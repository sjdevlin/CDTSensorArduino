//*****************************************************************************/
//  Author:        S. Devlin
//  Date:          Oct,2023
//  Version:       v1.0
//*******************************************************************************/

// state machine states
#define STOPPED 1
#define STARTING 2
#define RUNNING 3

// Two external classes used, one for BLE and one for Peak Detection
#include <ArduinoBLE.h>
#include "./PeakDetection.h"

//debug flag
const int DEBUG = 0;

//pins
const int POWER_PIN_OUT = 13;
const int PULSE_OFFSET_VOLTAGE_OUT = DAC;
const int PULSE_PEAK_VOLTAGE_IN = A3;
const int PULSE_VOLTAGE_IN = A2;
const int TEMP_VOLTAGE_IN = A1;

//constants
const int BLE_BUFFER_SIZE = 64;
const int MINIMUM_PULSE_DELAY_MS = 300;  // Used to reject any abnormal pulse peaks
const int POLLING_DELAY = 2;

//averaging rates for exponential smoothing
float ALPHA_PEAK = .05;  // these are out of 100
const float ALPHA_PULSE = .25;
const float ALPHA_HEART = .20;

//status
char status = STOPPED;

//working variables
float act_temperature, est_temperature;
float temperature_readings_array[3];  // this array is a rolling store of the last two temp readings used for prediction algorithm
float battery_voltage = 4.7;          // not curretly used - but may be added
long temperature_analog_in;
int differential_output, transimpedence_output;
float smoothed_transimpedence_output, smoothed_differential_output;
float last_smoothed_pulse_value;
float heart_rate;
float scale_factor = 1.05;
float smoothed_heart_rate;
int peak = 0;

//timing variables
unsigned long time_now, time_since_last_beat, time_of_last_beat;
unsigned long time_last_temp_reading = 0;

//counter for temperature prediction algorithm
int reading_counter = 0;
int temp;

// Custom object for peak detection using dispersion
PeakDetection peakDetection;

// create service and characteristics:
char ble_buffer[BLE_BUFFER_SIZE];
BLEService stringService("7DEF8317-7300-4EE6-8849-46FACE74CA2A");
BLEStringCharacteristic txCharacteristic("7DEF8317-7301-4EE6-8849-46FACE74CA2A", BLERead | BLENotify, BLE_BUFFER_SIZE);
BLEStringCharacteristic rxCharacteristic("00002A3D-0000-1000-8000-00805f9b34fb", BLEWrite | BLENotify, 5);

void setup() {

  if (DEBUG) Serial.begin(9600);

  peakDetection.begin(10, 2, 0.6);  // a) lag 300 points in 3 seconds, b) Threhold in STDEVS c) Not sure 0?

  // set resolution for ADC and DAC
  analogWriteResolution(12);
  analogReadResolution(12);

  // use external analogue reference. **investigate later !
  //analogReference(AR_EXTERNAL);  // not using at moment

  // set up pins for input and output
  pinMode(POWER_PIN_OUT, OUTPUT);
  pinMode(TEMP_VOLTAGE_IN, INPUT);
  pinMode(PULSE_VOLTAGE_IN, INPUT);
  pinMode(PULSE_PEAK_VOLTAGE_IN, INPUT);
  pinMode(PULSE_OFFSET_VOLTAGE_OUT, OUTPUT);

  // start bluetooth
  while (!BLE.begin()) {
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
  }

  // set up BLE service, eventhandlers and advertise
  BLE.setLocalName("BLE_String");
  BLE.setAdvertisedService(stringService);
  stringService.addCharacteristic(txCharacteristic);
  stringService.addCharacteristic(rxCharacteristic);
  BLE.setEventHandler(BLEConnected, connectHandler);
  BLE.setEventHandler(BLEDisconnected, disconnectHandler);
  rxCharacteristic.setEventHandler(BLEWritten, onRxCharValueUpdate);
  BLE.addService(stringService);
  BLE.advertise();

  //ensure that power is "off" to the sensor circuit to save battery
  digitalWrite(POWER_PIN_OUT, LOW);
}

// main loop

void loop() {

  time_now = millis();
  delay(POLLING_DELAY);
  BLE.poll();  // check for BLE comms from the iphone app

  switch (status) {
    case RUNNING:


      // temperature - measure every 1.5 secs
      if (time_now - time_last_temp_reading > 1500) {
        time_last_temp_reading = time_now;

        // following code takes 10 readings and averages to remove noise from ADC
        temperature_analog_in = 0;
        for (int counter = 0; counter < 10; counter++) {
          temperature_analog_in += analogRead(TEMP_VOLTAGE_IN);
        }
        temperature_analog_in = temperature_analog_in / 10;

        act_temperature = (float)(temperature_analog_in + 6690.0) / 227.0;  //this formula came from callibaration//;

        // Use the historical termperature like a shift register
        temperature_readings_array[0] = temperature_readings_array[1];
        temperature_readings_array[1] = temperature_readings_array[2];
        temperature_readings_array[2] = act_temperature;
        est_temperature = predict_temperature();
      }

      // calculate heart rate
      transimpedence_output = analogRead(PULSE_PEAK_VOLTAGE_IN);

      smoothed_transimpedence_output = (ALPHA_PEAK * (float)transimpedence_output) + ((1.0 - ALPHA_PEAK) * smoothed_transimpedence_output);

      if (smoothed_differential_output < 1000) scale_factor += 0.001;
      if (smoothed_differential_output >= 1000) scale_factor -= 0.001;  // this scaling ensures that we constantly keep output in a good range

      temp = (int)(smoothed_transimpedence_output * scale_factor);
      analogWrite(PULSE_OFFSET_VOLTAGE_OUT, temp);  // this is where we apply the offset

      differential_output = analogRead(PULSE_VOLTAGE_IN);
      last_smoothed_pulse_value = smoothed_differential_output;
      smoothed_differential_output = (ALPHA_PULSE * (float)differential_output) + ((1.0 - ALPHA_PULSE) * smoothed_differential_output);

      peakDetection.add((double)differential_output);
      if (peakDetection.getPeak() == 1) {
        if (peak != 1 && time_now - time_since_last_beat > MINIMUM_PULSE_DELAY_MS) {
          heart_rate = 60000.0 / (float)(time_now - time_since_last_beat);
          smoothed_heart_rate = (ALPHA_HEART * heart_rate) + ((1.0 - ALPHA_HEART) * smoothed_heart_rate);
          peak = 1;
          time_since_last_beat = time_now;
        }
      } else {
        peak = 0;
      }

      if (DEBUG) {
        Serial.print(peakDetection.getFilt());
        Serial.print(",");
        Serial.print(differential_output);
        Serial.print(",");
        Serial.print(peak * 500);
        Serial.print(",");
        Serial.println(smoothed_heart_rate * 10);
      }

      updateBLE();
      break;

    case STOPPED:
      digitalWrite(POWER_PIN_OUT, LOW);
      break;

    case STARTING:
      // switch on power to sensing ciircuit
      digitalWrite(POWER_PIN_OUT, HIGH);
      heart_rate = 70.0;
      smoothed_heart_rate = 70.0;  // start off with reasonable number so it converges quicker
      // set the smoothed average as the initial value to avoid ramp up
      smoothed_transimpedence_output = (float)analogRead(PULSE_PEAK_VOLTAGE_IN);
      smoothed_differential_output = 0;

      //get intial temperature
      temperature_analog_in = 0;
      for (int counter = 0; counter < 10; counter++) {
        temperature_analog_in += analogRead(TEMP_VOLTAGE_IN);
      }
      temperature_analog_in = temperature_analog_in / 10;

      act_temperature = (float)(temperature_analog_in + 6690.0) / 227.0;  //this formula came from callibaration//;
      temperature_readings_array[2] = act_temperature;
      // Initialise historical termperature array with current value
      temperature_readings_array[0] = temperature_readings_array[2];
      temperature_readings_array[1] = temperature_readings_array[2];

      status = RUNNING;
      break;
  }
}
float predict_temperature() {
  if (temperature_readings_array[2] - temperature_readings_array[1] > 0.1 && temperature_readings_array[1] - temperature_readings_array[0] > 0.1) {
    double nominator = (temperature_readings_array[1] * temperature_readings_array[1]) - (temperature_readings_array[0] * temperature_readings_array[2]);
    double denominator = (2 * temperature_readings_array[1]) - temperature_readings_array[0] - temperature_readings_array[2];
    return (float)(nominator / denominator);
  } else {
    return (temperature_readings_array[2]);
  }
}

void updateBLE() {
  sprintf(ble_buffer, "%1.1f,%2.1f,%2.1f,%2.0f,%2.0f,", battery_voltage, est_temperature, act_temperature, smoothed_heart_rate, heart_rate);
  if (DEBUG) Serial.print("BLE TEXT:");
  if (DEBUG) Serial.println(ble_buffer);
  txCharacteristic.writeValue(ble_buffer);
}



void connectHandler(BLEDevice central) {
  // central connected event handler
  if (DEBUG) Serial.print("Connected event, central: ");
  if (DEBUG) Serial.println(central.address());
}

void disconnectHandler(BLEDevice central) {
  // central disconnected event handler
  if (DEBUG) Serial.print("Disconnected event, central: ");
  if (DEBUG) Serial.println(central.address());
}

void onRxCharValueUpdate(BLEDevice central, BLECharacteristic characteristic) {
  // this is called when a message is received from the central (iPhone)
  // this will just be a single character string (an array of 2 chars including the terminator)
  // depending on the value (G or S) the arduino will enable or disable the sensing circuit
  char tmp[2];
  rxCharacteristic.readValue(tmp, 1);
  if (tmp[0] == 'G') {
    status = STARTING;
    if (DEBUG) Serial.println("Started Sensing");
  } else if (tmp[0] == 'S') {
    status = STOPPED;
    if (DEBUG) Serial.println("Stopped Sensing.");
  }
}

