//*****************************************************************************/
//  Author:        S. Devlin
//  Date:          Oct,2023
//  Version:       v1.0
//
/*******************************************************************************/

#include <ArduinoBLE.h>
#define TIMEOUT 45000
#define DEBUG 1

const int delay_time = 100;
const int characteristicSize = 64;
bool start=false;
char ble_buffer[63];

float act_temp, est_temp, voltage;
int act_heart, est_heart;

// create service and characteristics:
BLEService stringService("7DEF8317-7300-4EE6-8849-46FACE74CA2A");
BLEStringCharacteristic txCharacteristic("7DEF8317-7301-4EE6-8849-46FACE74CA2A", BLERead | BLENotify, characteristicSize);
BLEStringCharacteristic rxCharacteristic("00002A3D-0000-1000-8000-00805f9b34fb", BLEWrite | BLENotify, 5);


void setup() {
  if (DEBUG) {
    Serial.begin(9600);
  }
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(A0, INPUT);

  while (!BLE.begin()) {
    digitalWrite(LED_BUILTIN, LOW); 
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH); 
    delay(500);
  }

  BLE.setLocalName("BLE_String");
  BLE.setAdvertisedService(stringService);
  stringService.addCharacteristic(txCharacteristic);
  stringService.addCharacteristic(rxCharacteristic);
  BLE.setEventHandler(BLEConnected, connectHandler);
  BLE.setEventHandler(BLEDisconnected, disconnectHandler);
  rxCharacteristic.setEventHandler(BLEWritten, onRxCharValueUpdate);
  BLE.addService(stringService);
  BLE.advertise();
}

void loop() {

  delay(delay_time);
  BLE.poll();

  if (start)

    {
    voltage = random(35)/10 ; 
    act_temp = random(5) + 33;  
    est_temp = random(5) + 34 ;
    act_heart = random(5) + 67 ;
    est_heart = random(5) + 69  ;
    updateBLE();
    }


}


void connectHandler(BLEDevice central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
}

void disconnectHandler(BLEDevice central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void onRxCharValueUpdate(BLEDevice central, BLECharacteristic characteristic) {
  // central disconnected event handler
  char tmp[2];
  rxCharacteristic.readValue(tmp, 1);
  if (tmp[0] == 'G') {
    start = true;
    if (DEBUG) Serial.println("Go");  
  } else if (tmp[0] == 'S')
  {
    start = false;
    if (DEBUG) Serial.println("Stop");  
  }
}


void updateBLE() {

  sprintf (ble_buffer, "%1.1f,%2.1f,%2.1f,%3d,%3d", voltage, act_temp, est_temp, act_heart, est_heart);
  Serial.print("BLE TEXT:");
  Serial.println(ble_buffer);
  txCharacteristic.writeValue(ble_buffer);
}

