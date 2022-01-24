//------------------------------------
// The aim of this application is to estime the wind speed. The device 
// composed by an esp32 and a 9 axis sensor (accelerometer, magnetometer, 
// gyroscope) have to be attached to a rotatinal moving object (e.g windmill blades) 
// to obtain the wind speed with the 9 axis data a previous calibration has been done
// Once the wind speed is obtained, it is sent via a BLE notification.
//
// Created on: JAN 21, 2022
// Author: Joaquín Sopena (joasop@gmail.com)
//------------------------------------


#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <string.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//GLOBAL VARIABLES

//SENSOR
struct Three_axis_sensor_pressure {
  float aX, gY;
};
QueueHandle_t SensorQueueS;

//BLE
bool deviceConnected = false;

//BLE server name
#define bleServerName "ESP32-JSA"
#define SERVICE_UUID BLEUUID((uint16_t)0x181A)   //Enviromental Sensing
BLECharacteristic bmeaccXCharacteristic(BLEUUID((uint16_t)0x2A70), BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeaccXDescriptor(BLEUUID((uint16_t)0x2902));
BLECharacteristic bmeaccXCharacteristic2(BLEUUID((uint16_t)0x2A72), BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeaccXDescriptor2(BLEUUID((uint16_t)0x2902));
 
//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


//SENSOR SENT VARIABLES
char acc_X[10], aux1[18], aux2[18];
//float gy, speed_rbd;
uint16_t gy = 0, ax = 0;

void capture_three_axis( void *pvParameters) {
  //Variables
  struct Three_axis_sensor_pressure TASP_S;
  //Struct to store the read values 

  const TickType_t xDelay250ms = pdMS_TO_TICKS (250);
  TickType_t xLastWakeTime;
  uint8_t cnt = 0;

  //Objects
  Adafruit_BMP280 bme; // Pressure sensor
  MPU9250_asukiaaa three_axis; //Accelerometer, Magnetometer,Gyroscope meter

  //Initialization.
  Wire.begin(21, 22);
  three_axis.setWire(&Wire);
  bme.begin();
  three_axis.beginAccel();
  three_axis.beginGyro();

  while (1) {
    if (cnt < 4) {
      three_axis.accelUpdate();
      three_axis.gyroUpdate();
      TASP_S.aX = TASP_S.aX + three_axis.accelX();
      TASP_S.gY = TASP_S.gY + three_axis.gyroY();
      cnt += 1;
    }

    else if (cnt == 4) {
      //Filter and send new data every second
      TASP_S.aX = TASP_S.aX / 4;
      TASP_S.gY = TASP_S.gY / 4;
      xQueueSendToBack(SensorQueueS, (void *) &TASP_S, 0);
      memset(&TASP_S, 0, sizeof(TASP_S)); //Set values to zero
      cnt = 0;
    }
    vTaskDelayUntil(&xLastWakeTime, xDelay250ms);
  }
}

void send_sensor_data_ble( void *pvParameters) {
  struct Three_axis_sensor_pressure TASP_R;
  BaseType_t xStatus; //To receive data from queue
  const TickType_t xDelay1s = pdMS_TO_TICKS (1000);
  TickType_t xLastWakeTime;

  float ax_r, speed_rbd_ax = 0, gy_r, speed_rbd_gy = 0;

  //BLE Peripheral
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *bmeService = pServer->createService(SERVICE_UUID);

  bmeService->addCharacteristic(&bmeaccXCharacteristic);
  bmeaccXCharacteristic.addDescriptor(&bmeaccXDescriptor);
  bmeService->addCharacteristic(&bmeaccXCharacteristic2);
  bmeaccXCharacteristic2.addDescriptor(&bmeaccXDescriptor2);

  // Start the service
  bmeService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  while (1) {
    if ( uxQueueSpacesAvailable != 0) {
      while ((xQueueReceive(SensorQueueS, (void *) &TASP_R, 0) == pdPASS)) {
      }
    }

    //variable auxiliar y concatenar
    //CONVERSION A INT.
    gy_r = (TASP_R.gY + 2000);
    ax_r = (TASP_R.aX+16);
    speed_rbd_ax = -0.0623*ax_r*ax_r + 3.0295*ax_r -32.592;
    speed_rbd_gy = -0.0157*gy_r + 32.914; 
    Serial.print(speed_rbd_ax);
    Serial.print(" ");
    Serial.println(speed_rbd_gy);
    snprintf(aux1, sizeof(aux1), "%.3f", speed_rbd_ax);  //estimated speed
    snprintf(aux2, sizeof(aux2), "%.3f", speed_rbd_gy);  //estimated speed
    
    if (deviceConnected) {
      bmeaccXCharacteristic.setValue(aux1);
      bmeaccXCharacteristic.notify();
      bmeaccXCharacteristic2.setValue(aux2);
      bmeaccXCharacteristic2.notify();
    }
    vTaskDelayUntil(&xLastWakeTime, xDelay1s);
  }
}


void setup() {
  Serial.begin(9600);
  SensorQueueS = xQueueCreate(1, sizeof( struct Three_axis_sensor_pressure));
  if (  SensorQueueS != NULL) { //Create tasks only if the queue is created succesfully
    xTaskCreate(capture_three_axis, "capture_three_axis", 2000, NULL, 1, NULL);
    xTaskCreate(send_sensor_data_ble, "send_sensor_data_ble", 3000, NULL, 1, NULL);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
