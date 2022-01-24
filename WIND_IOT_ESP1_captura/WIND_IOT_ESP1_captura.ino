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
//----------------------------------------------------

//GLOBAL VARIABLES

//SENSOR
struct Three_axis_sensor_pressure {
  float aX, aY, aZ, gX, gY, gZ, mDirection, mX, mY, mZ, temp, pressure;
};
QueueHandle_t SensorQueueS;

//BLE
bool deviceConnected = false;

//BLE server name
#define bleServerName "ESP32-JSA"
#define SERVICE_UUID BLEUUID((uint16_t)0x181A)   //Enviromental Sensing
BLECharacteristic bmeaccXCharacteristic("8843c2c8-5b64-11ec-bf63-0242ac130002", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeaccXDescriptor(BLEUUID((uint16_t)0x2902));
BLECharacteristic bmeaccXCharacteristic2("f45902ea-60c1-11ec-8607-0242ac130002", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeaccXDescriptor2(BLEUUID((uint16_t)0x2902));
BLECharacteristic bmeaccXCharacteristic3("0512bf0e-60c2-11ec-8607-0242ac130002", BLECharacteristic::PROPERTY_NOTIFY);
BLEDescriptor bmeaccXDescriptor3(BLEUUID((uint16_t)0x2902));
//Setup callbacks onConnect and onDisconnect
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


//SENSOR SENDED VARIABLES
char acc_X[10], aux1[18],aux2[18],aux3[18], aux4[46];
char three_axis_v[20];
char three_axis_v_aux[20];
char acc_comma[2] = ",";
int16_t ax, ay, az;
uint32_t gx, gy, gz, mx, my, mz;
float aux_f = 0;


void capture_three_axis( void *pvParameters) {
  //Variables
  struct Three_axis_sensor_pressure TASP_S;
  //Struct to store the read values from both i2c sensor not filtered

  const TickType_t xDelay250ms = pdMS_TO_TICKS (250);
  TickType_t xLastWakeTime;
  uint8_t cnt = 0;

  //Objects
  Adafruit_BMP280 bme; // Pressure sensor
  MPU9250_asukiaaa three_axis; //Accelerometer, Magnetometer,Gyroscope meter

  //Initialization.
#ifdef _ESP32_HAL_I2C_H_ // For ESP32
  Wire.begin(21, 22);
  three_axis.setWire(&Wire);
#endif
  bme.begin();
  three_axis.beginAccel();
  three_axis.beginGyro();
  three_axis.beginMag();

  while (1) {
    if (cnt < 4) {
      three_axis.accelUpdate();
      three_axis.gyroUpdate();
      three_axis.magUpdate();
      TASP_S.aX = TASP_S.aX + three_axis.accelX();
      TASP_S.aY = TASP_S.aY + three_axis.accelY();
      TASP_S.aZ = TASP_S.aZ + three_axis.accelZ();
      TASP_S.gX = TASP_S.gX + three_axis.gyroX();
      TASP_S.gY = TASP_S.gY + three_axis.gyroY();
      TASP_S.gZ = TASP_S.gZ + three_axis.gyroZ();
      TASP_S.mDirection = TASP_S.mDirection + three_axis.magHorizDirection();
      TASP_S.mX = TASP_S.mX + three_axis.magX();
      TASP_S.mY = TASP_S.mY + three_axis.magY();
      TASP_S.mZ = TASP_S.mZ + three_axis.magZ();
      TASP_S.temp = TASP_S.temp + bme.readTemperature();
      TASP_S.pressure = TASP_S.pressure + bme.readPressure();
      cnt += 1;
    }

    else if (cnt == 4) {
      //Filter and send new data every second

      TASP_S.aX = TASP_S.aX / 4;
      TASP_S.aY = TASP_S.aY / 4;
      TASP_S.aZ = TASP_S.aZ / 4;
      TASP_S.gX = TASP_S.gX / 4;
      TASP_S.gY = TASP_S.gY / 4;
      TASP_S.gZ = TASP_S.gZ / 4;
      TASP_S.mDirection = TASP_S.mDirection / 4;
      TASP_S.mX = TASP_S.mX / 4;
      TASP_S.mY = TASP_S.mY / 4;
      TASP_S.mZ = TASP_S.mZ / 4;
      TASP_S.temp = TASP_S.temp / 4;
      TASP_S.pressure = TASP_S.pressure / 4;
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

  //BLE Peripheral
  BLEDevice::init(bleServerName);

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *bmeService = pServer->createService(SERVICE_UUID);

  bmeService->addCharacteristic(&bmeaccXCharacteristic);
  bmeaccXDescriptor.setValue("BME axayazgx");
  bmeaccXCharacteristic.addDescriptor(&bmeaccXDescriptor);

  bmeService->addCharacteristic(&bmeaccXCharacteristic2);
  bmeaccXDescriptor2.setValue("BME gygzmx");
  bmeaccXCharacteristic2.addDescriptor(&bmeaccXDescriptor2);

  bmeService->addCharacteristic(&bmeaccXCharacteristic3);
  bmeaccXDescriptor3.setValue("BME gygzmx");
  bmeaccXCharacteristic3.addDescriptor(&bmeaccXDescriptor3);
  

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
    aux_f = (TASP_R.aX + 16) * 1000;
    ax = (int16_t)aux_f;
    aux_f = (TASP_R.aY + 16) * 1000;
    ay = (int16_t)aux_f;
    aux_f = (TASP_R.aZ + 16) * 1000;
    az = (int16_t)aux_f;
    aux_f = (TASP_R.gX + 2000) * 100;
    gx = (int32_t) aux_f;
    aux_f = (TASP_R.gY + 2000) * 100;
    gy = (int32_t) aux_f;
    aux_f = (TASP_R.gZ + 2000) * 100;
    gz = (int32_t) aux_f;
    aux_f = (TASP_R.mX + 16384) * 100;
    mx = (int32_t) aux_f;
    aux_f = (TASP_R.mY + 16384) * 100;
    my = (int32_t) aux_f;
    aux_f = (TASP_R.mZ + 16384) * 100;
    mz = (int32_t) aux_f;
    snprintf(aux1, sizeof(aux1), "%04x%04x%04x%05x", ax, ay, az, gx);
    snprintf(aux2, sizeof(aux2), "%05x%05x%06x", gy, gz, mx);
    snprintf(aux3, sizeof(aux3), "%06x%06x", my, mz);
    snprintf(aux4, sizeof(aux4), "%s%s%s", aux1, aux2, aux3);
    //Serial.println(aux4);
//    Serial.println(sensor_t);
//    Serial.print(aux1);
//    Serial.print(aux2);
//    Serial.println(aux3);


    //snprintf(aux, sizeof(aux), "%f", TASP_R.aX);  //acceleration x obtained and to be sended via ble
    //snprintf(aux, sizeof(aux), "%f", TASP_R.aX);  //acceleration x obtained and to be sended via ble
    //snprintf(three_axis_v, sizeof(three_axis_v), "%02X%s",aux, acc_comma);
    //three_axis_v_aux =
    //snprintf(aux, sizeof(aux), "%f", TASP_R.aY);  //acceleration x obtained and to be sended via ble
    //snprintf(three_axis_v, sizeof(three_axis_v), "%s%s%s",three_axis_v,acc_comma,aux);
    //snprintf(aux, sizeof(aux), "%f", TASP_R.aZ);  //acceleration x obtained and to be sended via ble
    //snprintf(three_axis_v, sizeof(three_axis_v), "%s%s%s",three_axis_v,acc_comma,aux);
    //snprintf(acc_X, sizeof(acc_X), "%f", TASP_R.aX);  //acceleration x obtained and to be sended via ble

    if (deviceConnected) {
      bmeaccXCharacteristic.setValue(aux1);
      bmeaccXCharacteristic.notify();
      bmeaccXCharacteristic2.setValue(aux2);
      bmeaccXCharacteristic2.notify();
      bmeaccXCharacteristic3.setValue(aux3);
      bmeaccXCharacteristic3.notify();
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
