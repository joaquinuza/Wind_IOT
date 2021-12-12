#include <MPU9250_asukiaaa.h>
#include <Adafruit_BMP280.h>
#include <string.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <stdio.h>

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
char acc_X[10];
uint16_t value = 0;


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
  bmeaccXDescriptor.setValue("BME temperature Celsius");
  bmeaccXCharacteristic.addDescriptor(&bmeaccXDescriptor);

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
    snprintf(acc_X, sizeof(acc_X), "%f", TASP_R.aX);  //acceleration x obtained and to be sended via ble

    if (deviceConnected) {
      bmeaccXCharacteristic.setValue(acc_X);
      bmeaccXCharacteristic.notify(); 
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
