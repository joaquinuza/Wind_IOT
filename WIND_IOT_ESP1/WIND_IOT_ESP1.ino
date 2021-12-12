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
#define SERVICE_UUID        BLEUUID((uint16_t)0x181A)   //Enviromental Sensing
#define CHAR_READ_UUID      BLEUUID((uint16_t)0x2713)   //Acceleration

// --------
// Global variables
// --------
static BLEServer* g_pServer = nullptr;
static BLECharacteristic* g_pCharRead = nullptr;
static bool g_centralConnected = false;
static std::string g_cmdLine;
//SENSOR SENDED VARIABLES
char acc_X[10];
uint16_t value = 0;

// --------
// Bluetooth event callbacks
// --------
class MyServerCallbacks: public BLEServerCallbacks
{
    void onConnect(BLEServer* pServer) override
    {
      Serial.println("onConnect");
      g_centralConnected = true;
    }

    void onDisconnect(BLEServer* pServer) override
    {
      Serial.println("onDisconnect, will start advertising");
      g_centralConnected = false;
      BLEDevice::startAdvertising();
    }
};

class MyCharPrintingCallbacks: public BLECharacteristicCallbacks
{
  public:
    explicit MyCharPrintingCallbacks(const char* name) : m_name(name) {}

  private:
    void PrintEvent(const char* event, const char* value)
    {
      Serial.print(event);
      Serial.print("(");
      Serial.print(m_name.c_str());
      Serial.print(")");
      if (value)
      {
        Serial.print(" value='");
        Serial.print(value);
        Serial.print("'");
      }
      Serial.println();
    }

  private:
    void onRead(BLECharacteristic* pCharacteristic) override
    {
      PrintEvent("onRead", pCharacteristic->getValue().c_str());
    }

    void onWrite(BLECharacteristic* pCharacteristic) override
    {
      PrintEvent("onWrite", pCharacteristic->getValue().c_str());
    }

    void onNotify(BLECharacteristic* pCharacteristic) override
    {
      PrintEvent("onNotify", pCharacteristic->getValue().c_str());
    }

    void onStatus(BLECharacteristic* pCharacteristic, Status status, uint32_t code) override
    {
      std::string event("onStatus:");
      switch (status)
      {
        case SUCCESS_INDICATE: event += "SUCCESS_INDICATE"; break;
        case SUCCESS_NOTIFY: event += "SUCCESS_NOTIFY"; break;
        case ERROR_INDICATE_DISABLED: event += "ERROR_INDICATE_DISABLED"; break;
        case ERROR_NOTIFY_DISABLED: event += "ERROR_NOTIFY_DISABLED"; break;
        case ERROR_GATT: event += "ERROR_GATT"; break;
        case ERROR_NO_CLIENT: event += "ERROR_NO_CLIENT"; break;
        case ERROR_INDICATE_TIMEOUT: event += "ERROR_INDICATE_TIMEOUT"; break;
        case ERROR_INDICATE_FAILURE: event += "ERROR_INDICATE_FAILURE"; break;
      }
      event += ":";
      event += String(code).c_str();
      PrintEvent(event.c_str(), nullptr);
    }

  private:
    std::string m_name;
};
//----------------------------------------------------

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
  BLEDevice::init("ESP32-JSA");
  g_pServer = BLEDevice::createServer();
  g_pServer->setCallbacks(new MyServerCallbacks());    //onConnect, onDisconnect
  BLEService* pService = g_pServer->createService(SERVICE_UUID);

    // characteristic for read //HERE SENSOR DATA CHARACTERISITCS SHOULD BE ADDED
  {
    uint32_t propertyFlags = BLECharacteristic::PROPERTY_READ;
    BLECharacteristic* pCharRead = pService->createCharacteristic(CHAR_READ_UUID, propertyFlags);
    pCharRead->setCallbacks(new MyCharPrintingCallbacks("CharRead"));
    g_pCharRead = pCharRead;
  }
  
  pService->start();
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  // this fixes iPhone connection issue (don't know how it works)
  {
    pAdvertising->setMinPreferred(0x06);
    pAdvertising->setMinPreferred(0x12);
  }
  BLEDevice::startAdvertising();

  Serial.println("BLE Peripheral setup done, advertising");

  while (1) {
    if ( uxQueueSpacesAvailable != 0) {
      while ((xQueueReceive(SensorQueueS, (void *) &TASP_R, 0) == pdPASS)) {
      }
    }
    snprintf(acc_X, sizeof(acc_X), "%f", TASP_R.aX);
    g_pCharRead->setValue(acc_X);
    vTaskDelayUntil(&xLastWakeTime, xDelay1s);
  }
}



void setup() {
  Serial.begin(9600);
  SensorQueueS = xQueueCreate(1, sizeof( struct Three_axis_sensor_pressure));
  if (  SensorQueueS != NULL) { //Create tasks only if the queue is created succesfully
    xTaskCreate(capture_three_axis, "capture_three_axis", 2000, NULL, 1, NULL);
    xTaskCreate(send_sensor_data_ble, "send_sensor_data_ble", 2000, NULL, 1, NULL);
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
