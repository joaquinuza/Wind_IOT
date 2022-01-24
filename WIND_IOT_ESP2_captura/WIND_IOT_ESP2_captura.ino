
#include "BLEDevice.h"


char sensor_t[51];
char adcval[5],sensor1[18],sensor2[18],sensor3[18];

//BLE GLOBAL
//----------------------------------------------------------------------------------------
//BLE Server name (the other ESP32 name running the server sketch)
#define bleServerName "ESP32-JSA"

/* UUID's of the service, characteristic that we want to read*/
// BLE Service
#define bmeServiceUUID        BLEUUID((uint16_t)0x181A)   //Enviromental Sensing

// BLE Characteristics
static BLEUUID bmesensor1CharacteristicUUID("8843c2c8-5b64-11ec-bf63-0242ac130002");
static BLEUUID bmesensor2CharacteristicUUID("f45902ea-60c1-11ec-8607-0242ac130002");
static BLEUUID bmesensor3CharacteristicUUID("0512bf0e-60c2-11ec-8607-0242ac130002");

//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;

//Characteristicd that we want to read
static BLERemoteCharacteristic* bmeasensor1Characteristic;
static BLERemoteCharacteristic* bmeasensor2Characteristic;
static BLERemoteCharacteristic* bmeasensor3Characteristic;

//Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};


//Variables to store temperature and humidity
char* sensor1Char;
char* sensor2Char;
char* sensor3Char;


//Flags to check whether new temperature and humidity readings are available
boolean newsensor1 = false, newsensor2 = false, newsensor3 = false;


//Connect to the BLE Server that has the name, Service, and Characteristics
bool connectToServer(BLEAddress pAddress) {
  BLEClient* pClient = BLEDevice::createClient();

  // Connect to the remove BLE Server.
  pClient->connect(pAddress);
  Serial.println(" - Connected to server");

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(bmeServiceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(bmeServiceUUID.toString().c_str());
    return (false);
  }

  // Obtain a reference to the characteristics in the service of the remote BLE server.
 bmeasensor1Characteristic = pRemoteService->getCharacteristic(bmesensor1CharacteristicUUID);
 bmeasensor2Characteristic = pRemoteService->getCharacteristic(bmesensor2CharacteristicUUID);
 bmeasensor3Characteristic = pRemoteService->getCharacteristic(bmesensor3CharacteristicUUID);

  if ( (bmeasensor1Characteristic == nullptr) ||  (bmeasensor2Characteristic == nullptr) ||  (bmeasensor3Characteristic == nullptr)) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");

  //Assign callback functions for the Characteristics
  bmeasensor1Characteristic->registerForNotify(sensor1NotifyCallback);
  bmeasensor2Characteristic->registerForNotify(sensor2NotifyCallback);
  bmeasensor3Characteristic->registerForNotify(sensor3NotifyCallback);
  return true;
}

//Callback function that gets called, when another device's advertisement has been received
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      if (advertisedDevice.getName() == bleServerName) { //Check if the name of the advertiser matches
        advertisedDevice.getScan()->stop(); //Scan can be stopped, we found what we are looking for
        pServerAddress = new BLEAddress(advertisedDevice.getAddress()); //Address of advertiser is the one we need
        doConnect = true; //Set indicator, stating that we are ready to connect
        Serial.println("Device found. Connecting!");
      }
    }
};

//When the BLE Server sends a new temperature reading with the notify property
static void sensor1NotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                               uint8_t* pData, size_t length, bool isNotify) {
  //store temperature value
  sensor1Char = (char*)pData;
  newsensor1 = true;
}

//When the BLE Server sends a new temperature reading with the notify property
static void sensor2NotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                               uint8_t* pData, size_t length, bool isNotify) {
  //store temperature value
  sensor2Char = (char*)pData;
  newsensor2 = true;
}

//When the BLE Server sends a new temperature reading with the notify property
static void sensor3NotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                               uint8_t* pData, size_t length, bool isNotify) {
  //store temperature value
  sensor3Char = (char*)pData;
  newsensor3 = true;
}
//----------------------------------------------------------------------------------------

void receive_data_ble( void *pvParameters) {
  /*Connects to a server and obtains data from its characteristics*/
  const TickType_t xDelay1s = pdMS_TO_TICKS (1000);
  TickType_t xLastWakeTime;

  //Init BLE device
  BLEDevice::init("");
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 30 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(60);

  while (1) {
    // If the flag "doConnect" is true then we have scanned for and found the desired
    // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
    // connected we set the connected flag to be true.
    if (doConnect == true) {
      if (connectToServer(*pServerAddress)) {
        Serial.println("We are now connected to the BLE Server.");
        //Activate the Notify property of each Characteristic
        bmeasensor1Characteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
        bmeasensor2Characteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
        bmeasensor3Characteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
        
        connected = true;
      } else {
        Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
      }
      doConnect = false;
    }
    //if new readings are available from ble server
    if (newsensor1) {
      newsensor1 = false;
      //Serial.println(accXChar);
      snprintf(sensor1, sizeof(sensor1), "%s", sensor1Char); 
    }
    if (newsensor2) {
      newsensor2 = false;
      //Serial.println(accXChar);
      snprintf(sensor2, sizeof(sensor2), "%s", sensor2Char); 
    }
    if (newsensor3) {
      newsensor3 = false;
      //Serial.println(accXChar);
      snprintf(sensor3, sizeof(sensor3), "%s", sensor3Char); 
    }


    vTaskDelayUntil(&xLastWakeTime, xDelay1s);
  }

}

void capture_send_data( void *pvParameters){
  /* Send data capture from other sensor via serial port*/
  const TickType_t xDelay1s = pdMS_TO_TICKS (1000);
  TickType_t xLastWakeTime;
  const uint16_t annemometerPin = 4;  //ADC2_CH0
  uint16_t annemometer_value = 0;
  float annemometer_voltage = 0;
  float annemometer_speed = 0;

  while(1){
      annemometer_value = analogRead(annemometerPin);
      //annemometer_voltage = (annemometer_value * 3.3)/4095; 
      //annemometer_speed = ((annemometer_voltage -0.3)/1.6)*32.4; 
      //Serial.println(annemometer_value);
      
//      Serial.print(sensor2);
//      Serial.println(sensor3);
//    Serial.print("Annemometer voltage ");
//    Serial.print(annemometer_voltage);
//    Serial.print(" Annemometer speed ");
//    Serial.println(annemometer_speed);
    
    snprintf(adcval, sizeof(adcval), "%04x", annemometer_value);
    //Serial.println(adcval);
    snprintf(sensor_t, sizeof(sensor_t), "%s%s%s%s", adcval, sensor1, sensor2, sensor3);
    Serial.println(sensor_t);   
      vTaskDelayUntil(&xLastWakeTime, xDelay1s); 
    }   
  } 


void setup() {

  //Start serial communication
  Serial.begin(9600);
  Serial.println("Starting Arduino BLE Client application...");
  xTaskCreate(receive_data_ble, "receive_data_ble", 3000, NULL, 2, NULL);
  xTaskCreate(capture_send_data, "capture_send_data", 2000, NULL, 1, NULL);

}

void loop() {
}
