/*********
  Rui Santos
  Complete instructions at https://RandomNerdTutorials.com/esp32-ble-server-client/
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files.
  The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
*********/

#include "BLEDevice.h"


char acc_X[10];

//BLE GLOBAL
//----------------------------------------------------------------------------------------
//BLE Server name (the other ESP32 name running the server sketch)
#define bleServerName "ESP32-JSA"

/* UUID's of the service, characteristic that we want to read*/
// BLE Service
#define bmeServiceUUID        BLEUUID((uint16_t)0x181A)   //Enviromental Sensing

// BLE Characteristics
static BLEUUID bmeaccXCharacteristicUUID("8843c2c8-5b64-11ec-bf63-0242ac130002");


//Flags stating if should begin connecting and if the connection is up
static boolean doConnect = false;
static boolean connected = false;

//Address of the peripheral device. Address will be found during scanning...
static BLEAddress *pServerAddress;

//Characteristicd that we want to read
static BLERemoteCharacteristic* bmeaccXCharacteristic;

//Activate notify
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t notificationOff[] = {0x0, 0x0};


//Variables to store temperature and humidity
char* accXChar;


//Flags to check whether new temperature and humidity readings are available
boolean newaccX = false;


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
  bmeaccXCharacteristic = pRemoteService->getCharacteristic(bmeaccXCharacteristicUUID);

  if ( bmeaccXCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID");
    return false;
  }
  Serial.println(" - Found our characteristics");

  //Assign callback functions for the Characteristics
  bmeaccXCharacteristic->registerForNotify(accXNotifyCallback);
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
static void accXNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic,
                               uint8_t* pData, size_t length, bool isNotify) {
  //store temperature value
  accXChar = (char*)pData;
  newaccX = true;
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
        bmeaccXCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
        connected = true;
      } else {
        Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
      }
      doConnect = false;
    }
    //if new readings are available from ble server
    if (newaccX) {
      newaccX = false;
      //Serial.println(accXChar);
      snprintf(acc_X, sizeof(acc_X), "%s", accXChar);
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

  while(1){
      annemometer_value = analogRead(annemometerPin);
      Serial.print(annemometer_value);
      Serial.print(" ");
      Serial.println(acc_X);
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
