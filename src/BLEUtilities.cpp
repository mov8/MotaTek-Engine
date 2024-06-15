#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEDescriptor.h>
#include <BLE2902.h>
#include <BLE2904.h>
#include "esp_ota_ops.h"

#include "BLEUtilities.h"


#define DEBUG

// Defined in main.cpp 
// https://github.com/espressif/esp-idf/blob/master/examples/bluetooth/bluedroid/ble/gatt_server/tutorial/Gatt_Server_Example_Walkthrough.md

/*
abd5a5cf-38b0-46ec-a766-1a6fad72a197

aaae6fb2-3b23-4a60-825f-ec242599bae1
*/

#define SERVER_NAME                           "Engine"
#define SERVICE_UUID                          "abd5a5cf-38b0-46ec-a766-1a6fad72a197"
#define CHARACTERISTIC_PC_DATA_UUID           "aaae6fb2-3b23-4a60-825f-ec242599bae1" 

// #define SERVER_NAME                           "Sentinel"
// #define SERVICE_UUID                          "19b10010-e8f2-537e-4f6c-d104768a1214"

const uint8_t version_major = 1;
const uint8_t version_minor = 0;
const uint8_t version_patch = 1;


/*SERVER -> PHONE*/
BLECharacteristic *pCharacteristicPhoneData;
BLECharacteristic *pCharacteristicPhoneControl;

/*PHONE -> SERVER*/
BLECharacteristic *pCharacteristicOtaServerData;
BLECharacteristic *pCharacteristicOtaControl;
BLECharacteristic *pCharacteristicOtaRepeaterData;

/*SERVER -> REPEATER*/
BLECharacteristic *pCharacteristicRepeaterData;
BLECharacteristic *pCharacteristicRepeaterControl;
BLECharacteristic *pCharacteristicOtaForwardedData;

/*REPEATER -> SERVER*/


int connectedDevices = 0;
int _confirmation = 0;
uint8_t dataSent = 0;
OtaToken otaStatus = OTA_IDLE;
OtaToken otaRemote = OTA_IDLE;
OtaTarget otaTarget = MONITOR;
esp_ota_handle_t otaHandler = 0;
uint16_t chunkSize = 200;
uint16_t chunks = 0;
uint16_t chunksSent = 0;
uint8_t chunksPerIncrement = 0;
uint8_t chunkIncrementer = 0;
uint8_t percentSent = 0;

CommandToken dataToSend = START_POLLING;

bool BLEStatus::otaUpdating = false;
int  BLEStatus::otaTimeout = 0;

/*
FORWARD DECLARATIONS
*/

uint8_t* convert16to8bitValues();
void sendVersion();

/*
*/

const char* otaCommands[11] = { 
  "OTA_IDLE",
  "OTA_ERROR",
  "OTA_RESTART",
  "OTA_REPEATER_REQUEST",
  "OTA_REPEATER_READY",
  "OTA_REPEATER_UPDATING",
  "OTA_REPEATER_UPDATED",
  "OTA_MONITOR_REQUEST",
  "OTA_MONITOR_READY",
  "OTA_MONITOR_UPDATING",
  "OTA_MONITOR_UPDATED",
  };



void bluetoothStatus(int delay) {
  char buffer[10];
  sprintf(buffer, "paired: %u", connectedDevices); 
  Serial.println("Bluetooth connected devices:");
  Serial.println(buffer);
}

void processSentData(std::string value) {
  if (value.length() > 0) {
    Serial.println("ProcessSentData");
    for (int i = 0; i < 6; i++) {
      BLEPortValues::values[i] = static_cast<uint8_t>(value[i]);
    }
    char buff[40];
    sprintf(buff, "Characteristic:[%u, %u, %u, %u, %u, %u]", BLEPortValues::values[0], 
      BLEPortValues::values[1], BLEPortValues::values[2], BLEPortValues::values[3], 
      BLEPortValues::values[4], BLEPortValues::values[5]);
    Serial.println(buff);
  } 
}


class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    BLEStatus::deviceConnected = true;
    ++connectedDevices;
    bluetoothStatus(20);
    Serial.println("Server - new device connected..");
    if(connectedDevices < 2) {      // Connect turns off advertising 
      Serial.println("Server advertising..");
      pServer->getAdvertising()->start();  
    } else {
      Serial.println("Server NOT advertising..");        
    }
    otaStatus = OTA_IDLE;
  };

  void onDisconnect(BLEServer* pServer) {
    if (otaStatus == OTA_RESTART) {
      Serial.println("Restarting after OTA");
      otaStatus == OTA_IDLE;
      delay(2000);
      esp_restart();
    } else {
      Serial.println("Device disconnected - restart advertising");
      BLEStatus::deviceConnected = --connectedDevices > 0;
      pServer->getAdvertising()->start();
      bluetoothStatus(50);
    }
  }
};


/*
CALLBACKS - FOR BOTH THE PHONE (MONITOR) AND REPEATER
  ONREAD IS USED TO SEND BACK THE VERSION 
  ONWRITE TAKES THE DATA SENT BY THE PHONE - PORT DATA - AND PROCESSES IT
*/
class BtPCDataCallbacks: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic  *pCharacteristicPhoneData) {
    Serial.println("PC read data - ");
    uint8_t values[3] = {version_major, version_minor, version_patch};
    pCharacteristicPhoneData->setValue((uint8_t*)&values, 3);
    // pCharacteristicPhoneData->notify();
  }
  void onWrite(BLECharacteristic  *pCharacteristicPhoneData) {
    Serial.println("PC wrote data - ");
    std::string sentValue = pCharacteristicPhoneData->getValue();
    processSentData(sentValue);
  }
};
 
/*
ANCILLARY FUNCTIONS TO PROCESS DATA
*/
uint8_t* convert16to8bitValues() {  // Going to return a address of the array
  // BLE can only send an array of 8 bit values so have to combine 2 to get 16 bit values <= 65,535
  static uint8_t values [12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  for (int i = 0; i < 6; i++) {
    values[i*2] = (BLEPortValues::values[i] >> 8) & 0xFF;
    values[i*2+1] = (BLEPortValues::values[i] >> 0) & 0xFF;
  }
  return values;  // returns address of array
}



/*
TRANSMIT() SENDS BOTH PORT SETUP DATA AND THE RESULTS TO THE 
REPEATER "19b10013-e8f2-537e-4f6c-d104768a1214"
*/
void transmitData() {
  if (dataToSend < START_POLLING - 10) {
 //   std::string jsonString = portDetailToJson(Details::portDetails[(uint8_t)dataSent]); 
 //   Serial.println(jsonString.c_str());
 //   pCharacteristicRepeaterData->setValue(jsonString);
  } else if (dataToSend == START_POLLING) {
    uint8_t* values = convert16to8bitValues();  // converts reurned pointer to array
    pCharacteristicRepeaterData->setValue(values, 12);
    pCharacteristicPhoneData->setValue(values, 12);
  }
  pCharacteristicRepeaterData->notify();
  pCharacteristicPhoneData->notify();
//  char buff[30];
//  sprintf(buff, "Transmitting: %u", (uint8_t)dataToSend);
//  Serial.println(buff);
  return;
}

/*
=> Data Characteristic
-> Control Characteristic

Flashing Monitor
Phone -   OTA_MONITOR_REQUEST  -> Monitor
Phone <-  OTA_MONITOR_READY     - Monitor
Phone =   byteList             => Monitor
Phone <-  OTA_MONITOR_UPDATING  - Monitor
Phone =   Firware chunks       => Monitor
Phone <-  OTA_MONITOR_UPDATED   - Monitor
Phone -   OTA_RESTART          -> Monitor  

Flashing Repeater
Phone -   OTA_REPEATER_REQUEST -> Monitor -  OTA_REPEATER_REQUEST  -> Repeater
Phone <-  OTA_REPEATER_READY   -  Monitor <- OTA_REPEATER_READY    -  Repeater
Phone =   byteList             => Monitor =   byteList             => Repeater
Phone <-  OTA_REPEATER_UPDATING - Monitor <-  OTA_REPEATER_UPDATING - Repeater
Phone =   Firmware chunks      => Monitor =   Firmware stream      => Repeater
Phone <-  OTA_REPEATER_UPDATED -  Monitor <-  OTA_REPEATER_UPDATED  - Repeater
        

*/

/*
CONTROL THE REPEATER'S OTA DATA COMING FROM THE PHONE ON
19b10014-e8f2-537e-4f6c-d104768a1214
*/


/*
REQUESTS FROM THE REPEATER
The repeater does a pCharacteristicRepeaterControl->readValue
pCharacteristicRepeaterControl 19b10017-e8f2-537e-4f6c-d104768a1214
pCharacteristicRepeaterData 19b10013-e8f2-537e-4f6c-d104768a1214
then sends the json data for each of the six ports
portDetails is and array of repeater poer values as json
*/      


class BtRepeaterControlCallbacks: public BLECharacteristicCallbacks {
  /*
  Handles requests from the Repeater on pCharacteristicRepeaterControl
  UUD = 19b10017...
  */
  void onWrite(BLECharacteristic  *pCharacteristicRepeaterControl) {
    Serial.println("pCharacteristicRepeaterControl onWrite callback");
    uint8_t dataSent = *pCharacteristicRepeaterControl->getData();
    // When Reeater sarts it asks the Monitor for the port details
    std::string jsonString;
    switch (dataSent) {
      case SEND_PORT_1 ... START_POLLING: 
        Serial.println("Request for port details");
      //  jsonString = portDetailToJson(Details::portDetails[dataSent-11]); 
        Serial.println(jsonString.c_str());
        pCharacteristicRepeaterData->setValue(jsonString);
        pCharacteristicRepeaterData->notify();
        break;
      case OTA_REPEATER_READY:
        /* Sent from repeater to say it's ready for firmware data
          so now tell the phone to send the firmware */
        Serial.println("OTA_REPEATER_READY - Tell phone Repeater is ready for OTA");
        BLEStatus::otaUpdating = true;
        BLEStatus::otaTimeout = 0;
        otaStatus = OTA_REPEATER_READY;
     //   writeToDisplay("Flashing ", "Repeater", 10);      
        // Tell the phone we're ready to go..
        pCharacteristicOtaControl->setValue((uint8_t*)&otaStatus, 1);
        pCharacteristicOtaControl->notify();
        break;
      case OTA_REPEATER_UPDATING:
        Serial.println("OTA_REPEATER_UPDATING - Tell phone Repeater is updating");
        otaStatus = OTA_REPEATER_UPDATING; 
        pCharacteristicOtaControl->setValue((uint8_t*)&otaStatus, 1);
        pCharacteristicOtaControl->notify();
        break;
      case OTA_REPEATER_UPDATED:
        Serial.println("OTA_REPEATER_UPDATED - Tell phone Repeater is updated");
        otaStatus = OTA_REPEATER_UPDATED;
        pCharacteristicOtaControl->setValue((uint8_t*)&otaStatus, 1);
        pCharacteristicOtaControl->notify();
        break;        
    }

    char buff[20];
    sprintf(buff, "dataSent = %u", (uint8_t)dataSent);
    Serial.println(buff);
  }

  void onRead(BLECharacteristic  *pCharacteristicRepeaterControl) {
    /*    
    REPEATER ONLY SENDS COMMANDS TO THE MONITOR - NEVER READS THIS CHANNEL
    */
  }
};



/*

 Handle the OTA data being sent for Monitor - the server
 Initially set 4 uint8_t bytes that give the size of the data in chunks
 chunks and the MTU size - chunkSize
 starts esp_ota_begin and ads all sebsequent reads to
 esp_ota_write() that stores the bytes in the ESP's ota partition
 Assumes that transfer complte when a chunk < chunkSize is received
 esp_ota_end() is then called followed by esp_ota_set_boot_partition()
 which returns ESP_OK and sets the chips boot partition to the esp
 partition just downloaded - the ESP is then reset 

*/


/*
=> Data Characteristic
-> Control Characteristic

Phone -   OTA_MONITOR_REQUEST  -> Monitor
Phone <-  OTA_MONITOR_READY     - Monitor
Phone =   byteList             => Monitor
Phone <-  OTA_MONITOR_UPDATING  - Monitor
Phone =   Firware chunks       => Monitor
Phone <-  OTA_MONITOR_UPDATED   - Monitor
Phone -   OTA_RESTART          -> Monitor       


Phone -   OTA_REPEATER_REQUEST -> Monitor -  OTA_REPEATER_REQUEST  -> Repeater
Phone <-  OTA_REPEATER_READY   -  Monitor <- OTA_REPEATER_READY    -  Repeater
Phone =   byteList             => Monitor =   byteList             => Repeater
Phone <-  OTA_REPEATER_UPDATING - Monitor <-  OTA_REPEATER_UPDATING - Repeater
Phone =   Firmware stream      => Monitor =   Firmware stream      => Repeater
Phone <-  OTA_REPEATER_UPDATED -  Monitor <-  OTA_REPEATER_UPDATED  - Repeater
        

*/



void setupBLE(void){
  BLEDevice::init("Engine");
  BLEServer *pServer = BLEDevice::createServer();    // Create device
  
  pServer->setCallbacks(new MyServerCallbacks());
  
  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID), 30, 0);  // Set device as a Server and increase No of handles to allow > 5 Characteristics

  /* 
  SETUP CHARACTERISTICS TO:
  NORMAL WORKING MODE:
    RECEIVE SETUP DATA FROM THE PHONE (PORTS ETC)
    SEND SENSOR DATA TO THE PHONE
    SEND PORTS PARAMETERS TO THE REPEATER
    SEND SENSOR DATA TO THE REPEATER
  FIRMWARE UPDATE MODE:
    RECEIVE OTA FIRMWARE UPDATE FROM THE PHONE FOR THIS DEVICE
    RECEIVE OTA FIRMWARE UPDATE FROM THE PHONE FOR THE REPEATER
    FORWARD THE REPEATERS OTA DATA TO THE REPEATER
  
  COMMUNICATIONS FOR BOTH DEVICES CONSISTS OF TWO CHANNELS
    DATA CHANNEL - BOTH READ AND WRITE HANDLES BOT SENSOR AND SETUP DATA
    CONTROL CHANNEL READ ONLY RECEIVES COMMANDS FROM DEVICES USING COMMON
    ENUM SINGLE BYTE COMMANDS
  */

  /*
  NORMAL WORKING MODE PHONE COMMUNICATIONS
  */
  pCharacteristicPhoneData = pService->createCharacteristic(               // Write to phone only.
                                         CHARACTERISTIC_PC_DATA_UUID,  
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );

  BLEDescriptor descriptorToPhone(BLEUUID((uint16_t)0x2902));
  descriptorToPhone.setValue("Data to Phone");
  pCharacteristicPhoneData->addDescriptor(&descriptorToPhone);   
  pCharacteristicPhoneData->setCallbacks(new BtPCDataCallbacks());
  
  
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLESetp run");
}


