#ifndef BLE_UTILITIES_H
#define BLE_UTILITIES_H


// THese enums must be exactly the same as those in the Flutter app
// Don't change them without checking the Dart code in btHelper.dart

class Version {
  public:
  uint8_t version_major = 1;
  uint8_t version_minor = 0;
  uint8_t version_patch = 1;
  Version() = default;
  Version(uint8_t major, uint8_t minor, uint8_t patch) {
    version_major = major;
    version_minor = minor;
    version_patch = patch;
  }
};

/*
enum OtaToken1 {
  START,
  RESTART,
  ABORT,
  STARTED,
  READING,
  READ,
  FINISHED,
  ERROR,
  IDLE,
  REQUEST_VERSION,
  FORWARDING,
  FORWARD,
};
*/


enum OtaToken {
  OTA_IDLE,
  OTA_ERROR,
  OTA_RESTART,
  OTA_REPEATER_REQUEST,
  OTA_REPEATER_READY,
  OTA_REPEATER_UPDATING,
  OTA_REPEATER_UPDATED,
  OTA_MONITOR_REQUEST,
  OTA_MONITOR_READY,
  OTA_MONITOR_UPDATING,
  OTA_MONITOR_UPDATED,
  REQUEST_VERSION = 18,
};


enum CommandToken {
  SEND_PORT_1 = 11,
  SEND_PORT_2 = 12,
  SEND_PORT_3 = 13,
  SEND_PORT_4 = 14,
  SEND_PORT_5 = 15,
  SEND_PORT_6 = 16,
  START_POLLING = 17,

};

enum ServerCommands {
  SEND_PORTS,
  SEND_DATA,
};

enum PortTypes {
  // 1 = I/O 2 = I/O inverted 3 = Analog 4 = Digital
  NONE, 
  ON_OFF,
  ON_OFF_INVERTED,
  ANALOG_GAUGE,
  DIGITAL_GAUGE,
};

enum OtaTarget {
    MONITOR,
    REPEATER
};



enum OtaResponse { NO, YES };

void processSentData(std::string value);
void setupBLE(void);

void transmitData(void);

class BLEPortValues {
    public:
        static uint8_t values[6];
};

class BLEStatus {
    public:
        static bool repeaterConnected;
        static bool updatingPortDetails;
        static bool portDetailsUpdated;
        static bool deviceConnected;
        static bool otaUpdating;
        static int otaTimeout;
        static int  writeMessageDelay;
        static int portsToLoad;
        static ServerCommands serverCommand;
};

// BLEStatus::otaUpdating;


#endif
