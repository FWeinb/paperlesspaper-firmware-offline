#pragma once
#include <Arduino.h>

// Display selection comes from the PlatformIO env (epd13 / epd7)
#if defined(EPD_TYPE_13INCH) == defined(EPD_TYPE_7INCH)
#error "Define exactly one of EPD_TYPE_13INCH or EPD_TYPE_7INCH (build with -e epd13 or -e epd7)"
#endif

#define SOFTWARE_VERSION "0.0.0"
#define DEBUG 1

#define DISPLAY_SPI_SPEED 20000000
#define QR_VERSION 3
#define QR_QUIET_ZONE 4

typedef enum
{
   SYSTEM_RESET = 0,
   BUTTON = 1,
   MOTION = 2,
   TIMER = 3,
} wakeup_reason_t;

enum DisplayInfoKey
{
   VERSION,
   BATTERY_INFO,
   BATTERY_LOW_BIG,
   WIFI_SIGNAL,
   DEVICE_INFO_STRING,
   WIFI_OFFLINE_BIG
};

struct WifiSettings
{
   String bleSSID;
   String blePASS;
   String ssid;
   String pss;
   int wifiQuality;
   uint8_t wifiRetries;
   bool wifiIsConnected;
   bool wifiOnboardingFailed;
   bool wifiConfig;
   bool bleInitOk;
   bool isDeployWifi;
   String clientId;
};

struct DisplayInfo
{
   bool version;
   bool batteryInfo;
   bool batteryLowBig;
   bool wifiSignal;
   bool deviceInfoString;
   bool wifiOfflineBig;
};

struct DisplaySettings
{
   uint8_t rotationText;
   uint8_t rotationPicture;
   bool quickRefresh;
   bool globalQuickRefreshDisable;
   int displayQuickRefreshTime;
   int displayQuickRefreshWipeTime;
   uint16_t colorWhiteFast;
   uint16_t colorBlackFast;
   uint8_t colorWipeFast;
   uint8_t displayType;
};

struct Settings
{
   int timeout;
   String lut;
   bool clearscreen;
   bool showBatteryWarning;
   bool showWifiWarning;
   bool sleepDisabled;
   String downloadUrl;
   String httpAuthUser;
   String httpAuthPassword;
   String lastModified;
   uint32_t downloadCrc; // CRC32 of the last rendered download, 0 = none
   int imageMode;
   bool motionWakeup;
   bool chargerMode;
   String settingsUrl;
   String settingsLastModified;
   bool autoRotation;
   bool telemetry; // append device telemetry as query params to the image URL
   bool forceDownload;
   bool skipSetupMode;
   bool imageValid; // IMAGE_FILE holds a complete image
};

struct SystemData
{
   wakeup_reason_t wakeupCause;
   int vddValue;
   u_int8_t ledDimValue;
   int sleepPrediction;
   bool newSleepTimeSet;
   bool usbConnected;
   int deviceOrientation;
   bool displayPowerOn;
   bool sdIsInit;
   bool sdReady;
};
