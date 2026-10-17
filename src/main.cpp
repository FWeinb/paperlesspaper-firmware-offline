#include <FS.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <cstdint>
#include <Arduino.h>
#include <ArduinoJson.h>
#include <NimBLEDevice.h>
#include <Preferences.h>
#ifdef EPD_TYPE_13INCH
#include "SdFat.h" // SD NAND only on the 13" board
#endif
#include "Ticker.h"
#include "driver/rtc_io.h"
#include "kxtj3-1057.h"
#include <JPEGDEC.h>
#include <SPI.h>
#include <SerialFlash.h>
#include <Update.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <atomic>
#include <rom/crc.h>
#include <rom/rtc.h>

#include "epaper_display.h"
#include "types.h"

#ifdef ARDUHAL_LOG_LEVEL
#undef ARDUHAL_LOG_LEVEL
#define ARDUHAL_LOG_LEVEL ARDUHAL_LOG_LEVEL_NONE
#endif
#define BUTTON_PIN_BITMASK(GPIO) (1ULL << GPIO) // 2 ^ GPIO_NUMBER in hex

// Pins
#ifdef EPD_TYPE_13INCH
#define CS_SD_PIN 13
#endif
#define DISP_POWER 12

// flash
#define CS_FLASH_PIN 21

// SPI Pins
#define SCK_PIN 15
#define MOSI_PIN 4 // DIN
#define MISO_PIN 5 // DIN

// I2C
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7
#define INT_PIN 0
#define ACC_ADDR 0x0E
#define USBC_ADDR 0x21

// Pins Periphery
#define LED_PIN 14
#define BAT_VOLT_SENSE_PIN 2
#define BAT_VOLT_EN_PIN 3
#define CHG_EN_PIN 22
#define CHG_STAT_PIN 23

#define MAX_EPD_WIDTH 1600

#ifdef EPD_TYPE_13INCH
#define EPD_TYPE_IDENTIFIER "epd13-" // Type of device (screen type)
#define VDD_CORRECTION_FACTOR 6.68
#else
#define EPD_TYPE_IDENTIFIER "epd7-" // Type of device (screen type)
#define VDD_CORRECTION_FACTOR 2.28
#endif

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define SETTINGS_NAMESPACE "settings" // NVS namespace for all persistent settings
#define SETTINGS_VERSION 2            // NVS schema version (key "cfgVer"), independent of SOFTWARE_VERSION; bump + add a migration step when the layout changes
#define BLE_BUFFER_SIZE 19200

#define DEFAULT_SLEEP 3600 // Default time how long to sleep after update
#define MIN_SLEEP_TIMEOUT 60     // Lower bound for a configured sleep timeout (seconds)
#define MAX_SLEEP_TIMEOUT 604800 // Upper bound for a configured sleep timeout (7 days)
#define FAILSAVE_TIMER 180 // Time to shutdown the device if no reaction or hangup
#define SLEEP_RECALCULATION_PERIOD_SECONDS 30
#define SETUP_MODE_TIMEOUT 30 // seconds to wait in setup mode for BLE connection before switching to fetch/refresh mode
#define LED_DIM_VALUE 5       // brightness of led in percent (0-100)

// SerialFlash cannot delete or resize files, so both files are created once with a fixed size and reused
#define IMAGE_FILE "tmp.bmp"                                   // processed 4-bit image that gets rendered
#define IMAGE_FILE_SIZE ((EPD_WIDTH * EPD_HEIGHT / 2) + 200)   // raw 4-bit rows (+ room for a BMP header from older firmware)
#define ROTATE_FILE "rotate.bin"                               // RGB565 view of a portrait image before rotating it into IMAGE_FILE
#define ROTATE_FILE_SIZE (EPD_WIDTH * EPD_HEIGHT * 2)
#define DOWNLOAD_FILE "download.bin"                           // raw HTTP download (replaces the old tmp_raw.bin)
#ifdef EPD_TYPE_13INCH
#define DOWNLOAD_FILE_MAX (6 * 1024 * 1024) // fits a 24-bit 1600x1200 BMP
#else
#define DOWNLOAD_FILE_MAX (2 * 1024 * 1024) // fits a 24-bit 800x480 BMP
#endif

// processImageFile() results
#define IMG_OK 0
#define IMG_REJECTED 1     // unsupported or invalid image, IMAGE_FILE untouched
#define IMG_FAILED 2       // failed while writing, IMAGE_FILE is broken
#define JPEG_STRIP_ROWS 16 // max JPEG MCU height

// downloadAndSaveFile() results
#define DL_OK 0
#define DL_NOT_MODIFIED 1
#define DL_ERR_NETWORK (-2)     // connection / TLS / request failed (retry)
#define DL_ERR_HTTP_STATUS (-3) // non-2xx/304 status other than 5xx (no retry)
#define DL_ERR_STREAM (-4)      // transfer aborted (retry)
#define DL_ERR_SERVER (-5)      // 5xx (retry)
#define DL_ERR_TOO_LARGE (-6)   // download exceeds DOWNLOAD_FILE (no retry)
#define DL_ERR_FLASH (-7)       // download file missing (no retry)
#define DL_ERR_SIZE (-8)        // received byte count does not match Content-Length (retry)


WifiSettings wifiSettings = {
    .ssid = "",
    .pss = "",
    .wifiIsConnected = false,
    .wifiOnboardingFailed = false,
    .bleInitOk = false,
    .isDeployWifi = false,
};

Settings settings = {
    .timeout = DEFAULT_SLEEP,
    .sleepDisabled = false,
    .downloadUrl = "",
    .httpAuthUser = "",
    .httpAuthPassword = "",
    .lastModified = "",
    .downloadCrc = 0,
    .imageMode = 1,
    .motionWakeup = false,
    .chargerMode = false,
    .settingsUrl = "",
    .settingsLastModified = "",
    .autoRotation = true,
    .telemetry = true,
    .forceDownload = false,
    .skipSetupMode = false,
    .imageValid = true};

SystemData systemData = {
    .wakeupCause = SYSTEM_RESET,
    .vddValue = 5000,
    .ledDimValue = 100,
    .sleepPrediction = DEFAULT_SLEEP,
    .newSleepTimeSet = false,
    .usbConnected = true,
    .deviceOrientation = 0,
    .displayPowerOn = false,
    .sdReady = false,
};

KXTJ3 myIMU(ACC_ADDR); // Address can be 0x0E or 0x0F
Preferences preferences;
Preferences settingsStore; // opened once in setup(), stays open until deep sleep
Ticker tickerFailsave;
Ticker perdiodicLed;
Ticker perdiodicLedOff;
Ticker periodicAccCheck;
Ticker tickerStatupCounter;
SerialFlashFile saveFile;
SerialFlashFile rawOutFile;
SerialFlashFile jpgInFile;
#ifdef EPD_TYPE_13INCH
SdFs sd;
#endif
RTC_DATA_ATTR timeval previousWakeup = timeval{.tv_sec = 0, .tv_usec = 0};
// Download failures (#15): keep the last image, retry with backoff, show an error badge after DL_FAIL_BADGE_AFTER failures
#define DL_FAIL_BADGE_AFTER 3
RTC_DATA_ATTR uint8_t dlFailCount = 0;         // consecutive failed downloads
RTC_DATA_ATTR bool dlErrorShown = false;       // error badge or error text is on the display
JPEGDEC jpeg;

NimBLECharacteristic *wifiConnectedCharacteristic;
NimBLECharacteristic *wifiInfoCharacteristic;
NimBLECharacteristic *wifiScanCharacteristic;
NimBLECharacteristic *systemInfoCharacteristic;

NimBLEAdvertising *pAdvertising;
static NimBLEServer *pServer;

int periodicLedTimeout = 0;
String tempLastModified = "";
uint32_t tempDownloadCrc = 0;
uint32_t downloadedLength = 0; // real byte count in DOWNLOAD_FILE (its allocated size is larger)
uint32_t jpgInputLength = 0;
int StartCounter = 0;
uint16_t bleWriteBufferPos = 0;
uint32_t bleBytesReceived = 0;
unsigned long lastDisconnectTime = 0;
bool wifiScanRequested = false;
bool periodicLedIsOn = false;
bool downloadStart = true;
bool applyPending = false;
bool bleImageApplied = false;
bool forceExitSetup = false;
bool isBleClientConnected = false;
bool buttonWake = false; // true if wakeup via reset button
volatile bool fwUpdateInProgress = false;
bool stopAccRecheck = false;
volatile int pendingOrientation = -1; // written by the orientation ticker, applied by applyPendingOrientation()
int16_t err_curr[MAX_EPD_WIDTH * 3];
int16_t err_next[MAX_EPD_WIDTH * 3];
char CLIENT_ID[20];
uint8_t *bleWriteBuffer = nullptr; // RAM buffer receiving BLE upload packets

// BLE upload worker (#17): BLE callbacks only buffer data and queue jobs, the worker task does the flash / OTA work
enum BleJobType : uint8_t {
   BLE_JOB_START_IMAGE,
   BLE_JOB_START_FW,
   BLE_JOB_FLUSH,
   BLE_JOB_END_IMAGE,
   BLE_JOB_END_FW,
   BLE_JOB_APPLY,
   BLE_JOB_ABORT,
   BLE_JOB_RESET
};
struct BleJob {
   BleJobType type;
   uint16_t len;
};
#define BLE_STATUS_ERROR 0xFFFF // cmd characteristic read: last upload failed
#define BLE_STATUS_BUSY 0xFFFE  // cmd characteristic read: worker still processing (only reported while the RAM buffer is empty)
QueueHandle_t bleJobQueue = nullptr;
SemaphoreHandle_t spiMutex = nullptr; // serializes SPI use (flash vs display) between the BLE worker and the main loop
std::atomic<int> blePendingJobs{0};
uint8_t *bleFlushBuffer = nullptr; // filled buffer handed to the worker
volatile bool bleFlushBufferBusy = false;
volatile bool bleUploadActive = false; // data packets are accepted
volatile bool bleStatusError = false;
uint32_t bleRxCrc = 0;      // CRC32 of all handed-off upload data (BLE task)
uint32_t bleWrittenCrc = 0; // CRC32 of all data the worker wrote (worker task); both must match the uploaded file
volatile uint16_t bleUploadConnHandle = 0xFFFF; // connection that started the current upload (BLE_HS_CONN_HANDLE_NONE)

void ledBlink(int timeout, bool on, int dimValue = 100);
void logHeap(void);
bool BleInit(String deviceId, bool enable);
int storeSleepTimeMem(int updateTime = 0);
int clampSleepTimeout(int seconds);
void gotToDeepSleep(int seconds, bool motionWake = true);
int accInit(bool skipInit = false);
bool accIntSet(int sensity);
void checkOrientationInBackground(bool isRunning = true);
bool applyPendingOrientation();
bool chargeMode(bool enable);
bool usbCheckConnect();
int calculateSleepDuration(int defaultTimeout, bool forceReset, bool getDataOnly = false);
bool resetAll(bool resetWifi);
wakeup_reason_t getWakeupReason();
void runSetupMode();
int processHttpDownload(String fileName);
#ifdef EPD_TYPE_13INCH
bool sdInit(void);
#endif
void accUpdateOrient();
bool connectWifi(uint32_t timeoutMs = 20000);
SerialFlashFile openErasedFile(const char *fileName);
void invalidateDownloadCache();
void setImageValid(bool valid);
void bleWorkerInit();
void bleWorkerShutdown();
bool copyFlashFile(SerialFlashFile &inFile, uint32_t rawLength, SerialFlashFile &outFile);

// Debug builds: log the minimum free stack (bytes) the calling task has had so far
void logStackHighWater(const char *where) {
   DEBUG_LOG("[STACK] %s: %s min free %u bytes\n", where, pcTaskGetName(NULL), (unsigned)uxTaskGetStackHighWaterMark(NULL));
}

void spiLock() {
   if (spiMutex)
      xSemaphoreTakeRecursive(spiMutex, portMAX_DELAY);
}
void spiUnlock() {
   if (spiMutex)
      xSemaphoreGiveRecursive(spiMutex);
}

bool blePostJob(BleJobType type, uint16_t len = 0) {
   BleJob job = {type, len};
   blePendingJobs++;
   if (!bleJobQueue || xQueueSend(bleJobQueue, &job, pdMS_TO_TICKS(100)) != pdTRUE) {
      blePendingJobs--;
      Serial.printf("[BLE] ERROR: could not queue job %d\n", type);
      bleStatusError = true;
      return false;
   }
   return true;
}

bool bleAllocBuffers() {
   if (!bleWriteBuffer)
      bleWriteBuffer = (uint8_t *)malloc(BLE_BUFFER_SIZE);
   if (!bleFlushBuffer)
      bleFlushBuffer = (uint8_t *)malloc(BLE_BUFFER_SIZE);
   return bleWriteBuffer && bleFlushBuffer;
}

// Hand the filled RAM buffer to the worker and continue receiving into the other one.
// Only waits if the worker has not finished writing the previous buffer yet.
bool bleHandOffBuffer() {
   if (bleWriteBufferPos == 0)
      return true;
   unsigned long waitStart = millis();
   while (bleFlushBufferBusy) {
      if (millis() - waitStart > 1500) {
         Serial.println("[BLE] ERROR: flash worker too slow, dropping data");
         bleStatusError = true;
         bleWriteBufferPos = 0;
         return false;
      }
      vTaskDelay(pdMS_TO_TICKS(5));
   }
   uint8_t *filled = bleWriteBuffer;
   bleWriteBuffer = bleFlushBuffer;
   bleFlushBuffer = filled;
   uint16_t len = bleWriteBufferPos;
   bleRxCrc = crc32_le(bleRxCrc, filled, len);
   bleWriteBufferPos = 0;
   bleFlushBufferBusy = true;
   return blePostJob(BLE_JOB_FLUSH, len);
}

// Drop a "#fragment": it is never sent to the server and would end up in front of appended query params
String stripUrlFragment(String url) {
   int hashPos = url.indexOf('#');
   return (hashPos >= 0) ? url.substring(0, hashPos) : url;
}

uint32_t calcCRC32(const uint8_t *data, size_t len) {
   return crc32_le(0, data, len);
}

const uint32_t acep_palette[7] = {
    0x000000, // 0: Black
    0x0000FF, // 1: Blue
    0x00FF00, // 2: Green
    0xFF0000, // 3: Red
    0xFFFFFF, // 4: Unused (skipped)
    0xFFFF00, // 5: Yellow
    0xFFFFFF  // 6: White
};

void snapColor(int &r, int &g, int &b) {
   int t = 25; // Aggressive threshold for snapping to pure colors
   if (r < t && g < t && b < t) {
      r = 0;
      g = 0;
      b = 0;
      return;
   } // Black
   if (r > 255 - t && g > 255 - t && b > 255 - t) {
      r = 255;
      g = 255;
      b = 255;
      return;
   } // White
   if (r > 255 - t && g < t && b < t) {
      r = 255;
      g = 0;
      b = 0;
      return;
   } // Red
   if (r < (t + 10) && g > 255 - (t + 10) && b < (t + 10)) {
      r = 0;
      g = 255;
      b = 0;
      return;
   } // Green
   if (r < t && g < t && b > 255 - t) {
      r = 0;
      g = 0;
      b = 255;
      return;
   } // Blue
   if (r > 255 - (t + 25) && g > 255 - (t + 25) && b < (t + 25)) {
      r = 255;
      g = 255;
      b = 0;
      return;
   } // Yellow
}

uint8_t findClosestColor(int r, int g, int b) {
   uint8_t best = 6; // Default to white
   long min_dist = 2000000000;
   uint8_t valid_indices[] = {0, 1, 2, 3, 5, 6};

   for (int i = 0; i < 6; i++) {
      uint8_t idx = valid_indices[i];
      int pr = (acep_palette[idx] >> 16) & 0xFF;
      int pg = (acep_palette[idx] >> 8) & 0xFF;
      int pb = acep_palette[idx] & 0xFF;
      long dist = (long)(r - pr) * (r - pr) + (long)(g - pg) * (g - pg) + (long)(b - pb) * (b - pb);
      if (dist < min_dist) {
         min_dist = dist;
         best = idx;
      }
   }
   return best;
}

void *myOpen(const char *filename, int32_t *size) {
   jpgInFile = SerialFlash.open(filename);
   if (jpgInFile) {
      *size = jpgInputLength;
      return (void *)1;
   }
   return NULL;
}
void myClose(void *handle) { jpgInFile.close(); }
int32_t myRead(JPEGFILE *pFile, uint8_t *pBuf, int32_t iLen) { return jpgInFile.read(pBuf, iLen); }
int32_t mySeek(JPEGFILE *pFile, int32_t iPosition) {
   jpgInFile.seek(iPosition);
   return iPosition;
}

// Open a fixed-size flash file and erase it so it can be rewritten from offset 0
SerialFlashFile openErasedFile(const char *fileName) {
   SerialFlashFile f = SerialFlash.open(fileName);
   if (f) {
      f.erase();
      f.seek(0);
   }
   else {
      Serial.printf("[FLASH] File %s missing\n", fileName);
   }
   return f;
}

// Copy rawLength bytes from inFile to the start of outFile
bool copyFlashFile(SerialFlashFile &inFile, uint32_t rawLength, SerialFlashFile &outFile) {
   if (rawLength > outFile.size()) {
      Serial.printf("[IMAGE] Payload too large: %u > %u\n", rawLength, outFile.size());
      return false;
   }
   // Heap buffer: keeps the 8 KB loop task's stack free
   const uint32_t bufSize = 2048;
   uint8_t *buf = (uint8_t *)malloc(bufSize);
   if (!buf) {
      Serial.println("[IMAGE] OOM allocating copy buffer");
      return false;
   }
   inFile.seek(0);
   uint32_t left = rawLength;
   bool ok = true;
   while (left > 0) {
      uint32_t n = inFile.read(buf, left < bufSize ? left : bufSize);
      if (n == 0) {
         ok = false;
         break;
      }
      outFile.write(buf, n);
      left -= n;
   }
   free(buf);
   return ok;
}

// Image decoding (#20): every source format is turned into RGB565 rows in "view" space (the image as the viewer
// sees it, scaled and center-cropped to fill the panel). These rows are dithered to the panel colors and written to
// IMAGE_FILE as raw 4-bit rows (EPD_WIDTH x EPD_HEIGHT, top-down, no header). With EPD_VIEW_ROTATED the panel is
// viewed in portrait while the buffer is landscape: portrait images are then collected in ROTATE_FILE and rotated
// by 90 degrees into the buffer.
struct ImageJob {
   int viewW, viewH;               // size of the view the source is fitted into
   bool rotate;                    // view is rotated by 90 degrees relative to the buffer
   int cropX, cropY, cropW, cropH; // source window that fills the view
   int nextViewY;                  // next view row to emit
   uint16_t *sxMap;                // source x for each view x
   uint8_t *outRow;                // one dithered buffer row (EPD_WIDTH / 2)
   bool writeError;
   SerialFlashFile rotFile;
};
ImageJob img;

uint16_t *jpgStrip = nullptr; // JPEG_STRIP_ROWS view rows sampled from the current MCU row
int jpgStripY = -1;           // source y of the first strip row
int jpgStripRows = 0;
uint16_t bmpPalette[256];     // RGB565

static inline void rgb565To888(uint16_t c, int &r, int &g, int &b) {
   r = (c >> 8) & 0xF8;
   r |= r >> 5;
   g = (c >> 3) & 0xFC;
   g |= g >> 6;
   b = (c << 3) & 0xF8;
   b |= b >> 5;
}

static inline uint16_t rgb888To565(int r, int g, int b) {
   return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// Floyd-Steinberg dither one buffer row (EPD_WIDTH RGB565 pixels) to the panel colors and append it to rawOutFile
void ditherRow(const uint16_t *row) {
   uint8_t *out_row = img.outRow;
   for (int x = 0; x < EPD_WIDTH; x++) {
      int orig_r, orig_g, orig_b;
      rgb565To888(row[x], orig_r, orig_g, orig_b);

      // Clean white & black threshold to prevent noise from JPEG artifacts
      snapColor(orig_r, orig_g, orig_b);

      int r = orig_r + err_curr[x * 3];
      int g = orig_g + err_curr[x * 3 + 1];
      int b = orig_b + err_curr[x * 3 + 2];

      r = (r < 0) ? 0 : (r > 255) ? 255
                                  : r;
      g = (g < 0) ? 0 : (g > 255) ? 255
                                  : g;
      b = (b < 0) ? 0 : (b > 255) ? 255
                                  : b;

      uint8_t best = findClosestColor(r, g, b);
      int pr = (acep_palette[best] >> 16) & 0xFF;
      int pg = (acep_palette[best] >> 8) & 0xFF;
      int pb = acep_palette[best] & 0xFF;

      int err_r = r - pr;
      int err_g = g - pg;
      int err_b = b - pb;

      if (x < EPD_WIDTH - 1) {
         err_curr[(x + 1) * 3] += (err_r * 7) >> 4;
         err_curr[(x + 1) * 3 + 1] += (err_g * 7) >> 4;
         err_curr[(x + 1) * 3 + 2] += (err_b * 7) >> 4;
      }
      if (x > 0) {
         err_next[(x - 1) * 3] += (err_r * 3) >> 4;
         err_next[(x - 1) * 3 + 1] += (err_g * 3) >> 4;
         err_next[(x - 1) * 3 + 2] += (err_b * 3) >> 4;
      }
      err_next[x * 3] += (err_r * 5) >> 4;
      err_next[x * 3 + 1] += (err_g * 5) >> 4;
      err_next[x * 3 + 2] += (err_b * 5) >> 4;

      if (x < EPD_WIDTH - 1) {
         err_next[(x + 1) * 3] += (err_r * 1) >> 4;
         err_next[(x + 1) * 3 + 1] += (err_g * 1) >> 4;
         err_next[(x + 1) * 3 + 2] += (err_b * 1) >> 4;
      }

      if (x % 2 == 0)
         out_row[x / 2] = (out_row[x / 2] & 0x0F) | (best << 4);
      else
         out_row[x / 2] = (out_row[x / 2] & 0xF0) | (best & 0x0F);
   }
   if (rawOutFile.write(out_row, EPD_WIDTH / 2) != EPD_WIDTH / 2)
      img.writeError = true;
   memcpy(err_curr, err_next, sizeof(err_curr));
   memset(err_next, 0, sizeof(err_next));
}

// Pick the view for a source of w x h: rotate portrait images when the panel is viewed in portrait
void imageJobInit(int w, int h) {
   img = ImageJob();
   img.viewW = EPD_WIDTH;
   img.viewH = EPD_HEIGHT;
#if EPD_VIEW_ROTATED
   if (h > w) {
      if (SerialFlash.exists(ROTATE_FILE)) {
         img.rotate = true;
         img.viewW = EPD_HEIGHT;
         img.viewH = EPD_WIDTH;
      }
      else {
         Serial.println("[IMAGE] " ROTATE_FILE " missing, not rotating");
      }
   }
#endif
}

void imageJobFree() {
   free(img.sxMap);
   free(img.outRow);
   free(jpgStrip);
   img.sxMap = nullptr;
   img.outRow = nullptr;
   jpgStrip = nullptr;
}

// Center-crop the srcW x srcH source to the view's aspect ratio, allocate buffers and open the output file(s).
// Nothing is written before this, so a rejected image leaves IMAGE_FILE intact.
bool imageJobBegin(int srcW, int srcH, const char *outFileName) {
   if ((int64_t)srcW * img.viewH > (int64_t)srcH * img.viewW) {
      img.cropH = srcH;
      img.cropW = (int)((int64_t)srcH * img.viewW / img.viewH);
   }
   else {
      img.cropW = srcW;
      img.cropH = (int)((int64_t)srcW * img.viewH / img.viewW);
   }
   img.cropW = max(img.cropW, 1);
   img.cropH = max(img.cropH, 1);
   img.cropX = (srcW - img.cropW) / 2;
   img.cropY = (srcH - img.cropH) / 2;
   Serial.printf("[IMAGE] Source %dx%d, crop %dx%d at %d,%d -> %dx%d%s\n", srcW, srcH, img.cropW, img.cropH, img.cropX,
                 img.cropY, img.viewW, img.viewH, img.rotate ? " (rotated 90)" : "");

   img.sxMap = (uint16_t *)malloc(img.viewW * sizeof(uint16_t));
   img.outRow = (uint8_t *)malloc(EPD_WIDTH / 2);
   if (!img.sxMap || !img.outRow) {
      Serial.println("[IMAGE] OOM allocating image buffers");
      imageJobFree();
      return false;
   }
   for (int vx = 0; vx < img.viewW; vx++) {
      img.sxMap[vx] = img.cropX + (int)((int64_t)vx * img.cropW / img.viewW);
   }
   memset(err_curr, 0, sizeof(err_curr));
   memset(err_next, 0, sizeof(err_next));

   rawOutFile = openErasedFile(outFileName);
   if (!rawOutFile) {
      imageJobFree();
      return false;
   }
   if (img.rotate) {
      img.rotFile = openErasedFile(ROTATE_FILE);
      if (!img.rotFile) {
         rawOutFile.close();
         imageJobFree();
         return false;
      }
   }
   return true;
}

// Source row that view row vy samples
int imageViewToSrcY(int vy) {
   return img.cropY + (int)((int64_t)vy * img.cropH / img.viewH);
}

// Append the next view row (img.viewW RGB565 pixels)
void imageEmitViewRow(const uint16_t *row) {
   if (img.nextViewY >= img.viewH)
      return;
   if (img.rotate) {
      if (img.rotFile.write(row, img.viewW * 2) != (uint32_t)img.viewW * 2)
         img.writeError = true;
   }
   else {
      ditherRow(row);
   }
   img.nextViewY++;
}

// Rotate the view stored in ROTATE_FILE (EPD_HEIGHT x EPD_WIDTH) into buffer rows and dither them.
// The panel is viewed in rotationText = rotationPicture + 1, i.e. GFX rotates the view by 90 degrees clockwise:
// buffer pixel (bx, by) = view pixel (by, EPD_WIDTH - 1 - bx).
bool imageWriteRotatedRows() {
   const int chunkCols = 16; // view columns (= buffer rows) per pass
   uint16_t *chunk = (uint16_t *)malloc(chunkCols * img.viewH * sizeof(uint16_t));
   uint16_t *bufRow = (uint16_t *)malloc(EPD_WIDTH * sizeof(uint16_t));
   if (!chunk || !bufRow) {
      Serial.println("[IMAGE] OOM allocating rotation buffers");
      free(chunk);
      free(bufRow);
      return false;
   }
   for (int c = 0; c < img.viewW; c += chunkCols) {
      int n = min(chunkCols, img.viewW - c);
      for (int vy = 0; vy < img.viewH; vy++) {
         img.rotFile.seek(((uint32_t)vy * img.viewW + c) * 2);
         img.rotFile.read(chunk + vy * chunkCols, n * 2);
      }
      for (int j = 0; j < n; j++) {
         for (int bx = 0; bx < EPD_WIDTH; bx++) {
            bufRow[bx] = chunk[(EPD_WIDTH - 1 - bx) * chunkCols + j];
         }
         ditherRow(bufRow);
      }
   }
   free(chunk);
   free(bufRow);
   return true;
}

// Pad missing view rows with white, rotate if needed, close files. Returns IMG_OK or IMG_FAILED.
int imageJobEnd() {
   bool ok = true;
   if (img.nextViewY < img.viewH) {
      Serial.printf("[IMAGE] Only %d of %d rows decoded, padding with white\n", img.nextViewY, img.viewH);
      uint16_t *white = (uint16_t *)malloc(img.viewW * sizeof(uint16_t));
      if (white) {
         for (int i = 0; i < img.viewW; i++)
            white[i] = 0xFFFF;
         while (img.nextViewY < img.viewH)
            imageEmitViewRow(white);
         free(white);
      }
      else {
         ok = false;
      }
   }
   if (ok && img.rotate)
      ok = imageWriteRotatedRows();
   if (img.rotate)
      img.rotFile.close();
   rawOutFile.close();
   imageJobFree();
   if (img.writeError) {
      Serial.println("[IMAGE] Flash write failed");
      ok = false;
   }
   return ok ? IMG_OK : IMG_FAILED;
}

// Emit every view row that samples one of the buffered strip rows
void flushJpegStrip() {
   for (int r = 0; r < jpgStripRows; r++) {
      while (img.nextViewY < img.viewH && imageViewToSrcY(img.nextViewY) == jpgStripY + r) {
         imageEmitViewRow(jpgStrip + r * img.viewW);
      }
   }
   jpgStripRows = 0;
}

int JPEGDraw(JPEGDRAW *pDraw) {
   if (pDraw->y != jpgStripY) {
      flushJpegStrip();
      jpgStripY = pDraw->y;
   }
   int h = min(pDraw->iHeight, JPEG_STRIP_ROWS);
   jpgStripRows = max(jpgStripRows, h);
   int x0 = pDraw->x;
   int x1 = x0 + pDraw->iWidth;
   for (int vx = 0; vx < img.viewW; vx++) {
      int sx = img.sxMap[vx];
      if (sx < x0)
         continue;
      if (sx >= x1)
         break;
      for (int y = 0; y < h; y++) {
         jpgStrip[y * img.viewW + vx] = pDraw->pPixels[y * pDraw->iWidth + (sx - x0)];
      }
   }
   return img.writeError ? 0 : 1; // 0 aborts the decode
}

int processJpeg(const char *rawFileName, uint32_t rawLength, const char *outFileName) {
   jpgInputLength = rawLength;
   if (!jpeg.open(rawFileName, myOpen, myClose, myRead, mySeek, JPEGDraw)) {
      Serial.printf("[IMAGE] Invalid JPEG (error %d)\n", jpeg.getLastError());
      return IMG_REJECTED;
   }
   int w = jpeg.getWidth();
   int h = jpeg.getHeight();
   if (w <= 0 || h <= 0) {
      Serial.println("[IMAGE] Invalid JPEG size");
      jpeg.close();
      return IMG_REJECTED;
   }
   imageJobInit(w, h);

   // Largest JPEGDEC downscale (flags, not shifts) that still fills the view
   int div = 1;
   int scale = 0;
   if (jpeg.getJPEGType() == JPEG_MODE_PROGRESSIVE) {
      // JPEGDEC only decodes the first scan of a progressive JPEG, at 1/8 size
      div = 8;
      scale = JPEG_SCALE_EIGHTH;
      Serial.printf("[IMAGE] Progressive JPEG: only %dx%d available, use baseline JPEGs for full quality\n", w / 8, h / 8);
   }
   else {
      const int divs[] = {8, 4, 2};
      const int scales[] = {JPEG_SCALE_EIGHTH, JPEG_SCALE_QUARTER, JPEG_SCALE_HALF};
      for (int i = 0; i < 3; i++) {
         if (w / divs[i] >= img.viewW && h / divs[i] >= img.viewH) {
            div = divs[i];
            scale = scales[i];
            break;
         }
      }
   }
   Serial.printf("[IMAGE] JPEG %dx%d, decoding at 1/%d\n", w, h, div);
   int srcW = max(w / div, 1);
   int srcH = max(h / div, 1);

   jpgStrip = (uint16_t *)malloc(JPEG_STRIP_ROWS * img.viewW * sizeof(uint16_t));
   if (!jpgStrip) {
      Serial.println("[IMAGE] OOM allocating JPEG strip");
      jpeg.close();
      return IMG_REJECTED;
   }
   uint16_t *strip = jpgStrip;
   jpgStrip = nullptr; // imageJobBegin frees jpgStrip on failure
   if (!imageJobBegin(srcW, srcH, outFileName)) {
      free(strip);
      jpeg.close();
      return IMG_FAILED;
   }
   jpgStrip = strip;
   jpgStripY = -1;
   jpgStripRows = 0;

   int decoded = jpeg.decode(0, 0, scale);
   flushJpegStrip();
   int lastError = jpeg.getLastError();
   jpeg.close();
   int result = imageJobEnd();
   if (!decoded) {
      Serial.printf("[IMAGE] JPEG decode failed (error %d)\n", lastError);
      return IMG_FAILED;
   }
   return result;
}

static inline uint32_t le32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }
static inline uint16_t le16(const uint8_t *p) { return p[0] | (p[1] << 8); }

int processBmp(SerialFlashFile &inFile, uint32_t rawLength, const char *outFileName) {
   uint8_t hdr[54];
   inFile.seek(0);
   if (rawLength < sizeof(hdr) || inFile.read(hdr, sizeof(hdr)) != sizeof(hdr)) {
      Serial.println("[IMAGE] BMP header truncated");
      return IMG_REJECTED;
   }
   uint32_t dataOffset = le32(hdr + 0x0A);
   uint32_t dibSize = le32(hdr + 0x0E);
   int32_t w = (int32_t)le32(hdr + 0x12);
   int32_t hRaw = (int32_t)le32(hdr + 0x16);
   uint16_t bpp = le16(hdr + 0x1C);
   uint32_t compression = le32(hdr + 0x1E);
   uint32_t colorsUsed = le32(hdr + 0x2E);
   bool bottomUp = hRaw > 0; // standard BMPs store the last row first
   int32_t h = bottomUp ? hRaw : -hRaw;

   Serial.printf("[IMAGE] BMP %dx%d, %u bpp, %s\n", (int)w, (int)h, bpp, bottomUp ? "bottom-up" : "top-down");
   if (dibSize < 40 || w <= 0 || h <= 0 || w > 10000 || h > 10000) {
      Serial.println("[IMAGE] Invalid BMP header");
      return IMG_REJECTED;
   }
   if (bpp != 4 && bpp != 8 && bpp != 24 && bpp != 32) {
      Serial.printf("[IMAGE] Unsupported BMP bit depth %u\n", bpp);
      return IMG_REJECTED;
   }
   if (compression != 0 && !(compression == 3 && bpp == 32)) { // BI_RGB, or BI_BITFIELDS assumed BGRX
      Serial.printf("[IMAGE] Unsupported BMP compression %u\n", compression);
      return IMG_REJECTED;
   }
   uint32_t stride = ((uint32_t)w * bpp + 31) / 32 * 4;
   if (dataOffset + stride * (uint32_t)h > rawLength) {
      Serial.printf("[IMAGE] BMP truncated: needs %u bytes, got %u\n", dataOffset + stride * (uint32_t)h, rawLength);
      return IMG_REJECTED;
   }

   // Palette (B, G, R, 0 entries after the DIB header)
   int paletteSize = 0;
   if (bpp <= 8) {
      paletteSize = (colorsUsed > 0 && colorsUsed < (1u << bpp)) ? colorsUsed : (1 << bpp);
      for (int i = 0; i < 256; i++)
         bmpPalette[i] = 0xFFFF;
      inFile.seek(14 + dibSize);
      for (int i = 0; i < paletteSize; i++) {
         uint8_t e[4];
         if (inFile.read(e, 4) != 4)
            break;
         bmpPalette[i] = rgb888To565(e[2], e[1], e[0]);
      }
   }

   // Pre-dithered 4-bit image at panel size: copy rows (top-down), mapping the palette to panel color indices
   if (bpp == 4 && w == EPD_WIDTH && h == EPD_HEIGHT) {
      uint8_t remap[16];
      uint8_t seen = 0; // bit per panel color index
      for (int i = 0; i < 16; i++) {
         int r, g, b;
         rgb565To888(bmpPalette[i], r, g, b);
         remap[i] = findClosestColor(r, g, b);
         if (i < paletteSize)
            seen |= 1 << remap[i];
      }
      const uint8_t allColors = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3) | (1 << 5) | (1 << 6);
      bool identity = true;
      if ((seen & allColors) == allColors) {
         for (int i = 0; i < paletteSize; i++)
            identity = identity && (remap[i] == i || i == 4);
         Serial.printf("[IMAGE] 4-bit BMP, palette %s\n", identity ? "in panel order" : "remapped to panel colors");
      }
      else {
         // Not a 6-color palette: assume the indices already are panel colors (what older firmware did)
         Serial.println("[IMAGE] 4-bit BMP, palette ignored (not a 6-color palette), using indices as panel colors");
      }

      rawOutFile = openErasedFile(outFileName);
      if (!rawOutFile)
         return IMG_FAILED;
      uint8_t *row = (uint8_t *)malloc(EPD_WIDTH / 2);
      if (!row) {
         rawOutFile.close();
         return IMG_FAILED;
      }
      bool ok = true;
      for (int by = 0; by < EPD_HEIGHT && ok; by++) {
         uint32_t fileRow = bottomUp ? (EPD_HEIGHT - 1 - by) : by;
         inFile.seek(dataOffset + fileRow * stride);
         ok = inFile.read(row, EPD_WIDTH / 2) == EPD_WIDTH / 2;
         if (!identity) {
            for (int i = 0; i < EPD_WIDTH / 2; i++)
               row[i] = (remap[row[i] >> 4] << 4) | remap[row[i] & 0x0F];
         }
         ok = ok && rawOutFile.write(row, EPD_WIDTH / 2) == EPD_WIDTH / 2;
      }
      free(row);
      rawOutFile.close();
      return ok ? IMG_OK : IMG_FAILED;
   }

   imageJobInit(w, h);
   const uint32_t readBufSize = 1024;
   uint8_t *readBuf = (uint8_t *)malloc(readBufSize);
   uint16_t *viewRow = (uint16_t *)malloc(img.viewW * sizeof(uint16_t));
   if (!readBuf || !viewRow) {
      Serial.println("[IMAGE] OOM allocating BMP buffers");
      free(readBuf);
      free(viewRow);
      return IMG_REJECTED;
   }
   if (!imageJobBegin(w, h, outFileName)) {
      free(readBuf);
      free(viewRow);
      return IMG_FAILED;
   }

   uint32_t pixelBytes = (bpp >= 8) ? bpp / 8 : 1;
   int prevSy = -1;
   while (img.nextViewY < img.viewH && !img.writeError) {
      int sy = imageViewToSrcY(img.nextViewY);
      if (sy != prevSy) {
         uint32_t rowStart = dataOffset + (uint32_t)(bottomUp ? (h - 1 - sy) : sy) * stride;
         uint32_t bufStart = 0;
         uint32_t bufLen = 0;
         for (int vx = 0; vx < img.viewW; vx++) {
            int sx = img.sxMap[vx];
            uint32_t off = (uint32_t)sx * bpp / 8;
            if (bufLen == 0 || off < bufStart || off + pixelBytes > bufStart + bufLen) {
               uint32_t n = min(readBufSize, stride - off);
               inFile.seek(rowStart + off);
               bufLen = inFile.read(readBuf, n);
               bufStart = off;
               if (bufLen < pixelBytes) {
                  bufLen = 0;
                  viewRow[vx] = 0xFFFF;
                  continue;
               }
            }
            const uint8_t *p = readBuf + (off - bufStart);
            switch (bpp) {
            case 4:
               viewRow[vx] = bmpPalette[(sx & 1) ? (p[0] & 0x0F) : (p[0] >> 4)];
               break;
            case 8:
               viewRow[vx] = bmpPalette[p[0]];
               break;
            default: // 24 / 32: B, G, R(, X)
               viewRow[vx] = rgb888To565(p[2], p[1], p[0]);
               break;
            }
         }
         prevSy = sy;
      }
      imageEmitViewRow(viewRow);
   }
   free(readBuf);
   free(viewRow);
   return imageJobEnd();
}

// Convert the downloaded file (JPEG, BMP or raw panel data) into IMAGE_FILE.
// Returns IMG_OK, IMG_REJECTED (unsupported/invalid, IMAGE_FILE untouched) or IMG_FAILED (IMAGE_FILE is broken).
int processImageFile(const char *rawFileName, uint32_t rawLength, const char *outFileName) {
   SerialFlashFile inFile = SerialFlash.open(rawFileName);
   if (!inFile || rawLength < 2)
      return IMG_REJECTED;
   uint8_t magic[2];
   inFile.read(magic, 2);
   Serial.printf("[IMAGE] Read magic: 0x%02X 0x%02X from file '%s' (%u bytes)\n", magic[0], magic[1], rawFileName, rawLength);
   inFile.seek(0);

   int result = IMG_REJECTED;
   if (magic[0] == 0xFF && magic[1] == 0xD8) {
      inFile.close();
      return processJpeg(rawFileName, rawLength, outFileName);
   }
   else if (magic[0] == 'B' && magic[1] == 'M') {
      result = processBmp(inFile, rawLength, outFileName);
   }
   else if (rawLength == EPD_WIDTH * EPD_HEIGHT / 2) {
      Serial.println("[IMAGE] Raw panel payload detected.");
      rawOutFile = openErasedFile(outFileName);
      if (!rawOutFile) {
         result = IMG_FAILED;
      }
      else {
         result = copyFlashFile(inFile, rawLength, rawOutFile) ? IMG_OK : IMG_FAILED;
         rawOutFile.close();
      }
   }
   else {
      Serial.printf("[IMAGE] Unknown format (expected JPEG, BMP or %u bytes of raw panel data)\n", EPD_WIDTH * EPD_HEIGHT / 2);
   }
   inFile.close();
   return result;
}

// Power supply display
#ifdef EPD_TYPE_13INCH
bool powerSupplyDisplay(bool enable) {
   bool tempState = systemData.displayPowerOn;
   if (enable) {
      pinMode(DISP_POWER, OUTPUT);
      digitalWrite(DISP_POWER, HIGH);
      systemData.displayPowerOn = true;
   }
   else {
      pinMode(DISP_POWER, INPUT);
      digitalWrite(DISP_POWER, LOW);
      systemData.displayPowerOn = false;
   }
   if (tempState != systemData.displayPowerOn) {
      if (enable) {
         Serial.printf("[POWER] Supply ON\n");
         delay(100);
         sdInit();
      }
      return true;
   }
   else {
      return false;
   }
}
#else
bool powerSupplyDisplay(bool enable) {
   return true;
}
#endif

void WiFiEvent(WiFiEvent_t event) {
   switch (event) {
   case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      DEBUG_LOG("[NETWORK] WiFi connected\n");
      Serial.printf("[NETWORK] IP address: ");
      Serial.println(WiFi.localIP());

      if (wifiSettings.wifiIsConnected != true) {
         wifiSettings.wifiIsConnected = true;
         if (wifiSettings.bleInitOk) {
            uint8_t val = 1;
            wifiConnectedCharacteristic->setValue(&val, 1);
            wifiConnectedCharacteristic->notify();

            if (wifiInfoCharacteristic) {
               JsonDocument doc;
               doc["ip"] = WiFi.localIP().toString();
               doc["rssi"] = WiFi.RSSI();
               String json;
               serializeJson(doc, json);
               wifiInfoCharacteristic->setValue(json.c_str());
               wifiInfoCharacteristic->notify();
            }
         }
      }

      break;
   case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      Serial.println("[NETWORK] WiFi lost connection");
      wifiSettings.wifiOnboardingFailed = true;
      if (wifiSettings.wifiIsConnected != false) {
         wifiSettings.wifiIsConnected = false;
         if (wifiSettings.bleInitOk) {
            uint8_t val = 0;
            wifiConnectedCharacteristic->setValue(&val, 1);
            wifiConnectedCharacteristic->notify();

            if (wifiInfoCharacteristic) {
               wifiInfoCharacteristic->setValue("{}");
               wifiInfoCharacteristic->notify();
            }
         }
      }
      break;
   }
}

void timeoutFailsafe(int time) {
   Serial.println("[MAIN] Timeout Failsafe");

   gotToDeepSleep(DEFAULT_SLEEP, false);
}

void ledBlinkFunctionOff() {
   analogWrite(LED_PIN, 0);
   perdiodicLedOff.detach();
}

void ledBlinkFunction() {
   analogWrite(LED_PIN, systemData.ledDimValue);
   perdiodicLedOff.attach_ms(periodicLedTimeout * 0.2, ledBlinkFunctionOff);
}

// set blink with timeout in ms
void ledBlink(int timeout, bool on, int dimValue) {
   pinMode(LED_PIN, OUTPUT);
   if (dimValue > 100)
      dimValue = 100;
   if (dimValue < 0)
      dimValue = 0;
   systemData.ledDimValue = (dimValue * 255) / 100;
   DEBUG_LOG("[LED] Set DIM: %d Set Time: %d On: %d (dimInput:%d)\n", systemData.ledDimValue, timeout, on, dimValue);

   if (on) {
      if (timeout <= 0) {
         perdiodicLed.detach();
         perdiodicLedOff.detach();
         analogWrite(LED_PIN, systemData.ledDimValue); // Dim brightness (0-255)
         return;
      }
      periodicLedIsOn = true;
      periodicLedTimeout = timeout * 2;
      perdiodicLed.attach_ms(periodicLedTimeout, ledBlinkFunction);
   }
   else {
      perdiodicLed.detach();
      perdiodicLedOff.detach();
      analogWrite(LED_PIN, 0);
      delay(periodicLedTimeout * 0.3);
      analogWrite(LED_PIN, 0);
      periodicLedIsOn = false;
   }
}

int readVDD(bool singleReading) {
   pinMode(BAT_VOLT_EN_PIN, OUTPUT);
#ifdef EPD_TYPE_13INCH
   digitalWrite(BAT_VOLT_EN_PIN, HIGH);
#else
   digitalWrite(BAT_VOLT_EN_PIN, LOW);
#endif
   delay(3);
   int retries = 1;
   if (!singleReading) {
      retries = 5;
   }
   float rawValue = 0;
   int count = 0;
   for (size_t i = 0; i < retries; i++) {
      float readValue = analogRead(BAT_VOLT_SENSE_PIN);
      delayMicroseconds(18000);
      rawValue += readValue;
      count++;
   }
   if (count > 0) {
      rawValue = rawValue / count;
   }
   pinMode(BAT_VOLT_EN_PIN, INPUT);
   int vdd = int(round(rawValue * VDD_CORRECTION_FACTOR));
   Serial.printf("[BAT] ADC Raw: %.1f -> VDD: %d mV\n", rawValue, vdd);
   return vdd;
}

bool settingsStoreInit() {
   if (!settingsStore.begin(SETTINGS_NAMESPACE, false)) {
      Serial.println("[MEM] failed to init settings store");
      delay(1000);
      return false;
   }
   Serial.println("[MEM] settings store init OK");
   return true;
}

bool settingsStoreClear() {
   settingsStore.clear();
   // A cleared store holds no old-layout data, so it is already at the current schema version
   settingsStore.putInt("cfgVer", SETTINGS_VERSION);
   Serial.println("[MEM] settings store clear OK");
   return true;
}

// Upgrade the NVS layout step by step (N -> N+1) from the stored "cfgVer" (missing = v0) to SETTINGS_VERSION.
// Runs before restoreSettings(). Each step must be safe to re-run if power is lost before cfgVer is written.
void runMigrations() {
   int ver = settingsStore.getInt("cfgVer", 0);
   if (ver > SETTINGS_VERSION) {
      Serial.printf("[MEM] Settings v%d are newer than this firmware (v%d), leaving them as they are\n", ver, SETTINGS_VERSION);
      return;
   }
   while (ver < SETTINGS_VERSION) {
      Serial.printf("[MEM] Migrating settings v%d -> v%d\n", ver, ver + 1);
      switch (ver) {
      case 0:
         // v0 -> v1: introduces the cfgVer key itself, no data changes
         break;
      case 1:
         // v1 -> v2: battWarn / wifiWarn were never used
         settingsStore.remove("battWarn");
         settingsStore.remove("wifiWarn");
         break;
      }
      ver++;
      settingsStore.putInt("cfgVer", ver);
   }
}

void saveSettings() {
   settingsStore.putBool("sleepDisabled", settings.sleepDisabled);
   settingsStore.putInt("imageMode", settings.imageMode);
   settingsStore.putString("downloadUrl", settings.downloadUrl);
   settingsStore.putString("lastModified", settings.lastModified);
   settingsStore.putUInt("dlCrc", settings.downloadCrc);
   settingsStore.putString("httpAuthUser", settings.httpAuthUser);
   settingsStore.putString("httpAuthPass", settings.httpAuthPassword);
   settingsStore.putBool("motionWakeup", settings.motionWakeup);
   settingsStore.putBool("chargerMode", settings.chargerMode);
   settingsStore.putString("settingsUrl", settings.settingsUrl);
   settingsStore.putString("settingsLastMod", settings.settingsLastModified);
   settingsStore.putBool("autoRotation", settings.autoRotation);
   settingsStore.putBool("telemetry", settings.telemetry);
   Serial.println("[MEM] Settings saved");
}

void restoreSettings() {
   wifiSettings.ssid = settingsStore.getString("ssid", "");
   wifiSettings.pss = settingsStore.getString("pss", "");
   settings.timeout = storeSleepTimeMem(0);
   settings.sleepDisabled = settingsStore.getBool("sleepDisabled", false);
   settings.imageMode = settingsStore.getInt("imageMode", 1);
   settings.downloadUrl = stripUrlFragment(settingsStore.getString("downloadUrl", ""));
   settings.lastModified = settingsStore.getString("lastModified", "");
   settings.downloadCrc = settingsStore.getUInt("dlCrc", 0);
   settings.imageValid = settingsStore.getBool("imgValid", true);
   settings.httpAuthUser = settingsStore.getString("httpAuthUser", "");
   settings.httpAuthPassword = settingsStore.getString("httpAuthPass", "");
   settings.motionWakeup = settingsStore.getBool("motionWakeup", false);
   settings.chargerMode = settingsStore.getBool("chargerMode", false);
   settings.settingsUrl = stripUrlFragment(settingsStore.getString("settingsUrl", ""));
   settings.settingsLastModified = settingsStore.getString("settingsLastMod", "");
   settings.autoRotation = settingsStore.getBool("autoRotation", true);
   settings.telemetry = settingsStore.getBool("telemetry", true);
   Serial.println("[MEM] Settings restored");
   DEBUG_LOG("[MEM] Settings - SleepDisabled: %d ImageMode: %d MotionWakeup: %d ChargerMode: %d\n", settings.sleepDisabled, settings.imageMode, settings.motionWakeup, settings.chargerMode);
}


class ServerCallbacks : public NimBLEServerCallbacks
{
   void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo) override {
      Serial.printf("\n[BLE]Client address: %s\n", connInfo.getAddress().toString().c_str());
      // Only one client at a time: advertising stops on connect and is not restarted until it disconnects
      if (pServer->getConnectedCount() > 1) {
         Serial.println("[BLE] Already connected to another client, rejecting");
         pServer->disconnect(connInfo.getConnHandle());
         return;
      }
      pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 180);
      isBleClientConnected = true;
   }
   void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason) override {
      Serial.printf("\n[BLE] Client disconnected (%u clients left)", pServer->getConnectedCount());
      // Several clients may be connected (e.g. a second browser tab): only the last disconnect counts
      isBleClientConnected = pServer->getConnectedCount() > 0;
      lastDisconnectTime = millis();
      // Only abort if the client that is uploading went away
      if ((bleUploadActive || fwUpdateInProgress) && connInfo.getConnHandle() == bleUploadConnHandle) {
         bleUploadActive = false;
         bleUploadConnHandle = 0xFFFF;
         blePostJob(BLE_JOB_ABORT);
      }
      if (wifiSettings.bleInitOk && pServer->getConnectedCount() == 0) {
         Serial.printf("\n[BLE] restart advertising...\n[");
         NimBLEDevice::startAdvertising();
      }
   };
   void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo) override {
      Serial.printf("\n[BLE] MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
   }

} serverCallbacks;

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
   void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
      std::string uuidStr = pCharacteristic->getUUID().toString();
      if (uuidStr == "10000004-0000-0000-0000-000000000001") {
         // Return current bytes in the RAM buffer, or an error / busy status
         uint16_t status = bleWriteBufferPos;
         if (bleStatusError)
            status = BLE_STATUS_ERROR;
         else if (bleWriteBufferPos == 0 && blePendingJobs > 0)
            status = BLE_STATUS_BUSY;
         pCharacteristic->setValue((uint8_t *)&status, 2);
         return;
      }
      Serial.printf("%s : onRead(), value: %s\n",
                    pCharacteristic->getUUID().toString().c_str(),
                    pCharacteristic->getValue().c_str());
   };

   void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo) override {
      std::string uuidStr = pCharacteristic->getUUID().toString();
      size_t dataLen = pCharacteristic->getValue().length();

      // Serial.printf("%s : onWrite(), value len: %d\n", uuidStr.c_str(), dataLen);

      // Für String-Werte eine sichere Konvertierung mit dem richtigen c_str()
      if (uuidStr == "a62eed84-7b0d-11ed-a1eb-0242ac120002") {
         wifiSettings.pss = pCharacteristic->getValue().c_str();
         Serial.println("[BLE] Set wifi password");
         wifiSettings.isDeployWifi = true;
      }
      else if (uuidStr == "090b0ef2-7b0d-11ed-a1eb-0242ac120002") {
         wifiSettings.ssid = pCharacteristic->getValue().c_str();
         Serial.printf("[BLE] Set wifi SSID: %s\n", wifiSettings.ssid.c_str());
      }
      else if (uuidStr == "10000001-0000-0000-0000-000000000001") {
         settings.downloadUrl = stripUrlFragment(pCharacteristic->getValue().c_str());
         invalidateDownloadCache(); // force a full download for the new URL
         Serial.printf("[BLE] Set URL: %s\n", settings.downloadUrl.c_str());
         if (settings.downloadUrl.length() > 0) {
            settings.imageMode = 1;
         }
      }
      else if (uuidStr == "10000007-0000-0000-0000-000000000001") {
         settings.httpAuthUser = pCharacteristic->getValue();
         Serial.printf("[BLE] Set HTTP Auth user: %s\n", settings.httpAuthUser.c_str());
      }
      else if (uuidStr == "10000008-0000-0000-0000-000000000001") {
         settings.httpAuthPassword = pCharacteristic->getValue();
         Serial.println("[BLE] Set HTTP Auth password");
      }
      else if (uuidStr == "10000002-0000-0000-0000-000000000001") {
         settings.imageMode = atoi(pCharacteristic->getValue().c_str());
         Serial.printf("[BLE] Set Image Mode: %d\n", settings.imageMode);
      }
      else if (uuidStr == "10000005-0000-0000-0000-000000000001") {
         settings.timeout = clampSleepTimeout(atoi(pCharacteristic->getValue().c_str()));
         Serial.printf("[BLE] Set Timeout/Sleep: %d\n", settings.timeout);
      }
      else if (pCharacteristic->getUUID().toString() == "1000000a-0000-0000-0000-000000000001") {
         String val = pCharacteristic->getValue().c_str();
         settings.chargerMode = (val == "1" || val == "true");
         Serial.printf("[BLE] Charger Mode updated to: %d\n", settings.chargerMode);
         saveSettings();
         chargeMode(settings.chargerMode);
      }
      else if (pCharacteristic->getUUID().toString() == "1000000c-0000-0000-0000-000000000001") {
         String val = pCharacteristic->getValue().c_str();
         settings.autoRotation = (val == "1" || val == "true");
         Serial.printf("[BLE] Auto Rotation updated to: %d\n", settings.autoRotation);
         saveSettings();
      }
      else if (uuidStr == "1000000d-0000-0000-0000-000000000001") {
         String val = pCharacteristic->getValue().c_str();
         settings.telemetry = (val == "1" || val == "true");
         Serial.printf("[BLE] Telemetry updated to: %d\n", settings.telemetry);
         saveSettings();
      }
      else if (pCharacteristic->getUUID().toString() == "1000000b-0000-0000-0000-000000000001") {
         settings.settingsUrl = stripUrlFragment(pCharacteristic->getValue().c_str());
         Serial.printf("[BLE] Set Settings URL: %s\n", settings.settingsUrl.c_str());
         settings.settingsLastModified = ""; // Reset cache on new URL
      }
      else if (uuidStr == "10000009-0000-0000-0000-000000000001") {
         String strVal = pCharacteristic->getValue().c_str();
         settings.motionWakeup = (strVal == "1" || strVal == "true");
         Serial.printf("[BLE] Set Motion Wakeup: %d\n", settings.motionWakeup);
      }
      else if (uuidStr == "10000004-0000-0000-0000-000000000001") {
         String cmd = pCharacteristic->getValue().c_str();
         Serial.printf("[BLE] Upload CMD: %s\n", cmd.c_str());
         if (cmd == "SCAN_WIFI") {
            wifiScanRequested = true;
            Serial.println("[BLE] WiFi Scan requested via BLE.");
         }
         else if (cmd == "EXIT_SETUP") {
            Serial.println("[BLE] EXIT_SETUP received. Forcing exit...");
            forceExitSetup = true;
         }
         else if (cmd == "START" || cmd == "START_FW") {
            if (!bleAllocBuffers()) {
               Serial.println("[BLE] ERROR: Failed to allocate memory for upload buffers!");
               bleStatusError = true;
               return;
            }
            bleStatusError = false;
            bleRxCrc = 0;
            bleWriteBufferPos = 0;
            bleUploadActive = true;
            bleUploadConnHandle = connInfo.getConnHandle();
            if (cmd == "START_FW") {
               tickerFailsave.detach();
               blePostJob(BLE_JOB_START_FW);
            }
            else {
               blePostJob(BLE_JOB_START_IMAGE);
            }
         }
         else if (cmd == "FLUSH") {
            bleHandOffBuffer();
         }
         else if (cmd == "CLEAR") {
            bleWriteBufferPos = 0;
            Serial.println("[BLE] Buffer CLEARED due to packet loss!");
         }
         else if (cmd == "END" || cmd == "END_FW") {
            bleHandOffBuffer();
            bleUploadActive = false;
            blePostJob(cmd == "END_FW" ? BLE_JOB_END_FW : BLE_JOB_END_IMAGE);
         }
         else if (cmd == "APPLY") {
            Serial.println("[BLE] Settings APPLY received.");
            blePostJob(BLE_JOB_APPLY);
         }
         else if (cmd == "SAVE_SETTINGS") {
            Serial.println("[BLE] SAVE_SETTINGS received.");
            saveSettings();
         }
         else if (cmd == "RESET") {
            Serial.println("[BLE] Settings RESET received.");
            blePostJob(BLE_JOB_RESET);
         }
      }
      else if (uuidStr == "10000003-0000-0000-0000-000000000001") {
         // Keep the value alive: getValue() returns a copy, a pointer into the temporary would dangle
         NimBLEAttValue value = pCharacteristic->getValue();
         const uint8_t *pData = value.data();
         dataLen = value.length();
         if (bleUploadActive && pData && dataLen > 4 && bleWriteBuffer != nullptr) {
            uint32_t packetCrc = pData[0] | (pData[1] << 8) | (pData[2] << 16) | (pData[3] << 24);
            size_t actualDataLen = dataLen - 4;
            uint32_t calculatedCrc = calcCRC32(pData + 4, actualDataLen);

            if (packetCrc == calculatedCrc) {
               if (bleWriteBufferPos + actualDataLen <= BLE_BUFFER_SIZE) {
                  memcpy(bleWriteBuffer + bleWriteBufferPos, pData + 4, actualDataLen);
                  bleWriteBufferPos += actualDataLen;
               }
               else {
                  Serial.println("[BLE] ERROR: RAM buffer overflow! Checkpoint logic failed.");
               }
            }
            else {
               Serial.printf("[BLE] CRC Error! Expected 0x%08X, got 0x%08X\n", calculatedCrc, packetCrc);
            }
         }
      }
   };

   void onStatus(NimBLECharacteristic *pCharacteristic, int code) override {
      Serial.printf("Notification/Indication return code: %d, %s\n", code, NimBLEUtils::returnCodeToString(code));
   }
} chrCallbacks;

/** Handler class for descriptor actions */
class DescriptorCallbacks : public NimBLEDescriptorCallbacks
{
   void onWrite(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo) override {
      std::string dscVal = pDescriptor->getValue();
      Serial.printf("[BLE] Descriptor written value: %s\n", dscVal.c_str());
   }

   void onRead(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo) override {
      Serial.printf("[BLE] %s Descriptor read\n", pDescriptor->getUUID().toString().c_str());
   }
} dscCallbacks;

bool BleInit(String deviceId, bool enable) {
   if (!enable) {
      if (wifiSettings.bleInitOk) {
         ledBlink(500, true);
         wifiSettings.bleInitOk = false;
         pAdvertising->stop();
         esp_bt_controller_enable(ESP_BT_MODE_BLE);
         esp_bt_controller_disable();
         esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);
         // NimBLEDevice::deinit(false); //crashes with panic
         Serial.println("[BLE] BLE stopped");
      }
      return true;
   }
   if (wifiSettings.bleInitOk) {
      Serial.println("[BLE] already initialized, skip...");
      return true;
   }
   String wifiSsidScan;
   WiFi.mode(WIFI_STA);
   delay(1);

   NimBLEDevice::init(deviceId.c_str());
   // NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
   NimBLEDevice::setSecurityAuth(false, false, true);

#ifdef ESP_PLATFORM
   NimBLEDevice::setPower(ESP_PWR_LVL_P9);
#else
   NimBLEDevice::setPower(9);
#endif

   pServer = NimBLEDevice::createServer();
   pServer->setCallbacks(&serverCallbacks);

   NimBLEService *deviceDataService = pServer->createService("7f74170e-7b0e-11ed-a1eb-0242ac120002");
   wifiConnectedCharacteristic = deviceDataService->createCharacteristic("4c578d4c-7b0e-11ed-a1eb-0242ac120002", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
   wifiInfoCharacteristic = deviceDataService->createCharacteristic("4c578d4d-7b0e-11ed-a1eb-0242ac120002", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
   wifiScanCharacteristic = deviceDataService->createCharacteristic("5131a3fc-7b0e-11ed-a1eb-0242ac120002", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
   systemInfoCharacteristic = deviceDataService->createCharacteristic("60000001-7b0e-11ed-a1eb-0242ac120002", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
   if (wifiSsidScan.length() > 0) {
      wifiScanCharacteristic->setValue(wifiSsidScan.c_str());
   }

   NimBLEService *wifiDataService = pServer->createService("0515c086-7b0c-11ed-a1eb-0242ac120002");
   NimBLECharacteristic *wifiSsidCharacteristic = wifiDataService->createCharacteristic("090b0ef2-7b0d-11ed-a1eb-0242ac120002", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
   NimBLECharacteristic *wifiPwCharacteristic = wifiDataService->createCharacteristic("a62eed84-7b0d-11ed-a1eb-0242ac120002", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

   wifiSsidCharacteristic->setCallbacks(&chrCallbacks);
   wifiPwCharacteristic->setCallbacks(&chrCallbacks);
   wifiConnectedCharacteristic->setCallbacks(&chrCallbacks);
   wifiInfoCharacteristic->setCallbacks(&chrCallbacks);
   wifiScanCharacteristic->setCallbacks(&chrCallbacks);

   NimBLE2904 *wifiSsidDescriptor = (NimBLE2904 *)wifiSsidCharacteristic->createDescriptor("2904");
   NimBLE2904 *wifiPwDescriptor = (NimBLE2904 *)wifiPwCharacteristic->createDescriptor("2904");
   NimBLE2904 *wifiScanDescriptor = (NimBLE2904 *)wifiScanCharacteristic->createDescriptor("2904");

   wifiSsidDescriptor->setDescription(0x00);
   wifiSsidDescriptor->setNamespace(0x00);
   wifiSsidDescriptor->setFormat(NimBLE2904::FORMAT_UTF8);
   wifiSsidDescriptor->setCallbacks(&dscCallbacks);

   wifiPwDescriptor->setDescription(0x00);
   wifiPwDescriptor->setNamespace(0x00);
   wifiPwDescriptor->setFormat(NimBLE2904::FORMAT_UTF8);
   wifiPwDescriptor->setCallbacks(&dscCallbacks);

   wifiScanDescriptor->setDescription(0x00);
   wifiScanDescriptor->setNamespace(0x00);
   wifiScanDescriptor->setFormat(NimBLE2904::FORMAT_UTF8);
   wifiScanDescriptor->setCallbacks(&dscCallbacks);

   NimBLEService *epaperSettingsService = pServer->createService("10000000-0000-0000-0000-000000000001");
   NimBLECharacteristic *urlCharacteristic = epaperSettingsService->createCharacteristic("10000001-0000-0000-0000-000000000001", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
   NimBLECharacteristic *imageModeCharacteristic = epaperSettingsService->createCharacteristic("10000002-0000-0000-0000-000000000001", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
   NimBLECharacteristic *uploadDataCharacteristic = epaperSettingsService->createCharacteristic("10000003-0000-0000-0000-000000000001", NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
   NimBLECharacteristic *uploadCmdCharacteristic = epaperSettingsService->createCharacteristic("10000004-0000-0000-0000-000000000001", NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::READ);
   NimBLECharacteristic *timeoutCharacteristic = epaperSettingsService->createCharacteristic("10000005-0000-0000-0000-000000000001", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
   NimBLECharacteristic *settingsUrlCharacteristic = epaperSettingsService->createCharacteristic("1000000b-0000-0000-0000-000000000001", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
   NimBLECharacteristic *httpAuthUserCharacteristic = epaperSettingsService->createCharacteristic("10000007-0000-0000-0000-000000000001", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
   NimBLECharacteristic *httpAuthPasswordCharacteristic = epaperSettingsService->createCharacteristic("10000008-0000-0000-0000-000000000001", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
   NimBLECharacteristic *motionWakeupCharacteristic = epaperSettingsService->createCharacteristic("10000009-0000-0000-0000-000000000001", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
   NimBLECharacteristic *chargerModeCharacteristic = epaperSettingsService->createCharacteristic("1000000a-0000-0000-0000-000000000001", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
   NimBLECharacteristic *autoRotationCharacteristic = epaperSettingsService->createCharacteristic("1000000c-0000-0000-0000-000000000001", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);
   NimBLECharacteristic *telemetryCharacteristic = epaperSettingsService->createCharacteristic("1000000d-0000-0000-0000-000000000001", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE);

   wifiSsidCharacteristic->setCallbacks(&chrCallbacks);
   wifiPwCharacteristic->setCallbacks(&chrCallbacks);
   urlCharacteristic->setCallbacks(&chrCallbacks);
   imageModeCharacteristic->setCallbacks(&chrCallbacks);
   uploadDataCharacteristic->setCallbacks(&chrCallbacks);
   uploadCmdCharacteristic->setCallbacks(&chrCallbacks);
   timeoutCharacteristic->setCallbacks(&chrCallbacks);
   settingsUrlCharacteristic->setCallbacks(&chrCallbacks);
   httpAuthUserCharacteristic->setCallbacks(&chrCallbacks);
   ;
   httpAuthPasswordCharacteristic->setCallbacks(&chrCallbacks);
   motionWakeupCharacteristic->setCallbacks(&chrCallbacks);
   chargerModeCharacteristic->setCallbacks(&chrCallbacks);
   autoRotationCharacteristic->setCallbacks(&chrCallbacks);
   telemetryCharacteristic->setCallbacks(&chrCallbacks);

   // Services are started when the server is started

   wifiSsidCharacteristic->setValue(wifiSettings.ssid.c_str());
   wifiPwCharacteristic->setValue(wifiSettings.pss.c_str());
   uint8_t initVal = wifiSettings.wifiIsConnected ? 1 : 0;
   wifiConnectedCharacteristic->setValue(&initVal, 1);
   wifiInfoCharacteristic->setValue("{}");
   wifiScanCharacteristic->setValue(wifiSsidScan);

   JsonDocument sysDoc;
   sysDoc["voltage"] = systemData.vddValue;
   sysDoc["usb"] = systemData.usbConnected;
   bool isChargingInit = false;
   if (settings.chargerMode) {
      isChargingInit = chargeMode(settings.chargerMode);
   }
   sysDoc["charging"] = isChargingInit;
   String sysOut;
   serializeJson(sysDoc, sysOut);
   systemInfoCharacteristic->setValue(sysOut.c_str());

   urlCharacteristic->setValue(settings.downloadUrl.c_str());
   char imgModeStr[8];
   sprintf(imgModeStr, "%d", settings.imageMode);
   imageModeCharacteristic->setValue(imgModeStr);

   char timeoutStr[8];
   sprintf(timeoutStr, "%d", settings.timeout);
   timeoutCharacteristic->setValue(timeoutStr);
   settingsUrlCharacteristic->setValue(settings.settingsUrl.c_str());
   httpAuthUserCharacteristic->setValue(settings.httpAuthUser.c_str());
   ;
   httpAuthPasswordCharacteristic->setValue(settings.httpAuthPassword.c_str());
   motionWakeupCharacteristic->setValue(settings.motionWakeup ? "1" : "0");
   chargerModeCharacteristic->setValue(settings.chargerMode ? "1" : "0");
   autoRotationCharacteristic->setValue(settings.autoRotation ? "1" : "0");
   telemetryCharacteristic->setValue(settings.telemetry ? "1" : "0");

   pAdvertising = NimBLEDevice::getAdvertising();
   pAdvertising->setName(deviceId.c_str());
   pAdvertising->addServiceUUID(wifiDataService->getUUID());
   pAdvertising->addServiceUUID(deviceDataService->getUUID());
   pAdvertising->addServiceUUID(epaperSettingsService->getUUID());
   pAdvertising->enableScanResponse(true);
   pAdvertising->start();
   ledBlink(200, true);
   Serial.printf("[BLE] BLE Advertising started: %s \n", deviceId.c_str());
   wifiSettings.bleInitOk = true;
   return true;
}
// Stream sink for HTTPClient::writeToStream that writes into a pre-allocated SerialFlash file,
// erasing 64 KB blocks just before they are first written. Rejects data beyond the file size.
class FlashFileWriter : public Stream {
 public:
   explicit FlashFileWriter(SerialFlashFile &file) : _file(file) {}
   size_t write(uint8_t b) override { return write(&b, 1); }
   size_t write(const uint8_t *buf, size_t len) override {
      if (_written + len > _file.size()) {
         _overflow = true;
         setWriteError();
         return 0;
      }
      while (_erased < _written + len) {
         SerialFlash.eraseBlock(_file.getFlashAddress() + _erased);
         _erased += SerialFlash.blockSize();
      }
      _file.seek(_written);
      _file.write(buf, len);
      _crc = crc32_le(_crc, buf, len);
      _written += len;
      return len;
   }
   int available() override { return 0; }
   int read() override { return -1; }
   int peek() override { return -1; }
   uint32_t length() const { return _written; }
   uint32_t crc() const { return _crc; }
   bool overflow() const { return _overflow; }

 private:
   SerialFlashFile &_file;
   uint32_t _written = 0;
   uint32_t _erased = 0;
   uint32_t _crc = 0;
   bool _overflow = false;
};

// ---- BLE upload worker ----
// Image uploads are staged in DOWNLOAD_FILE and only copied to IMAGE_FILE (and imageMode switched)
// after END with the exact image size, so a failed upload keeps the old image and mode (#18).
SerialFlashFile bleStagingFile;
FlashFileWriter *bleStagingWriter = nullptr;
bool bleImageReady = false; // END succeeded, APPLY may render

void bleRunJob(const BleJob &job) {
   switch (job.type) {
   case BLE_JOB_START_IMAGE:
      delete bleStagingWriter;
      bleStagingWriter = nullptr;
      bleStagingFile = SerialFlash.open(DOWNLOAD_FILE);
      if (bleStagingFile)
         bleStagingWriter = new FlashFileWriter(bleStagingFile);
      else
         bleStatusError = true;
      bleBytesReceived = 0;
      bleWrittenCrc = 0;
      bleImageReady = false;
      Serial.println("[BLE] Image upload STARTED.");
      break;

   case BLE_JOB_START_FW:
      bleBytesReceived = 0;
      bleWrittenCrc = 0;
      if (Update.begin(UPDATE_SIZE_UNKNOWN)) {
         fwUpdateInProgress = true;
         Serial.println("[BLE] Firmware Update STARTED.");
      }
      else {
         Update.printError(Serial);
         bleStatusError = true;
      }
      break;

   case BLE_JOB_FLUSH:
      bleWrittenCrc = crc32_le(bleWrittenCrc, bleFlushBuffer, job.len);
      if (fwUpdateInProgress) {
         if (Update.write(bleFlushBuffer, job.len) != job.len) {
            Update.printError(Serial);
            bleStatusError = true;
         }
      }
      else if (bleStagingWriter) {
         if (bleStagingWriter->write(bleFlushBuffer, job.len) != job.len) {
            Serial.println("[BLE] ERROR: image upload too large");
            bleStatusError = true;
         }
      }
      bleBytesReceived += job.len;
      bleFlushBufferBusy = false;
      break;

   case BLE_JOB_END_IMAGE: {
      const uint32_t expected = EPD_WIDTH * EPD_HEIGHT / 2;
      Serial.printf("[BLE] Upload ENDED. Bytes: %u CRC32 received: 0x%08X written: 0x%08X\n", bleBytesReceived, bleRxCrc, bleWrittenCrc);
      if (!bleStagingWriter || bleStatusError || bleBytesReceived != expected) {
         Serial.printf("[BLE] ERROR: upload rejected (expected %u bytes, got %u). Keeping current image and mode.\n", expected, bleBytesReceived);
         bleStatusError = true;
      }
      else {
         SerialFlashFile out = openErasedFile(IMAGE_FILE);
         if (out && copyFlashFile(bleStagingFile, bleBytesReceived, out)) {
            setImageValid(true);
            invalidateDownloadCache();
            settings.imageMode = 0;
            saveSettings();
            bleImageReady = true;
            Serial.println("[BLE] Image stored. Switched to ImageMode 0.");
         }
         else {
            setImageValid(false);
            bleStatusError = true;
         }
      }
      delete bleStagingWriter;
      bleStagingWriter = nullptr;
      break;
   }

   case BLE_JOB_END_FW:
      fwUpdateInProgress = false;
      Serial.printf("[BLE] Firmware upload ENDED. Bytes: %u CRC32 received: 0x%08X written: 0x%08X\n", bleBytesReceived, bleRxCrc, bleWrittenCrc);
      if (!bleStatusError && Update.end(true)) {
         Serial.println("[BLE] Firmware Update SUCCESS. Rebooting...");
         delay(500);
         ESP.restart();
      }
      Update.printError(Serial);
      if (Update.isRunning())
         Update.abort();
      Serial.println("[BLE] Firmware Update FAILED.");
      bleStatusError = true;
      break;

   case BLE_JOB_APPLY:
      if (bleImageReady) {
         bleImageReady = false;
         applyPending = true;
      }
      else {
         Serial.println("[BLE] APPLY ignored, no valid uploaded image.");
      }
      break;

   case BLE_JOB_ABORT:
      if (fwUpdateInProgress) {
         Update.abort();
         fwUpdateInProgress = false;
         Serial.println("[BLE] Firmware update aborted (disconnect).");
      }
      bleStatusError = true; // a client still polling the status sees the upload failed
      delete bleStagingWriter;
      bleStagingWriter = nullptr;
      bleFlushBufferBusy = false;
      break;

   case BLE_JOB_RESET:
      resetAll(true);
      ESP.restart();
      break;
   }
}

void bleWorkerTask(void *) {
   BleJob job;
   for (;;) {
      if (xQueueReceive(bleJobQueue, &job, portMAX_DELAY) != pdTRUE)
         continue;
      spiLock();
      bleRunJob(job);
      spiUnlock();
      if (job.type == BLE_JOB_END_IMAGE || job.type == BLE_JOB_END_FW)
         logStackHighWater("after BLE upload");
      blePendingJobs--;
   }
}

void bleWorkerInit() {
   if (bleJobQueue)
      return;
   spiMutex = xSemaphoreCreateRecursiveMutex();
   bleJobQueue = xQueueCreate(8, sizeof(BleJob));
   xTaskCreate(bleWorkerTask, "bleWorker", 6144, nullptr, 2, nullptr);
}

// After setup mode: let queued jobs finish, then release the upload buffers
void bleWorkerShutdown() {
   unsigned long waitStart = millis();
   while (blePendingJobs > 0 && millis() - waitStart < 30000) {
      delay(10);
   }
   // An unfinished upload must not leave a half-written OTA partition or staging writer behind
   if (bleUploadActive || fwUpdateInProgress) {
      Serial.println("[BLE] Setup mode ended during an upload, aborting it");
      bleUploadActive = false;
      if (blePostJob(BLE_JOB_ABORT)) {
         waitStart = millis();
         while (blePendingJobs > 0 && millis() - waitStart < 5000) {
            delay(10);
         }
      }
   }
   free(bleWriteBuffer);
   bleWriteBuffer = nullptr;
   free(bleFlushBuffer);
   bleFlushBuffer = nullptr;
}

// Download url into fileName. Returns DL_OK, DL_NOT_MODIFIED or a DL_ERR_* code.
// On DL_OK, downloadedLength holds the byte count and tempLastModified / tempDownloadCrc
// are pending until commitDownloadCache() is called after a successful render.
int downloadAndSaveFile(const char *fileName, String url) {
   tempLastModified = "";
   tempDownloadCrc = 0;
   downloadedLength = 0;
   SerialFlashFile file = SerialFlash.open(fileName);
   if (!file) {
      Serial.printf("[DL] Download file %s missing\n", fileName);
      return DL_ERR_FLASH;
   }

   WiFi.setSleep(false);
   WiFiClientSecure secureClient;
   secureClient.setInsecure();
   HTTPClient http;
   http.setTimeout(10000);
   http.setReuse(false);
   // Same-protocol redirects are followed by HTTPClient; http <-> https hops are followed below
   http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);

   bool useCache = !settings.forceDownload;
   const char *headerKeys[] = {"Last-Modified"};
   int httpCode = 0;
   for (int hop = 0; hop < 3; hop++) {
      if (url.startsWith("https:")) {
         Serial.println("[DL] Download HTTPS");
         http.begin(secureClient, url);
      }
      else {
         Serial.println("[DL] Download HTTP");
         http.begin(url);
      }

      if (settings.httpAuthUser.length() > 0) {
         http.setAuthorization(settings.httpAuthUser.c_str(), settings.httpAuthPassword.c_str());
         Serial.println("[DL] HTTP Auth enabled");
      }
      if (useCache && settings.lastModified.length() > 0) {
         http.addHeader("If-Modified-Since", settings.lastModified);
      }
      http.collectHeaders(headerKeys, 1);

      httpCode = http.GET();
      bool isRedirect = httpCode == 301 || httpCode == 302 || httpCode == 303 || httpCode == 307 || httpCode == 308;
      if (!isRedirect || http.getLocation().length() == 0)
         break;
      url = http.getLocation();
      Serial.printf("[DL] Redirect (%d) to %s\n", httpCode, url.c_str());
      http.end();
   }

   if (httpCode < 0) {
      Serial.printf("[DL] Request failed: %s\n", http.errorToString(httpCode).c_str());
      http.end();
      return DL_ERR_NETWORK;
   }
   if (httpCode == HTTP_CODE_NOT_MODIFIED) {
      Serial.println("[DL] 304 Not Modified");
      http.end();
      return DL_NOT_MODIFIED;
   }
   if (httpCode != HTTP_CODE_OK) {
      Serial.printf("[DL] HTTP status %d\n", httpCode);
      http.end();
      return (httpCode >= 500) ? DL_ERR_SERVER : DL_ERR_HTTP_STATUS;
   }

   String lastMod = http.header("Last-Modified");
   if (useCache && lastMod.length() > 0 && lastMod == settings.lastModified) {
      Serial.println("[DL] File not modified (Last-Modified), skipping download.");
      http.end();
      return DL_NOT_MODIFIED;
   }

   int len = http.getSize();
   Serial.printf("[DL] Download Size: %d (capacity %u)\n", len, file.size());
   if (len > (int)file.size()) {
      Serial.println("[DL] ERROR: Download larger than flash file");
      http.end();
      return DL_ERR_TOO_LARGE;
   }

   FlashFileWriter writer(file);
   int ret = http.writeToStream(&writer);
   http.end();
   if (writer.overflow()) {
      Serial.println("[DL] ERROR: Download larger than flash file");
      return DL_ERR_TOO_LARGE;
   }
   if (ret < 0) {
      Serial.printf("[DL] Transfer failed after %u bytes: %s\n", writer.length(), http.errorToString(ret).c_str());
      return DL_ERR_STREAM;
   }
   if ((len >= 0 && writer.length() != (uint32_t)len) || writer.length() == 0) {
      Serial.printf("[DL] Size mismatch: expected %d, got %u\n", len, writer.length());
      return DL_ERR_SIZE;
   }
   Serial.printf("[DL] Received %u bytes, CRC 0x%08X\n", writer.length(), writer.crc());

   if (useCache && settings.downloadCrc != 0 && writer.crc() == settings.downloadCrc) {
      Serial.println("[DL] Content unchanged (CRC), skipping refresh.");
      return DL_NOT_MODIFIED;
   }
   downloadedLength = writer.length();
   tempDownloadCrc = writer.crc();
   tempLastModified = lastMod;
   return DL_OK;
}

// Persist Last-Modified / CRC of the download that was just rendered
void commitDownloadCache() {
   if (tempDownloadCrc == 0 && tempLastModified.length() == 0)
      return;
   settings.lastModified = tempLastModified;
   settings.downloadCrc = tempDownloadCrc;
   tempLastModified = "";
   tempDownloadCrc = 0;
   saveSettings();
}

// Forget what was last downloaded so the next download is rendered unconditionally
void invalidateDownloadCache() {
   settings.lastModified = "";
   settings.downloadCrc = 0;
}

void setImageValid(bool valid) {
   if (settings.imageValid == valid)
      return;
   settings.imageValid = valid;
   settingsStore.putBool("imgValid", valid);
}

// Create IMAGE_FILE and DOWNLOAD_FILE once with fixed sizes. Existing files are reused.
void initFlashFiles() {
   uint8_t id[5];
   SerialFlash.readID(id);
   Serial.printf("[FLASH] Capacity %u bytes\n", SerialFlash.capacity(id));

   for (int attempt = 0; attempt < 2; attempt++) {
      bool ok = true;
      SerialFlashFile img = SerialFlash.open(IMAGE_FILE);
      if (!img) {
         ok = SerialFlash.createErasable(IMAGE_FILE, IMAGE_FILE_SIZE);
      }
      else if (img.size() < IMAGE_FILE_SIZE) {
         Serial.printf("[FLASH] %s too small (%u)\n", IMAGE_FILE, img.size());
         ok = false;
      }

      if (ok && !SerialFlash.exists(DOWNLOAD_FILE)) {
         uint32_t size = DOWNLOAD_FILE_MAX;
         while (!SerialFlash.createErasable(DOWNLOAD_FILE, size) && size / 2 >= IMAGE_FILE_SIZE) {
            size /= 2;
         }
         ok = SerialFlash.exists(DOWNLOAD_FILE);
      }

      if (ok) {
         SerialFlashFile dl = SerialFlash.open(DOWNLOAD_FILE);
         Serial.printf("[FLASH] %s: %u bytes, %s: %u bytes\n", IMAGE_FILE, SerialFlash.open(IMAGE_FILE).size(), DOWNLOAD_FILE, dl.size());
#if EPD_VIEW_ROTATED
         // Optional: without it portrait images are not rotated
         if (!SerialFlash.exists(ROTATE_FILE) && !SerialFlash.createErasable(ROTATE_FILE, ROTATE_FILE_SIZE)) {
            Serial.println("[FLASH] Could not create " ROTATE_FILE);
         }
#endif
         return;
      }
      if (attempt == 0) {
         // Last resort: the layout is unusable (old undersized file or flash full), start over
         Serial.println("[FLASH] Erasing external flash to recreate files...");
         SerialFlash.eraseAll();
         while (!SerialFlash.ready()) {
            delay(100);
         }
         setImageValid(false);
         invalidateDownloadCache();
         saveSettings();
      }
   }
   Serial.println("[FLASH] ERROR: could not create image files");
}

// https://github.com/zenmanenergy/ESP8266-Arduino-Examples/blob/master/helloWorld_urlencoded/urlencode.ino
// Seconds per retry step; build with -DDL_RETRY_UNIT=2 for a quick test (10 s, 30 s, 60 s)
#ifndef DL_RETRY_UNIT
#define DL_RETRY_UNIT 60
#endif

// Retry delay after the n-th consecutive download failure: 5, 15, then 30 min
int downloadRetryDelay(int failCount) {
   if (failCount <= 1)
      return 5 * DL_RETRY_UNIT;
   if (failCount == 2)
      return 15 * DL_RETRY_UNIT;
   return 30 * DL_RETRY_UNIT;
}

int clampSleepTimeout(int seconds) {
   int clamped = constrain(seconds, MIN_SLEEP_TIMEOUT, MAX_SLEEP_TIMEOUT);
   if (clamped != seconds)
      Serial.printf("[MAIN] Timeout %d out of range, clamped to %d\n", seconds, clamped);
   return clamped;
}

int storeSleepTimeMem(int updateTime) {
   int returnSleepTime = settingsStore.getInt("timeout", DEFAULT_SLEEP);
   if (updateTime > 0) {
      if (returnSleepTime != updateTime) {
         systemData.newSleepTimeSet = true;
         DEBUG_LOG("[MEM] Update Sleep Time to %d\n", updateTime);
         settingsStore.putInt("timeout", updateTime);
      }
      returnSleepTime = updateTime;
   }
   else {
      DEBUG_LOG("[MEM] Get Sleep Time: %d\n", returnSleepTime);
   }
   return returnSleepTime;
}

void logHeap() {
   DEBUG_LOG("[MAIN] heap free: %d/%d\n", ESP.getFreeHeap(), ESP.getHeapSize());
}

#ifdef EPD_TYPE_13INCH
bool sdInit() {
   if (systemData.sdReady) {
      DEBUG_LOG("[SD] skip, init is done...\n");
      return true;
   }
   if (!sd.begin(SdSpiConfig(CS_SD_PIN, SHARED_SPI, DISPLAY_SPI_SPEED))) {
      Serial.println("[SD] mount failed, attempting to format the card...");
      FatFormatter fatFormatter;
      uint8_t buffer[512];
      SdCardFactory cardFactory;
      SdCard *m_card = cardFactory.newCard(SdSpiConfig(CS_SD_PIN, SHARED_SPI, DISPLAY_SPI_SPEED));

      if (!m_card || m_card->errorCode()) {
         Serial.println("[SD] Hardware error: could not detect SD card!");
         return false;
      }
      else {
         bool formatSuccess = fatFormatter.format(m_card, buffer, &Serial);
         if (formatSuccess) {
            Serial.println("Format complete! Retrying mount...");
            if (sd.begin(SdSpiConfig(CS_SD_PIN, SHARED_SPI, DISPLAY_SPI_SPEED))) {
               Serial.println("[SD] mounted successfully after format.");
               systemData.sdReady = true;
               return true;
            }
            else {
               Serial.println("[SD] mount failed even after format.");
               return false;
            }
         }
         else {
            Serial.println("[SD] Format failed!");
            return false;
         }
      }
   }
   else {
      Serial.println("[SD] initialization done.");
      systemData.sdReady = true;
      return true;
   }
   return false;
}
#endif

// Append device telemetry as query parameters if the telemetry setting is on (stored URLs never contain a #fragment)
String appendTelemetryParams(String url) {
   if (!settings.telemetry)
      return url;
   bool isCharging = false;
   if (settings.chargerMode) {
      isCharging = chargeMode(settings.chargerMode);
   }
   // With autoRotation the orientation was read at the start of loop(); otherwise read it without touching the display rotation
   int orientation = settings.autoRotation ? systemData.deviceOrientation : accInit();

   url += (url.indexOf('?') >= 0) ? "&" : "?";
   url += "voltage=" + String(systemData.vddValue);
   url += "&usb=" + String(systemData.usbConnected ? 1 : 0);
   url += "&charging=" + String(isCharging ? 1 : 0);
   if (orientation >= 0)
      url += "&orientation=" + String(orientation);
   url += "&width=" + String(EPD_WIDTH);
   url += "&height=" + String(EPD_HEIGHT);
   url += "&fw=" + String(SOFTWARE_VERSION);
   return url;
}

int loadImageFromWeb(String url, String fileName) {
   if (url.length() < 1)
      return -2;
   url = appendTelemetryParams(url);
   Serial.print("[DL] Download Image file: ");
   Serial.println(url);
   int downloadOk = DL_ERR_NETWORK;
   for (int i = 0; i <= 5; i++) {
      logHeap();
      downloadOk = downloadAndSaveFile(fileName.c_str(), url);
      bool retry = (downloadOk == DL_ERR_NETWORK || downloadOk == DL_ERR_STREAM || downloadOk == DL_ERR_SERVER || downloadOk == DL_ERR_SIZE);
      if (!retry)
         return downloadOk;
      Serial.printf("[DL] Attempt %d failed (%d), retrying\n", i + 1, downloadOk);
      if (WiFi.status() != WL_CONNECTED) {
         WiFi.disconnect(true);
         connectWifi(10000);
      }
      else {
         delay(1000);
      }
   }
   return downloadOk;
}

String setLeadingZero(String input) {
   String result = "";
   if (input.length() == 1) {
      result = "0" + input;
   }
   else {
      result = input;
   }

   return result;
}

void setDeviceUid() {
   byte mac[6];
   WiFi.mode(WIFI_STA);
   WiFi.macAddress(mac);
   DEBUG_LOG("[WIFI] MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
   String uniq = setLeadingZero(String(mac[0], HEX)) + setLeadingZero(String(mac[1], HEX)) + setLeadingZero(String(mac[2], HEX)) + setLeadingZero(String(mac[3], HEX)) + setLeadingZero(String(mac[4], HEX)) + setLeadingZero(String(mac[5], HEX));
   const uint8_t MSG_BUF_SIZE = 100;
   char mesg[MSG_BUF_SIZE] = {'\0'}; // Serial input message buffer
   uniq.toCharArray(mesg, MSG_BUF_SIZE);
   sprintf(CLIENT_ID, "%s%s", EPD_TYPE_IDENTIFIER, mesg);
   Serial.print("[MAIN] UID: ");
   Serial.println(CLIENT_ID);
}

// sleep x seconds
void gotToDeepSleep(int wakeuptimeout, bool motionWake) {
   Serial.printf("[MAIN] Going to Sleep for %d seconds (MotionWake: %d)\n", wakeuptimeout, motionWake);
   checkOrientationInBackground(false);
   if (!settings.sleepDisabled)
      WiFi.disconnect(true);
   if (motionWake) {
      accIntSet(80); // Set acc int wakeup /TODO: disable if no motion wakeup
   }
   else {
      accIntSet(0);
   }
   waitDisplayComplete(false);
   ledBlink(0, false);
   delay(5);
   display.hibernate();
   powerSupplyDisplay(false);
   delay(5);
   Serial.flush();
   delay(10);
   if (!settings.sleepDisabled)
      setCpuFrequencyMhz(40);
   delay(5);
   pinMode(RST_PIN, INPUT);
   pinMode(CS_EPD_PIN, INPUT);
   pinMode(DC_PIN, INPUT);
   if (!settings.sleepDisabled)
      pinMode(LED_PIN, INPUT);
   pinMode(CS_FLASH_PIN, INPUT);
   pinMode(SCK_PIN, INPUT);
   pinMode(MOSI_PIN, INPUT);
   pinMode(MISO_PIN, INPUT);
   pinMode(I2C_SDA_PIN, INPUT);
   pinMode(I2C_SCL_PIN, INPUT);
   pinMode(BAT_VOLT_EN_PIN, INPUT);
   pinMode(CHG_EN_PIN, INPUT);
   pinMode(12, INPUT); // USB TEST PINS
   pinMode(13, INPUT); // USB TEST PINS
   if (settings.sleepDisabled) {
      Serial.printf("[SLEEP] enter soft sleep\n");
      tickerFailsave.detach();
      ledBlink(2000, true);
      while (true) {
         // TODO: add trigger for reboot via wifi or ble for soft sleep
         delay(100);
      }
   }
   esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
   if (wakeuptimeout > 0) {
      esp_sleep_enable_timer_wakeup(wakeuptimeout * uS_TO_S_FACTOR);
   }
   else {
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
   }
   if (motionWake) {
      esp_sleep_enable_ext1_wakeup_io(BUTTON_PIN_BITMASK(GPIO_NUM_0), ESP_EXT1_WAKEUP_ANY_HIGH);
      rtc_gpio_pulldown_en(GPIO_NUM_0);
      rtc_gpio_pullup_dis(GPIO_NUM_0);
   }
   logStackHighWater("before deep sleep");
   esp_deep_sleep_start();
   ESP.restart(); // will never reach in deep sleep
}

int calculateSleepDuration(int defaultTimeout, bool forceReset, bool getDataOnly) {
   struct timeval now;
   gettimeofday(&now, NULL);

   if (getDataOnly) {
      time_t diff = now.tv_sec - previousWakeup.tv_sec;
      int predictedSleep = defaultTimeout - diff;
      if (predictedSleep < SLEEP_RECALCULATION_PERIOD_SECONDS || predictedSleep <= 0) {
         predictedSleep = defaultTimeout;
      }
      Serial.printf("[SLEEP] Return predicted sleep data only: %d \n", predictedSleep);

      return predictedSleep;
   }

   if (systemData.wakeupCause == wakeup_reason_t::TIMER) {
      DEBUG_LOG("[SLEEP] no manual wakeup (timer) - using full wakeup period...\n");
      previousWakeup = now;
      return defaultTimeout;
   }
   if (systemData.wakeupCause == wakeup_reason_t::BUTTON) {
      // TODO: cloud time feedback is needed here to calculate new wakeup
      DEBUG_LOG("[SLEEP] button wakeup - not able to calculate new sleep internally...\n");
      previousWakeup = now;
      return defaultTimeout;
   }
   if (forceReset) {
      DEBUG_LOG("[SLEEP] force reset - using full wakeup period...\n");
      previousWakeup = now;
      return defaultTimeout;
   }

   if (previousWakeup.tv_sec > now.tv_sec) {
      DEBUG_LOG("[SLEEP] detected invalid previous wakeup, resetting...\n");
      previousWakeup = timeval{
          .tv_sec = 0,
          .tv_usec = 0};
   }

   time_t diff = now.tv_sec - previousWakeup.tv_sec;

   if (diff <= 0) {
      DEBUG_LOG("[SLEEP] detected non positive sleep difference, resetting...\n");
      previousWakeup = now;
      diff = 0;
   }

   Serial.printf("[SLEEP] current time: %lld, previous time: %lld, difference: %lld\n", now.tv_sec, previousWakeup.tv_sec, diff);
   int predictedSleep = defaultTimeout - diff;
   // if the user refreshes less than one minute before the actual refresh will occur, skip that next refresh and use the current one instead.
   if (predictedSleep < SLEEP_RECALCULATION_PERIOD_SECONDS || predictedSleep <= 0) {
      Serial.printf("[SLEEP] manual reset within %d seconds of wakeup window. Reset to full window next\n", SLEEP_RECALCULATION_PERIOD_SECONDS);
      previousWakeup = now;
      return defaultTimeout;
   }

   return predictedSleep;
}

wakeup_reason_t getWakeupReason() {
   int resetReason0 = rtc_get_reset_reason(0);
   int resetReason1 = rtc_get_reset_reason(1);
   int wakeupReason = esp_sleep_get_wakeup_cause();
   DEBUG_LOG("[WAKE] Reset Reason 0: %d\n", resetReason0);
   DEBUG_LOG("[WAKE] Reset Reason 1: %d\n", resetReason1);
   DEBUG_LOG("[WAKE] Wake Reason: %d\n", wakeupReason);
   if (wakeupReason == esp_sleep_wakeup_cause_t::ESP_SLEEP_WAKEUP_UNDEFINED && resetReason0 == 1 && resetReason1 == 1) {
      Serial.printf("[WAKE] Got Button Wakeup or Power Loss Wakeup\n");
      return wakeup_reason_t::BUTTON;
   }
   if (wakeupReason == esp_sleep_wakeup_cause_t::ESP_SLEEP_WAKEUP_EXT1) {
      Serial.printf("[WAKE] Got Motion Wakeup\n");
      return wakeup_reason_t::MOTION;
   }
   if (wakeupReason == esp_sleep_wakeup_cause_t::ESP_SLEEP_WAKEUP_TIMER) {
      Serial.printf("[WAKE] Got Timer Wakeup \n");
      return wakeup_reason_t::TIMER;
   }
   Serial.printf("[WAKE] Got Button System Reset or Others\n");
   return wakeup_reason_t::SYSTEM_RESET;
}

bool usbCheckConnect() {
   Wire.beginTransmission(USBC_ADDR);
   Wire.write(0x11);
   Wire.endTransmission(true); // end write operation, as we just wanted to select the starting register

   Wire.requestFrom(USBC_ADDR, 1); // request 6 bytes from peripheral device #8
   char c = Wire.read();           // receive a byte as character
   char usbPower = (c >> 3) & 0x01;
   char usbPowerLow = (c >> 6) & 0x01;
   if (usbPower && !usbPowerLow) {
      return true;
   }
   else {
      return false;
   }
}

int accInit(bool skipInit) {
   int orientation = 0;
   if (!skipInit) {
      if (myIMU.begin(6.25, 2, false) == IMU_SUCCESS) {
         DEBUG_LOG("[ACC] initialized.\n");
      }
      else {
         Serial.println("[ACC] Failed to initialize.");
         return -1;
         // while (true);  // stop running sketch if failed
      }

      uint8_t readData = 0;

// Get the ID:
#if DEBUG
      if (myIMU.readRegister(&readData, KXTJ3_WHO_AM_I) ==
          IMU_SUCCESS) {
      }
      else {
         Serial.println("[ACC] Communication error, stopping.");
         return -1;
         // while (true);  // stop running sketch if failed
      }
#endif
   }
   myIMU.standby(false);

   uint8_t dataLowRes = 0;
   int acc_x = 0;
   int acc_y = 0;
   int acc_z = 0;

   if (myIMU.readRegister(&dataLowRes, KXTJ3_XOUT_H) ==
       IMU_SUCCESS) {
      // Read accelerometer data in mg as Float
      float acc_x_loc = myIMU.axisAccel(X);
      acc_x = round((int)(acc_x_loc * 10));
   }

   if (myIMU.readRegister(&dataLowRes, KXTJ3_YOUT_H) ==
       IMU_SUCCESS) {
      // Read accelerometer data in mg as Float
      float acc_y_loc = myIMU.axisAccel(Y);
      acc_y = round((int)(acc_y_loc * 10));
   }

   if (myIMU.readRegister(&dataLowRes, KXTJ3_ZOUT_H) ==
       IMU_SUCCESS) {
      // Read accelerometer data in mg as Float
      float acc_z_loc = myIMU.axisAccel(Z);
      acc_z = round((int)(acc_z_loc * 10));
   }

   if (acc_x > 5 && acc_y < 5 && acc_y > -5) {
      orientation = 0;
   }
   if (acc_x < 5 && acc_x > -5 && acc_y > 5) {
      orientation = 1;
   }
   if (acc_x < -5 && acc_y < 5 && acc_y > -5) {
      orientation = 2;
   }
   if (acc_x < 5 && acc_x > -5 && acc_y < -5) {
      orientation = 3;
   }

   if (!skipInit)
      DEBUG_LOG("[ACC] Values: X:%d Y:%d Z:%d Orient: %d \n", acc_x, acc_y, acc_z, orientation);

   // Put IMU back into standby
   myIMU.resetInterrupt();
   myIMU.standby(true);
   return orientation;
}

bool accIntSet(int sensity) {
   pinMode(INT_PIN, INPUT);
   if (sensity == 0) {
      myIMU.resetInterrupt();
      myIMU.standby(true);
      return true;
   }
   myIMU.resetInterrupt();
   delay(1);
   myIMU.intConf(sensity, 3, 10, HIGH, -1, true, false, true, false, true);
   delay(1);
   return true;
}

// Runs on the esp_timer task: only sample the accelerometer, never touch NVS or the display here.
void recheckAccOrient() {
   if (stopAccRecheck)
      return;
   int accCheck = accInit(true);
   if (accCheck >= 0)
      pendingOrientation = accCheck;
}

// Apply an orientation change seen by the background ticker. Call only from the main loop at points
// where no display update is running. Returns true if the orientation changed.
bool applyPendingOrientation() {
   int orient = pendingOrientation;
   if (orient < 0 || orient == systemData.deviceOrientation)
      return false;
   systemData.deviceOrientation = orient;
   Serial.printf("[ACC] Orientation changed to %d\n", orient);
   if (settingsStore.getInt("orientation", 0) != orient)
      settingsStore.putInt("orientation", orient);
   displaySetRotation((orient == 2 || orient == 3) ? 1 : 0);
   return true;
}

void checkOrientationInBackground(bool isRunning) {
   if (isRunning) {
      stopAccRecheck = false;
      pendingOrientation = -1;
      Serial.printf("[ACC] Background update start\n");

      periodicAccCheck.attach_ms(1000, recheckAccOrient);
   }
   else {
      Serial.printf("[ACC] Background update stop\n");
      stopAccRecheck = true;
      periodicAccCheck.detach();
      return;
   }

   return;
}

bool chargeMode(bool enable) {
   // charge pin high disables the charger (GND enabled)
   bool chargeState = false;
   bool isCharging = false;
   pinMode(CHG_STAT_PIN, INPUT);
   delay(1);
   if (enable) {
      pinMode(CHG_EN_PIN, INPUT);
      delay(2);
      chargeState = digitalRead(CHG_STAT_PIN);
      if (chargeState == LOW) {
         Serial.println("[CHARGE] on - Charging");
         isCharging = true;
      }
      else {
         Serial.println("[CHARGE] on - Charge Done");
         digitalWrite(CHG_EN_PIN, LOW);
      }
   }
   else {
      pinMode(CHG_EN_PIN, OUTPUT);
      digitalWrite(CHG_EN_PIN, HIGH);
      delay(2);
      chargeState = digitalRead(CHG_STAT_PIN);
      if (chargeState == LOW) {
         Serial.println("[CHARGE] off - Charging");
         isCharging = true;
      }
      else {
         Serial.println("[CHARGE] off - Charge Done");
         digitalWrite(CHG_EN_PIN, LOW);
      }
   }
   return isCharging;
}

bool resetAll(bool resetWifi) {
   checkOrientationInBackground(false);
   Serial.printf("[MAIN] Reset - WIFI %d \n", resetWifi);
   settingsStore.putInt("orientation", 0); // restore screen orient to default
   storeSleepTimeMem(DEFAULT_SLEEP); // restore sleep mem store to default
   openErasedFile(IMAGE_FILE);
   setImageValid(false);
   invalidateDownloadCache();
   settings.downloadUrl = "";
   settings.imageMode = 1;
   settings.timeout = DEFAULT_SLEEP;
   settings.httpAuthUser = "";
   settings.httpAuthPassword = "";
   if (resetWifi) {
      wifiSettings.ssid = "";
      wifiSettings.pss = "";
      settingsStoreClear();
   }
   saveSettings();
   return true;
}

void startupCounter(int reset) {
   preferences.begin("my-app", false);
   unsigned int counter = preferences.getUInt("counter", 0);
   counter++;
   if (reset || counter > 16) {
      if (StartCounter >= 5) {
         Serial.println("[MAIN] +5 button presses");
         StartCounter = 0;
         counter = 0;
         preferences.putUInt("counter", counter);
         preferences.end();
         resetAll(true);
         ESP.restart();
         return;
      }
      StartCounter = 0;
      counter = 0;
      Serial.println("[MAIN] Startup Counter RESET");
   }
   preferences.putUInt("counter", counter);
   preferences.end();
   StartCounter = counter;
   if (reset == 0)
      tickerStatupCounter.once_ms(3000, startupCounter, 1);
   return;
}

void runSetupMode() {
   Serial.println("[MAIN] Setup Mode Triggered");
   bleWorkerInit();

   unsigned long setupModeStart = millis();
   unsigned long bleConnectedTime = 0;
   bool bleWasConnected = false;
   bool initialWifiCheckDone = false;

   // Wait for completion, disconnect, or timeout
   while (true) {
      if (wifiScanRequested) {
         wifiScanRequested = false;
         Serial.println("[NETWORK] WiFi Scan requested via BLE, starting...");
         WiFi.mode(WIFI_STA);
         delay(1);
         int n = WiFi.scanNetworks();
         String wifiSsidScan = "";
         if (n == 0) {
            Serial.println("[NETWORK] no networks found");
         }
         else {
            Serial.printf("[NETWORK] %d networks found\n", n);
            for (int i = 0; i < n; ++i) {
               wifiSsidScan = wifiSsidScan + WiFi.SSID(i) + "´";
               wifiSsidScan = wifiSsidScan + WiFi.RSSI(i) + "´´";
               if (wifiSsidScan.length() > 460) {
                  wifiSsidScan = wifiSsidScan + "´...´...´";
                  break;
               }
            }
         }
         if (wifiScanCharacteristic) {
            wifiScanCharacteristic->setValue(wifiSsidScan.c_str());
            wifiScanCharacteristic->notify();
         }
         Serial.println("[NETWORK] Scan completed and notified.");
      }

      if (pServer->getConnectedCount() > 0) {
         if (!bleWasConnected) {
            bleWasConnected = true;
            bleConnectedTime = millis();
         }
         if (!initialWifiCheckDone && wifiSettings.ssid.length() > 0 && (millis() - bleConnectedTime > 8000)) {
            initialWifiCheckDone = true;
            wifiSettings.isDeployWifi = true;
         }
      }
      else {
         bleWasConnected = false;
      }

      if (wifiSettings.isDeployWifi) {
         wifiSettings.wifiIsConnected = false;
         wifiSettings.wifiOnboardingFailed = false;
         wifiSettings.isDeployWifi = false;

         if (wifiSettings.bleInitOk && wifiConnectedCharacteristic) {
            uint8_t val = 0;
            wifiConnectedCharacteristic->setValue(&val, 1);
            wifiConnectedCharacteristic->notify();

            if (wifiInfoCharacteristic) {
               wifiInfoCharacteristic->setValue(String("{}").c_str());
               wifiInfoCharacteristic->notify();
            }
         }

         Serial.println("[MAIN] Connecting WiFi...");
         WiFi.disconnect(true);
         delay(100);
         WiFi.mode(WIFI_STA);
         delay(100);

         if (connectWifi(10000)) {
            Serial.println("[WIFI] Connected successfully!");
            // check valid wifi connection
            settingsStore.putString("ssid", wifiSettings.ssid);
            settingsStore.putString("pss", wifiSettings.pss);
         }
         else {
            Serial.println("[WIFI] Connection timeout!");
         }
      }

      if (applyPending) {
         applyPending = false;
         bleImageApplied = true;
         Serial.println("[MAIN] Applying new image within setup mode...");
         spiLock();
         powerSupplyDisplay(true);
         if (settings.autoRotation) {
            accUpdateOrient();
         }
         setImageFromFS(IMAGE_FILE);
         spiUnlock();
      }

      if (forceExitSetup) {
         Serial.println("[MAIN] Manual exit requested via Web UI. Switching to fetch/refresh mode.");
         break;
      }

      if (!isBleClientConnected && !bleUploadActive && !fwUpdateInProgress) {
         if (millis() - lastDisconnectTime > SETUP_MODE_TIMEOUT * 1000) {
            Serial.printf("[MAIN] No BLE connection for %ds. Switching to fetch/refresh mode.\n", SETUP_MODE_TIMEOUT);
            break;
         }
      }
      else {
         lastDisconnectTime = millis(); // Reset timer while connected
      }

      static unsigned long lastTelemetryUpdate = 0;
      if (millis() - lastTelemetryUpdate > 10000) {
         lastTelemetryUpdate = millis();
         systemData.usbConnected = usbCheckConnect();
         systemData.vddValue = readVDD(false);
         bool isCharging = false;
         if (settings.chargerMode) {
            isCharging = chargeMode(settings.chargerMode);
         }

         JsonDocument sysDoc;
         sysDoc["voltage"] = systemData.vddValue;
         sysDoc["usb"] = systemData.usbConnected;
         sysDoc["charging"] = isCharging;
         String sysOut;
         serializeJson(sysDoc, sysOut);

         if (systemInfoCharacteristic) {
            systemInfoCharacteristic->setValue(sysOut.c_str());
            if (pServer->getConnectedCount() > 0) {
               systemInfoCharacteristic->notify();
            }
         }
      }

      delay(100);
   }
}

// Connect to the stored WiFi (no-op if already connected). Returns true when connected.
bool connectWifi(uint32_t timeoutMs) {
   if (WiFi.status() == WL_CONNECTED)
      return true;
   WiFi.begin(wifiSettings.ssid.c_str(), wifiSettings.pss.c_str());
   unsigned long startWait = millis();
   while (WiFi.status() != WL_CONNECTED && millis() - startWait < timeoutMs) {
      delay(100);
   }
   return WiFi.status() == WL_CONNECTED;
}

int processHttpDownload(String fileName) {
   int dlSuccess = 0;
   if (connectWifi()) {
      dlSuccess = loadImageFromWeb(settings.downloadUrl, DOWNLOAD_FILE);
      if (dlSuccess == DL_OK) {
         int imgResult = processImageFile(DOWNLOAD_FILE, downloadedLength, fileName.c_str());
         logStackHighWater("after image processing");
         if (imgResult == IMG_OK) {
            dlSuccess = 0;
            setImageValid(true);
         }
         else {
            // A rejected image never touched IMAGE_FILE, so the last image stays valid
            Serial.printf("[IMAGE] Failed to process image file (%s)\n", imgResult == IMG_REJECTED ? "rejected, last image kept" : "image file broken");
            if (imgResult == IMG_FAILED)
               setImageValid(false);
            dlSuccess = -1;
         }
      }
      WiFi.setSleep(true);
   }
   else {
      Serial.println("[DL] WiFi Connection Failed");
      dlSuccess = -1;
   }
   return dlSuccess;
}

void initFirstBoot(void) {
   // First Boot Check
   if (!settingsStore.getBool("initialized", false)) {
      Serial.println("[MEM] First boot detected, applying default settings.");
      settingsStoreClear();
      settings.timeout = 3600;
      settings.sleepDisabled = false;
      settings.imageMode = 1;
      settings.motionWakeup = false;
      settings.chargerMode = false;
      settings.settingsUrl = "";
      settings.settingsLastModified = "";
      settings.downloadUrl = "";
      settings.httpAuthUser = "";
      settings.httpAuthPassword = "";
      settings.lastModified = "";
      settings.downloadCrc = 0;

      storeSleepTimeMem(3600);
      saveSettings();
      settingsStore.putBool("initialized", true);
   }
}

void fetchRemoteSettings() {
   if (settings.settingsUrl.length() == 0)
      return;
   if (WiFi.status() != WL_CONNECTED)
      return;

   Serial.println("[HTTP] Fetching remote JSON settings...");
   HTTPClient http;
   http.begin(settings.settingsUrl);
   if (settings.settingsLastModified.length() > 0) {
      http.addHeader("If-Modified-Since", settings.settingsLastModified);
   }

   const char *headerKeys[] = {"Last-Modified"};
   http.collectHeaders(headerKeys, 1);

   int httpCode = http.GET();
   if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      JsonDocument doc;
      DeserializationError error = deserializeJson(doc, payload);
      if (!error) {
         bool changed = false;

         if (doc["timeout"].is<int>()) {
            settings.timeout = clampSleepTimeout(doc["timeout"].as<int>());
            changed = true;
         }
         if (doc["motionWakeup"].is<bool>()) {
            settings.motionWakeup = doc["motionWakeup"];
            changed = true;
         }
         if (doc["chargerMode"].is<bool>()) {
            settings.chargerMode = doc["chargerMode"];
            changed = true;
         }
         if (doc["autoRotation"].is<bool>()) {
            settings.autoRotation = doc["autoRotation"];
            changed = true;
         }
         if (doc["telemetry"].is<bool>()) {
            settings.telemetry = doc["telemetry"];
            changed = true;
         }
         if (doc["downloadUrl"].is<String>()) {
            String newUrl = stripUrlFragment(doc["downloadUrl"].as<String>());
            if (newUrl != settings.downloadUrl)
               invalidateDownloadCache();
            settings.downloadUrl = newUrl;
            changed = true;
            settings.imageMode = 1;
         }
         if (doc["httpAuthUser"].is<String>()) {
            settings.httpAuthUser = doc["httpAuthUser"].as<String>();
            changed = true;
         }
         if (doc["httpAuthPassword"].is<String>()) {
            settings.httpAuthPassword = doc["httpAuthPassword"].as<String>();
            changed = true;
         }

         if (http.hasHeader("Last-Modified")) {
            settings.settingsLastModified = http.header("Last-Modified");
            changed = true;
         }

         if (changed) {
            Serial.println("[HTTP] JSON settings applied and saved.");
            saveSettings();
         }
      }
      else {
         Serial.println("[HTTP] JSON parse failed");
      }
   }
   else if (httpCode == HTTP_CODE_NOT_MODIFIED) {
      Serial.println("[HTTP] JSON settings not modified.");
   }
   else {
      Serial.printf("[HTTP] GET failed, error: %s\n", http.errorToString(httpCode).c_str());
   }
   http.end();
}

void accUpdateOrient() {
   systemData.deviceOrientation = accInit();
   if (systemData.deviceOrientation == 2 || systemData.deviceOrientation == 3) {
      displaySetRotation(1);
   }
   else {
      displaySetRotation(0);
   }
   return;
}

void setup() {
   chargeMode(false); // enable charge mode
   powerSupplyDisplay(false);
   pinMode(BAT_VOLT_EN_PIN, OUTPUT);
#ifdef EPD_TYPE_13INCH
   digitalWrite(BAT_VOLT_EN_PIN, HIGH);
#else
   digitalWrite(BAT_VOLT_EN_PIN, LOW);
#endif
   Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
   pinMode(LED_PIN, OUTPUT);
   digitalWrite(LED_PIN, LOW);
   Serial.begin(115200);
   analogReadResolution(12);
   systemData.usbConnected = usbCheckConnect();
   delay(80);

   systemData.vddValue = readVDD(false);
   if (!systemData.usbConnected && systemData.vddValue < 4000) {
      DEBUG_LOG("[MAIN] Bat low protection: %dmV\n", systemData.vddValue);
      analogWrite(LED_PIN, LED_DIM_VALUE);
      gotToDeepSleep(86000, false);
   }
   systemData.wakeupCause = getWakeupReason();
   tickerFailsave.once_ms(FAILSAVE_TIMER * 1000, timeoutFailsafe, 0);

   if (systemData.wakeupCause == wakeup_reason_t::BUTTON) {
      buttonWake = true;
      startupCounter(false);
   }

   settingsStoreInit();
   pinMode(INT_PIN, INPUT);
   pinMode(BUSY_PIN, INPUT);
   pinMode(RST_PIN, OUTPUT);
   pinMode(CS_EPD_PIN, OUTPUT);
   pinMode(EPD_CS_S, OUTPUT);
#ifdef EPD_TYPE_13INCH
   pinMode(CS_SD_PIN, OUTPUT);
#endif
   pinMode(SCK_PIN, OUTPUT);
   pinMode(MOSI_PIN, OUTPUT);
   digitalWrite(CS_EPD_PIN, HIGH);
   digitalWrite(EPD_CS_S, HIGH);
#ifdef EPD_TYPE_13INCH
   digitalWrite(CS_SD_PIN, HIGH);
#endif

   gpio_set_drive_capability((gpio_num_t)SCK_PIN, GPIO_DRIVE_CAP_1);
   gpio_set_drive_capability((gpio_num_t)MOSI_PIN, GPIO_DRIVE_CAP_1);
   initFirstBoot();
   runMigrations();

   // Load credentials & settings from NVS
   restoreSettings();
   displaySetOverlayOption(DEVICE_INFO_STRING, true);
   displaySetRotation(0);
   char firmwareVersion[] = SOFTWARE_VERSION;

   ledBlink(500, true);

   Serial.printf("[MAIN] INIT Device V: %s\n", firmwareVersion);
   DEBUG_LOG("[MAIN] Current counter value: %u VDD: %d\n", StartCounter, systemData.vddValue);
   if (StartCounter >= 2) {
      settings.forceDownload = true;
      settings.skipSetupMode = true;
   }
   WiFi.onEvent(WiFiEvent);
   WiFi.mode(WIFI_STA);

   SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN); // SCK(), MISO(),MOSI(), SS()
   SPI.setFrequency(DISPLAY_SPI_SPEED);
   SerialFlash.begin(CS_FLASH_PIN, DISPLAY_SPI_SPEED); // proceed even if begin() fails
   initFlashFiles();
   powerSupplyDisplay(true);
   initEpaperDisplay(SPI);
   powerSupplyDisplay(false);
   chargeMode(settings.chargerMode);

   setDeviceUid();
   setDisplayData(CLIENT_ID, systemData.vddValue);
   // TODO: maybe do a function to generally check updated mem values
   float temperature = temperatureRead();
   DEBUG_LOG("[MAIN] Temp Main: %.2f °C\n", temperature);

   if (buttonWake && !settings.skipSetupMode) {
      tickerFailsave.detach();
      BleInit(CLIENT_ID, true);
      runSetupMode();

      Serial.println("[MAIN] Setup Mode Completed or Timeout");
      BleInit(CLIENT_ID, false); // Stop BLE advertising to save memory
      bleWorkerShutdown();
      // Always proceed to loop to fetch URL or refresh picture
      downloadStart = true;
      tickerFailsave.once_ms(FAILSAVE_TIMER * 1000, timeoutFailsafe, 0);
   }
   else {
      Serial.println("[MAIN] skip setup Mode");
   }

   ledBlink(500, true);
   saveSettings();
   storeSleepTimeMem(settings.timeout);

   systemData.sleepPrediction = calculateSleepDuration(settings.timeout, systemData.newSleepTimeSet, false);

   delay(10);
}

void loop() {

   if (downloadStart) {
      BleInit(CLIENT_ID, false);
      powerSupplyDisplay(true);
      downloadStart = false;
      if (settings.autoRotation) {
         accUpdateOrient();
         checkOrientationInBackground(true);
      }

      String fileName = IMAGE_FILE;
      int dlSuccess = 0;

      // Fallback: If in BLE mode but the file is missing and we have a download URL configured, switch to URL mode
      if (settings.imageMode == 0 && !settings.imageValid && settings.downloadUrl.length() > 0) {
         Serial.println("[IMAGE] BLE mode active but no valid uploaded image. Fallback to URL mode.");
         settings.imageMode = 1;
         saveSettings();
      }

      // Ensure WiFi if we need it for Settings OR Image
      if (settings.settingsUrl.length() > 0 || settings.imageMode == 1) {
         if (connectWifi()) {
            int prevTimeout = settings.timeout;
            bool prevChargerMode = settings.chargerMode;
            fetchRemoteSettings(); // This might change imageMode or downloadUrl!

            // Sleep time and charger mode were already applied in setup(), redo it so remote changes take effect this cycle
            if (settings.timeout != prevTimeout) {
               storeSleepTimeMem(settings.timeout);
               systemData.sleepPrediction = calculateSleepDuration(settings.timeout, true, false);
            }
            if (settings.chargerMode != prevChargerMode) {
               chargeMode(settings.chargerMode);
            }
         }
      }

      if (settings.imageMode == 1) { // URL Mode
         dlSuccess = processHttpDownload(fileName);
      }
      else { // BLE Mode (Mode 0)
         Serial.println("[IMAGE] Using BLE uncompressed tmp.bmp");
      }

      waitDisplayComplete(false);
      delay(25);

      int setSuccess = 0;
      bool dlFailed = false;
      if (dlSuccess == 0 || dlSuccess == 1) { // 0 = downloaded/ok, 1 = not modified
         if (dlFailCount > 0) {
            Serial.printf("[DL] Recovered after %d failed attempt(s)\n", dlFailCount);
            dlFailCount = 0;
         }
         if (dlSuccess == 0 && bleImageApplied) {
            Serial.println("[IMAGE] Image was already applied during setup mode, skipping refresh.");
            setSuccess = 0;
            dlErrorShown = false;
            commitDownloadCache();
         }
         else if (dlSuccess == 0 || settings.imageMode == 0 || dlErrorShown) {
            // dlErrorShown: redraw even if not modified, to remove the error badge / text
            WiFi.disconnect(true);
            applyPendingOrientation();
            setSuccess = setImageFromFS(fileName);
            // Device was rotated while drawing: redraw once with the new rotation
            if (setSuccess == 0 && applyPendingOrientation()) {
               waitDisplayComplete(false);
               setSuccess = setImageFromFS(fileName);
            }
            if (setSuccess == 0) {
               dlErrorShown = false;
               if (dlSuccess == 0)
                  commitDownloadCache();
            }
         }
         else {
            Serial.println("[IMAGE] Not rendering, image not modified");
            setSuccess = 0;
         }
      }
      else {
         // Keep the last image on the display and retry sooner (#15)
         dlFailed = true;
         if (dlFailCount < 255)
            dlFailCount++;
         Serial.printf("[DL] Download failed (%d), %d failure(s) in a row\n", dlSuccess, dlFailCount);
         WiFi.disconnect(true);
         applyPendingOrientation();
         if (!settings.imageValid) {
            // Nothing to fall back to (no image yet, or the failed processing overwrote it)
            if (!dlErrorShown) {
               displaySetText("Error: Picture download failed, please try again", false);
               dlErrorShown = true;
            }
         }
         else if (dlFailCount == DL_FAIL_BADGE_AFTER) {
            Serial.println("[IMAGE] Redrawing last image with error badge");
            displaySetErrorBadge(true);
            if (setImageFromFS(fileName) == 0)
               dlErrorShown = true;
            displaySetErrorBadge(false);
         }
         else {
            Serial.println("[IMAGE] Keeping last image");
         }
         setSuccess = -1;
      }

      logHeap();
      Serial.println("[MAIN] End of Update");
      if (settings.imageMode == 0) {
         gotToDeepSleep(0, false);
      }
      bool doMotionWake = (settings.imageMode == 1) ? settings.motionWakeup : false;
      int sleepSeconds = (settings.timeout > 0) ? systemData.sleepPrediction : DEFAULT_SLEEP;
      if (dlFailed && downloadRetryDelay(dlFailCount) < sleepSeconds) {
         sleepSeconds = downloadRetryDelay(dlFailCount);
         Serial.printf("[DL] Retrying in %d s\n", sleepSeconds);
      }
      gotToDeepSleep(sleepSeconds, doMotionWake);
   }

   // Keep alive for setup mode / BLE configs
   delay(500);
}