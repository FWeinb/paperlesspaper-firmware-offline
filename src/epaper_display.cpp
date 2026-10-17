#include "epaper_display.h"
#include <Arduino.h>
#include <SerialFlash.h>
#include <WiFi.h>
#include <qrcode.h>

// Externs from main.cpp
extern SerialFlashFile saveFile;

DisplayInfo displayInfos = {
    .version = false,
    .batteryInfo = false,
    .batteryLowBig = false,
    .wifiSignal = false,
    .deviceInfoString = false,
    .wifiOfflineBig = false};

DisplaySettings displaySettings = {
    .rotationText = 3,
    .rotationPicture = 2,
    .quickRefresh = true,
    .globalQuickRefreshDisable = false,
#ifdef EPD_TYPE_13INCH
    .displayQuickRefreshTime = 2500,
    .displayQuickRefreshWipeTime = 2000,
    .colorWhiteFast = GxEPD_BLACK,
    .colorBlackFast = GxEPD_BLUE,
    .colorWipeFast = 1, // blue
#else
    .displayQuickRefreshTime = 960,
    .displayQuickRefreshWipeTime = 960,
    .colorWhiteFast = GxEPD_WHITE_I,
    .colorBlackFast = GxEPD_BLACK_I,
    .colorWipeFast = 1, // blue
#endif
};

QRCode QR;

bool epaperIsUpdating = false;
static bool displayIsInit = false;
static SPIClass *epd_spi_bus = nullptr;
static char epd_client_id[20] = {0};
static int epd_vdd_value = 0;

#ifdef EPD_TYPE_13INCH
DisplayType display(GxEPD2_1330c_EL133UF3(/*CS=*/CS_EPD_PIN, /*CS-S=*/EPD_CS_S, /*DC=*/-1, /*RST=*/RST_PIN, BUSY_PIN));
#else
DisplayType display(GxEPD2_730c_GDEP073E01(/*CS=*/CS_EPD_PIN, /*DC=*/DC_PIN, /*RST=*/RST_PIN, /*BUSY=*/BUSY_PIN));
#endif

U8G2_FOR_ADAFRUIT_GFX u8g2_for_adafruit_gfx;

void setDisplayData(const char *clientId, int vddValue) {
   if (clientId) {
      strncpy(epd_client_id, clientId, sizeof(epd_client_id) - 1);
      epd_client_id[sizeof(epd_client_id) - 1] = '\0';
   }
   epd_vdd_value = vddValue;
}

void initEpaperDisplay(SPIClass &spiBus) {
   epd_spi_bus = &spiBus;
   if (displayIsInit)
      return;
   pinMode(RST_PIN, OUTPUT);
   pinMode(CS_EPD_PIN, OUTPUT);
   pinMode(DC_PIN, OUTPUT);
   pinMode(EPD_CS_S, OUTPUT);
   pinMode(BUSY_PIN, INPUT);
   pinMode(DC_PIN, OUTPUT);
   digitalWrite(EPD_CS_S, HIGH);
   digitalWrite(CS_EPD_PIN, HIGH);
   display.epd2.selectSPI(spiBus, SPISettings(DISPLAY_SPI_SPEED, MSBFIRST, SPI_MODE0));
   u8g2_for_adafruit_gfx.begin(display);
   displayTypeDetect();
   displayIsInit = true;
}

bool isEpaperActive() {
   return epaperIsUpdating;
}

void deinitDisplay() {
   pinMode(RST_PIN, OUTPUT);
   digitalWrite(RST_PIN, 1);
   delay(50); // needs a little longer
   digitalWrite(RST_PIN, 0);
   delay(20);
   displayIsInit = false;
   display.hibernate();
   pinMode(RST_PIN, INPUT);
   pinMode(CS_EPD_PIN, INPUT);
   pinMode(EPD_CS_S, INPUT);
   pinMode(DC_PIN, INPUT);
}

void displayTypeDetect() {
   uint8_t reg9A[2] = {0};
   uint8_t patternDKE1[2] = {0x36, 0x42};
   uint8_t patternDKE2[2] = {0x36, 0x36};
   uint8_t patternOKRA1[2] = {0x31, 0xC2};
   uint8_t patternOKRA2[2] = {0x33, 0x00}; // Das zweite Byte wird bei OKRA 2 manchmal nicht gesendet, wir prüfen primär das erste

   display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
   display.init(115200);

   epd_spi_bus->endTransaction();

   // Register 0x9A auslesen
   epd_spi_bus->beginTransaction(SPISettings(DISPLAY_SPI_SPEED, MSBFIRST, SPI_MODE0));
   digitalWrite(DC_PIN, LOW);
   digitalWrite(CS_EPD_PIN, LOW);
   epd_spi_bus->transfer(0x9A);
   digitalWrite(DC_PIN, HIGH);

   for (int i = 0; i < 2; i++) {
      reg9A[i] = epd_spi_bus->transfer(0x00);
   }
   digitalWrite(CS_EPD_PIN, HIGH);
   epd_spi_bus->endTransaction();

#ifdef USE_QUICK_REFRESH

   // Pattern Matching
   if (memcmp(reg9A, patternDKE1, 2) == 0 || memcmp(reg9A, patternDKE2, 2) == 0) {
      Serial.println("[EPD] Match Found: DKE Display");
      displaySettings.displayQuickRefreshTime = 1500;
      displaySettings.displayQuickRefreshWipeTime = 500;
      displaySettings.colorWhiteFast = GxEPD_RED;
      displaySettings.colorBlackFast = GxEPD_BLUE;
      displaySettings.colorWipeFast = 1; // 3
   }
   else if (memcmp(reg9A, patternOKRA1, 2) == 0 || memcmp(reg9A, patternOKRA2, 2) == 0) {
      Serial.println("[EPD] Match Found: OKRA Display");
      displaySettings.displayQuickRefreshTime = 1500; // 1400-1700
      displaySettings.displayQuickRefreshWipeTime = 4000;
      displaySettings.colorWhiteFast = GxEPD_YELLOW;
      displaySettings.colorBlackFast = GxEPD_WHITE;
      displaySettings.colorWipeFast = 1; // blue
   }
   else {
      Serial.println("[EPD] No matching Type.");
      Serial.printf("[EPD] Register 0x9A Read (2 bytes): 0x%02X 0x%02X\n", reg9A[0], reg9A[1]);
      displaySettings.globalQuickRefreshDisable = true;
   }

#else
   Serial.printf("[EPD] Register 0x9A Read (2 bytes): 0x%02X 0x%02X\n", reg9A[0], reg9A[1]);
   displaySettings.globalQuickRefreshDisable = true;
   displaySettings.quickRefresh = false;

#endif
}

// Error badge (red square with "!") drawn into the top right corner of the image by setImageFromFS
static bool errorBadge = false;

void displaySetErrorBadge(bool enable) {
   errorBadge = enable;
}

void displaySetOverlayOption(DisplayInfoKey key, bool value) {
   switch (key) {
   case DisplayInfoKey::VERSION:
      displayInfos.version = value;
      break;
   case DisplayInfoKey::BATTERY_INFO:
      displayInfos.batteryInfo = value;
      break;
   case DisplayInfoKey::BATTERY_LOW_BIG:
      displayInfos.batteryLowBig = value;
      break;
   case DisplayInfoKey::WIFI_SIGNAL:
      displayInfos.wifiSignal = value;
      break;
   case DisplayInfoKey::DEVICE_INFO_STRING:
      displayInfos.deviceInfoString = value;
      break;
   case DisplayInfoKey::WIFI_OFFLINE_BIG:
      displayInfos.wifiOfflineBig = value;
      break;
   }
}

void displayOverlays(DisplayType &display, DisplayInfo displayData, bool invertColors, bool fullcolor) {
   int16_t tw = 0;
   int foreGround = GxEPD_WHITE_I;
   int backGround = GxEPD_BLACK_I;
   if (fullcolor) {
      foreGround = GxEPD_WHITE;
      backGround = GxEPD_BLACK;
   }
   if (invertColors) {
      // invert again
      int tempStore = foreGround;
      foreGround = backGround;
      backGround = tempStore;
   }
   char charBuffer[128];
   char charBuffer2[128];
   int pos = 0;

   if (displayData.deviceInfoString) {
      int deviceIdPos = 20;

      u8g2_for_adafruit_gfx.setForegroundColor(backGround); // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(foreGround); // apply Adafruit GFX color

      sprintf(charBuffer, "ID: %s", epd_client_id);
      String wifiSSID = WiFi.SSID();
      if (wifiSSID.length() > 1) {
         sprintf(charBuffer2, "%s WiFi: %s", charBuffer, wifiSSID.c_str());
         sprintf(charBuffer, "%s", charBuffer2);
      }
      else {
         sprintf(charBuffer2, "%s WiFi: NOT SET", charBuffer);
         sprintf(charBuffer, "%s", charBuffer2);
      }

      u8g2_for_adafruit_gfx.setFont(FONT_INFO);                                        // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer);                             // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, EPD_WIDTH - deviceIdPos); // start writing at this position
      u8g2_for_adafruit_gfx.print(charBuffer);
   }

   if (displayData.batteryLowBig) {
      int batteryLowPos = 500;

      u8g2_for_adafruit_gfx.setForegroundColor(foreGround); // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround); // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setFont(FONT_BIG);              // extended font
      u8g2_for_adafruit_gfx.setFontMode(1);                 // use u8g2 transparent mode (this is default)

      sprintf(charBuffer, "BATTERY LOW");
      int16_t tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer);                                  // text box width
      int16_t ta = u8g2_for_adafruit_gfx.getFontAscent();                                           // positive
      int16_t td = u8g2_for_adafruit_gfx.getFontDescent();                                          // negative; in mathematicians view
      int16_t th = ta - td;                                                                         // text box height
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, (EPD_WIDTH + batteryLowPos - th) / 2); // start writing at this position
      display.fillRect((EPD_HEIGHT - tw) / 2 - 2, (EPD_WIDTH + batteryLowPos - th) / 2 - 20, tw + 5, 25, backGround);
      u8g2_for_adafruit_gfx.print(charBuffer);
   }

   // Version Display bottom right
   if (displayData.version) {
      u8g2_for_adafruit_gfx.setFont(FONT_VERSION);          // extended font
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround); // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround); // apply Adafruit GFX color

      sprintf(charBuffer, "%s", SOFTWARE_VERSION);
#if DEBUG
      sprintf(charBuffer2, "DEV: %s", charBuffer);
      sprintf(charBuffer, "%s", charBuffer2);
#endif
      tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer); // text box width
      display.fillRect(EPD_HEIGHT - tw - 2 - pos, EPD_WIDTH - 7, tw + 3, 8, backGround);
      u8g2_for_adafruit_gfx.setCursor(EPD_HEIGHT - tw - 1 - pos, EPD_WIDTH - 1);
      u8g2_for_adafruit_gfx.print(charBuffer);
      pos = pos + 60;
   }
   if (displayData.batteryInfo) {
      u8g2_for_adafruit_gfx.setFont(FONT_VERSION);          // extended font
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround); // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround); // apply Adafruit GFX color
      int tempVdd = epd_vdd_value;

      sprintf(charBuffer, "Bat: %dV", tempVdd);

      tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer); // text box width
      display.fillRect(EPD_HEIGHT - tw - 2 - pos, EPD_WIDTH - 7, tw + 3, 8, backGround);
      u8g2_for_adafruit_gfx.setCursor(EPD_HEIGHT - tw - 1 - pos, EPD_WIDTH - 1);
      u8g2_for_adafruit_gfx.print(charBuffer);
      pos = pos + 60;
   }

   if (displayData.wifiSignal) {
      u8g2_for_adafruit_gfx.setFont(FONT_VERSION);          // extended font
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround); // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround); // apply Adafruit GFX color
      int wifiSignal = WiFi.RSSI();

      sprintf(charBuffer, "WiFi Sig: %d", wifiSignal);
      tw = u8g2_for_adafruit_gfx.getUTF8Width(charBuffer); // text box width
      display.fillRect(EPD_HEIGHT - tw - 2 - pos, EPD_WIDTH - 7, tw + 3, 8, backGround);
      u8g2_for_adafruit_gfx.setCursor(EPD_HEIGHT - tw - 1 - pos, EPD_WIDTH - 1);
      u8g2_for_adafruit_gfx.print(charBuffer);
      pos = pos + 60;
   }
}

// tmp.bmp holds the image in panel order: PANEL_ROWS rows of PANEL_COLS packed 4-bit pixels (high nibble =
// left pixel). The 13" panel is portrait in memory; each row is split across its two controllers.
#ifdef EPD_TYPE_13INCH
#define PANEL_COLS EPD_HEIGHT
#define PANEL_ROWS EPD_WIDTH
#else
#define PANEL_COLS EPD_WIDTH
#define PANEL_ROWS EPD_HEIGHT
#endif
#define PANEL_ROW_BYTES (PANEL_COLS / 2)

// Palette nibble as stored in tmp.bmp -> native color code (same on both panels)
static const uint8_t nativeColor[16] = {
    0x0, // 0 black
    0x5, // 1 blue
    0x6, // 2 green
    0x3, // 3 red
    0x1, // 4 unused -> white
    0x2, // 5 yellow
    0x1, // 6 white
    0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1, 0x1};

// The badge sits in the top right corner as the viewer sees it, i.e. in text coordinates (rotationText).
// Map panel pixel (px, py) to text coordinates (inverse of the GFX rotation).
static void panelToText(int px, int py, int &tx, int &ty) {
   switch (displaySettings.rotationText) {
   case 1:
      tx = py;
      ty = PANEL_COLS - 1 - px;
      break;
   case 2:
      tx = PANEL_COLS - 1 - px;
      ty = PANEL_ROWS - 1 - py;
      break;
   case 3:
      tx = PANEL_ROWS - 1 - py;
      ty = px;
      break;
   default:
      tx = px;
      ty = py;
      break;
   }
}

// Returns the palette nibble for text pixel (x, y): badge color inside the badge, else the image's nibble
static uint8_t errorBadgeNibble(int x, int y, int textWidth, uint8_t nibble) {
   const int size = EPD_WIDTH / 20;
   const int margin = size / 2;
   const int lx = x - (textWidth - margin - size);
   const int ly = y - margin;
   if (lx < 0 || ly < 0 || lx >= size || ly >= size)
      return nibble;
   const int border = max(2, size / 16);
   if (lx < border || ly < border || lx >= size - border || ly >= size - border)
      return 0; // black
   const int barHalfWidth = max(2, size / 12);
   if (lx >= size / 2 - barHalfWidth && lx < size / 2 + barHalfWidth) {
      if ((ly >= size * 20 / 100 && ly < size * 20 / 100 + size * 42 / 100) || (ly >= size * 72 / 100 && ly < size * 72 / 100 + size * 12 / 100))
         return 6; // white
   }
   return 3; // red
}

// Apply the badge to one panel row (py) of palette nibbles
static void errorBadgeRow(uint8_t *row, int py) {
   const int size = EPD_WIDTH / 20;
   const int margin = size / 2;
   const int textWidth = (displaySettings.rotationText & 1) ? PANEL_ROWS : PANEL_COLS;
   // Skip rows the badge can't touch: one text coordinate depends on py only
   int tx, ty;
   panelToText(0, py, tx, ty);
   if (displaySettings.rotationText & 1) {
      if (tx < textWidth - margin - size || tx >= textWidth - margin)
         return;
   }
   else if (ty < margin || ty >= margin + size)
      return;
   for (int b = 0; b < PANEL_ROW_BYTES; b++) {
      uint8_t nibbles[2] = {uint8_t(row[b] >> 4), uint8_t(row[b] & 0x0F)};
      for (int n = 0; n < 2; n++) {
         panelToText(b * 2 + n, py, tx, ty);
         nibbles[n] = errorBadgeNibble(tx, ty, textWidth, nibbles[n]);
      }
      row[b] = (nibbles[0] << 4) | nibbles[1];
   }
}

// Panel specific transfer of native rows. Callers hold the SPI bus (flash and display share it), and the
// display CS is released between chunks so the flash can be read in between.
#ifdef EPD_TYPE_13INCH
static void beginImageWrite() {
   display.init(115200);
   // The controller init is private in the driver; a tiny write runs it (the image overwrites the patch)
   static const uint8_t white[4] = {0x11, 0x11, 0x11, 0x11};
   display.epd2.writeImage(white, 0, 0, 4, 2);
}

// No DC line: every CS frame starts with a command, so each chunk gets its own window and DTM per controller
static void writeImageRows(const uint8_t *buf, int py0, int rows) {
   const uint16_t HRST = 0;
   const uint16_t HRED = PANEL_COLS - 1; // window units are half the controller width in pixels
   const uint16_t VRST = py0 / 2;
   const uint16_t VRED = (py0 + rows) / 2 - 1;
   const uint8_t window[9] = {uint8_t(HRST >> 8), uint8_t(HRST & 0xFF), uint8_t(HRED >> 8), uint8_t(HRED & 0xFF),
                              uint8_t(VRST >> 8), uint8_t(VRST & 0xFF), uint8_t(VRED >> 8), uint8_t(VRED & 0xFF),
                              0x01}; // PTLW_ENABLE
   const int halfBytes = PANEL_ROW_BYTES / 2;

   epd_spi_bus->beginTransaction(SPISettings(DISPLAY_SPI_SPEED, MSBFIRST, SPI_MODE0));
   for (int half = 0; half < 2; half++) {
      const int csPin = (half == 0) ? CS_EPD_PIN : EPD_CS_S;
      digitalWrite(csPin, LOW);
      epd_spi_bus->transfer(0x83); // PTLW (partial window)
      epd_spi_bus->writeBytes(window, sizeof(window));
      digitalWrite(csPin, HIGH);
      digitalWrite(csPin, LOW);
      epd_spi_bus->transfer(0x91); // PTIN (partial in)
      digitalWrite(csPin, HIGH);
      digitalWrite(csPin, LOW);
      epd_spi_bus->transfer(0x10); // DTM
      for (int r = 0; r < rows; r++)
         epd_spi_bus->writeBytes(buf + r * PANEL_ROW_BYTES + half * halfBytes, halfBytes);
      digitalWrite(csPin, HIGH);
   }
   epd_spi_bus->endTransaction();
}

static void endImageWrite() {
   // Disable the partial window on both controllers so the refresh covers the full screen
   const uint8_t noWindow[9] = {0};
   epd_spi_bus->beginTransaction(SPISettings(DISPLAY_SPI_SPEED, MSBFIRST, SPI_MODE0));
   for (int half = 0; half < 2; half++) {
      const int csPin = (half == 0) ? CS_EPD_PIN : EPD_CS_S;
      digitalWrite(csPin, LOW);
      epd_spi_bus->transfer(0x83);
      epd_spi_bus->writeBytes(noWindow, sizeof(noWindow));
      digitalWrite(csPin, HIGH);
   }
   epd_spi_bus->endTransaction();
}
#else
static void beginImageWrite() {
   display.init(115200);
   display.epd2.setPaged(); // runs the controller init if needed and sends DTM (0x10), pixel data follows
}

// Pixel data continues across CS toggles, so one DTM covers all chunks
static void writeImageRows(const uint8_t *buf, int py0, int rows) {
   epd_spi_bus->beginTransaction(SPISettings(DISPLAY_SPI_SPEED, MSBFIRST, SPI_MODE0));
   digitalWrite(DC_PIN, HIGH);
   digitalWrite(CS_EPD_PIN, LOW);
   epd_spi_bus->writeBytes(buf, rows * PANEL_ROW_BYTES);
   digitalWrite(CS_EPD_PIN, HIGH);
   epd_spi_bus->endTransaction();
}

static void endImageWrite() {
}
#endif

// Stream the image from flash to the controller in one pass (no GFX page buffer): read a chunk of rows,
// flip it if needed, draw the error badge, convert to native colors and send it.
int setImageFromFS(String fileName, bool doRefresh) {
   const unsigned long tStart = millis();
   epaperIsUpdating = true;
   saveFile = SerialFlash.open(fileName.c_str());
   if (!saveFile) {
      Serial.println("[BMP] File missing");
      epaperIsUpdating = false;
      return -1;
   }

   uint32_t offsetData = 0;

   // Check if the file is a standard BMP (starts with 'BM')
   uint8_t magic[2];
   if (saveFile.read(magic, 2) == 2) {
      if (magic[0] == 'B' && magic[1] == 'M') {
         // It's a standard BMP file. Read the pixel data offset at byte 0x0A
         saveFile.seek(0x0A);
         saveFile.read((uint8_t *)&offsetData, 4);
         Serial.printf("[BMP] Detected Windows BMP. Pixel Data starts at offset: %u\n", offsetData);
      }
      else {
         Serial.println("[BMP] Detected RAW payload (no BM magic). Reading from byte 0.");
      }
   }

   Serial.printf("[BMP] Loading Image H: %d W: %d\n", PANEL_ROWS, PANEL_COLS);

   // About 24 KB per chunk; an even row count keeps the 13" window rows aligned
   int rowsPerChunk = (24 * 1024 / PANEL_ROW_BYTES) & ~1;
   uint8_t *chunkBuffer = (uint8_t *)malloc(rowsPerChunk * PANEL_ROW_BYTES);
   if (!chunkBuffer) {
      rowsPerChunk = 2;
      chunkBuffer = (uint8_t *)malloc(rowsPerChunk * PANEL_ROW_BYTES);
      if (!chunkBuffer) {
         Serial.println("[BMP] Malloc for chunk buffer failed! Aborting.");
         saveFile.close();
         epaperIsUpdating = false;
         return -1;
      }
   }

   // Both pixels of a byte at once: palette byte -> native byte
   uint8_t nativeByte[256];
   for (int i = 0; i < 256; i++)
      nativeByte[i] = (nativeColor[i >> 4] << 4) | nativeColor[i & 0x0F];

   display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
   beginImageWrite();
   Serial.print("[EPD] Update Display... \n");

   const bool rotate180 = (displaySettings.rotationPicture > 0);

   for (int py0 = 0; py0 < PANEL_ROWS; py0 += rowsPerChunk) {
      const int rows = min(rowsPerChunk, PANEL_ROWS - py0);
      const int len = rows * PANEL_ROW_BYTES;
      // Panel rows [py0, py0 + rows) come from file rows [PANEL_ROWS - py0 - rows, PANEL_ROWS - py0) when flipped
      const int fileRow = rotate180 ? PANEL_ROWS - py0 - rows : py0;
      saveFile.seek(offsetData + uint32_t(fileRow) * PANEL_ROW_BYTES);
      int got = saveFile.read(chunkBuffer, len);
      if (got < len) {
         Serial.printf("[BMP] Short read at row %d (%d of %d bytes), padding white\n", fileRow, got, len);
         memset(chunkBuffer + max(got, 0), 0x66, len - max(got, 0));
      }

      if (rotate180) {
         // Reversing the whole chunk reverses the row order and the bytes in each row; swap the nibbles so
         // the pixels inside each byte are reversed too
         for (int i = 0, j = len - 1; i < j; i++, j--) {
            uint8_t t = chunkBuffer[i];
            chunkBuffer[i] = chunkBuffer[j];
            chunkBuffer[j] = t;
         }
         for (int i = 0; i < len; i++)
            chunkBuffer[i] = (chunkBuffer[i] << 4) | (chunkBuffer[i] >> 4);
      }

      if (errorBadge) {
         for (int r = 0; r < rows; r++)
            errorBadgeRow(chunkBuffer + r * PANEL_ROW_BYTES, py0 + r);
      }

      for (int i = 0; i < len; i++)
         chunkBuffer[i] = nativeByte[chunkBuffer[i]];

      writeImageRows(chunkBuffer, py0, rows);
   }

   free(chunkBuffer);
   saveFile.close();
   endImageWrite();

   const unsigned long tData = millis();
   Serial.printf("[EPD] Image data sent in %lu ms\n", tData - tStart);
   if (doRefresh) {
      display.epd2.refresh(false);
      display.epd2.powerOff();
      const unsigned long tEnd = millis();
      Serial.printf("[EPD] Refresh %lu ms, total %lu ms\n", tEnd - tData, tEnd - tStart);
   }

   epaperIsUpdating = false;
   return 0;
}

void displayWipe(bool quick) {
   if (quick) {
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
      display.init(115200);
   }
   else {
      display.init(115200);
   }

   display.clearScreen(0x01);
   display.refresh();
}

void displaySetText(String info, bool isBlackboard, bool quickRefresh) {
   int foreGround = GxEPD_BLACK_I;
   int backGround = GxEPD_WHITE_I;
   bool invert = false;

   if (!displaySettings.quickRefresh) {
      if (isBlackboard) {
         invert = true;
         foreGround = GxEPD_WHITE;
         backGround = GxEPD_BLACK;
      }
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
      display.init(115200);
   }
   else {
      if (isBlackboard) {
         invert = true;
         foreGround = GxEPD_WHITE_I;
         backGround = GxEPD_BLACK_I;
      }
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, false);
      display.init(115200);
   }

   display.setRotation(displaySettings.rotationText);
   display.firstPage();
   do {
      display.fillScreen(backGround);
      u8g2_for_adafruit_gfx.setFontDirection(0);            // left to right (this is default)
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround); // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround); // apply Adafruit GFX color

      u8g2_for_adafruit_gfx.setFont(FONT_BIG); // extended font
      u8g2_for_adafruit_gfx.setFontMode(1);    // use u8g2 transparent mode (this is default)

      int16_t tw = u8g2_for_adafruit_gfx.getUTF8Width(info.c_str());                // text box width
      int16_t ta = u8g2_for_adafruit_gfx.getFontAscent();                           // positive
      int16_t td = u8g2_for_adafruit_gfx.getFontDescent();                          // negative; in mathematicians view
      int16_t th = ta - td;                                                         // text box height
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, (EPD_WIDTH - th) / 2); // start writing at this position

      u8g2_for_adafruit_gfx.print(info);
      if (info.length() > 1) {
         displayInfos.deviceInfoString = true;
      }

      displayOverlays(display, displayInfos, invert, false);
   }
   while (display.nextPage());
}

bool waitDisplayComplete(bool quick) {
   int counter = 0;
   while (counter < 20) {
      counter++;
      if (quick)
         counter = 21;
      bool pinState = digitalRead(BUSY_PIN);
      if (pinState) {
         return true;
         break;
      }
      DEBUG_LOG("[EPD] Wait for Display...\n");
      delay(500);
   }
   return false;
}

void displayTurnOn() {
   String info = "Ich schlafe ...";
   String info2 = "Drücke die Taste auf der Rückseite";
   String info3 = "um mich zu wecken.";

   char msg[128];
   sprintf(msg, "%s%s%s", "https://paperlesspaper.de/b?d=", epd_client_id, "&w=99");

   uint8_t QRData[qrcode_getBufferSize(QR_VERSION)];
   uint8_t blockSize;
   uint8_t page = 0;
   qrcode_initText(&QR, QRData, QR_VERSION, ECC_LOW, msg);
   blockSize = 2;
   uint16_t x0 = (EPD_HEIGHT - 30 * blockSize) / 2;
   uint16_t y0 = 680;

   int foreGround = GxEPD_WHITE_I;
   int backGround = GxEPD_BLACK_I;
   bool fullColor = false;

   Serial.print(F("\n[EPD] Press to turn on Screen Loading - "));
   if (displaySettings.quickRefresh) {
      display.init(115200);
      display.enableQuickRefresh(displaySettings.displayQuickRefreshTime, true);
   }
   else {
      display.enableQuickRefresh(0, false);
      display.init(115200);
      foreGround = GxEPD_WHITE;
      backGround = GxEPD_BLACK;
      fullColor = true;
   }
   display.setRotation(displaySettings.rotationText);
   display.firstPage();
   // Display 600*448
   do {
      int16_t tw = 0;
      display.fillScreen(backGround);
      u8g2_for_adafruit_gfx.setForegroundColor(foreGround); // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setBackgroundColor(backGround); // apply Adafruit GFX color
      u8g2_for_adafruit_gfx.setFontDirection(0);            // left to right (this is default)
      u8g2_for_adafruit_gfx.setFontMode(1);                 // use u8g2 transparent mode (this is default)

      display.fillRect(x0 + 2, y0 + 2, QR.size * blockSize + QR_QUIET_ZONE + blockSize - 2, QR.size * blockSize + QR_QUIET_ZONE + blockSize - 2, foreGround);

      // For each vertical module
      for (uint8_t y = 0; y < QR.size; y++) {
         // For each horizontal module
         for (uint8_t x = 0; x < QR.size; x++) {
            if (qrcode_getModule(&QR, x, y))
               display.fillRect(x0 + (x * blockSize) + QR_QUIET_ZONE,
                                y0 + (y * blockSize) + QR_QUIET_ZONE,
                                blockSize, blockSize,
                                (qrcode_getModule(&QR, x, y)) ? backGround : foreGround);
         }
      }

      u8g2_for_adafruit_gfx.setFont(FONT_MAIN); // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info.c_str());
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 300); // start writing at this position
      u8g2_for_adafruit_gfx.print(info);

      u8g2_for_adafruit_gfx.setFont(FONT_BIG);                     // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width(info2.c_str());      // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 370); // start writing at this position
      u8g2_for_adafruit_gfx.print(info2);                          // UTF-8 string: "<" 550 448 664 ">"

      tw = u8g2_for_adafruit_gfx.getUTF8Width(info3.c_str());      // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 395); // start writing at this position
      u8g2_for_adafruit_gfx.print(info3);

      u8g2_for_adafruit_gfx.setFont(FONT_NORMAL);                  // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("I am sleeping..."); // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 560); // start writing at this position
      u8g2_for_adafruit_gfx.print("I am sleeping...");

      u8g2_for_adafruit_gfx.setFont(FONT_SMALL);                                              // extended font
      tw = u8g2_for_adafruit_gfx.getUTF8Width("Press the button on the back to wake me up."); // text box width
      u8g2_for_adafruit_gfx.setCursor((EPD_HEIGHT - tw) / 2, 590);                            // start writing at this position
      u8g2_for_adafruit_gfx.print("Press the button on the back to wake me up.");

      displayOverlays(display, displayInfos, true, fullColor);
   }
   while (display.nextPage());
}

void displaySetRotation(int orientation) {
   switch (orientation) {
   case 0:
#ifdef EPD_TYPE_13INCH
      displaySettings.rotationText = 0;
      displaySettings.rotationPicture = 0;
#else
      displaySettings.rotationText = 3;
      displaySettings.rotationPicture = 2;
#endif
      break;
   case 1:
#ifdef EPD_TYPE_13INCH
      displaySettings.rotationText = 2;
      displaySettings.rotationPicture = 2;
#else
      displaySettings.rotationText = 1;
      displaySettings.rotationPicture = 0;
#endif
      break;
   default:
#ifdef EPD_TYPE_13INCH
      displaySettings.rotationText = 0;
      displaySettings.rotationPicture = 0;
#else
      displaySettings.rotationText = 3;
      displaySettings.rotationPicture = 2;
#endif
   }
}

void displaySetQuickRefresh(bool enable, int refreshTime, int wipeTime) {
   if (displaySettings.globalQuickRefreshDisable) {
      displaySettings.quickRefresh = false;
      return;
   }
   if (enable) {
      displaySettings.quickRefresh = true;
   }
   else {
      displaySettings.quickRefresh = false;
   }
   if (refreshTime > 0) {
      displaySettings.displayQuickRefreshTime = refreshTime;
   }
   if (wipeTime > 0) {
      displaySettings.displayQuickRefreshWipeTime = wipeTime;
   }
}
