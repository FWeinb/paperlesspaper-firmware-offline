#pragma once
#include "types.h"
#include <Arduino.h>


#define BUSY_PIN 18
#define DC_PIN 19
#define EPD_CS_S 19
#define RST_PIN 1
#define CS_EPD_PIN 20

#ifdef EPD_TYPE_13INCH
#define FONT_MAIN u8g2_font_helvB24_tf // Font for main text
#define FONT_BIG u8g2_font_helvB18_tf  // Font for big text
#define FONT_NORMAL u8g2_font_helvB14_tf
#define FONT_SMALL u8g2_font_helvR12_tf
#define FONT_INFO u8g2_font_7x14_tf
#define FONT_VERSION u8g2_font_tom_thumb_4x6_tf
#else
#define FONT_MAIN u8g2_font_helvB24_tf // Font for main text
#define FONT_BIG u8g2_font_helvB14_tf  // Font for big text
#define FONT_NORMAL u8g2_font_helvB12_tf
#define FONT_SMALL u8g2_font_helvR08_tf
#define FONT_INFO u8g2_font_7x14_tf
#define FONT_VERSION u8g2_font_tom_thumb_4x6_tf
#endif

#include "Adafruit_GFX.h"
#include <GxEPD2_7C.h>
#include <SPI.h>
#include <U8g2_for_Adafruit_GFX.h>

#ifdef EPD_TYPE_13INCH
#define EPD_WIDTH 1600
#define EPD_HEIGHT 1200
#define SCREEN_OFFSET 200
#define EPD_VIEW_ROTATED 0 // viewed like the buffer (rotationText == rotationPicture)
using DisplayType = GxEPD2_7C<GxEPD2_1330c_EL133UF3, GxEPD2_1330c_EL133UF3::HEIGHT / 8>;
#else
#define EPD_WIDTH 800
#define EPD_HEIGHT 480
#define SCREEN_OFFSET 100
#define EPD_VIEW_ROTATED 1 // viewed in portrait: rotationText == rotationPicture + 1 (see displaySetRotation)
using DisplayType = GxEPD2_7C<GxEPD2_730c_GDEP073E01, GxEPD2_730c_GDEP073E01::HEIGHT / 4>;
#endif

extern DisplayType display;
extern U8G2_FOR_ADAFRUIT_GFX u8g2_for_adafruit_gfx;

void setDisplayData(const char *clientId, int vddValue);
void initEpaperDisplay(SPIClass &spiBus);
void displaySetRotation(int orientation);
void displaySetOverlayOption(DisplayInfoKey key, bool value);
void displaySetErrorBadge(bool enable);
void displayOverlays(DisplayType &display, DisplayInfo displayData, bool invertColors, bool fullcolor = false);
int setImageFromFS(String fileName, bool doRefresh = true);
void displaySetText(String info, bool isBlackboard);
bool waitDisplayComplete(bool quick);
