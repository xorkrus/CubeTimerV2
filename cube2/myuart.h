#ifndef MYUART_H
#define MYUART_H

#include <Arduino.h>
#include <EEPROM.h>
#include <TFT_eSPI.h>

#define FONT_COUNT 10

// Структуры конфигурации
struct FontConfig {
  int8_t baseOffsetX;
  int8_t baseOffsetY;
  float scale;        // 1.0 = оригинальный размер
};

struct SideFontExtra {
  int8_t extraOffsetX[6]; // для сторон 1-6 (индекс 0-5)
  int8_t extraOffsetY[6];
};

struct SideConfig {
  uint32_t timer;        // секунды
  uint8_t fontIndex;     // 0-9
  uint16_t textColor;    // RGB565
  uint16_t bgColor;      // RGB565
};

struct BatteryConfig {
  uint8_t fontIndex;
  float scale;
  int8_t offsetX[6];
  int8_t offsetY[6];
  uint16_t textColor[6];
};

struct MessageConfig {
  uint8_t fontIndex;
  float scale;
  int8_t offsetX;
  int8_t offsetY;
  uint16_t textColor;
  uint16_t bgColor;
};

struct GlobalConfig {
  float stableThreshold;     // м/с²
  uint8_t stableCountReq;
  float tapThreshold;        // м/с²
  uint32_t tapWindow;        // мс
  uint8_t confirmSeconds;    // длительность подтверждения (с)
  uint32_t shutdownSeconds;  // время показа OFF (с)
  uint8_t ledBrightness;     // 0-255
};

// Глобальные переменные конфигурации
extern FontConfig fonts[FONT_COUNT];
extern SideFontExtra fontExtras[FONT_COUNT];
extern SideConfig sides[6];
extern BatteryConfig battery;
extern MessageConfig message;
extern GlobalConfig global;

// Массив указателей на шрифты (заполняется в main.ino)
extern const GFXfont* fontList[FONT_COUNT];

// Флаги режимов UART
extern bool sensMode;
extern bool threshMode;

// Данные акселерометра (для режимов sens/thresh)
extern float ax, ay, az;
extern float processedAx, processedAy, processedAz;

// Функции
void loadConfig();
void saveConfig();
void resetConfig();
void handleUART();
void printConfig(int side = -1, int fontIdx = -1);
void printFontInfo(int fontIdx);

// Вспомогательные функции для цветов
uint16_t rgbToRGB565(uint32_t rgb);
uint32_t rgb565ToRGB(uint16_t color);

#endif