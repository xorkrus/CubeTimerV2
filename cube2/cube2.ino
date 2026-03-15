/**
 * Проект: Умный куб (таймер-куб) с аппаратным управлением питанием
 * Версия: 7.4 — добавлена UART-конфигурация, 10 шрифтов, сохранение в EEPROM
 * 
 * Аппаратура:
 * - Waveshare RP2040-Zero
 * - LSM6DS3 (акселерометр/гироскоп) I2C: SDA=GP0, SCL=GP1
 * - GC9A01A дисплей 240x240 SPI: CS=GP17, DC=GP20, RST=GP21, MOSI=GP19, SCK=GP18
 * - Встроенный NeoPixel: GP16
 * - Управление питанием (транзистор): GP15 (HIGH = удерживать питание)
 * - Пьезодинамик: GP27
 * - Измерение батареи: GP26 (ADC0) через постоянно включённый делитель
 * 
 * Библиотеки: TFT_eSPI (настроенный), Adafruit LSM6DS, Adafruit NeoPixel
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_LSM6DS.h>
#include <Adafruit_NeoPixel.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "myuart.h"

// Подключение пользовательских шрифтов (10 файлов)
#include "fonts/mf00.h"
#include "fonts/mf01.h"
#include "fonts/mf02.h"
#include "fonts/mf03.h"
#include "fonts/mf04.h"
#include "fonts/mf05.h"
#include "fonts/mf06.h"
#include "fonts/mf07.h"
#include "fonts/mf08.h"
#include "fonts/mf09.h"

// Массив указателей на шрифты
const GFXfont* fontList[FONT_COUNT] = {
  &mf00, &mf01, &mf02, &mf03, &mf04,
  &mf05, &mf06, &mf07, &mf08, &mf09
};

// Пины
#define PIN_I2C_SDA      0
#define PIN_I2C_SCL      1
#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL     16
#endif
#define PIN_HOLD         15
#define PIN_BUZZER       27
#define PIN_VBAT_ADC     26

// Адрес LSM6DS3
#define LSM6DS_ADDR      0x6B

// Глобальные объекты
Adafruit_LSM6DS lsm6ds;
Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite spr = TFT_eSprite(&tft);
bool useLibrary = false;                // флаг использования библиотеки LSM6DS
unsigned long lastSensorOutput = 0;    // для вывода данных в режимах sens/thresh

// Данные акселерометра
float ax, ay, az;
float processedAx, processedAy, processedAz;
float prevAx, prevAy, prevAz;
int stableCount = 0;
bool lastStableState = false;
int currentSide = 0;

// Состояния
enum State {
  STATE_IDLE,
  STATE_CONFIRMING,
  STATE_COUNTDOWN,
  STATE_SHUTDOWN
};
State currentState = STATE_IDLE;
unsigned long stateStartTime = 0;
int countdownValue = 0;
int confirmedSide = 0;

// Батарея
float batteryVoltage = 0.0;
int batteryPercent = 0;
unsigned long lastBatteryRead = 0;
const unsigned long BATTERY_READ_INTERVAL = 5000;

// Мелодия
const int melody[] = { 262, 330, 392, 523, 0 };
const int noteDuration[] = { 200, 200, 200, 400, 0 };
int melodyIndex = 0;
unsigned long nextNoteTime = 0;
bool melodyPlaying = false;

// Тапы
unsigned long lastTapTime = 0;
int tapCount = 0;
bool tapGestureDetected = false;
int tapGestureType = 0;   // 2 или 3

// Повороты для сторон (заданы жёстко)
const uint8_t rotationForBottom[7] = {0,1,3,2,0,0,2};

// Прототипы
void initSensor();
void updateSensor();
void applyInversion();
int  getSide(float ax, float ay, float az);
void setLed(int side, uint8_t brightness);
void drawTime(int seconds, int side);
void drawMessage(const char* msg);
void drawBattery(int side);
void updateBattery();
void shortBeep(int freq = 2000, int duration = 100);
void startMelody();
void stopMelody();
void updateMelody();
void detectTap(float rawAx, float rawAy, float rawAz);
void powerOff();
bool initLSM6DSDirect();
bool readAccelDirect(float &ax, float &ay, float &az);

// ---------- Управление питанием ----------
void setupPowerHold() {
  pinMode(PIN_HOLD, OUTPUT);
  digitalWrite(PIN_HOLD, HIGH);
}

void powerOff() {
  digitalWrite(PIN_HOLD, LOW);
  while (1) delay(100);
}

// ---------- Инициализация датчика (как в оригинале) ----------
void initSensor() {
  if (lsm6ds.begin_I2C(LSM6DS_ADDR, &Wire)) {
    lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
    useLibrary = true;
    Serial.println("LSM6DS инициализирован через библиотеку");
  } else {
    if (initLSM6DSDirect()) {
      useLibrary = false;
      Serial.println("LSM6DS инициализирован через прямой доступ");
    } else {
      Serial.println("КРИТИЧЕСКАЯ ОШИБКА: датчик не отвечает!");
      while (1) delay(100);
    }
  }
}

bool initLSM6DSDirect() {
  Wire.beginTransmission(LSM6DS_ADDR);
  Wire.write(0x10);
  Wire.write(0x60);
  return (Wire.endTransmission() == 0);
}

bool readAccelDirect(float &ax, float &ay, float &az) {
  Wire.beginTransmission(LSM6DS_ADDR);
  Wire.write(0x28);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom(LSM6DS_ADDR, 6);
  if (Wire.available() < 6) return false;
  uint8_t xl = Wire.read(), xh = Wire.read();
  uint8_t yl = Wire.read(), yh = Wire.read();
  uint8_t zl = Wire.read(), zh = Wire.read();
  int16_t rawX = (int16_t)((xh << 8) | xl);
  int16_t rawY = (int16_t)((yh << 8) | yl);
  int16_t rawZ = (int16_t)((zh << 8) | zl);
  const float SCALE = 0.000598f;
  ax = rawX * SCALE;
  ay = rawY * SCALE;
  az = rawZ * SCALE;
  return true;
}

void updateSensor() {
  if (useLibrary) {
    sensors_event_t accel, gyro, temp;
    if (lsm6ds.getEvent(&accel, &gyro, &temp)) {
      ax = accel.acceleration.x;
      ay = accel.acceleration.y;
      az = accel.acceleration.z;
    } else {
      useLibrary = false;
      initLSM6DSDirect();
    }
  }
  if (!useLibrary) {
    readAccelDirect(ax, ay, az);
  }
  applyInversion();
}

// Инверсия осей (можно настраивать через конфиг, но пока жёстко)
#define INVERT_X true
#define INVERT_Y true
#define INVERT_Z true

void applyInversion() {
  processedAx = INVERT_X ? -ax : ax;
  processedAy = INVERT_Y ? -ay : ay;
  processedAz = INVERT_Z ? -az : az;
}

// Определение стороны по инвертированным значениям
int getSide(float ax, float ay, float az) {
  float absX = fabs(ax);
  float absY = fabs(ay);
  float absZ = fabs(az);
  if (absX > absY && absX > absZ) return (ax > 0) ? 1 : 2;
  else if (absY > absX && absY > absZ) return (ay > 0) ? 3 : 4;
  else return (az > 0) ? 5 : 6;
}

// Светодиод
void setLed(int side, uint8_t brightness) {
  // Цвета для сторон (можно позже вынести в конфиг)
  const uint32_t ledColor[7] = {
    0,
    pixel.Color(255, 0, 0),
    pixel.Color(0, 255, 0),
    pixel.Color(0, 0, 255),
    pixel.Color(255, 255, 0),
    pixel.Color(255, 0, 255),
    pixel.Color(0, 255, 255)
  };
  pixel.setBrightness(brightness);
  if (side == 0) pixel.setPixelColor(0, 0);
  else pixel.setPixelColor(0, ledColor[side]);
  pixel.show();
}

// ---------- Батарея ----------
void updateBattery() {
  int raw = 0;
  for (int i = 0; i < 8; i++) raw += analogRead(PIN_VBAT_ADC);
  raw /= 8;
  float adcVoltage = raw * 3.3f / 1023.0f;
  const float R1 = 100000.0, R2 = 100000.0;
  batteryVoltage = adcVoltage * (R1 + R2) / R2;
  const float VBAT_MAX = 4.2, VBAT_MIN = 3.0;
  batteryPercent = (int)((batteryVoltage - VBAT_MIN) / (VBAT_MAX - VBAT_MIN) * 100);
  if (batteryPercent < 0) batteryPercent = 0;
  if (batteryPercent > 100) batteryPercent = 100;
}

// ---------- Отрисовка ----------
void drawTime(int seconds, int side) {
  int idx = side - 1;
  uint8_t fontIdx = sides[idx].fontIndex;
  const GFXfont* fnt = fontList[fontIdx];
  float scale = fonts[fontIdx].scale;
  int16_t offX = fonts[fontIdx].baseOffsetX + fontExtras[fontIdx].extraOffsetX[idx];
  int16_t offY = fonts[fontIdx].baseOffsetY + fontExtras[fontIdx].extraOffsetY[idx];
  uint16_t textColor = sides[idx].textColor;
  uint16_t bgColor = sides[idx].bgColor;

  tft.setRotation(rotationForBottom[side]);

  spr.fillScreen(bgColor);

  char buf[6];
  if (seconds >= 60) {
    int mins = seconds / 60;
    int secs = seconds % 60;
    sprintf(buf, "%02d:%02d", mins, secs);
  } else {
    sprintf(buf, "%d", seconds);
  }

  spr.setFreeFont(fnt);
  spr.setTextSize(scale);
  spr.setTextColor(textColor, bgColor);

  int16_t x = (240 - spr.textWidth(buf)) / 2 + offX;
  int16_t y = (240 - spr.fontHeight()) / 2 + offY;
  spr.setCursor(x, y);
  spr.print(buf);

  drawBattery(side);
  spr.pushSprite(0, 0);
}

void drawMessage(const char* msg) {
  uint8_t fontIdx = message.fontIndex;
  const GFXfont* fnt = fontList[fontIdx];
  float scale = message.scale;
  int16_t offX = message.offsetX;
  int16_t offY = message.offsetY;
  uint16_t textColor = message.textColor;
  uint16_t bgColor = message.bgColor;

  int rot = (currentSide != 0) ? rotationForBottom[currentSide] : 0;
  tft.setRotation(rot);

  spr.fillScreen(bgColor);

  spr.setFreeFont(fnt);
  spr.setTextSize(scale);
  spr.setTextColor(textColor, bgColor);

  int16_t x = (240 - spr.textWidth(msg)) / 2 + offX;
  int16_t y = (240 - spr.fontHeight()) / 2 + offY;
  spr.setCursor(x, y);
  spr.print(msg);

  drawBattery(currentSide ? currentSide : 1);
  spr.pushSprite(0, 0);
}

void drawBattery(int side) {
  int idx = side - 1;
  uint8_t fontIdx = battery.fontIndex;
  const GFXfont* fnt = fontList[fontIdx];
  float scale = battery.scale;
  int16_t offX = battery.offsetX[idx];
  int16_t offY = battery.offsetY[idx];
  uint16_t textColor = battery.textColor[idx];

  spr.setFreeFont(fnt);
  spr.setTextSize(scale);
  spr.setTextColor(textColor);

  String batStr = String(batteryPercent) + "%";
  int16_t x = (240 - spr.textWidth(batStr)) / 2 + offX;
  int16_t y = 240 - spr.fontHeight() - 4 + offY;
  spr.setCursor(x, y);
  spr.print(batStr);
}

// ---------- Звук ----------
void shortBeep(int freq, int duration) {
  tone(PIN_BUZZER, freq, duration);
}

void startMelody() {
  melodyPlaying = true;
  melodyIndex = 0;
  nextNoteTime = millis();
}

void stopMelody() {
  melodyPlaying = false;
  noTone(PIN_BUZZER);
}

void updateMelody() {
  if (!melodyPlaying) return;
  unsigned long now = millis();
  if (now >= nextNoteTime) {
    if (melodyIndex < sizeof(melody)/sizeof(melody[0]) && melody[melodyIndex] != 0) {
      tone(PIN_BUZZER, melody[melodyIndex], noteDuration[melodyIndex]);
      nextNoteTime = now + noteDuration[melodyIndex] + 30;
      melodyIndex++;
    } else {
      stopMelody();
    }
  }
}

// ---------- Детекция тапов ----------
void detectTap(float rawAx, float rawAy, float rawAz) {
  float magnitude = sqrt(rawAx*rawAx + rawAy*rawAy + rawAz*rawAz);
  unsigned long now = millis();

  if (magnitude > global.tapThreshold) {
    if (now - lastTapTime < global.tapWindow) {
      tapCount++;
    } else {
      tapCount = 1;
    }
    lastTapTime = now;

    if (tapCount == 2) {
      tapGestureDetected = true;
      tapGestureType = 2;
    } else if (tapCount == 3) {
      tapGestureDetected = true;
      tapGestureType = 3;
    }
  }

  if (now - lastTapTime > global.tapWindow) {
    tapCount = 0;
    tapGestureDetected = false;
  }
}

// ---------- SETUP ----------
void setup() {
  setupPowerHold();
  Serial.begin(115200);
  delay(2000);

  loadConfig();  // загружаем настройки из EEPROM

  pixel.begin();
  pixel.setBrightness(global.ledBrightness);
  setLed(0, global.ledBrightness);

  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);

  Wire.setSDA(PIN_I2C_SDA);
  Wire.setSCL(PIN_I2C_SCL);
  Wire.begin();

  initSensor();
  updateSensor();
  prevAx = processedAx; prevAy = processedAy; prevAz = processedAz;

  tft.init();
  tft.setRotation(0);
  spr.createSprite(240, 240);

  drawMessage("WAITING");

  updateBattery();
  lastBatteryRead = millis();

  Serial.println(F("System ready. Type 'help' for commands."));
}

// ---------- MAIN LOOP ----------
void loop() {
  unsigned long now = millis();

  // Режимы UART (sens / thresh)
  if (sensMode) {
    if (now - lastSensorOutput >= 1000) {
      lastSensorOutput = now;
      Serial.print(F("AX:")); Serial.print(processedAx);
      Serial.print(F(" AY:")); Serial.print(processedAy);
      Serial.print(F(" AZ:")); Serial.println(processedAz);
    }
    if (Serial.available()) {
      while(Serial.available()) Serial.read();
      sensMode = false;
      Serial.println(F("Exiting sens mode."));
    }
  }
  else if (threshMode) {
    if (now - lastSensorOutput >= 1000) {
      lastSensorOutput = now;
      float mag = sqrt(ax*ax + ay*ay + az*az);
      Serial.print(F("ST=")); Serial.print(global.stableThreshold);
      Serial.print(F(" TT=")); Serial.print(global.tapThreshold);
      Serial.print(F(" Cur:")); Serial.print(ax); Serial.print(F(",")); Serial.print(ay); Serial.print(F(",")); Serial.print(az);
      Serial.print(F(" mag=")); Serial.println(mag);
    }
    if (Serial.available()) {
      while(Serial.available()) Serial.read();
      threshMode = false;
      Serial.println(F("Exiting thresh mode."));
    }
  }
  else {
    handleUART();
  }

  // Обновление датчика
  updateSensor();
  detectTap(ax, ay, az);

  // Батарея
  if (now - lastBatteryRead > BATTERY_READ_INTERVAL) {
    updateBattery();
    lastBatteryRead = now;
    if (currentState == STATE_IDLE) {
      drawMessage("WAITING");
    }
  }

  // Стабильность
  float diffX = fabs(processedAx - prevAx);
  float diffY = fabs(processedAy - prevAy);
  float diffZ = fabs(processedAz - prevAz);
  bool currentlyStable = false;
  if (diffX < global.stableThreshold && diffY < global.stableThreshold && diffZ < global.stableThreshold) {
    stableCount++;
    if (stableCount >= global.stableCountReq) {
      currentlyStable = true;
      currentSide = getSide(processedAx, processedAy, processedAz);
    }
  } else {
    stableCount = 0;
    currentlyStable = false;
    currentSide = 0;
  }

  // Обработка двойного тапа (сброс)
  if (tapGestureDetected && tapGestureType == 2) {
    currentState = STATE_IDLE;
    confirmedSide = 0;
    setLed(0, global.ledBrightness);
    drawMessage("WAITING");
    tapGestureDetected = false;
    tapGestureType = 0;
    delay(200);
  }

  // Обработка одиночного тапа
  static unsigned long singleTapTime = 0;
  static bool singleTapPossible = false;
  if (tapCount == 1 && !singleTapPossible) {
    singleTapPossible = true;
    singleTapTime = lastTapTime;
  }
  if (singleTapPossible && (now - singleTapTime > global.tapWindow)) {
    if (currentlyStable) {
      if (currentState == STATE_IDLE || currentState == STATE_CONFIRMING || currentState == STATE_SHUTDOWN) {
        currentState = STATE_SHUTDOWN;
        stateStartTime = now;
        drawMessage("OFF");
        setLed(0, global.ledBrightness);
        shortBeep(1000, 500);
      }
    }
    singleTapPossible = false;
  }
  if (tapCount >= 2) {
    singleTapPossible = false;
  }

  // Конечный автомат
  switch (currentState) {
    case STATE_IDLE:
      if (currentlyStable && !lastStableState && currentSide != 0) {
        confirmedSide = currentSide;
        currentState = STATE_CONFIRMING;
        stateStartTime = now;
        countdownValue = global.confirmSeconds;
        drawTime(countdownValue, confirmedSide);
        setLed(confirmedSide, global.ledBrightness);
        shortBeep(2000, 100);
      }
      break;

    case STATE_CONFIRMING:
      if (!currentlyStable || currentSide != confirmedSide) {
        currentState = STATE_IDLE;
        setLed(0, global.ledBrightness);
        drawMessage("WAITING");
        break;
      }
      if (now - stateStartTime >= 1000) {
        stateStartTime = now;
        countdownValue--;
        if (countdownValue > 0) {
          drawTime(countdownValue, confirmedSide);
        } else {
          currentState = STATE_COUNTDOWN;
          countdownValue = sides[confirmedSide-1].timer;
          stateStartTime = now;
          drawTime(countdownValue, confirmedSide);
        }
      }
      break;

    case STATE_COUNTDOWN:
      if (now - stateStartTime >= 1000) {
        stateStartTime = now;
        countdownValue--;
        if (countdownValue > 0) {
          drawTime(countdownValue, confirmedSide);
        } else {
          currentState = STATE_SHUTDOWN;
          stateStartTime = now;
          drawMessage("OFF");
          setLed(0, global.ledBrightness);
          startMelody();
        }
      }
      break;

    case STATE_SHUTDOWN:
      if (now - stateStartTime >= global.shutdownSeconds * 1000UL) {
        powerOff();
      }
      break;
  }

  updateMelody();

  prevAx = processedAx;
  prevAy = processedAy;
  prevAz = processedAz;
  lastStableState = currentlyStable;

  delay(10);
}