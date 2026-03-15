#include "myuart.h"
#include <Arduino.h>

// Глобальные переменные
FontConfig fonts[FONT_COUNT];
SideFontExtra fontExtras[FONT_COUNT];
SideConfig sides[6];
BatteryConfig battery;
MessageConfig message;
GlobalConfig global;

//const GFXfont* fontList[FONT_COUNT];  // заполняется в main.ino

bool sensMode = false;
bool threshMode = false;

// Данные акселерометра (будут определены в main.ino)
extern float ax, ay, az;
extern float processedAx, processedAy, processedAz;

#define CONFIG_VERSION 0x0100
#define EEPROM_SIZE 2048

struct EEPROMData {
  uint16_t version;
  FontConfig fonts[FONT_COUNT];
  SideFontExtra fontExtras[FONT_COUNT];
  SideConfig sides[6];
  BatteryConfig battery;
  MessageConfig message;
  GlobalConfig global;
  uint16_t checksum;
};

static EEPROMData eepromData;

static uint16_t calculateChecksum(const EEPROMData& data) {
  uint16_t sum = 0;
  const uint8_t* ptr = (const uint8_t*)&data;
  for (size_t i = 0; i < sizeof(EEPROMData) - sizeof(uint16_t); i++) {
    sum += ptr[i];
  }
  return sum;
}

void loadConfig() {
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.get(0, eepromData);
  if (eepromData.version != CONFIG_VERSION || calculateChecksum(eepromData) != eepromData.checksum) {
    resetConfig();
  } else {
    memcpy(fonts, eepromData.fonts, sizeof(fonts));
    memcpy(fontExtras, eepromData.fontExtras, sizeof(fontExtras));
    memcpy(sides, eepromData.sides, sizeof(sides));
    memcpy(&battery, &eepromData.battery, sizeof(battery));
    memcpy(&message, &eepromData.message, sizeof(message));
    memcpy(&global, &eepromData.global, sizeof(global));
  }
  EEPROM.end();
}

void saveConfig() {
  memcpy(eepromData.fonts, fonts, sizeof(fonts));
  memcpy(eepromData.fontExtras, fontExtras, sizeof(fontExtras));
  memcpy(eepromData.sides, sides, sizeof(sides));
  memcpy(&eepromData.battery, &battery, sizeof(battery));
  memcpy(&eepromData.message, &message, sizeof(message));
  memcpy(&eepromData.global, &global, sizeof(global));
  eepromData.version = CONFIG_VERSION;
  eepromData.checksum = calculateChecksum(eepromData);

  EEPROM.begin(EEPROM_SIZE);
  EEPROM.put(0, eepromData);
  EEPROM.commit();
  EEPROM.end();
}

void resetConfig() {
  for (int i = 0; i < FONT_COUNT; i++) {
    fonts[i].baseOffsetX = 0;
    fonts[i].baseOffsetY = 0;
    fonts[i].scale = 1.0f;
    for (int s = 0; s < 6; s++) {
      fontExtras[i].extraOffsetX[s] = 0;
      fontExtras[i].extraOffsetY[s] = 0;
    }
  }

  uint32_t defTimers[6] = {45,50,55,65,70,75};
  for (int s = 0; s < 6; s++) {
    sides[s].timer = defTimers[s];
    sides[s].fontIndex = s % FONT_COUNT;
    sides[s].textColor = TFT_WHITE;
    sides[s].bgColor = TFT_BLACK;
  }

  battery.fontIndex = 0;
  battery.scale = 1.0f;
  for (int s = 0; s < 6; s++) {
    battery.offsetX[s] = 0;
    battery.offsetY[s] = 0;
    battery.textColor[s] = TFT_WHITE;
  }

  message.fontIndex = 0;
  message.scale = 1.0f;
  message.offsetX = 0;
  message.offsetY = 0;
  message.textColor = TFT_WHITE;
  message.bgColor = TFT_BLACK;

  global.stableThreshold = 0.4f;
  global.stableCountReq = 10;
  global.tapThreshold = 2.5f * 9.81f;
  global.tapWindow = 500;
  global.confirmSeconds = 3;
  global.shutdownSeconds = 10;
  global.ledBrightness = 50;
}

uint16_t rgbToRGB565(uint32_t rgb) {
  uint8_t r = (rgb >> 16) & 0xFF;
  uint8_t g = (rgb >> 8) & 0xFF;
  uint8_t b = rgb & 0xFF;
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

uint32_t rgb565ToRGB(uint16_t color) {
  uint8_t r = (color >> 8) & 0xF8;
  uint8_t g = (color >> 3) & 0xFC;
  uint8_t b = (color << 3) & 0xF8;
  return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

// ---------- Парсер команд ----------
static void printHelp();
static void parseSet(String args);
static void parseSetTimer(String args);
static void parseSetBat(String args);
static void parseSetMessage(String args);
static void parseSetSn(String args);
static void parseSetGlobal(String args);

void handleUART() {
  static String inputLine = "";
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      inputLine.trim();
      if (inputLine.length() > 0) {
        int idx = inputLine.indexOf(' ');
        String cmd = (idx == -1) ? inputLine : inputLine.substring(0, idx);
        String args = (idx == -1) ? "" : inputLine.substring(idx + 1);
        args.trim();

        if (cmd.equalsIgnoreCase("help")) {
          printHelp();
        }
        else if (cmd.equalsIgnoreCase("ver")) {
          Serial.print(F("Version: 7.4 ("));
          Serial.print(__DATE__);
          Serial.print(F(" "));
          Serial.print(__TIME__);
          Serial.println(F(")"));
        }
        else if (cmd.equalsIgnoreCase("reboot")) {
          Serial.println(F("Rebooting..."));
          delay(100);
          rp2040.reboot();
        }
        else if (cmd.equalsIgnoreCase("save")) {
          saveConfig();
          Serial.println(F("Configuration saved."));
        }
        else if (cmd.equalsIgnoreCase("list")) {
          if (args.length() > 0) {
            int n = args.toInt();
            printFontInfo(n);
          } else {
            printConfig();
          }
        }
        else if (cmd.equalsIgnoreCase("sens")) {
          sensMode = true;
          Serial.println(F("Entering sens mode. Send any character to exit."));
        }
        else if (cmd.equalsIgnoreCase("thresh")) {
          threshMode = true;
          Serial.println(F("Entering thresh mode. Send any character to exit."));
        }
        else if (cmd.equalsIgnoreCase("set")) {
          parseSet(args);
        }
        else {
          Serial.print(F("Unknown command: "));
          Serial.println(cmd);
        }
      }
      inputLine = "";
    } else {
      inputLine += c;
    }
  }
}

static void printHelp() {
  Serial.println(F("Доступные команды:"));
  Serial.println(F("  list - вывод всех настроек"));
  Serial.println(F("  list <n> - вывод информации о шрифте n"));
  Serial.println(F("  set timer sfs <side> <fontIdx> - выбор шрифта для стороны"));
  Serial.println(F("  set timer fc <fontIdx> <x>:<y> - базовая коррекция шрифта"));
  Serial.println(F("  set timer fcs <side> <fontIdx> <x>:<y> - доп. коррекция шрифта на стороне"));
  Serial.println(F("  set timer fs <fontIdx> <value|+d|-d> - установка масштаба (1.0 = 100%)"));
  Serial.println(F("  set timer tm <side> <seconds> - установка таймера (сек)"));
  Serial.println(F("  set timer sc f <side> <RRGGBB> - цвет текста стороны"));
  Serial.println(F("  set timer sc b <side> <RRGGBB> - цвет фона стороны"));
  Serial.println(F("  set bat fc <x>:<y> - базовая коррекция для сообщений (для bat не используется)"));
  Serial.println(F("  set bat fcs <side> <x>:<y> - коррекция батареи на стороне"));
  Serial.println(F("  set bat fs <value|+d|-d> - масштаб шрифта батареи"));
  Serial.println(F("  set bat sc f <side> <RRGGBB> - цвет текста батареи на стороне"));
  Serial.println(F("  set bat font <fontIdx> - выбор шрифта батареи"));
  Serial.println(F("  set message fc <x>:<y> - коррекция сообщений"));
  Serial.println(F("  set message fs <value|+d|-d> - масштаб сообщений"));
  Serial.println(F("  set message sc f <RRGGBB> - цвет текста сообщений"));
  Serial.println(F("  set message sc b <RRGGBB> - цвет фона сообщений"));
  Serial.println(F("  set message font <fontIdx> - выбор шрифта сообщений"));
  Serial.println(F("  set sn f <value> - чувствительность фиксации (м/с²)"));
  Serial.println(F("  set sn t <value> - чувствительность тапа (м/с²)"));
  Serial.println(F("  set global confirm <sec> - длительность подтверждения (с)"));
  Serial.println(F("  set global shutdown <sec> - время показа OFF (с)"));
  Serial.println(F("  set global led <0-255> - яркость светодиода"));
  Serial.println(F("  sens - вывод данных акселерометра (любой символ для выхода)"));
  Serial.println(F("  thresh - вывод порогов и данных (любой символ для выхода)"));
  Serial.println(F("  save - сохранение в EEPROM"));
  Serial.println(F("  ver - версия прошивки"));
  Serial.println(F("  reboot - перезагрузка"));
  Serial.println(F("  help - эта справка"));
}

static void parseSet(String args) {
  int idx = args.indexOf(' ');
  if (idx == -1) {
    Serial.println(F("set: missing subcommand"));
    return;
  }
  String sub = args.substring(0, idx);
  sub.toLowerCase();
  String subArgs = args.substring(idx + 1);
  subArgs.trim();

  if (sub == "timer") {
    parseSetTimer(subArgs);
  } else if (sub == "bat") {
    parseSetBat(subArgs);
  } else if (sub == "message") {
    parseSetMessage(subArgs);
  } else if (sub == "sn") {
    parseSetSn(subArgs);
  } else if (sub == "global") {
    parseSetGlobal(subArgs);
  } else {
    Serial.print(F("Unknown set category: "));
    Serial.println(sub);
  }
}

static void parseSetTimer(String args) {
  int idx = args.indexOf(' ');
  if (idx == -1) {
    Serial.println(F("timer: missing command"));
    return;
  }
  String cmd = args.substring(0, idx);
  cmd.toLowerCase();
  String params = args.substring(idx + 1);
  params.trim();

  if (cmd == "sfs") {
    int side, fontIdx;
    if (sscanf(params.c_str(), "%d %d", &side, &fontIdx) == 2) {
      if (side >=1 && side <=6 && fontIdx >=0 && fontIdx < FONT_COUNT) {
        sides[side-1].fontIndex = fontIdx;
        Serial.print(F("Side ")); Serial.print(side); Serial.print(F(" font set to ")); Serial.println(fontIdx);
      } else {
        Serial.println(F("Invalid side or font index"));
      }
    } else {
      Serial.println(F("Usage: sfs <side> <fontIdx>"));
    }
  }
  else if (cmd == "fc") {
    int fontIdx, x, y;
    if (sscanf(params.c_str(), "%d %d:%d", &fontIdx, &x, &y) == 3) {
      if (fontIdx >=0 && fontIdx < FONT_COUNT) {
        fonts[fontIdx].baseOffsetX = x;
        fonts[fontIdx].baseOffsetY = y;
        Serial.print(F("Font ")); Serial.print(fontIdx); Serial.print(F(" base offset set to ")); Serial.print(x); Serial.print(F(",")); Serial.println(y);
      } else {
        Serial.println(F("Invalid font index"));
      }
    } else {
      Serial.println(F("Usage: fc <fontIdx> <x>:<y>"));
    }
  }
  else if (cmd == "fcs") {
    int side, fontIdx, x, y;
    if (sscanf(params.c_str(), "%d %d %d:%d", &side, &fontIdx, &x, &y) == 4) {
      if (side >=1 && side <=6 && fontIdx >=0 && fontIdx < FONT_COUNT) {
        fontExtras[fontIdx].extraOffsetX[side-1] = x;
        fontExtras[fontIdx].extraOffsetY[side-1] = y;
        Serial.print(F("Font ")); Serial.print(fontIdx); Serial.print(F(" extra offset for side ")); Serial.print(side); Serial.print(F(" set to ")); Serial.print(x); Serial.print(F(",")); Serial.println(y);
      } else {
        Serial.println(F("Invalid side or font index"));
      }
    } else {
      Serial.println(F("Usage: fcs <side> <fontIdx> <x>:<y>"));
    }
  }
  else if (cmd == "fs") {
    int fontIdx;
    char mod[10];
    if (sscanf(params.c_str(), "%d %s", &fontIdx, mod) == 2) {
      if (fontIdx >=0 && fontIdx < FONT_COUNT) {
        float newScale = fonts[fontIdx].scale;
        if (mod[0] == '+' || mod[0] == '-') {
          float delta = atof(mod+1);
          newScale += (mod[0]=='+' ? delta : -delta);
        } else {
          newScale = atof(mod);
        }
        if (newScale < 0.1f) newScale = 0.1f;
        if (newScale > 10.0f) newScale = 10.0f;
        fonts[fontIdx].scale = newScale;
        Serial.print(F("Font ")); Serial.print(fontIdx); Serial.print(F(" scale set to ")); Serial.println(newScale, 2);
      } else {
        Serial.println(F("Invalid font index"));
      }
    } else {
      Serial.println(F("Usage: fs <fontIdx> <value|+d|-d>"));
    }
  }
  else if (cmd == "tm") {
    int side, sec;
    if (sscanf(params.c_str(), "%d %d", &side, &sec) == 2) {
      if (side >=1 && side <=6) {
        sides[side-1].timer = sec;
        Serial.print(F("Side ")); Serial.print(side); Serial.print(F(" timer set to ")); Serial.print(sec); Serial.println(F(" s"));
      } else {
        Serial.println(F("Invalid side"));
      }
    } else {
      Serial.println(F("Usage: tm <side> <seconds>"));
    }
  }
  else if (cmd == "sc") {
    char type;
    int side;
    uint32_t rgb;
    if (sscanf(params.c_str(), "%c %d %x", &type, &side, &rgb) == 3) {
      if (side >=1 && side <=6) {
        uint16_t color565 = rgbToRGB565(rgb);
        if (type == 'f' || type == 'F') {
          sides[side-1].textColor = color565;
          Serial.print(F("Side ")); Serial.print(side); Serial.print(F(" text color set to ")); Serial.println(rgb, HEX);
        } else if (type == 'b' || type == 'B') {
          sides[side-1].bgColor = color565;
          Serial.print(F("Side ")); Serial.print(side); Serial.print(F(" bg color set to ")); Serial.println(rgb, HEX);
        } else {
          Serial.println(F("Type must be f or b"));
        }
      } else {
        Serial.println(F("Invalid side"));
      }
    } else {
      Serial.println(F("Usage: sc f <side> <RRGGBB> or sc b <side> <RRGGBB>"));
    }
  }
  else {
    Serial.print(F("Unknown timer command: "));
    Serial.println(cmd);
  }
}

static void parseSetBat(String args) {
  int idx = args.indexOf(' ');
  if (idx == -1) {
    Serial.println(F("bat: missing command"));
    return;
  }
  String cmd = args.substring(0, idx);
  cmd.toLowerCase();
  String params = args.substring(idx + 1);
  params.trim();

  if (cmd == "fc") {
    int x, y;
    if (sscanf(params.c_str(), "%d:%d", &x, &y) == 2) {
      message.offsetX = x;
      message.offsetY = y;
      Serial.print(F("Message offset set to ")); Serial.print(x); Serial.print(F(",")); Serial.println(y);
    } else {
      Serial.println(F("Usage: fc <x>:<y>"));
    }
  }
  else if (cmd == "fcs") {
    int side, x, y;
    if (sscanf(params.c_str(), "%d %d:%d", &side, &x, &y) == 3) {
      if (side >=1 && side <=6) {
        battery.offsetX[side-1] = x;
        battery.offsetY[side-1] = y;
        Serial.print(F("Battery offset for side ")); Serial.print(side); Serial.print(F(" set to ")); Serial.print(x); Serial.print(F(",")); Serial.println(y);
      } else {
        Serial.println(F("Invalid side"));
      }
    } else {
      Serial.println(F("Usage: fcs <side> <x>:<y>"));
    }
  }
  else if (cmd == "fs") {
    char mod[10];
    if (sscanf(params.c_str(), "%s", mod) == 1) {
      float newScale = battery.scale;
      if (mod[0] == '+' || mod[0] == '-') {
        float delta = atof(mod+1);
        newScale += (mod[0]=='+' ? delta : -delta);
      } else {
        newScale = atof(mod);
      }
      if (newScale < 0.1f) newScale = 0.1f;
      if (newScale > 10.0f) newScale = 10.0f;
      battery.scale = newScale;
      Serial.print(F("Battery scale set to ")); Serial.println(newScale, 2);
    } else {
      Serial.println(F("Usage: fs <value|+d|-d>"));
    }
  }
  else if (cmd == "sc") {
    char type;
    int side;
    uint32_t rgb;
    if (sscanf(params.c_str(), "%c %d %x", &type, &side, &rgb) == 3) {
      if (side >=1 && side <=6 && (type == 'f' || type == 'F')) {
        battery.textColor[side-1] = rgbToRGB565(rgb);
        Serial.print(F("Battery text color for side ")); Serial.print(side); Serial.print(F(" set to ")); Serial.println(rgb, HEX);
      } else {
        Serial.println(F("Invalid side or type (only f supported)"));
      }
    } else {
      Serial.println(F("Usage: sc f <side> <RRGGBB>"));
    }
  }
  else if (cmd == "font") {
    int fontIdx;
    if (sscanf(params.c_str(), "%d", &fontIdx) == 1) {
      if (fontIdx >=0 && fontIdx < FONT_COUNT) {
        battery.fontIndex = fontIdx;
        Serial.print(F("Battery font set to ")); Serial.println(fontIdx);
      } else {
        Serial.println(F("Invalid font index"));
      }
    } else {
      Serial.println(F("Usage: font <fontIdx>"));
    }
  }
  else {
    Serial.print(F("Unknown bat command: "));
    Serial.println(cmd);
  }
}

static void parseSetMessage(String args) {
  int idx = args.indexOf(' ');
  if (idx == -1) {
    Serial.println(F("message: missing command"));
    return;
  }
  String cmd = args.substring(0, idx);
  cmd.toLowerCase();
  String params = args.substring(idx + 1);
  params.trim();

  if (cmd == "fc") {
    int x, y;
    if (sscanf(params.c_str(), "%d:%d", &x, &y) == 2) {
      message.offsetX = x;
      message.offsetY = y;
      Serial.print(F("Message offset set to ")); Serial.print(x); Serial.print(F(",")); Serial.println(y);
    } else {
      Serial.println(F("Usage: fc <x>:<y>"));
    }
  }
  else if (cmd == "fs") {
    char mod[10];
    if (sscanf(params.c_str(), "%s", mod) == 1) {
      float newScale = message.scale;
      if (mod[0] == '+' || mod[0] == '-') {
        float delta = atof(mod+1);
        newScale += (mod[0]=='+' ? delta : -delta);
      } else {
        newScale = atof(mod);
      }
      if (newScale < 0.1f) newScale = 0.1f;
      if (newScale > 10.0f) newScale = 10.0f;
      message.scale = newScale;
      Serial.print(F("Message scale set to ")); Serial.println(newScale, 2);
    } else {
      Serial.println(F("Usage: fs <value|+d|-d>"));
    }
  }
  else if (cmd == "sc") {
    char type;
    uint32_t rgb;
    if (sscanf(params.c_str(), "%c %x", &type, &rgb) == 2) {
      if (type == 'f' || type == 'F') {
        message.textColor = rgbToRGB565(rgb);
        Serial.print(F("Message text color set to ")); Serial.println(rgb, HEX);
      } else if (type == 'b' || type == 'B') {
        message.bgColor = rgbToRGB565(rgb);
        Serial.print(F("Message bg color set to ")); Serial.println(rgb, HEX);
      } else {
        Serial.println(F("Type must be f or b"));
      }
    } else {
      Serial.println(F("Usage: sc f <RRGGBB> or sc b <RRGGBB>"));
    }
  }
  else if (cmd == "font") {
    int fontIdx;
    if (sscanf(params.c_str(), "%d", &fontIdx) == 1) {
      if (fontIdx >=0 && fontIdx < FONT_COUNT) {
        message.fontIndex = fontIdx;
        Serial.print(F("Message font set to ")); Serial.println(fontIdx);
      } else {
        Serial.println(F("Invalid font index"));
      }
    } else {
      Serial.println(F("Usage: font <fontIdx>"));
    }
  }
  else {
    Serial.print(F("Unknown message command: "));
    Serial.println(cmd);
  }
}

static void parseSetSn(String args) {
  char type;
  float val;
  if (sscanf(args.c_str(), "%c %f", &type, &val) == 2) {
    if (type == 'f' || type == 'F') {
      global.stableThreshold = val;
      Serial.print(F("Stability threshold set to ")); Serial.print(val); Serial.println(F(" m/s²"));
    } else if (type == 't' || type == 'T') {
      global.tapThreshold = val;
      Serial.print(F("Tap threshold set to ")); Serial.print(val); Serial.println(F(" m/s²"));
    } else {
      Serial.println(F("Type must be f or t"));
    }
  } else {
    Serial.println(F("Usage: sn f <value> or sn t <value>"));
  }
}

static void parseSetGlobal(String args) {
  String sub;
  int idx = args.indexOf(' ');
  if (idx == -1) {
    sub = args;
    args = "";
  } else {
    sub = args.substring(0, idx);
    args = args.substring(idx + 1);
    args.trim();
  }
  sub.toLowerCase();

  if (sub == "confirm") {
    int sec = args.toInt();
    if (sec > 0) {
      global.confirmSeconds = sec;
      Serial.print(F("Confirm seconds set to ")); Serial.println(sec);
    } else {
      Serial.println(F("Invalid value"));
    }
  }
  else if (sub == "shutdown") {
    int sec = args.toInt();
    if (sec > 0) {
      global.shutdownSeconds = sec;
      Serial.print(F("Shutdown seconds set to ")); Serial.println(sec);
    } else {
      Serial.println(F("Invalid value"));
    }
  }
  else if (sub == "led") {
    int bright = args.toInt();
    if (bright >=0 && bright <=255) {
      global.ledBrightness = bright;
      Serial.print(F("LED brightness set to ")); Serial.println(bright);
    } else {
      Serial.println(F("Value must be 0-255"));
    }
  }
  else {
    Serial.print(F("Unknown global param: "));
    Serial.println(sub);
  }
}

void printConfig(int side, int fontIdx) {
  Serial.println(F("===== Current Configuration ====="));
  Serial.print(F("Global: stableThr=")); Serial.print(global.stableThreshold);
  Serial.print(F(" stableCnt=")); Serial.print(global.stableCountReq);
  Serial.print(F(" tapThr=")); Serial.print(global.tapThreshold);
  Serial.print(F(" tapWin=")); Serial.print(global.tapWindow);
  Serial.print(F(" confirm=")); Serial.print(global.confirmSeconds);
  Serial.print(F(" shutdown=")); Serial.print(global.shutdownSeconds);
  Serial.print(F(" ledBright=")); Serial.println(global.ledBrightness);

  Serial.println(F("Sides:"));
  for (int s=0; s<6; s++) {
    Serial.print(F("  Side ")); Serial.print(s+1);
    Serial.print(F(": timer=")); Serial.print(sides[s].timer);
    Serial.print(F(" font=")); Serial.print(sides[s].fontIndex);
    Serial.print(F(" textColor=0x")); Serial.print(rgb565ToRGB(sides[s].textColor), HEX);
    Serial.print(F(" bgColor=0x")); Serial.println(rgb565ToRGB(sides[s].bgColor), HEX);
  }

  Serial.println(F("Fonts:"));
  for (int f=0; f<FONT_COUNT; f++) {
    Serial.print(F("  Font ")); Serial.print(f);
    Serial.print(F(": baseOff=")); Serial.print(fonts[f].baseOffsetX); Serial.print(F(",")); Serial.print(fonts[f].baseOffsetY);
    Serial.print(F(" scale=")); Serial.print(fonts[f].scale, 2);
    Serial.print(F(" extras:"));
    for (int s=0; s<6; s++) {
      if (fontExtras[f].extraOffsetX[s] != 0 || fontExtras[f].extraOffsetY[s] != 0) {
        Serial.print(F(" s")); Serial.print(s+1); Serial.print(F("=")); Serial.print(fontExtras[f].extraOffsetX[s]); Serial.print(F(",")); Serial.print(fontExtras[f].extraOffsetY[s]);
      }
    }
    Serial.println();
  }

  Serial.println(F("Battery:"));
  Serial.print(F("  font=")); Serial.print(battery.fontIndex);
  Serial.print(F(" scale=")); Serial.print(battery.scale, 2);
  Serial.println();
  for (int s=0; s<6; s++) {
    Serial.print(F("  Side ")); Serial.print(s+1);
    Serial.print(F(": offset=")); Serial.print(battery.offsetX[s]); Serial.print(F(",")); Serial.print(battery.offsetY[s]);
    Serial.print(F(" textColor=0x")); Serial.println(rgb565ToRGB(battery.textColor[s]), HEX);
  }

  Serial.println(F("Message:"));
  Serial.print(F("  font=")); Serial.print(message.fontIndex);
  Serial.print(F(" scale=")); Serial.print(message.scale, 2);
  Serial.print(F(" offset=")); Serial.print(message.offsetX); Serial.print(F(",")); Serial.print(message.offsetY);
  Serial.print(F(" textColor=0x")); Serial.print(rgb565ToRGB(message.textColor), HEX);
  Serial.print(F(" bgColor=0x")); Serial.println(rgb565ToRGB(message.bgColor), HEX);
  Serial.println(F("================================"));
}

void printFontInfo(int fontIdx) {
  if (fontIdx < 0 || fontIdx >= FONT_COUNT) {
    Serial.println(F("Invalid font index"));
    return;
  }
  Serial.print(F("Font ")); Serial.print(fontIdx); Serial.println(F(":"));
  Serial.print(F("  base offset: ")); Serial.print(fonts[fontIdx].baseOffsetX); Serial.print(F(", ")); Serial.println(fonts[fontIdx].baseOffsetY);
  Serial.print(F("  scale: ")); Serial.println(fonts[fontIdx].scale, 2);
  Serial.println(F("  extra offsets per side:"));
  for (int s=0; s<6; s++) {
    if (fontExtras[fontIdx].extraOffsetX[s] != 0 || fontExtras[fontIdx].extraOffsetY[s] != 0) {
      Serial.print(F("    side ")); Serial.print(s+1); Serial.print(F(": "));
      Serial.print(fontExtras[fontIdx].extraOffsetX[s]); Serial.print(F(", ")); Serial.println(fontExtras[fontIdx].extraOffsetY[s]);
    }
  }
}