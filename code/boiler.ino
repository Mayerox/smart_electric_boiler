#include <U8g2lib.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <math.h>

// ================== ДИСПЛЕЙ ==================
// Для SSD1315 используем совместимый драйвер SSD1306 (128x64, I2C, full buffer)
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);
// Если в вашей версии U8g2 есть явный конструктор под SSD1315 — можно так:
 //U8G2_SSD1315_128X64_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ================== ДАТЧИКИ DS18B20 ==================
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ================== ПИНЫ (ESP32) ==================
#define BTN_MENU 25
#define BTN_EXIT 26
#define BTN_UP 27
#define BTN_DOWN 14

#define PIN_TENA_1 12
#define PIN_TENA_2 13
#define PIN_TENA_3 15 
#define PIN_TENA_4 17
#define PIN_BUZZER 16

// ================== АНТИДРЕБЕЗГ ==================
#define DEBOUNCE_DELAY 50
unsigned long lastDebounceTime[4] = {0, 0, 0, 0};
bool buttonState[4] = {HIGH, HIGH, HIGH, HIGH};
bool lastButtonState[4] = {HIGH, HIGH, HIGH, HIGH};

// ================== АДРЕСА ДАТЧИКОВ ==================
DeviceAddress sensorOutdoor = { 0x28, 0xB2, 0x27, 0x45, 0xD4, 0xD8, 0x5E, 0xD2 };
DeviceAddress sensorIndoor  = { 0x28, 0xDB, 0x2F, 0x44, 0xD4, 0xE1, 0x3C, 0x70 };
DeviceAddress sensorCoolant = { 0x28, 0x1E, 0x28, 0x45, 0xD4, 0x60, 0x6B, 0x69 };

// ================== EEPROM АДРЕСА ==================
#define EEPROM_TARGET_TEMP_ADDR         0
#define EEPROM_TARGET_COOLANT_ADDR      4
#define EEPROM_MAX_COOLANT_ADDR         8

#define EEPROM_MIN_COOLANT_ADDR         12
#define EEPROM_CURVE_SCALE_ADDR         16
#define EEPROM_ROOM_IMPACT_ADDR         20

#define EEPROM_HEAT_START_DELTA_ADDR    24
#define EEPROM_COOL_UPPER_DELTA_ADDR    28

#define EEPROM_TEST_INTERVAL_ADDR       32
#define EEPROM_TEST_DELTA_ADDR          36

#define EEPROM_RAMP_REEVAL_MIN_ADDR     40
#define EEPROM_SLOW_RATE_TH_ADDR        44
#define EEPROM_MOD_RATE_TH_ADDR         48
#define EEPROM_ETA_FAST_MIN_ADDR        52
#define EEPROM_ETA_MOD_MIN_ADDR         56

// ================== ДЕФОЛТЫ ==================
#define DEFAULT_TARGET_TEMP     22.0f
#define DEFAULT_TARGET_COOLANT  25.0f
#define DEFAULT_MAX_COOLANT     80.0f
#define DEFAULT_MIN_COOLANT     23.0f

#define DEFAULT_CURVE_SCALE     1.00f   // 0.6..1.6
#define DEFAULT_ROOM_IMPACT_K   1.50f

#define DEFAULT_HEAT_START_DELTA 5.0f
#define DEFAULT_COOL_UPPER_DELTA 3.0f

#define DEFAULT_TEST_INTERVAL_S 20.0f
#define DEFAULT_TEST_DELTA_C    1.0f

#define DEFAULT_RAMP_REEVAL_MIN 3.0f
#define DEFAULT_SLOW_RATE_TH    0.2f
#define DEFAULT_MOD_RATE_TH     0.5f
#define DEFAULT_ETA_FAST_MIN    20.0f
#define DEFAULT_ETA_MOD_MIN     30.0f

// Границы для условий по комнате
#define INDOOR_BAND 1.0f
#define INDOOR_HYST 0.1f

// ================== ПЕРЕМЕННЫЕ ЦЕЛЕЙ ==================
float targetTemp = DEFAULT_TARGET_TEMP;
float targetCoolantTemp = DEFAULT_TARGET_COOLANT;
float maxCoolantTemp = DEFAULT_MAX_COOLANT;
float minCoolantTemp = DEFAULT_MIN_COOLANT;

// Настройки кривой
float curveScaleK = DEFAULT_CURVE_SCALE;
float roomImpactK = DEFAULT_ROOM_IMPACT_K;

// Пороги управления
float heatStartDelta = DEFAULT_HEAT_START_DELTA;
float coolUpperDelta = DEFAULT_COOL_UPPER_DELTA;

// Параметры эффективности и адаптивного повышения
float testIntervalSec = DEFAULT_TEST_INTERVAL_S;
float testDeltaC = DEFAULT_TEST_DELTA_C;

float rampReevalMin = DEFAULT_RAMP_REEVAL_MIN;
float slowRateTh = DEFAULT_SLOW_RATE_TH;
float modRateTh = DEFAULT_MOD_RATE_TH;
float etaFastMin = DEFAULT_ETA_FAST_MIN;
float etaModMin = DEFAULT_ETA_MOD_MIN;

// ================== МЕНЮ ==================
bool inMenu = false;
bool inService = false;
bool editMode = false;

int menuItem = 0;            // основной уровень
int serviceItem = 0;         // сервисный уровень

#define MENU_MAIN_ITEMS 4
#define MENU_SERVICE_ITEMS 12
enum {
  S_CURVE_SCALE = 0,
  S_ROOM_IMPACT,
  S_HEAT_START_DELTA,
  S_COOL_UPPER_DELTA,
  S_TEST_INTERVAL_S,
  S_TEST_DELTA_C,
  S_RAMP_REEVAL_MIN,
  S_SLOW_RATE_TH,
  S_MOD_RATE_TH,
  S_ETA_FAST_MIN,
  S_ETA_MOD_MIN,
  S_MIN_COOLANT
};

// Оригинальные для отката
float originalTargetTemp;
float originalTargetCoolantTemp;
float originalMaxCoolantTemp;
float originalMinCoolantTemp;

float originalCurveScaleK, originalRoomImpactK;
float originalHeatStartDelta, originalCoolUpperDelta;
float originalTestIntervalSec, originalTestDeltaC;
float originalRampReevalMin, originalSlowRateTh, originalModRateTh, originalEtaFastMin, originalEtaModMin;

// ================== ИНДИКАЦИЯ СОХРАНЕНИЯ ==================
bool showSavedMessage = false;
unsigned long savedMessageTime = 0;
#define SAVED_MESSAGE_DURATION 500

// ================== ОБНОВЛЕНИЕ ИЗМЕРЕНИЙ ==================
unsigned long lastTempUpdate = 0;
#define TEMP_UPDATE_INTERVAL 10000UL  // 10 c

// Быстрый UI (независимо от датчиков)
unsigned long lastUiRefresh = 0;
#define UI_REFRESH_INTERVAL 150UL

unsigned long lastMenuActivity = 0;
#define MENU_TIMEOUT 60000UL

// ================== ФИЛЬТРЫ ==================
bool filtersInitialized = false;
float filtOutdoor = NAN, filtIndoor = NAN, filtCoolant = NAN;
const float alpha = 0.3f;

// ================== Мониторинг датчиков ==================
struct SensorHealth {
  uint8_t failCount = 0;
  unsigned long lastOk = 0;
};
SensorHealth outH, inH, colH;

#define SENSOR_FAIL_LIMIT     2
#define SENSOR_STALE_TIMEOUT  30000UL

static inline void updateHealth(SensorHealth& h, float raw) {
  unsigned long now = millis();
  if (isnan(raw)) {
    if (h.failCount < 255) h.failCount++;
  } else {
    h.failCount = 0;
    h.lastOk = now;
  }
}

// ================== DS18B20 неблокирующий режим ==================
#define DS_RES_BITS 11  // 9:~95мс, 10:~188мс, 11:~375мс, 12:~750мс
uint16_t dsConvMs = 375;
bool dsConverting = false;
unsigned long dsConvStart = 0;
float lastRawOut = NAN, lastRawIn = NAN, lastRawCol = NAN;

static uint16_t convTimeForBits(uint8_t bits){
  switch(bits){
    case 9:  return 95;
    case 10: return 188;
    case 11: return 375;
    default: return 750;
  }
}

// ================== ПЕРЕГРЕВ/НЕДОГРЕВ ЛОГИКА ==================
unsigned long strongHoldStart   = 0;
unsigned long moderateHoldStart = 0;

unsigned long rampStartTime = 0;
float rampStartCoolant = NAN;
unsigned long lastRampEval = 0;

// ================== ТЭНы ==================
bool tena1State=false, tena2State=false, tena3State=false;
int activeTenas = 0;
bool heatingMode = false;
bool coolingMode = false;
unsigned long lastTenaCheckTime = 0;
unsigned long lastTenaToggleTime = 0;
float lastCoolantTempCheck = NAN;

// RoomHold
bool roomHoldActive = false;
#define ROOM_HOLD_OFF_HYST 0.4f

// ================== АВАРИИ ==================
bool alarmActive = false;
bool alarmMuted = false;
String alarmMessage = "";
unsigned long lastBeepToggle = 0;
bool buzzerOn = false;
#define BEEP_ON_MS  300UL
#define BEEP_OFF_MS 700UL

bool tempsEverRead = false;

// ================== EEPROM ==================
void saveFloat(int addr, float val){ EEPROM.put(addr, val); }
float loadFloat(int addr, float def){ float v; EEPROM.get(addr, v); if (isnan(v)) return def; return v; }
void commitEEP(){ EEPROM.commit(); }

void saveTargetTemp(){ saveFloat(EEPROM_TARGET_TEMP_ADDR, targetTemp); commitEEP(); }
void saveTargetCoolantTemp(){ saveFloat(EEPROM_TARGET_COOLANT_ADDR, targetCoolantTemp); commitEEP(); }
void saveMaxCoolantTemp(){ saveFloat(EEPROM_MAX_COOLANT_ADDR, maxCoolantTemp); commitEEP(); }

void saveSettings(){
  saveFloat(EEPROM_MIN_COOLANT_ADDR,       minCoolantTemp);
  saveFloat(EEPROM_CURVE_SCALE_ADDR,       curveScaleK);
  saveFloat(EEPROM_ROOM_IMPACT_ADDR,       roomImpactK);

  saveFloat(EEPROM_HEAT_START_DELTA_ADDR,  heatStartDelta);
  saveFloat(EEPROM_COOL_UPPER_DELTA_ADDR,  coolUpperDelta);

  saveFloat(EEPROM_TEST_INTERVAL_ADDR,     testIntervalSec);
  saveFloat(EEPROM_TEST_DELTA_ADDR,        testDeltaC);

  saveFloat(EEPROM_RAMP_REEVAL_MIN_ADDR,   rampReevalMin);
  saveFloat(EEPROM_SLOW_RATE_TH_ADDR,      slowRateTh);
  saveFloat(EEPROM_MOD_RATE_TH_ADDR,       modRateTh);
  saveFloat(EEPROM_ETA_FAST_MIN_ADDR,      etaFastMin);
  saveFloat(EEPROM_ETA_MOD_MIN_ADDR,       etaModMin);

  commitEEP();
}

void loadAll(){
  float t;
  EEPROM.get(EEPROM_TARGET_TEMP_ADDR, t);
  targetTemp = (!isnan(t) && t>-50 && t<100) ? t : DEFAULT_TARGET_TEMP;
  if (isnan(t) || !(t>-50 && t<100)) saveTargetTemp();

  EEPROM.get(EEPROM_TARGET_COOLANT_ADDR, t);
  targetCoolantTemp = (!isnan(t) && t>=0 && t<=120) ? t : DEFAULT_TARGET_COOLANT;
  if (isnan(t) || !(t>=0 && t<=120)) saveTargetCoolantTemp();

  EEPROM.get(EEPROM_MAX_COOLANT_ADDR, t);
  maxCoolantTemp = (!isnan(t) && t>=30 && t<=120) ? t : DEFAULT_MAX_COOLANT;
  if (isnan(t) || !(t>=30 && t<=120)) saveMaxCoolantTemp();

  minCoolantTemp = loadFloat(EEPROM_MIN_COOLANT_ADDR, DEFAULT_MIN_COOLANT);
  curveScaleK    = loadFloat(EEPROM_CURVE_SCALE_ADDR, DEFAULT_CURVE_SCALE);
  roomImpactK    = loadFloat(EEPROM_ROOM_IMPACT_ADDR, DEFAULT_ROOM_IMPACT_K);

  heatStartDelta = loadFloat(EEPROM_HEAT_START_DELTA_ADDR, DEFAULT_HEAT_START_DELTA);
  coolUpperDelta = loadFloat(EEPROM_COOL_UPPER_DELTA_ADDR, DEFAULT_COOL_UPPER_DELTA);

  testIntervalSec = loadFloat(EEPROM_TEST_INTERVAL_ADDR, DEFAULT_TEST_INTERVAL_S);
  testDeltaC       = loadFloat(EEPROM_TEST_DELTA_ADDR, DEFAULT_TEST_DELTA_C);

  rampReevalMin  = loadFloat(EEPROM_RAMP_REEVAL_MIN_ADDR, DEFAULT_RAMP_REEVAL_MIN);
  slowRateTh     = loadFloat(EEPROM_SLOW_RATE_TH_ADDR, DEFAULT_SLOW_RATE_TH);
  modRateTh      = loadFloat(EEPROM_MOD_RATE_TH_ADDR, DEFAULT_MOD_RATE_TH);
  etaFastMin     = loadFloat(EEPROM_ETA_FAST_MIN_ADDR, DEFAULT_ETA_FAST_MIN);
  etaModMin      = loadFloat(EEPROM_ETA_MOD_MIN_ADDR, DEFAULT_ETA_MOD_MIN);

  // Валидация
  if (curveScaleK < 0.6f || curveScaleK > 1.6f) curveScaleK = DEFAULT_CURVE_SCALE;
  if (roomImpactK < 0.0f || roomImpactK > 4.0f) roomImpactK = DEFAULT_ROOM_IMPACT_K;

  if (minCoolantTemp < 15.0f) minCoolantTemp = 15.0f;
  if (minCoolantTemp > 50.0f) minCoolantTemp = 50.0f;

  if (heatStartDelta < 2.0f) heatStartDelta = 2.0f;
  if (heatStartDelta > 10.0f) heatStartDelta = 10.0f;

  if (coolUpperDelta < 1.0f) coolUpperDelta = 1.0f;
  if (coolUpperDelta > 10.0f) coolUpperDelta = 10.0f;

  if (testIntervalSec < 10.0f) testIntervalSec = 10.0f;
  if (testIntervalSec > 120.0f) testIntervalSec = 120.0f;

  if (testDeltaC < 0.3f) testDeltaC = 0.3f;
  if (testDeltaC > 3.0f) testDeltaC = 3.0f;

  if (rampReevalMin < 1.0f) rampReevalMin = 1.0f;
  if (rampReevalMin > 15.0f) rampReevalMin = 15.0f;
}

// ================== ЧТЕНИЕ ТЕМПЕРАТУР ==================
static float readTemperatureC(const DeviceAddress addr) {
  float t = sensors.getTempC(addr);
  if (t == DEVICE_DISCONNECTED_C) return NAN;
  if (t < -55.0f || t > 125.0f) return NAN;
  if (fabs(t - 85.0f) < 0.01f) return NAN; // игнор «85°C»
  return t;
}
float expFilter(float prev, float val){
  if (isnan(prev)) return val;
  if (isnan(val))  return prev;
  return prev + alpha*(val - prev);
}

// ================== КРИВАЯ ОТОПЛЕНИЯ ==================
const int CURVE_POINTS = 6;
const float curveX[CURVE_POINTS] = { 20.0,  5.0,   0.0,  -10.0, -20.0, -30.0 };
const float curveY[CURVE_POINTS] = { 23.0, 23.0,  30.0,  50.0,  65.0,  80.0 };

float interpCurve(const float* x, const float* y, int n, float xq){
  if (xq >= x[0]) return y[0];
  if (xq <= x[n-1]) return y[n-1];
  for(int i=0;i<n-1;i++){
    if (xq <= x[i] && xq >= x[i+1]){
      float x1=x[i], x2=x[i+1];
      float y1=y[i], y2=y[i+1];
      float k = (y2 - y1)/(x2 - x1);
      return y1 + k*(xq - x1);
    }
  }
  return y[n-1];
}

float calculateOptimalCoolantTemp(float outdoor){
  float base = interpCurve(curveX, curveY, CURVE_POINTS, outdoor);
  float scaled = minCoolantTemp + (base - minCoolantTemp)*curveScaleK;
  float shifted = scaled + (targetTemp - 22.0f)*roomImpactK;
  if (shifted < minCoolantTemp) shifted = minCoolantTemp;
  if (shifted > maxCoolantTemp) shifted = maxCoolantTemp;
  return shifted;
}

// ================== ЛОГИКА ЦЕЛЕЙ ТЕПЛОНОСИТЕЛЯ ==================
void setCoolantTargetToOptimal(float tOutdoor) {
  float opt = calculateOptimalCoolantTemp(tOutdoor);
  opt = constrain(opt, minCoolantTemp, maxCoolantTemp);
  if (fabs(opt - targetCoolantTemp) > 0.05f) {
    targetCoolantTemp = opt;
    saveTargetCoolantTemp();
    Serial.print("[manage] Установлена оптимальная цель теплоносителя: ");
    Serial.println(targetCoolantTemp, 1);
  } else {
    Serial.println("[manage] Оптимальная цель совпадает");
  }
}

void manageCoolantTargetTemp(float tIndoor, float tOutdoor, float tCoolant) {
  unsigned long now = millis();

  bool strong   = (tIndoor >= targetTemp + INDOOR_BAND - INDOOR_HYST);
  bool moderate = (tIndoor >  targetTemp + INDOOR_HYST) &&
                  (tIndoor <  targetTemp + INDOOR_BAND - INDOOR_HYST);
  bool under    = (tIndoor <= targetTemp - 1.0f - INDOOR_HYST);

  // Сильный перегрев 5 мин
  if (strong) {
    if (strongHoldStart == 0) strongHoldStart = now;
    if (now - strongHoldStart >= 5UL * 60UL * 1000UL) {
      setCoolantTargetToOptimal(tOutdoor);
      strongHoldStart   = 0;
      moderateHoldStart = 0;
    }
  } else {
    strongHoldStart = 0;
  }

  // Умеренный перегрев 2 часа
  if (moderate) {
    if (moderateHoldStart == 0) moderateHoldStart = now;
    if (now - moderateHoldStart >= 2UL * 60UL * 60UL * 1000UL) {
      setCoolantTargetToOptimal(tOutdoor);
      moderateHoldStart = 0;
      strongHoldStart   = 0;
    }
  } else {
    moderateHoldStart = 0;
  }

  // Недогрев — адаптивный рост цели
  if (under) {
    if (rampStartTime == 0 || isnan(rampStartCoolant)) {
      rampStartTime = now;
      rampStartCoolant = tCoolant;
      lastRampEval = now;
    }

    if (now - lastRampEval >= (unsigned long)(rampReevalMin * 60000.0f)) {
      lastRampEval = now;

      float dt_min = (now - rampStartTime) / 60000.0f;
      float dCoolant = tCoolant - rampStartCoolant;
      float rate = (dt_min > 0) ? (dCoolant / dt_min) : 0.0f;

      float remain = targetCoolantTemp - tCoolant; if (remain < 0) remain = 0;
      float eta_min = (rate > 0.01f) ? (remain / rate) : 1e6f;

      bool shouldBoost =
        (rate < slowRateTh && eta_min > etaFastMin) ||
        (rate < modRateTh  && eta_min > etaModMin);

      if (shouldBoost) {
        float newTarget = targetCoolantTemp + 2.0f;
        if (newTarget > maxCoolantTemp) newTarget = maxCoolantTemp;
        if (newTarget > targetCoolantTemp) {
          targetCoolantTemp = newTarget;
          saveTargetCoolantTemp();
          rampStartTime = now;
          rampStartCoolant = tCoolant;
          Serial.print("[manage] Недогрев — подняли цель подачи до ");
          Serial.println(targetCoolantTemp, 1);
        }
      }
    }
  } else {
    rampStartTime = 0;
    rampStartCoolant = NAN;
  }

  if (targetCoolantTemp > maxCoolantTemp) targetCoolantTemp = maxCoolantTemp;
  if (targetCoolantTemp < minCoolantTemp) targetCoolantTemp = minCoolantTemp;
}

// ================== RoomHold ==================
void updateRoomHold(float tIndoor, float tOutdoor) {
  if (!roomHoldActive && tIndoor >= targetTemp) {
    roomHoldActive = true;
    float opt = calculateOptimalCoolantTemp(tOutdoor);
    if (opt < targetCoolantTemp) {
      targetCoolantTemp = opt;
      saveTargetCoolantTemp();
    }
    Serial.println("[ROOM HOLD] Цель по комнате достигнута — снижаем цель подачи");
  } else if (roomHoldActive && tIndoor <= targetTemp - ROOM_HOLD_OFF_HYST) {
    roomHoldActive = false;
    Serial.println("[ROOM HOLD] Выход из HOLD");
  }
}

// ================== ТЭНы ==================
void activateTena(int n){
  switch(n){
    case 1: if (!tena1State){ digitalWrite(PIN_TENA_1, HIGH); tena1State=true; activeTenas++; } break;
    case 2: if (!tena2State){ digitalWrite(PIN_TENA_2, HIGH); tena2State=true; activeTenas++; } break;
    case 3: if (!tena3State){ digitalWrite(PIN_TENA_3, HIGH); tena3State=true; activeTenas++; } break;
  }
}
void deactivateTena(int n){
  switch(n){
    case 1: if (tena1State){ digitalWrite(PIN_TENA_1, LOW); tena1State=false; activeTenas--; } break;
    case 2: if (tena2State){ digitalWrite(PIN_TENA_2, LOW); tena2State=false; activeTenas--; } break;
    case 3: if (tena3State){ digitalWrite(PIN_TENA_3, LOW); tena3State=false; activeTenas--; } break;
  }
}
void allTenasOff(){ deactivateTena(1); deactivateTena(2); deactivateTena(3); activeTenas=0; }

void activateRandomTena(){
  int avail[3]; int cnt=0;
  if (!tena1State) avail[cnt++]=1;
  if (!tena2State) avail[cnt++]=2;
  if (!tena3State) avail[cnt++]=3;
  if (cnt>0){
    int idx = random(0, cnt);
    activateTena(avail[idx]);
  }
}
bool deactivateNextTena(){ // 3->2->1
  if (tena3State){ deactivateTena(3); return true; }
  if (tena2State){ deactivateTena(2); return true; }
  if (tena1State){ deactivateTena(1); return true; }
  return false;
}

void manageTenas(float tCoolant){
  unsigned long now = millis();

  // RoomHold: мягкое отключение и запрет на набор
  if (roomHoldActive) {
    heatingMode = false;
    if (activeTenas > 0) {
      if (now - lastTenaToggleTime >= 5000UL) {
        deactivateNextTena();
        lastTenaToggleTime = now;
      }
    } else {
      coolingMode = false;
    }
    return;
  }

  // Требуется нагрев
  if (tCoolant < targetCoolantTemp - heatStartDelta){
    if (!heatingMode){
      heatingMode = true; coolingMode = false;
      lastTenaCheckTime = now;
      lastCoolantTempCheck = tCoolant;
    }
    if (activeTenas == 0){
      activateRandomTena();
      lastTenaCheckTime = now;
      lastCoolantTempCheck = tCoolant;
    } else if (now - lastTenaCheckTime >= (unsigned long)(testIntervalSec*1000.0f)){
      float delta = tCoolant - lastCoolantTempCheck;
      if (delta < testDeltaC && activeTenas < 3){
        activateRandomTena();
      }
      lastTenaCheckTime = now;
      lastCoolantTempCheck = tCoolant;
    }
    return;
  }

  // Перегрев теплоносителя относительно цели — охлаждение
  if (tCoolant >= targetCoolantTemp + coolUpperDelta && activeTenas > 0){
    if (!coolingMode){ coolingMode = true; heatingMode = false; lastTenaToggleTime = now; }
    if (now - lastTenaToggleTime >= 5000UL){
      if (deactivateNextTena()){
        lastTenaToggleTime = now;
      } else {
        coolingMode = false;
      }
    }
    return;
  }

  // Прерывание охлаждения если опять ушли вниз
  if (coolingMode && tCoolant < targetCoolantTemp - heatStartDelta){
    coolingMode = false; heatingMode = true;
    lastTenaCheckTime = now;
    lastCoolantTempCheck = tCoolant;
  }
}

// ================== АВАРИЙНЫЙ КОНТРОЛЬ ==================
void enterAlarm(const String& msg){
  if (!alarmActive){
    alarmActive = true;
    alarmMuted = false;
    lastBeepToggle = millis();
    buzzerOn = false;
  }
  alarmMessage = msg;
  allTenasOff();
  heatingMode = false; coolingMode = false;
}

void clearAlarm(){
  alarmActive = false;
  alarmMessage = "";
  alarmMuted = false;
  buzzerOn = false;
  digitalWrite(PIN_BUZZER, LOW);
}

void tickBuzzer(){
  if (!alarmActive || alarmMuted){
    if (buzzerOn){ buzzerOn=false; digitalWrite(PIN_BUZZER, LOW); }
    return;
  }
  unsigned long now = millis();
  if (buzzerOn){
    if (now - lastBeepToggle >= BEEP_ON_MS){
      buzzerOn=false; lastBeepToggle=now; digitalWrite(PIN_BUZZER, LOW);
    }
  } else {
    if (now - lastBeepToggle >= BEEP_OFF_MS){
      buzzerOn=true; lastBeepToggle=now; digitalWrite(PIN_BUZZER, HIGH);
    }
  }
}

void checkAlarmsRaw(float rawOut, float rawIn, float rawCol) {
  if (!tempsEverRead) return;

  updateHealth(outH, rawOut);
  updateHealth(inH,  rawIn);
  updateHealth(colH, rawCol);

  unsigned long now = millis();

  // 1) Обрыв/застой любого датчика
  if (outH.failCount >= SENSOR_FAIL_LIMIT || (outH.lastOk > 0 && (now - outH.lastOk) > SENSOR_STALE_TIMEOUT)) {
    enterAlarm("Авария: датчик улица");
    return;
  }
  if (inH.failCount >= SENSOR_FAIL_LIMIT || (inH.lastOk > 0 && (now - inH.lastOk) > SENSOR_STALE_TIMEOUT)) {
    enterAlarm("Авария: датчик дом");
    return;
  }
  if (colH.failCount >= SENSOR_FAIL_LIMIT || (colH.lastOk > 0 && (now - colH.lastOk) > SENSOR_STALE_TIMEOUT)) {
    enterAlarm("Авария: датчик подача");
    return;
  }

  // 2) Перегрев теплоносителя
  if (!isnan(rawCol) && rawCol > maxCoolantTemp + 5.0f) {
    enterAlarm("Авария: перегрев теплоносителя");
    return;
  }

  // 3) Снятие аварии при нормализации
  if (alarmActive) {
    bool sensorsOk = (outH.failCount < SENSOR_FAIL_LIMIT) &&
                     (inH.failCount  < SENSOR_FAIL_LIMIT) &&
                     (colH.failCount < SENSOR_FAIL_LIMIT) &&
                     (now - outH.lastOk <= SENSOR_STALE_TIMEOUT) &&
                     (now - inH.lastOk  <= SENSOR_STALE_TIMEOUT) &&
                     (now - colH.lastOk <= SENSOR_STALE_TIMEOUT);

    bool coolantOk = isnan(rawCol) ? false : (rawCol <= maxCoolantTemp + 2.0f);

    if (sensorsOk && coolantOk) {
      clearAlarm();
    }
  }
}

// ================== КНОПКИ ==================
bool readButton(int idx, int pin){
  int reading = digitalRead(pin);
  if (reading != lastButtonState[idx]) lastDebounceTime[idx] = millis();
  bool pressed=false;
  if (millis()-lastDebounceTime[idx] > DEBOUNCE_DELAY){
    if (reading != buttonState[idx]){
      buttonState[idx]=reading;
      if (buttonState[idx]==LOW) pressed=true;
    }
  }
  lastButtonState[idx]=reading;
  return pressed;
}

void snapshotOriginals(){
  originalTargetTemp = targetTemp;
  originalTargetCoolantTemp = targetCoolantTemp;
  originalMaxCoolantTemp = maxCoolantTemp;
  originalMinCoolantTemp = minCoolantTemp;

  originalCurveScaleK = curveScaleK;
  originalRoomImpactK = roomImpactK;
  originalHeatStartDelta = heatStartDelta;
  originalCoolUpperDelta = coolUpperDelta;
  originalTestIntervalSec = testIntervalSec;
  originalTestDeltaC = testDeltaC;
  originalRampReevalMin = rampReevalMin;
  originalSlowRateTh = slowRateTh;
  originalModRateTh = modRateTh;
  originalEtaFastMin = etaFastMin;
  originalEtaModMin = etaModMin;
}

void restoreOriginals(){
  targetTemp = originalTargetTemp;
  targetCoolantTemp = originalTargetCoolantTemp;
  maxCoolantTemp = originalMaxCoolantTemp;
  minCoolantTemp = originalMinCoolantTemp;

  curveScaleK = originalCurveScaleK;
  roomImpactK = originalRoomImpactK;
  heatStartDelta = originalHeatStartDelta;
  coolUpperDelta = originalCoolUpperDelta;
  testIntervalSec = originalTestIntervalSec;
  testDeltaC = originalTestDeltaC;
  rampReevalMin = originalRampReevalMin;
  slowRateTh = originalSlowRateTh;
  modRateTh = originalModRateTh;
  originalEtaFastMin = originalEtaFastMin;
  originalEtaModMin = originalEtaModMin;
}

void handleButtons() {
  bool menuPressed = readButton(0, BTN_MENU);
  bool exitPressed = readButton(1, BTN_EXIT);
  bool upPressed   = readButton(2, BTN_UP);
  bool downPressed = readButton(3, BTN_DOWN);

  if (menuPressed || exitPressed || upPressed || downPressed) {
    lastMenuActivity = millis();
    if (alarmActive) {
      alarmMuted = true;
      digitalWrite(PIN_BUZZER, LOW);
      return;
    }
    // Если идёт показ "СОХРАНЕНО" — любое нажатие ускоряет выход
    if (showSavedMessage && inMenu) {
      showSavedMessage = false;
      inMenu = false;
      inService = false;
      editMode = false;
      return;
    }
  }

  if (menuPressed) {
    if (!inMenu) {
      inMenu = true; editMode = false; inService = false; menuItem = 0;
      snapshotOriginals();
    }
    else if (!inService && !editMode) {
      if (menuItem == 3) { inService = true; serviceItem = 0; editMode = false; }
      else { editMode = true; }
    }
    else if (inService && !editMode) {
      editMode = true;
    }
  }

  if (exitPressed) {
    if (inMenu && editMode) {
      if (!inService) {
        switch (menuItem) {
          case 0: saveTargetTemp();        originalTargetTemp = targetTemp; break;
          case 1: saveMaxCoolantTemp();    originalMaxCoolantTemp = maxCoolantTemp; break;
          case 2: saveSettings();          originalMinCoolantTemp = minCoolantTemp; break;
        }
      } else {
        saveSettings();
        snapshotOriginals();
      }
      editMode = false;
      showSavedMessage = true;
      savedMessageTime = millis();
      // Автовыход сделаем в loop()
    } else if (inMenu) {
      restoreOriginals();
      inMenu = false;
      inService = false;
      editMode = false;
    }
  }

  if (inMenu) {
    if (!editMode) {
      if (upPressed)   { if (!inService) { menuItem--; if (menuItem < 0) menuItem = MENU_MAIN_ITEMS - 1; } else { serviceItem--; if (serviceItem < 0) serviceItem = MENU_SERVICE_ITEMS - 1; } }
      if (downPressed) { if (!inService) { menuItem++; if (menuItem >= MENU_MAIN_ITEMS) menuItem = 0; } else { serviceItem++; if (serviceItem >= MENU_SERVICE_ITEMS) serviceItem = 0; } }
    } else {
      if (!inService) {
        if (upPressed) {
          switch (menuItem) {
            case 0: targetTemp += 0.5f; if (targetTemp > 35.0f) targetTemp = 35.0f; break;
            case 1: maxCoolantTemp += 1.0f; if (maxCoolantTemp > 100.0f) maxCoolantTemp = 100.0f; break;
            case 2: minCoolantTemp += 0.5f; if (minCoolantTemp > 50.0f) minCoolantTemp = 50.0f; break;
          }
        }
        if (downPressed) {
          switch (menuItem) {
            case 0: targetTemp -= 0.5f; if (targetTemp < 5.0f) targetTemp = 5.0f; break;
            case 1: maxCoolantTemp -= 1.0f; if (maxCoolantTemp < 30.0f) maxCoolantTemp = 30.0f; break;
            case 2: minCoolantTemp -= 0.5f; if (minCoolantTemp < 15.0f) minCoolantTemp = 15.0f; break;
          }
        }
      } else {
        if (upPressed) {
          switch (serviceItem) {
            case S_CURVE_SCALE:      curveScaleK     += 0.05f; if (curveScaleK > 1.6f) curveScaleK = 1.6f; break;
            case S_ROOM_IMPACT:      roomImpactK     += 0.1f;  if (roomImpactK > 4.0f) roomImpactK = 4.0f; break;
            case S_HEAT_START_DELTA: heatStartDelta  += 0.5f;  if (heatStartDelta > 10.0f) heatStartDelta = 10.0f; break;
            case S_COOL_UPPER_DELTA: coolUpperDelta  += 0.5f;  if (coolUpperDelta > 10.0f) coolUpperDelta = 10.0f; break;
            case S_TEST_INTERVAL_S:  testIntervalSec += 1.0f;  if (testIntervalSec > 120.0f) testIntervalSec = 120.0f; break;
            case S_TEST_DELTA_C:     testDeltaC      += 0.1f;  if (testDeltaC > 3.0f) testDeltaC = 3.0f; break;
            case S_RAMP_REEVAL_MIN:  rampReevalMin   += 0.5f;  if (rampReevalMin > 15.0f) rampReevalMin = 15.0f; break;
            case S_SLOW_RATE_TH:     slowRateTh      += 0.05f; if (slowRateTh > 1.0f) slowRateTh = 1.0f; break;
            case S_MOD_RATE_TH:      modRateTh       += 0.05f; if (modRateTh > 1.5f) modRateTh = 1.5f; break;
            case S_ETA_FAST_MIN:     etaFastMin      += 2.0f;  if (etaFastMin > 120.0f) etaFastMin = 120.0f; break;
            case S_ETA_MOD_MIN:      etaModMin       += 2.0f;  if (etaModMin > 180.0f) etaModMin = 180.0f; break;
            case S_MIN_COOLANT:      minCoolantTemp  += 0.5f;  if (minCoolantTemp > 50.0f) minCoolantTemp = 50.0f; break;
          }
        }
        if (downPressed) {
          switch (serviceItem) {
            case S_CURVE_SCALE:      curveScaleK     -= 0.05f; if (curveScaleK < 0.6f) curveScaleK = 0.6f; break;
            case S_ROOM_IMPACT:      roomImpactK     -= 0.1f;  if (roomImpactK < 0.0f) roomImpactK = 0.0f; break;
            case S_HEAT_START_DELTA: heatStartDelta  -= 0.5f;  if (heatStartDelta < 2.0f) heatStartDelta = 2.0f; break;
            case S_COOL_UPPER_DELTA: coolUpperDelta  -= 0.5f;  if (coolUpperDelta < 1.0f) coolUpperDelta = 1.0f; break;
            case S_TEST_INTERVAL_S:  testIntervalSec -= 1.0f;  if (testIntervalSec < 10.0f) testIntervalSec = 10.0f; break;
            case S_TEST_DELTA_C:     testDeltaC      -= 0.1f;  if (testDeltaC < 0.3f) testDeltaC = 0.3f; break;
            case S_RAMP_REEVAL_MIN:  rampReevalMin   -= 0.5f;  if (rampReevalMin < 1.0f) rampReevalMin = 1.0f; break;
            case S_SLOW_RATE_TH:     slowRateTh      -= 0.05f; if (slowRateTh < 0.05f) slowRateTh = 0.05f; break;
            case S_MOD_RATE_TH:      modRateTh       -= 0.05f; if (modRateTh < 0.1f)  modRateTh = 0.1f; break;
            case S_ETA_FAST_MIN:     etaFastMin      -= 2.0f;  if (etaFastMin < 5.0f) etaFastMin = 5.0f; break;
            case S_ETA_MOD_MIN:      etaModMin       -= 2.0f;  if (etaModMin < 10.0f) etaModMin = 10.0f; break;
            case S_MIN_COOLANT:      minCoolantTemp  -= 0.5f;  if (minCoolantTemp < 15.0f) minCoolantTemp = 15.0f; break;
          }
        }
      }
    }
  }

  if (inMenu && (millis() - lastMenuActivity > MENU_TIMEOUT)) {
    restoreOriginals();
    inMenu = false;
    inService = false;
    editMode = false;
  }
}

// ================== ДИСПЛЕЙ ==================
void drawHeader(const char* title){
  u8g2.setCursor(10, 12);
  u8g2.print(title);
}

void displayAlarmScreen(){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_t_cyrillic);

  u8g2.setCursor(20, 12);
  u8g2.print("*** АВАРИЯ ***");

  u8g2.setCursor(2, 28);
  u8g2.print(alarmMessage);

  u8g2.setCursor(2, 44);
  u8g2.print("ТЭНы отключены");

  u8g2.setCursor(2, 60);
  u8g2.print("Нажмите любую кнопку");
  u8g2.sendBuffer();
}

void displayMenu(){
  if (showSavedMessage && (millis() - savedMessageTime > SAVED_MESSAGE_DURATION)) showSavedMessage=false;

  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_t_cyrillic);

  if (showSavedMessage) drawHeader("*** СОХРАНЕНО ***");
  else if (editMode)    drawHeader( inService ? "*** СЕРВ-ПАРАМ ***" : "*** РЕДАКТИРОВАНИЕ ***");
  else                  drawHeader(inService ? "*** СЕРВИС ***" : "*** МЕНЮ ***");

  if (!inService){
    for (int i=0;i<MENU_MAIN_ITEMS;i++){
      u8g2.setCursor(2, 28 + i*12);
      if (i==menuItem && editMode) u8g2.print("* ");
      else if (i==menuItem) u8g2.print("> ");
      else u8g2.print("  ");

      switch(i){
        case 0: u8g2.print("Цел.комн: "); u8g2.print(targetTemp,1); u8g2.print("C"); break;
        case 1: u8g2.print("Макс.под: "); u8g2.print(maxCoolantTemp,1); u8g2.print("C"); break;
        case 2: u8g2.print("Мин.под:  "); u8g2.print(minCoolantTemp,1); u8g2.print("C"); break;
        case 3: u8g2.print("СЕРВИС ->"); break;
      }
    }
  } else {
    u8g2.setCursor(2, 28);
    if (editMode) u8g2.print("* "); else u8g2.print("> ");

    switch(serviceItem){
      case S_CURVE_SCALE:      u8g2.print("Масштаб кривой: "); u8g2.print(curveScaleK,2); break;
      case S_ROOM_IMPACT:      u8g2.print("Сдвиг/гр.комн: "); u8g2.print(roomImpactK,2); break;
      case S_HEAT_START_DELTA: u8g2.print("Δстарт нагрева: "); u8g2.print(heatStartDelta,1); u8g2.print("C"); break;
      case S_COOL_UPPER_DELTA: u8g2.print("Δоткл охлажд: "); u8g2.print(coolUpperDelta,1); u8g2.print("C"); break;
      case S_TEST_INTERVAL_S:  u8g2.print("Интервал теста: "); u8g2.print(testIntervalSec,0); u8g2.print("с"); break;
      case S_TEST_DELTA_C:     u8g2.print("Δтеста ТЭНа: "); u8g2.print(testDeltaC,1); u8g2.print("C"); break;
      case S_RAMP_REEVAL_MIN:  u8g2.print("Переоценка: "); u8g2.print(rampReevalMin,1); u8g2.print("м"); break;
      case S_SLOW_RATE_TH:     u8g2.print("Порог медл: "); u8g2.print(slowRateTh,2); u8g2.print("C/м"); break;
      case S_MOD_RATE_TH:      u8g2.print("Порог средн: "); u8g2.print(modRateTh,2); u8g2.print("C/м"); break;
      case S_ETA_FAST_MIN:     u8g2.print("ETA slow> : "); u8g2.print(etaFastMin,0); u8g2.print("м"); break;
      case S_ETA_MOD_MIN:      u8g2.print("ETA mod>  : "); u8g2.print(etaModMin,0); u8g2.print("м"); break;
      case S_MIN_COOLANT:      u8g2.print("Мин.под (серв): "); u8g2.print(minCoolantTemp,1); u8g2.print("C"); break;
    }

    u8g2.setCursor(2, 44);
    u8g2.print("MENU=редакт/далее");

    u8g2.setCursor(2, 60);
    u8g2.print("EXIT=сохран/назад");
  }

  u8g2.sendBuffer();
}

void displayNormalMode(float tOut, float tIn, float tCol){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_6x12_t_cyrillic);

  u8g2.setCursor(26, 12); u8g2.print("Температура, °C");

  u8g2.setCursor(2, 24);  u8g2.print("Улица/Дом/Цель комн");
  u8g2.setCursor(18, 36);
  if (isnan(tOut)) u8g2.print("--"); else u8g2.print(tOut,1);
  u8g2.print("/");
  if (isnan(tIn)) u8g2.print("--"); else u8g2.print(tIn,1);
  u8g2.print("/");
  u8g2.print(targetTemp,1);

  u8g2.setCursor(2, 45); u8g2.print("Подача/Цель подача");
  u8g2.setCursor(18, 55);
  if (isnan(tCol)) u8g2.print("--"); else u8g2.print(tCol,1);
  u8g2.print("/");
  u8g2.print(targetCoolantTemp,1);

  // Правый низ — статус ТЭНов/режим
  u8g2.setCursor(90, 63);
  if (tena1State || tena2State || tena3State){
    u8g2.print("T:");
    if (tena1State) u8g2.print("1");
    if (tena2State) u8g2.print("2");
    if (tena3State) u8g2.print("3");
  } else {
    u8g2.print("T:-");
  }

  u8g2.setCursor(0, 63);
  if (alarmActive) u8g2.print("[АВАР]");
  else if (roomHoldActive) u8g2.print("[ХОЛД]");
  else if (heatingMode) u8g2.print("[НАГРЕВ]");
  else if (coolingMode) u8g2.print("[ОХЛАЖД]");
  else u8g2.print("[Ожидание]");

  u8g2.sendBuffer();
}

// ================== SETUP / LOOP ==================
void setup(){
  Serial.begin(115200);
  EEPROM.begin(512);

  pinMode(BTN_MENU, INPUT_PULLUP);
  pinMode(BTN_EXIT, INPUT_PULLUP);
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);

  pinMode(PIN_TENA_1, OUTPUT);
  pinMode(PIN_TENA_2, OUTPUT);
  pinMode(PIN_TENA_3, OUTPUT);
  pinMode(PIN_TENA_4, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_TENA_1, LOW);
  digitalWrite(PIN_TENA_2, LOW);
  digitalWrite(PIN_TENA_3, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setBusClock(400000); // ускоряем I2C
  // Если изображение вверх ногами — раскомментируйте:
  // u8g2.setFlipMode(1);
  // Можно подстроить контраст:
  // u8g2.setContrast(200);

  sensors.begin();
  sensors.setWaitForConversion(false);                 // НЕ ждём конверсию
  sensors.setResolution(sensorOutdoor, DS_RES_BITS);   // 11 бит ~ 375мс
  sensors.setResolution(sensorIndoor,  DS_RES_BITS);
  sensors.setResolution(sensorCoolant, DS_RES_BITS);
  dsConvMs = convTimeForBits(DS_RES_BITS);

  loadAll();

  #ifdef ESP32
    randomSeed(esp_random());
  #else
    randomSeed(analogRead(0));
  #endif

  Serial.println("Отопление: запуск");
  digitalWrite(PIN_TENA_4, HIGH);
}

void loop(){
  handleButtons();
  tickBuzzer();

  unsigned long now = millis();

  // Быстрый UI (меню/авария) независимо от датчиков
  if (alarmActive) {
    if (now - lastUiRefresh >= UI_REFRESH_INTERVAL) {
      displayAlarmScreen();
      lastUiRefresh = now;
    }
  } else if (inMenu) {
    if (now - lastUiRefresh >= UI_REFRESH_INTERVAL) {
      displayMenu();
      lastUiRefresh = now;
    }
  }

  // Старт конвертации раз в TEMP_UPDATE_INTERVAL (неблокирующий)
  if ((now - lastTempUpdate >= TEMP_UPDATE_INTERVAL) && !dsConverting){
    lastTempUpdate = now;
    sensors.requestTemperatures();
    dsConverting = true;
    dsConvStart  = now;
  }

  // Когда конвертация завершилась — читаем и выполняем логику
  if (dsConverting && (now - dsConvStart >= dsConvMs)){
    dsConverting = false;

    float tOut = readTemperatureC(sensorOutdoor);
    float tIn  = readTemperatureC(sensorIndoor);
    float tCol = readTemperatureC(sensorCoolant);

    lastRawOut = tOut; lastRawIn = tIn; lastRawCol = tCol;
    tempsEverRead = true;

    // Аварии — на сырых значениях
    checkAlarmsRaw(tOut, tIn, tCol);

    // Сглаживание
    if (!filtersInitialized){
      filtOutdoor=tOut; filtIndoor=tIn; filtCoolant=tCol; filtersInitialized=true;
    } else {
      filtOutdoor=expFilter(filtOutdoor,tOut);
      filtIndoor =expFilter(filtIndoor,tIn);
      filtCoolant=expFilter(filtCoolant,tCol);
    }

    if (!alarmActive){
      if (!isnan(filtIndoor) && !isnan(filtOutdoor)) {
        updateRoomHold(filtIndoor, filtOutdoor);
      }
      if (!isnan(filtIndoor) && !isnan(filtOutdoor) && !isnan(filtCoolant)){
        manageCoolantTargetTemp(filtIndoor, filtOutdoor, filtCoolant);
        manageTenas(filtCoolant);
      }
    } else {
      allTenasOff();
      heatingMode = false;
      coolingMode = false;
    }

    // Обычный экран — по приходу новых данных
    if (!alarmActive && !inMenu) {
      displayNormalMode(tOut, tIn, tCol);
      lastUiRefresh = now;
    }

    // Отладка (можно отключить)
    Serial.print("Out: "); Serial.print(tOut,1); Serial.print ("/");Serial.print(filtOutdoor,2);
    Serial.print(" In: ");  Serial.print(tIn,1); Serial.print ("/");Serial.print(filtIndoor,2);
    Serial.print(" Col: "); Serial.print(tCol,1); Serial.print ("/");Serial.print(filtCoolant,2);
    Serial.print(" | TargetCol: "); Serial.print(targetCoolantTemp,1);
    Serial.print(" | Tenas:"); if (tena1State) Serial.print("1"); if (tena2State) Serial.print("2"); if (tena3State) Serial.print("3");
    Serial.print(" | Mode:"); if (alarmActive) Serial.print("ALRM"); else if (roomHoldActive) Serial.print("HOLD"); else if (heatingMode) Serial.print("HEAT"); else if (coolingMode) Serial.print("COOL"); else Serial.print("IDLE");
    Serial.println();
  }

  // Автовыход из меню через 1 сек после сохранения
  if (showSavedMessage && inMenu && (millis() - savedMessageTime > 1000)) {
    showSavedMessage = false;
    inMenu = false;
    inService = false;
  }

  delay(10);
}
