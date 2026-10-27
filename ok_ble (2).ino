/*
 * ESP32 - Khoa xe DIY phien ban BLE
 * - Mo khoa bang app (RFID tuy chon) + Van tay dieu khien dong co + Web Bluetooth (Bluefy)
 *
 * CU CHI VAN TAY (sau khi da mo khoa):
 *   - Cham 3 lan lien tiep (moi lan cham ngan)  -> KHOA XE
 *   - Cham roi nhan giu (> 0.6 giay)             -> RELAY DE chay trong luc giu (toi da 10 giay)
 *       + Neu da dang ky van tay: chi van tay da dang ky moi duoc de
 *       + Neu chua dang ky van tay nao: cham gi cung de duoc (che do du phong)
 *       + Chan chong dang ha -> khong de
 *
 * DANG KY VAN TAY: quet 5 lan, moi lan quet thanh cong den van tay nhay xanh la + bip
 *
 * Giao thuc BLE (1 service, 3 characteristic):
 *   CMD    (write)  : lenh dang chu "verb" hoac "verb:arg"
 *   STATUS (read)   : JSON trang thai truc tiep
 *   CONFIG (read)   : JSON cai dat + danh sach the RFID + danh sach van tay
 *
 * Lenh moi cho phan van tay:
 *   addfp        : dang ky van tay moi (tu chon ID trong)
 *   addfp:ID     : dang ky vao ID cu the (1..MAX_FP), ghi de neu da co
 *   cancelfp     : huy dang ky dang chay
 *   delfp:ID     : xoa 1 van tay
 *   clrfp        : xoa tat ca van tay
 * STATUS them: "fpAdd" (true/false), "fpStep" (so lan da quet thanh cong)
 * CONFIG them: "fps":[1,2,...]
 *
 * Bao mat: moi lenh (tru "auth") bi bo qua cho den khi web gui "auth:<PIN>" dung.
 * PIN mac dinh: 123456  -> NEN DOI NGAY trong phan Cai dat.
 *
 * Chon Partition Scheme: "Minimal SPIFFS (1.9MB APP with OTA)" hoac "Huge APP"
 */

// 1 = dung module RFID RC522, 0 = bo qua RFID (mo khoa bang nut tren app)
#define USE_RFID 0

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <Adafruit_Fingerprint.h>
#include <SPI.h>
#if USE_RFID
#include <MFRC522.h>
#endif
#include <Preferences.h>
#include <vector>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

#include "esp_sleep.h"
#include "driver/rtc_io.h"

#ifndef FINGERPRINT_LED_RED
  #define FINGERPRINT_LED_RED 0x01
#endif
#ifndef FINGERPRINT_LED_GREEN
  #define FINGERPRINT_LED_GREEN 0x02
#endif
#ifndef FINGERPRINT_LED_BLUE
  #define FINGERPRINT_LED_BLUE 0x04
#endif
#ifndef FINGERPRINT_LED_YELLOW
  #define FINGERPRINT_LED_YELLOW 0x03
#endif
#ifndef FINGERPRINT_LED_BREATHING
  #define FINGERPRINT_LED_BREATHING 0x01
#endif
#ifndef FINGERPRINT_LED_FLASHING
  #define FINGERPRINT_LED_FLASHING 0x02
#endif
#ifndef FINGERPRINT_LED_ON
  #define FINGERPRINT_LED_ON 0x03
#endif
#ifndef FINGERPRINT_LED_OFF
  #define FINGERPRINT_LED_OFF 0x04
#endif

// ============== DINH NGHIA CHAN PHAN CUNG (giu nguyen) ================
#define FINGER_RX        32
#define FINGER_TX        33

#define RFID_SS          5
#define RFID_RST         4
#define SPI_SCK          18
#define SPI_MISO         19
#define SPI_MOSI         23

#define RELAY_STARTER    25
#define RELAY_POWER      26
#define SIDESTAND_PIN    27   // LOW = chan chong dang ha
#define BATTERY_ADC_PIN  34
#define BUZZER_PIN       12
#define ONBOARD_LED      2
#define LED_D2_PIN       ONBOARD_LED

#define RELAY_ON   HIGH
#define RELAY_OFF  LOW

// ============== BLE UUID (phai trung voi trang web) ================
#define SERVICE_UUID "7a1c0001-5b6e-4c1d-9a3e-2f6d8b4c1e01"
#define CMD_UUID     "7a1c0002-5b6e-4c1d-9a3e-2f6d8b4c1e01"
#define STATUS_UUID  "7a1c0003-5b6e-4c1d-9a3e-2f6d8b4c1e01"
#define CONFIG_UUID  "7a1c0004-5b6e-4c1d-9a3e-2f6d8b4c1e01"

#define CMD_LEN        40
#define MAX_RFID_CARDS 12

// ============== NGU NONG (LIGHT SLEEP) ================
// Noi chan TOUCH OUT / WAKEUP cua ZW101 (muc CAO khi co ngon tay cham) vao GPIO nay.
// Phai la chan RTC con trong (13, 14, 15, 35, 36, 39). Mac dinh GPIO13.
#define FP_WAKE_PIN          13
// 1 = sau khi thuc day tu khoi dong lai (BLE len lai sach se, on dinh, mat ~1 giay)
// 0 = chay tiep ngay tai cho va bat lai BLE (nhanh hon nhung co the khong on dinh tuy phien ban core)
#define SLEEP_WAKE_RESTART   1
// RC522 khong danh thuc duoc ESP32 -> tat tu dong ngu khi dung RFID
#define SLEEP_ENABLED        (!USE_RFID)
#define SLEEP_MIN_SEC        30
#define SLEEP_MAX_SEC        3600

// ============== THAM SO VAN TAY ================
#define MAX_FP              20      // so van tay toi da
#define FP_ENROLL_SCANS     5       // so lan quet khi dang ky
#define FP_ENROLL_TIMEOUT   20000UL // ms cho moi lan quet
#define FP_HOLD_START_MS    600     // giu lau hon thoi gian nay -> de
#define FP_HOLD_MAX_MS      10000UL // gioi han an toan: de toi da 10 giay
#define FP_TAP_MIN_MS       40      // cham ngan toi thieu
#define FP_TAP_MAX_MS       500     // cham ngan toi da
#define FP_TAP_GAP_MS       800     // khoang cach toi da giua 2 lan cham
#define FP_TAPS_TO_LOCK     3
#define FP_RELEASE_DEBOUNCE 80      // ms chong nhieu khi nha tay

// ============== DOI TUONG & BIEN TOAN CUC ================
HardwareSerial fingerSerial(2);
Adafruit_Fingerprint finger(&fingerSerial);
#if USE_RFID
MFRC522 rfid(RFID_SS, RFID_RST);
#endif
Preferences prefs;

std::vector<String> validRfidUIDs;
std::vector<uint16_t> fpIds;        // danh sach ID van tay da dang ky (luu trong flash)

bool isUnlocked = false;
bool isEngineRunning = false;
bool isAlarmActive = false;
bool fpConnected = false;
bool rfidOk = true;
String lastStatusMsg = "He thong san sang";

// Cai dat luu trong flash
String bleName = "HQT-KhoaXe";
String blePin = "123456";
uint32_t starterTimeMs = 1500;
uint32_t delaySec = 3;
uint32_t odometerKm = 0;
uint32_t oilLimitKm = 1500;
uint32_t oilLimitDays = 90;
uint32_t lastOilOdo = 0;
String oilDate = "";
String servDate = "";
String airDate = "";

bool isAddingRfid = false;
unsigned long rfidAddTimer = 0;

unsigned long lastFingerLedTime = 0;
bool fingerLedNeedsReset = false;
unsigned long lastDebugPrintTime = 0;
unsigned long lastFingerPressTime = 0;
bool lastFingerState = false;
unsigned long lastRfidCheckTime = 0;

// --- Cu chi van tay ---
unsigned long lastFingerSeen = 0;
unsigned long lastTapRelease = 0;
uint8_t tapCount = 0;
bool holdVerified = false;
bool holdDenied = false;
bool holdStarted = false;

// --- Dang ky van tay ---
bool fpEnrolling = false;
uint8_t fpStep = 0;          // so lan quet da thanh cong
uint8_t fpPhase = 0;         // 0 = cho dat tay, 1 = cho nhac tay ra
uint8_t fpFailCount = 0;
uint16_t fpEnrollId = 0;
unsigned long fpStepTimer = 0;

// --- BLE ---
BLEServer* pServer = nullptr;
BLECharacteristic* chCmd = nullptr;
BLECharacteristic* chStatus = nullptr;
BLECharacteristic* chConfig = nullptr;

volatile bool bleConnected = false;
volatile bool bleAuthed = false;
volatile bool configDirty = true;
volatile unsigned long restartAdvAt = 0;
uint8_t authFails = 0;
unsigned long authLockUntil = 0;
bool lastAuthState = false;
unsigned long lastStatusUpdate = 0;

QueueHandle_t cmdQueue = nullptr;

// --- Tu dong ngu ---
uint32_t sleepSec = 120;          // 0 = tat tu dong ngu
unsigned long lastActivityMs = 0;
uint8_t sleepMode = 1;            // 1 = tiet kiem (van phat BLE), 0 = ngu hang (tat BLE, cham van tay de thuc)
bool ecoActive = false;
uint32_t normalCpuMhz = 240;

// ============== PROTOTYPES ================
void loadAllSettings();
void saveAllSettings();
void setFingerLED(uint8_t control, uint8_t speed, uint8_t color, uint8_t count = 0);
void blinkLedD2(int times, int delayMs = 30);
void beepBuzzer(uint16_t freq, uint16_t duration, uint8_t count = 1);
void startBLE();
void unlockBike();
void lockBike();
void startEngineTimerMode();
void triggerFindBike();
bool isFingerPressed();
void handleRFIDReader();
void handleFingerprintLogic();
void handleFpEnroll();
void startFpEnroll(int reqId);
void cancelFpEnroll(const char *msg);
bool fpHasId(uint16_t id);
bool verifyEnrolledFinger();
#if USE_RFID
String uidToString(MFRC522::Uid &uid);
#endif
void printSystemStatusToSerial();
void resetFingerLED();
void hardResetRFID();
void checkAndResetRfidModule();
float readBatteryVolt();
void processCommand(const String &c);
void updateStatusChar();
void updateConfigChar();
void handleAuth(const String &pin);
String jsonEscape(const String &s);
bool isDateStr(const String &s);
void checkAutoSleep();
void enterLightSleep();
void enterEco();
void exitEco();

// ============== BLE CALLBACKS ================
class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    bleConnected = true;
    bleAuthed = false;
    configDirty = true;
  }
  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    bleAuthed = false;
    restartAdvAt = millis() + 300;   // loop() se bat lai quang ba
  }
};

class CmdCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = String(c->getValue().c_str());
    v.trim();
    if (v.length() == 0 || v.length() >= CMD_LEN) return;

    if (v.startsWith("auth:")) {       // xu ly ngay, khong can cho loop()
      handleAuth(v.substring(5));
      return;
    }
    if (!bleAuthed) return;            // chua nhap PIN dung -> bo qua moi lenh

    char buf[CMD_LEN];
    strncpy(buf, v.c_str(), CMD_LEN - 1);
    buf[CMD_LEN - 1] = 0;
    xQueueSend(cmdQueue, buf, 0);
  }
};

void handleAuth(const String &pin) {
  if (millis() < authLockUntil) return;
  if (pin == blePin) {
    bleAuthed = true;
    authFails = 0;
    configDirty = true;
  } else {
    authFails++;
    if (authFails >= 5) {
      authLockUntil = millis() + 30000UL;   // khoa 30 giay sau 5 lan sai
      authFails = 0;
    }
  }
}

// ============== SETUP / LOOP ================
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(921600);
  delay(200);

  Serial.println("\n\n==========================================");
  Serial.println("[SYSTEM] Khoi dong he thong Khoa xe thong minh HQT v6 (BLE)...");
  Serial.println("==========================================");

  // Dat mode truoc roi moi ghi muc logic, tranh relay bi nhay luc khoi dong
  pinMode(RELAY_POWER, OUTPUT);
  pinMode(RELAY_STARTER, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_D2_PIN, OUTPUT);
  pinMode(SIDESTAND_PIN, INPUT_PULLUP);
  pinMode(FP_WAKE_PIN, INPUT_PULLDOWN);
#if USE_RFID
  pinMode(RFID_RST, OUTPUT);
#endif

  digitalWrite(RELAY_POWER, RELAY_OFF);
  digitalWrite(RELAY_STARTER, RELAY_OFF);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_D2_PIN, LOW);

  cmdQueue = xQueueCreate(8, CMD_LEN);
  lastActivityMs = millis();
  normalCpuMhz = getCpuFrequencyMhz();

  loadAllSettings();

  Serial.println("\n[INIT-FP] Dang ket noi cam bien van tay ZW101...");
  finger.begin(57600);
  delay(50);
  // Mo UART2 sau finger.begin() de chac chan dung dung chan RX/TX da chon
  fingerSerial.begin(57600, SERIAL_8N1, FINGER_RX, FINGER_TX);
  delay(200);

  if (finger.verifyPassword()) {
    fpConnected = true;
    finger.getParameters();   // can de tim kiem van tay (fingerSearch) biet dung luong thu vien
    Serial.println("[HW] OK: Cam bien van tay ZW101 san sang!");
    setFingerLED(FINGERPRINT_LED_FLASHING, 0x05, FINGERPRINT_LED_BLUE, 2);
  } else {
    fpConnected = false;
    Serial.println("[HW-ERR] CANH BAO: Khong the ket noi cam bien van tay ZW101!");
  }

#if USE_RFID
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, RFID_SS);
  delay(20);
  hardResetRFID();
#else
  Serial.println("[RFID] Dang TAT (USE_RFID = 0), mo khoa bang nut tren app");
#endif

  startBLE();
  beepBuzzer(2000, 80, 2);
  Serial.println("[SYSTEM] He thong da san sang!\n");
}

void loop() {
  handleRFIDReader();

  // Thuc thi lenh nhan tu BLE (chay o loop de khong chan tac vu BLE)
  char cbuf[CMD_LEN];
  while (xQueueReceive(cmdQueue, cbuf, 0) == pdTRUE) {
    processCommand(String(cbuf));
    configDirty = true;
  }

  // Quang ba lai sau khi mat ket noi
  if (restartAdvAt != 0 && millis() > restartAdvAt) {
    restartAdvAt = 0;
    BLEDevice::startAdvertising();
  }

  // Het thoi gian cho them the RFID
  if (isAddingRfid && millis() - rfidAddTimer > 15000) {
    isAddingRfid = false;
    lastStatusMsg = "Het thoi gian cho quet the";
  }

  // Cap nhat gia tri cho characteristic (CONFIG truoc, STATUS sau)
  if (bleConnected) {
    bool a = bleAuthed;
    if (a != lastAuthState) { lastAuthState = a; configDirty = true; }
    if (configDirty) { configDirty = false; updateConfigChar(); }
    if (millis() - lastStatusUpdate > 300) {
      lastStatusUpdate = millis();
      updateStatusChar();
    }
  } else {
    lastAuthState = false;
  }

  if (millis() - lastRfidCheckTime > 2000) {
    lastRfidCheckTime = millis();
    checkAndResetRfidModule();
  }

  if (millis() - lastDebugPrintTime > 5000) {
    lastDebugPrintTime = millis();
    printSystemStatusToSerial();
  }

  if (fingerLedNeedsReset && (millis() - lastFingerLedTime > 2000)) {
    resetFingerLED();
    fingerLedNeedsReset = false;
  }

  if (fpEnrolling) handleFpEnroll();
  else handleFingerprintLogic();

  // Co hoat dong -> lam moi dong ho cho ngu
  if (bleConnected || isUnlocked || fpEnrolling || isAddingRfid || lastFingerState ||
      digitalRead(RELAY_STARTER) == RELAY_ON) {
    lastActivityMs = millis();
    if (ecoActive) exitEco();
  }
  checkAutoSleep();

  // Che do tiet kiem: cho CPU nghi giua cac vong lap (van quet van tay moi 50ms)
  if (ecoActive) delay(50);
}

// ============== BLE ================
void startBLE() {
  BLEDevice::init(bleName.c_str());
  BLEDevice::setMTU(185);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCb());

  BLEService* svc = pServer->createService(SERVICE_UUID);

  chCmd = svc->createCharacteristic(CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  chCmd->setCallbacks(new CmdCb());

  chStatus = svc->createCharacteristic(STATUS_UUID, BLECharacteristic::PROPERTY_READ);
  chConfig = svc->createCharacteristic(CONFIG_UUID, BLECharacteristic::PROPERTY_READ);

  String stub = "{\"auth\":false}";
  chStatus->setValue((uint8_t*)stub.c_str(), stub.length());
  chConfig->setValue((uint8_t*)stub.c_str(), stub.length());

  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  Serial.printf("[BLE] Dang quang ba voi ten: %s\n", bleName.c_str());
}

String jsonEscape(const String &s) {
  String o;
  for (size_t i = 0; i < s.length(); i++) {
    char ch = s[i];
    if (ch == '"' || ch == '\\') { o += '\\'; o += ch; }
    else if (ch == '\n' || ch == '\r') o += ' ';
    else o += ch;
  }
  return o;
}

float readBatteryVolt() {
  int rawAdc = analogRead(BATTERY_ADC_PIN);
  float v = (rawAdc / 4095.0) * 3.3 * 4.3;
  if (v < 2.0) v = 12.6;
  return v;
}

void updateStatusChar() {
  String j;
  if (!bleAuthed) {
    long lockLeft = (long)(authLockUntil - millis());
    j = "{\"auth\":false,\"lock\":" + String(lockLeft > 0 ? (lockLeft / 1000 + 1) : 0) + "}";
  } else {
    j = "{\"auth\":true";
    j += ",\"unlocked\":" + String(isUnlocked ? "true" : "false");
    j += ",\"engine\":" + String(isEngineRunning ? "true" : "false");
    j += ",\"starter\":" + String(digitalRead(RELAY_STARTER) == RELAY_ON ? "true" : "false");
    j += ",\"alarm\":" + String(isAlarmActive ? "true" : "false");
    j += ",\"vBatt\":" + String(readBatteryVolt(), 1);
    j += ",\"standDown\":" + String(digitalRead(SIDESTAND_PIN) == LOW ? "true" : "false");
    j += ",\"fp\":" + String(fpConnected ? "true" : "false");
    j += ",\"rfid\":" + String(rfidOk ? "true" : "false");
    j += ",\"adding\":" + String(isAddingRfid ? "true" : "false");
    j += ",\"fpAdd\":" + String(fpEnrolling ? "true" : "false");
    j += ",\"fpStep\":" + String(fpStep);
    j += ",\"msg\":\"" + jsonEscape(lastStatusMsg) + "\"}";
  }
  chStatus->setValue((uint8_t*)j.c_str(), j.length());
}

void updateConfigChar() {
  String j;
  if (!bleAuthed) {
    j = "{\"auth\":false}";
  } else {
    j = "{\"auth\":true";
    j += ",\"name\":\"" + jsonEscape(bleName) + "\"";
    j += ",\"starterMs\":" + String(starterTimeMs);
    j += ",\"delay\":" + String(delaySec);
    j += ",\"sleepSec\":" + String(sleepSec);
    j += ",\"sleepMode\":" + String(sleepMode);
    j += ",\"odo\":" + String(odometerKm);
    j += ",\"oilKm\":" + String(oilLimitKm);
    j += ",\"oilDays\":" + String(oilLimitDays);
    j += ",\"lastOilOdo\":" + String(lastOilOdo);
    j += ",\"oilDate\":\"" + oilDate + "\"";
    j += ",\"servDate\":\"" + servDate + "\"";
    j += ",\"airDate\":\"" + airDate + "\"";
    j += ",\"cards\":[";
    for (size_t i = 0; i < validRfidUIDs.size(); i++) {
      if (i > 0) j += ",";
      j += "\"" + validRfidUIDs[i] + "\"";
    }
    j += "]";
    j += ",\"fps\":[";
    for (size_t i = 0; i < fpIds.size(); i++) {
      if (i > 0) j += ",";
      j += String(fpIds[i]);
    }
    j += "]}";
  }
  chConfig->setValue((uint8_t*)j.c_str(), j.length());
}

bool isDateStr(const String &s) {
  if (s.length() == 0) return true;       // cho phep xoa ngay
  if (s.length() != 10) return false;
  for (int i = 0; i < 10; i++) {
    if (i == 4 || i == 7) { if (s[i] != '-') return false; }
    else if (s[i] < '0' || s[i] > '9') return false;
  }
  return true;
}

// ============== XU LY LENH ================
void processCommand(const String &c) {
  int p = c.indexOf(':');
  String verb = (p < 0) ? c : c.substring(0, p);
  String arg  = (p < 0) ? "" : c.substring(p + 1);

  Serial.printf("[BLE-CMD] %s\n", c.c_str());

  if (verb == "unlock") {
    lastStatusMsg = "Mo khoa xe tu app";
    unlockBike();
  } else if (verb == "lock") {
    lastStatusMsg = "Khoa xe tu app";
    lockBike();
  } else if (verb == "start") {
    startEngineTimerMode();
  } else if (verb == "find") {
    triggerFindBike();
  } else if (verb == "starter") {
    long ms = arg.toInt();
    if (ms < 500) ms = 500;
    if (ms > 10000) ms = 10000;     // tran an toan: toi da 10 giay
    starterTimeMs = ms;
    saveAllSettings();
    lastStatusMsg = "Da luu thoi gian de: " + String(starterTimeMs) + "ms";
  } else if (verb == "delay") {
    long s = arg.toInt();
    if (s < 1) s = 1;
    if (s > 30) s = 30;
    delaySec = s;
    saveAllSettings();
    lastStatusMsg = "Da luu thoi gian cho";
  } else if (verb == "addrfid") {
    if ((int)validRfidUIDs.size() >= MAX_RFID_CARDS) {
      lastStatusMsg = "Da day bo nho the RFID";
    } else {
      isAddingRfid = true;
      rfidAddTimer = millis();
      lastStatusMsg = "Dang cho quet the RFID...";
    }
  } else if (verb == "delrfid") {
    int idx = arg.toInt();
    if (idx >= 0 && idx < (int)validRfidUIDs.size()) {
      validRfidUIDs.erase(validRfidUIDs.begin() + idx);
      saveAllSettings();
      lastStatusMsg = "Da xoa the RFID!";
    }
  } else if (verb == "sleepmode") {
    sleepMode = (arg.toInt() == 0) ? 0 : 1;
    if (ecoActive) exitEco();
    saveAllSettings();
    lastActivityMs = millis();
    lastStatusMsg = sleepMode ? "Che do ngu: Tiet kiem (van phat BLE)" : "Che do ngu: Ngu hang (cham van tay de thuc)";
  } else if (verb == "sleep") {
    long s = arg.toInt();
    if (s <= 0) {
      sleepSec = 0;
      lastStatusMsg = "Da tat tu dong ngu";
    } else {
      if (s < SLEEP_MIN_SEC) s = SLEEP_MIN_SEC;
      if (s > SLEEP_MAX_SEC) s = SLEEP_MAX_SEC;
      sleepSec = s;
      lastStatusMsg = "Se ngu sau " + String(sleepSec) + " giay khong mo khoa";
    }
    saveAllSettings();
    lastActivityMs = millis();
  } else if (verb == "addfp") {
    startFpEnroll(arg.length() ? (int)arg.toInt() : 0);
  } else if (verb == "cancelfp") {
    if (fpEnrolling) cancelFpEnroll("Da huy dang ky van tay");
  } else if (verb == "delfp") {
    int id = arg.toInt();
    if (!fpConnected) {
      lastStatusMsg = "Cam bien van tay chua ket noi";
    } else if (fpEnrolling) {
      lastStatusMsg = "Dang dang ky van tay, hay doi";
    } else if (id >= 1 && id <= MAX_FP) {
      uint8_t r = finger.deleteModel(id);
      // Du module bao loi (ID chua co) van go khoi danh sach ESP32 cho dong bo
      for (size_t i = 0; i < fpIds.size(); i++) {
        if (fpIds[i] == id) { fpIds.erase(fpIds.begin() + i); break; }
      }
      saveAllSettings();
      lastStatusMsg = (r == FINGERPRINT_OK) ? ("Da xoa van tay ID " + String(id))
                                            : ("Da go ID " + String(id) + " (module bao loi " + String(r) + ")");
    }
  } else if (verb == "clrfp") {
    if (!fpConnected) {
      lastStatusMsg = "Cam bien van tay chua ket noi";
    } else if (fpEnrolling) {
      lastStatusMsg = "Dang dang ky van tay, hay doi";
    } else {
      uint8_t r = finger.emptyDatabase();
      fpIds.clear();
      saveAllSettings();
      lastStatusMsg = (r == FINGERPRINT_OK) ? "Da xoa tat ca van tay"
                                            : ("Da xoa danh sach ESP32 (module bao loi " + String(r) + ")");
    }
  } else if (verb == "odo") {
    long v = arg.toInt();
    if (v < 0) v = 0;
    odometerKm = v;
    saveAllSettings();
    lastStatusMsg = "Da cap nhat odometer";
  } else if (verb == "oil") {
    int comma = arg.indexOf(',');
    if (comma > 0) {
      long km = arg.substring(0, comma).toInt();
      long days = arg.substring(comma + 1).toInt();
      if (km > 0) oilLimitKm = km;
      if (days > 0) oilLimitDays = days;
      saveAllSettings();
      lastStatusMsg = "Da luu dinh muc thay dau";
    }
  } else if (verb == "mo") {
    if (isDateStr(arg)) {
      if (arg != oilDate) { oilDate = arg; lastOilOdo = odometerKm; }
      saveAllSettings();
      lastStatusMsg = "Da luu ngay thay dau";
    }
  } else if (verb == "ms") {
    if (isDateStr(arg)) { servDate = arg; saveAllSettings(); lastStatusMsg = "Da luu ngay bao duong"; }
  } else if (verb == "ma") {
    if (isDateStr(arg)) { airDate = arg; saveAllSettings(); lastStatusMsg = "Da luu ngay thay loc gio"; }
  } else if (verb == "pin") {
    bool ok = (arg.length() >= 4 && arg.length() <= 8);
    for (size_t i = 0; ok && i < arg.length(); i++) if (arg[i] < '0' || arg[i] > '9') ok = false;
    if (ok) {
      blePin = arg;
      saveAllSettings();
      lastStatusMsg = "Da doi PIN";
    } else {
      lastStatusMsg = "PIN phai 4-8 chu so";
    }
  } else if (verb == "name") {
    String n = arg;
    n.trim();
    if (n.length() >= 1 && n.length() <= 12) {
      bleName = n;
      saveAllSettings();
      lastStatusMsg = "Da doi ten BLE, dang khoi dong lai...";
      updateStatusChar();
      delay(500);
      ESP.restart();
    }
  }
}

// ============== RFID ================
#if USE_RFID
void hardResetRFID() {
  Serial.println("[RFID] Thuc hien Reset cung (Hardware Reset) module RFID...");
  digitalWrite(RFID_RST, LOW);
  delay(50);
  digitalWrite(RFID_RST, HIGH);
  delay(50);
  rfid.PCD_Init();
  delay(10);
  rfid.PCD_SetAntennaGain(MFRC522::PCD_RxGain::RxGain_max);
}

void checkAndResetRfidModule() {
  byte v = rfid.PCD_ReadRegister(rfid.VersionReg);
  if (v == 0x00 || v == 0xFF) {
    rfidOk = false;
    Serial.println("[RFID-WARN] Phat hien module RFID bi treo/mat ket noi SPI do nhieu!");
    hardResetRFID();
  } else {
    rfidOk = true;
  }
}
#else
void hardResetRFID() {}
void checkAndResetRfidModule() {}
#endif

// ============== RELAY / KHOA ================
void resetFingerLED() {
  if (isUnlocked) {
    setFingerLED(FINGERPRINT_LED_FLASHING, 0x05, FINGERPRINT_LED_GREEN, 1);
  } else {
    setFingerLED(FINGERPRINT_LED_OFF, 0x00, 0x00, 0);
  }
}

void unlockBike() {
  isUnlocked = true;
  tapCount = 0;
  digitalWrite(RELAY_POWER, RELAY_ON);
  digitalWrite(LED_D2_PIN, HIGH);

  Serial.println("[RELAY] RELAY_POWER -> BAT (Mo khoa xe thanh cong)");
  setFingerLED(FINGERPRINT_LED_FLASHING, 0x05, FINGERPRINT_LED_GREEN, 2);
  beepBuzzer(2500, 100, 2);
}

void lockBike() {
  isUnlocked = false;
  isEngineRunning = false;
  tapCount = 0;
  digitalWrite(RELAY_STARTER, RELAY_OFF);
  digitalWrite(RELAY_POWER, RELAY_OFF);
  digitalWrite(LED_D2_PIN, LOW);

  Serial.println("[RELAY] RELAY_POWER & RELAY_STARTER -> TAT (Da khoa/tat may xe)");
  setFingerLED(FINGERPRINT_LED_FLASHING, 0x05, FINGERPRINT_LED_RED, 2);
  beepBuzzer(1000, 150, 1);

  delay(100);
  hardResetRFID();
}

void startEngineTimerMode() {
  if (!isUnlocked) {
    lastStatusMsg = "Chua mo khoa, khong the de!";
    return;
  }

  if (digitalRead(SIDESTAND_PIN) == LOW) {
    lastStatusMsg = "Canh bao: Chan chong dang ha!";
    beepBuzzer(3000, 100, 3);
    return;
  }

  Serial.println("[RELAY] Kich RELAY_STARTER qua app BLE...");
  digitalWrite(RELAY_STARTER, RELAY_ON);
  isEngineRunning = true;
  delay(starterTimeMs);
  digitalWrite(RELAY_STARTER, RELAY_OFF);
  isEngineRunning = false;
  lastStatusMsg = "De no qua app thanh cong!";
  beepBuzzer(2000, 80, 1);

  delay(100);
  hardResetRFID();
}

void triggerFindBike() {
  isAlarmActive = true;
  for (int i = 0; i < 4; i++) {
    beepBuzzer(2800, 80, 1);
    setFingerLED(FINGERPRINT_LED_FLASHING, 0x02, FINGERPRINT_LED_BLUE, 1);
    digitalWrite(LED_D2_PIN, !digitalRead(LED_D2_PIN));
    delay(80);
  }
  isAlarmActive = false;
  digitalWrite(LED_D2_PIN, isUnlocked ? HIGH : LOW);
  if (!isUnlocked) setFingerLED(FINGERPRINT_LED_OFF, 0x00, 0x00, 0);
}

void blinkLedD2(int times, int delayMs) {
  for (int i = 0; i < times; i++) {
    digitalWrite(LED_D2_PIN, HIGH);
    delay(delayMs);
    digitalWrite(LED_D2_PIN, LOW);
    delay(delayMs);
  }
  if (isUnlocked) digitalWrite(LED_D2_PIN, HIGH);
}

bool isFingerPressed() {
  uint8_t p = finger.getImage();
  return (p == FINGERPRINT_OK);
}

#if USE_RFID
String uidToString(MFRC522::Uid &uid) {
  String s = "";
  for (byte i = 0; i < uid.size; i++) {
    if (uid.uidByte[i] < 0x10) s += "0";
    s += String(uid.uidByte[i], HEX);
  }
  s.toUpperCase();
  return s;
}

void handleRFIDReader() {
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial()) return;

  String scannedUID = uidToString(rfid.uid);
  Serial.printf("[RFID] Quet duoc the UID: %s\n", scannedUID.c_str());

  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();

  if (isAddingRfid) {
    bool exists = false;
    for (const String &existingUid : validRfidUIDs) {
      if (existingUid.equalsIgnoreCase(scannedUID)) { exists = true; break; }
    }

    if (!exists) {
      validRfidUIDs.push_back(scannedUID);
      saveAllSettings();
      configDirty = true;
      lastStatusMsg = "Da luu the moi UID: " + scannedUID;
      blinkLedD2(3, 30);
      beepBuzzer(2500, 80, 2);
    } else {
      lastStatusMsg = "The nay da ton tai!";
      blinkLedD2(2, 60);
      beepBuzzer(1500, 100, 2);
    }
    isAddingRfid = false;
    return;
  }

  bool matched = false;
  for (const String &uid : validRfidUIDs) {
    if (uid.equalsIgnoreCase(scannedUID)) { matched = true; break; }
  }

  if (matched) {
    Serial.printf("[RFID] The RFID hop le: %s\n", scannedUID.c_str());
    if (!isUnlocked) {
      lastStatusMsg = "Mo khoa xe bang the UID: " + scannedUID;
      unlockBike();
    } else {
      lastStatusMsg = "Khoa/Tat may xe bang the UID: " + scannedUID;
      lockBike();
    }
  } else {
    Serial.printf("[RFID-ERR] The RFID khong hop le: %s\n", scannedUID.c_str());
    lastStatusMsg = "Canh bao: The RFID khong hop le!";
    blinkLedD2(2, 50);
    beepBuzzer(1000, 100, 2);
  }
}

#else
void handleRFIDReader() {}
#endif

// ============== VAN TAY: KIEM TRA ID ================
bool fpHasId(uint16_t id) {
  for (size_t i = 0; i < fpIds.size(); i++) {
    if (fpIds[i] == id) return true;
  }
  return false;
}

// Chi tra ve true khi module bao khop that su: ma tra ve OK, do tin cay > 0 va ID co trong danh sach ESP32.
// Neu chua dang ky van tay nao -> cho phep (che do du phong, giong hanh vi cu).
bool verifyEnrolledFinger() {
  if (fpIds.empty()) return true;
  if (finger.image2Tz(1) != FINGERPRINT_OK) return false;

  finger.fingerID = 0xFFFF;
  finger.confidence = 0;
  uint8_t p = finger.fingerSearch();   // neu bao loi bien dich khong co ham nay, doi thanh fingerFastSearch()
  Serial.printf("[FP] Search ma=%d ID=%d tin cay=%d\n", p, finger.fingerID, finger.confidence);
  return (p == FINGERPRINT_OK && finger.confidence > 0 && fpHasId(finger.fingerID));
}

// ============== VAN TAY: CU CHI (cham 3 lan = khoa, giu = de) ================
void handleFingerprintLogic() {
  unsigned long now = millis();
  bool raw = isFingerPressed();
  if (raw) lastFingerSeen = millis();

  // Chong nhieu: coi la da nha tay chi khi mat tin hieu lien tuc > FP_RELEASE_DEBOUNCE ms
  bool pressed = raw || (lastFingerState && (now - lastFingerSeen < FP_RELEASE_DEBOUNCE));

  // Het cua so cham -> reset bo dem
  if (tapCount > 0 && !pressed && (now - lastTapRelease > FP_TAP_GAP_MS)) {
    tapCount = 0;
  }

  // ---- Bat dau cham ----
  if (pressed && !lastFingerState) {
    lastFingerPressTime = now;
    holdVerified = false;
    holdDenied = false;
    holdStarted = false;
    if (!isUnlocked) {
      lastStatusMsg = "Xe dang khoa, hay mo khoa truoc!";
    }
  }

  // ---- Dang giu ----
  if (isUnlocked && pressed) {
    unsigned long holdDuration = now - lastFingerPressTime;

    if (holdDuration > FP_HOLD_START_MS && !holdDenied) {
      holdStarted = true;
      tapCount = 0;   // da giu thi khong tinh la cham

      if (digitalRead(SIDESTAND_PIN) == LOW) {
        lastStatusMsg = "Canh bao: Chan chong dang ha!";
        digitalWrite(RELAY_STARTER, RELAY_OFF);
        isEngineRunning = false;
      } else if (holdDuration > FP_HOLD_MAX_MS) {
        // Gioi han an toan: khong de qua 10 giay lien tuc
        digitalWrite(RELAY_STARTER, RELAY_OFF);
        isEngineRunning = false;
        holdDenied = true;
        lastStatusMsg = "De qua lau, da ngat relay de";
        beepBuzzer(3000, 100, 2);
      } else {
        if (!holdVerified) {
          // Xac thuc van tay mot lan cho moi lan giu
          if (verifyEnrolledFinger()) {
            holdVerified = true;
          } else {
            holdDenied = true;
            lastStatusMsg = "Van tay khong hop le, khong de";
            beepBuzzer(1000, 100, 2);
          }
          lastFingerSeen = millis();   // viec tim kiem mat thoi gian, tranh bi nhan nham la nha tay
        }
        if (holdVerified) {
          digitalWrite(RELAY_STARTER, RELAY_ON);
          isEngineRunning = true;
          lastStatusMsg = "Dang giu van tay -> De no...";

          setFingerLED(FINGERPRINT_LED_FLASHING, 0x05, FINGERPRINT_LED_GREEN, 1);
          lastFingerLedTime = millis();
          fingerLedNeedsReset = true;
        }
      }
    }
  }

  // ---- Nha tay ----
  if (!pressed && lastFingerState) {
    unsigned long holdDuration = now - lastFingerPressTime;

    if (isUnlocked) {
      digitalWrite(RELAY_STARTER, RELAY_OFF);
      isEngineRunning = false;

      if (!holdStarted && holdDuration >= FP_TAP_MIN_MS && holdDuration <= FP_TAP_MAX_MS) {
        tapCount++;
        lastTapRelease = now;
        Serial.printf("[FP] Cham %d/%d\n", tapCount, FP_TAPS_TO_LOCK);

        if (tapCount >= FP_TAPS_TO_LOCK) {
          tapCount = 0;
          lockBike();
          lastStatusMsg = "Cham 3 lan -> Da khoa xe";
        } else {
          lastStatusMsg = "Cham " + String(tapCount) + "/" + String(FP_TAPS_TO_LOCK) + " de khoa xe";
          beepBuzzer(2200, 25, 1);
        }
      } else {
        tapCount = 0;
        if (holdStarted && holdVerified && !holdDenied) {
          lastStatusMsg = "Da nha van tay, ngat de";
        }
      }
    }
    holdVerified = false;
    holdStarted = false;
  }

  lastFingerState = pressed;
}

// ============== VAN TAY: DANG KY 5 LAN ================
void startFpEnroll(int reqId) {
  if (!fpConnected) {
    lastStatusMsg = "Cam bien van tay chua ket noi";
    return;
  }
  if (fpEnrolling) {
    lastStatusMsg = "Dang dang ky van tay roi";
    return;
  }

  int id = reqId;
  if (id < 1 || id > MAX_FP) {
    id = 0;
    for (int i = 1; i <= MAX_FP; i++) {
      if (!fpHasId(i)) { id = i; break; }
    }
  }
  if (id == 0) {
    lastStatusMsg = "Da day bo nho van tay";
    return;
  }

  // Dam bao relay de da tat truoc khi dung cam bien
  digitalWrite(RELAY_STARTER, RELAY_OFF);
  isEngineRunning = false;
  tapCount = 0;

  fpEnrollId = id;
  fpStep = 0;
  fpPhase = 0;
  fpFailCount = 0;
  fpStepTimer = millis();
  fpEnrolling = true;
  lastStatusMsg = "Dang ky ID " + String(id) + ": dat ngon tay lan 1/" + String(FP_ENROLL_SCANS);
  Serial.printf("[FP] Bat dau dang ky ID %d\n", id);
  beepBuzzer(2000, 60, 1);
}

void cancelFpEnroll(const char *msg) {
  fpEnrolling = false;
  fpStep = 0;
  lastStatusMsg = msg;
  lastFingerState = false;
  beepBuzzer(1000, 100, 2);
  resetFingerLED();
}

void handleFpEnroll() {
  if (!fpEnrolling) return;

  if (millis() - fpStepTimer > FP_ENROLL_TIMEOUT) {
    cancelFpEnroll("Het thoi gian, huy dang ky van tay");
    return;
  }

  uint8_t p = finger.getImage();

  // Pha 1: cho nguoi dung nhac tay ra truoc khi quet lan ke tiep
  if (fpPhase == 1) {
    if (p == FINGERPRINT_NOFINGER) {
      fpPhase = 0;
      fpStepTimer = millis();
    }
    return;
  }

  // Pha 0: cho dat tay
  if (p != FINGERPRINT_OK) return;

  uint8_t slot = (fpStep == 0) ? 1 : 2;
  bool ok = true;
  const char *err = "";

  if (finger.image2Tz(slot) != FINGERPRINT_OK) {
    ok = false;
    err = "Anh kem, nhac tay ra va dat lai";
  } else if (fpStep > 0 && finger.createModel() != FINGERPRINT_OK) {
    // Lan quet nay khong khop voi lan dau -> khong tinh
    ok = false;
    err = "Khong khop lan dau, dat dung ngon cu";
  }

  if (!ok) {
    fpFailCount++;
    beepBuzzer(1000, 80, 2);
    if (fpFailCount >= 4) {
      cancelFpEnroll("Dang ky that bai, thu lai tu dau");
      return;
    }
    lastStatusMsg = String(err) + " (lan " + String(fpStep + 1) + "/" + String(FP_ENROLL_SCANS) + ")";
    fpPhase = 1;
    fpStepTimer = millis();
    return;
  }

  // Quet thanh cong -> nhay xanh la + bip
  fpStep++;
  fpFailCount = 0;
  setFingerLED(FINGERPRINT_LED_FLASHING, 0x05, FINGERPRINT_LED_GREEN, 1);
  beepBuzzer(2500, 60, 1);
  Serial.printf("[FP] Quet thanh cong %d/%d\n", fpStep, FP_ENROLL_SCANS);

  if (fpStep >= FP_ENROLL_SCANS) {
    // Du 5 lan -> luu mo hinh vao module
    uint8_t r = finger.storeModel(fpEnrollId);
    fpEnrolling = false;
    if (r == FINGERPRINT_OK) {
      if (!fpHasId(fpEnrollId)) fpIds.push_back(fpEnrollId);
      saveAllSettings();
      configDirty = true;
      lastStatusMsg = "Da luu van tay ID " + String(fpEnrollId);
      setFingerLED(FINGERPRINT_LED_FLASHING, 0x05, FINGERPRINT_LED_GREEN, 3);
      beepBuzzer(2500, 80, 3);
    } else {
      lastStatusMsg = "Luu van tay that bai (ma " + String(r) + ")";
      beepBuzzer(1000, 150, 2);
    }
    lastFingerState = false;
    fpStep = 0;
    return;
  }

  lastStatusMsg = "Tot! Nhac tay ra, quet lan " + String(fpStep + 1) + "/" + String(FP_ENROLL_SCANS);
  fpPhase = 1;
  fpStepTimer = millis();
}

// ============== NGU NONG (LIGHT SLEEP) ================
void checkAutoSleep() {
  if (sleepSec == 0) {
    if (ecoActive) exitEco();
    return;
  }
  if (isUnlocked || bleConnected || fpEnrolling || isAddingRfid) return;
  if (millis() - lastActivityMs < sleepSec * 1000UL) return;

  // Khi dung RFID thi khong ngu hang duoc -> tu dong dung che do tiet kiem
  bool useEco = (sleepMode == 1) || !SLEEP_ENABLED;
  if (useEco) {
    if (!ecoActive) enterEco();
    return;
  }

#if SLEEP_ENABLED
  // Dang co tay tren cam bien -> khong ngu (se thuc ngay lap tuc)
  if (digitalRead(FP_WAKE_PIN) == HIGH) {
    lastActivityMs = millis();
    return;
  }
  enterLightSleep();
#endif
}

// ---- Che do tiet kiem: ha xung CPU 80MHz + phat BLE thua (~1 giay/lan), BLE van hien ----
static void setAdvInterval(bool slow) {
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  if (!bleConnected) adv->stop();
  adv->setMinInterval(slow ? 0x640 : 0x20);   // don vi 0.625ms: 0x640 = 1000ms, 0x20 = 20ms
  adv->setMaxInterval(slow ? 0x7D0 : 0x40);   // 0x7D0 = 1250ms, 0x40 = 40ms
  if (!bleConnected) BLEDevice::startAdvertising();
}

void enterEco() {
  if (ecoActive) return;
  ecoActive = true;
  setCpuFrequencyMhz(80);   // BLE can toi thieu 80MHz
  setAdvInterval(true);
  Serial.println("[ECO] Vao che do tiet kiem (CPU 80MHz, BLE phat thua)");
}

void exitEco() {
  if (!ecoActive) return;
  ecoActive = false;
  setCpuFrequencyMhz(normalCpuMhz);
  setAdvInterval(false);
  Serial.println("[ECO] Thoat che do tiet kiem");
}

void enterLightSleep() {
#if SLEEP_ENABLED
  Serial.println("[SLEEP] Vao ngu nong. Cham van tay de danh thuc.");
  Serial.flush();

  // An toan: xe dang khoa, dam bao 2 relay tat
  digitalWrite(RELAY_STARTER, RELAY_OFF);
  digitalWrite(RELAY_POWER, RELAY_OFF);
  digitalWrite(LED_D2_PIN, LOW);
  setFingerLED(FINGERPRINT_LED_OFF, 0x00, 0x00, 0);
  delay(20);

  // Tat BLE truoc khi ngu (radio khong chay duoc trong light sleep thu cong)
  BLEDevice::getAdvertising()->stop();
  BLEDevice::deinit(false);
  pServer = nullptr; chCmd = nullptr; chStatus = nullptr; chConfig = nullptr;
  bleConnected = false;
  bleAuthed = false;

  // Danh thuc khi chan TOUCH OUT len muc CAO
  if (esp_sleep_enable_ext0_wakeup((gpio_num_t)FP_WAKE_PIN, 1) != ESP_OK) {
    Serial.println("[SLEEP] Loi: chan danh thuc khong dung (can chan RTC). Huy ngu.");
#if SLEEP_WAKE_RESTART
    ESP.restart();
#else
    startBLE();
    lastActivityMs = millis();
    return;
#endif
  }
  rtc_gpio_pullup_dis((gpio_num_t)FP_WAKE_PIN);
  rtc_gpio_pulldown_en((gpio_num_t)FP_WAKE_PIN);

  esp_light_sleep_start();

  // ---- DA THUC DAY ----
#if SLEEP_WAKE_RESTART
  ESP.restart();
#else
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_EXT0);
  rtc_gpio_pulldown_dis((gpio_num_t)FP_WAKE_PIN);
  rtc_gpio_deinit((gpio_num_t)FP_WAKE_PIN);
  pinMode(FP_WAKE_PIN, INPUT_PULLDOWN);

  while (fingerSerial.available()) fingerSerial.read();
  startBLE();
  lastActivityMs = millis();
  lastFingerState = false;
  configDirty = true;
  lastStatusMsg = "Da thuc day (cham van tay)";
  beepBuzzer(2000, 40, 1);
  setFingerLED(FINGERPRINT_LED_FLASHING, 0x05, FINGERPRINT_LED_BLUE, 1);
  Serial.println("[SLEEP] Da thuc day.");
#endif
#endif
}

// ============== TIEN ICH ================
void setFingerLED(uint8_t control, uint8_t speed, uint8_t color, uint8_t count) {
  finger.LEDcontrol(control, speed, color, count);
}

void beepBuzzer(uint16_t freq, uint16_t duration, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    tone(BUZZER_PIN, freq, duration);
    delay(duration + 10);
  }
  noTone(BUZZER_PIN);
}

void printSystemStatusToSerial() {
  Serial.println("\n==================== [ REAL-TIME MONITOR ] ====================");
  Serial.printf("Trang thai: %s | Dong co: %s\n", isUnlocked ? "MO KHOA" : "DA KHOA", isEngineRunning ? "DANG DE" : "TAT");
  Serial.printf("Dien ap: %.1fV\n", readBatteryVolt());
  Serial.printf("RFID: %s | ZW101: %s | Van tay da dang ky: %d | BLE: %s%s\n", rfidOk ? "OK" : "ERR", fpConnected ? "OK" : "ERR",
                (int)fpIds.size(), bleConnected ? "da ket noi" : "dang quang ba", bleAuthed ? " (da xac thuc)" : "");
  Serial.printf("Thong diep: %s\n", lastStatusMsg.c_str());
  Serial.println("===============================================================\n");
}

// ============== LUU / DOC CAI DAT ================
void loadAllSettings() {
  prefs.begin("bike_cfg", true);
  bleName = prefs.getString("bleName", "HQT-KhoaXe");
  blePin = prefs.getString("blePin", "123456");
  starterTimeMs = prefs.getUInt("starterTime", 1500);
  delaySec = prefs.getUInt("delaySec", 3);
  sleepSec = prefs.getUInt("sleepSec", 120);
  sleepMode = prefs.getUChar("sleepMode", 1);
  odometerKm = prefs.getUInt("odo", 0);
  oilLimitKm = prefs.getUInt("oilKm", 1500);
  oilLimitDays = prefs.getUInt("oilDays", 90);
  lastOilOdo = prefs.getUInt("lastOilOdo", 0);
  oilDate = prefs.getString("oilDate", "");
  servDate = prefs.getString("servDate", "");
  airDate = prefs.getString("airDate", "");

  String rfidCsv = prefs.getString("rfidList", "");
  validRfidUIDs.clear();
  int start = 0;
  while (start < (int)rfidCsv.length()) {
    int comma = rfidCsv.indexOf(',', start);
    if (comma == -1) comma = rfidCsv.length();
    String val = rfidCsv.substring(start, comma);
    if (val.length() > 0) validRfidUIDs.push_back(val);
    start = comma + 1;
  }

  String fpCsv = prefs.getString("fpList", "");
  fpIds.clear();
  start = 0;
  while (start < (int)fpCsv.length()) {
    int comma = fpCsv.indexOf(',', start);
    if (comma == -1) comma = fpCsv.length();
    int v = fpCsv.substring(start, comma).toInt();
    if (v >= 1 && v <= MAX_FP) fpIds.push_back((uint16_t)v);
    start = comma + 1;
  }
  prefs.end();
}

void saveAllSettings() {
  prefs.begin("bike_cfg", false);
  prefs.putString("bleName", bleName);
  prefs.putString("blePin", blePin);
  prefs.putUInt("starterTime", starterTimeMs);
  prefs.putUInt("delaySec", delaySec);
  prefs.putUInt("sleepSec", sleepSec);
  prefs.putUChar("sleepMode", sleepMode);
  prefs.putUInt("odo", odometerKm);
  prefs.putUInt("oilKm", oilLimitKm);
  prefs.putUInt("oilDays", oilLimitDays);
  prefs.putUInt("lastOilOdo", lastOilOdo);
  prefs.putString("oilDate", oilDate);
  prefs.putString("servDate", servDate);
  prefs.putString("airDate", airDate);

  String rfidCsv = "";
  for (size_t i = 0; i < validRfidUIDs.size(); i++) {
    if (i > 0) rfidCsv += ",";
    rfidCsv += validRfidUIDs[i];
  }
  prefs.putString("rfidList", rfidCsv);

  String fpCsv = "";
  for (size_t i = 0; i < fpIds.size(); i++) {
    if (i > 0) fpCsv += ",";
    fpCsv += String(fpIds[i]);
  }
  prefs.putString("fpList", fpCsv);
  prefs.end();
}
