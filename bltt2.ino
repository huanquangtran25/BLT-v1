/*
 * ESP32 - Khoa xe DIY phien ban BLE (RFID: PN532)
 * - Mo khoa bang app (RFID tuy chon) + Van tay dieu khien dong co + Web Bluetooth (Bluefy)
 *
 * CU CHI VAN TAY (moi lan cham deu doi chieu van tay, den XANH LA = dung, DO = sai):
 *   - Xe dang KHOA: cham dung van tay da dang ky       -> MO KHOA
 *   - Xe da mo khoa: cham ngan 3 lan lien tiep          -> KHOA XE
 *   - Xe da mo khoa: cham roi nhan giu (> holdMs, mac dinh 0.6 giay, chinh tren web 0.4-3 giay) -> RELAY DE chay trong luc giu (toi da 10 giay)
 *     Cham ngan (< holdMs) chi tinh la 1 lan cham de khoa, khong bao gio kich de
 *       + Chi van tay da dang ky moi duoc de / khoa
 *       + Chua dang ky van tay nao (xe mo khoa tu app): cham gi cung duoc, den xanh duong
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
 * CAP NHAT FIRMWARE QUA WIFI (OTA, khong can may tinh):
 *   wifi:SSID|MATKHAU : luu WiFi (hotspot dien thoai hoac WiFi nha)
 *   fwurl:URL         : luu link file firmware.bin tren web cua ban
 *   ota               : ESP32 ngat BLE, noi WiFi, tai firmware.bin ve, ghi vao flash, khoi dong lai
 *   (chi cho phep khi xe dang KHOA)
 * CONFIG them: "fw" (so phien ban), "wifi" (ten WiFi da luu), "fwUrl"
 *
 * Bao mat: moi lenh (tru "auth") bi bo qua cho den khi web gui "auth:<PIN>" dung.
 * PIN mac dinh: 123456  -> NEN DOI NGAY trong phan Cai dat.
 *
 * Chon Partition Scheme: "Minimal SPIFFS (1.9MB APP with OTA)" hoac "Huge APP"
 */

// 1 = dung module RFID/NFC PN532, 0 = bo qua RFID (mo khoa bang nut tren app)
#define USE_RFID 1

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <Adafruit_Fingerprint.h>
#include <SPI.h>
#if USE_RFID
#include <Wire.h>
#include <Adafruit_PN532.h>     // cai "Adafruit PN532" trong Library Manager (keo theo Adafruit BusIO)
#endif
#include <Preferences.h>
#include <vector>
#include <algorithm>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>

// --- OTA qua WiFi ---
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

// So phien ban firmware. MOI LAN BUILD BAN MOI, TANG SO NAY (va tang so trong version.txt tren web)
#define FP_POLL_SLEEP_MS     250
#define FW_VERSION 1

#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "driver/rtc_io.h"

// ============== MAU DEN LED CAM BIEN VAN TAY ==============
// ZW101 dung ma mau theo bit: 1 = xanh duong, 2 = xanh la, 4 = do (3 = lo, 5 = tim, 6 = vang, 7 = trang).
// Neu den len sai mau: bam "Thu mau den van tay" trong Cai dat, xem ma nao ra mau nao roi sua 3 dong duoi.
#define LEDC_BLUE   1
#define LEDC_GREEN  2
#define LEDC_RED    4
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

#define RFID_SS          5   // SS cua PN532 (SPI chuan, da xac nhan chay)
#define RFID_RST         4
#define SPI_SCK          18   // SPI chuan cua ESP32 (VSPI)
#define SPI_MISO         19
#define SPI_MOSI         23

// ---- PN532 ----
// 1 = noi bang SPI (dung chan RFID_SS/SPI_* o tren), 0 = noi bang I2C (khuyen dung, it day nhat)
// Cong tat DIP tren module: I2C = SW1 ON, SW2 OFF | SPI = SW1 OFF, SW2 ON (xem bang in tren module)
#define PN532_USE_SPI    1
// I2C dung chan chuan cua ESP32 (D21, D22), nam o canh phai board, xa chan van tay (D32, D33 o canh trai).
// SPI (PN532_USE_SPI 1) dung D18 (SCK), D19 (MISO), D23 (MOSI), D5 (SS). IRQ va RSTO cua PN532 de trong.
// => relay de da doi sang D21, coi sang D22 de khong trung chan SPI. (RSTO cua PN532 neu noi thi KHONG noi vao D21.)
#define PN532_SDA        21     // I2C: SDA -> D21, SCL -> D22
#define PN532_SCL        22
#define PN532_IRQ        18     // IRQ -> D18 (nen noi, thu vien dung de biet module san sang)
#define PN532_RESET      15     // KHONG NOI day RSTO (RSTO la chan ra cua PN532). D15 de trong, chi la chan gia cho thu vien.

#define RELAY_STARTER    21   // (khong dung D19 vi la MISO SPI; khong dung GPIO16/17 vi UART2 mac dinh cua thu vien van tay)
#define RELAY_POWER      26
#define SIDESTAND_PIN    27   // LOW = chan chong dang ha
#define BATTERY_ADC_PIN  34
#define BUZZER_PIN       22   // doi khoi GPIO12 (strapping) va D23 (MOSI SPI)
#define ONBOARD_LED      2

// ---- COI ----
// 1 = coi CHU DONG (active, cap dien la tu keu, 1 cao do co dinh) - loai "buzzer" 5V pho bien
// 0 = coi THU DONG (passive, can xung tan so de keu)
// Thu: noi coi truc tiep vao 5V + GND, neu tu keu lien tuc la loai chu dong.
#define BUZZER_ACTIVE    1
// Muc tin hieu lam coi keu: HIGH (coi/transistor thuong) hoac LOW (module coi kich muc thap)
#define BUZZER_ON_LEVEL  HIGH
static inline void buzzOn(uint16_t freq) {
#if BUZZER_ACTIVE
  (void)freq;
  digitalWrite(BUZZER_PIN, BUZZER_ON_LEVEL);
#else
  tone(BUZZER_PIN, freq);
#endif
}
static inline void buzzOff() {
#if BUZZER_ACTIVE
  digitalWrite(BUZZER_PIN, !BUZZER_ON_LEVEL);
#else
  noTone(BUZZER_PIN);
#endif
}
#define LED_D2_PIN       ONBOARD_LED

#define RELAY_ON   HIGH
#define RELAY_OFF  LOW

// ============== BLE UUID (phai trung voi trang web) ================
#define SERVICE_UUID "7a1c0001-5b6e-4c1d-9a3e-2f6d8b4c1e01"
#define CMD_UUID     "7a1c0002-5b6e-4c1d-9a3e-2f6d8b4c1e01"
#define STATUS_UUID  "7a1c0003-5b6e-4c1d-9a3e-2f6d8b4c1e01"
#define CONFIG_UUID  "7a1c0004-5b6e-4c1d-9a3e-2f6d8b4c1e01"

#define CMD_LEN        160   // du cho lenh wifi:SSID|PASS va fwurl:URL
#define MAX_RFID_CARDS 12


// ============== NGU NONG (LIGHT SLEEP) ================
// Noi chan TOUCH OUT / WAKEUP cua ZW101 (muc CAO khi co ngon tay cham) vao GPIO nay.
// Phai la chan RTC con trong (13, 14, 15, 35, 36, 39). Mac dinh GPIO13.
#define FP_WAKE_PIN          13
// 1 = sau khi thuc day tu khoi dong lai (BLE len lai sach se, on dinh, mat ~1 giay)
// 0 = chay tiep ngay tai cho va bat lai BLE (nhanh hon nhung co the khong on dinh tuy phien ban core)
#define SLEEP_WAKE_RESTART   1
// Ngu hang (light sleep) luon bat. Khi ngu (ca tiet kiem lan ngu hang) RFID bi VO HIEU HOA (PN532 giu o trang thai reset);
// cham van tay se khoi dong lai RFID.
#define SLEEP_ENABLED        1
#define SLEEP_MIN_SEC        10
// 1 = keu coi 3 tieng khi vao ngu (ca che do Tiet kiem lan Ngu hang), 0 = tat
#define SLEEP_BEEP           1
#define SLEEP_MAX_SEC        100

// ============== THAM SO VAN TAY ================
#define MAX_FP              20      // so van tay toi da
#define FP_ENROLL_SCANS     5       // so lan quet khi dang ky
#define FP_ENROLL_TIMEOUT   20000UL // ms cho moi lan quet
#define FP_HOLD_START_MS    600     // MAC DINH: giu lau hon thoi gian nay -> de (chinh duoc tren web)
#define FP_HOLD_MIN_MS      400     // gioi han duoi (tranh cham 3 lan bi nham thanh de)
#define FP_HOLD_MAX_CFG_MS  3000    // gioi han tren cua cai dat
#define FP_HOLD_MAX_MS      10000UL // gioi han an toan: de toi da 10 giay
#define FP_TAP_MIN_MS       30      // cham ngan toi thieu
#define FP_TAP_GAP_MS       1000    // khoang cach toi da giua 2 lan cham (mac dinh)
#define FP_TAPS_TO_LOCK     3
#define FP_TAP_GAP_MIN_MS   100     // gioi han cai dat: khoang cach giua 2 lan cham
#define FP_TAP_GAP_MAX_MS   1000
#define FP_RELEASE_DEBOUNCE 150     // ms mat tin hieu (NOFINGER) moi coi la nha tay
#define FP_ERR_GRACE        400     // ms bo qua loi doc tam thoi khi dang giu tay

// ============== DOI TUONG & BIEN TOAN CUC ================
HardwareSerial fingerSerial(2);
Adafruit_Fingerprint finger(&fingerSerial);
#if USE_RFID
#if PN532_USE_SPI
Adafruit_PN532 nfc(RFID_SS, &SPI);
#else
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET, &Wire);
#endif
#endif
Preferences prefs;

std::vector<String> validRfidUIDs;
std::vector<uint16_t> fpIds;        // danh sach ID van tay da dang ky (luu trong flash)

bool isUnlocked = false;
bool isEngineRunning = false;
bool isAlarmActive = false;
bool fpConnected = false;
bool rfidOk = true;
bool rfidSleeping = false;       // RFID dang bi tat khi vao che do ngu
bool rfidWakePending = false;    // can khoi dong lai RFID ngay sau khi nha tay khoi van tay
String lastStatusMsg = "He thong san sang";

// OTA qua WiFi (luu trong flash)
String wifiSsid = "";
String wifiPass = "";
String wifiListJson = "";            // ket qua quet WiFi (JSON mang)
unsigned long wifiListUntil = 0;       // gui danh sach WiFi qua CONFIG den thoi diem nay
String fwUrl = "";          // vd https://tenmiencuaban.com/firmware/firmware.bin

// Cai dat luu trong flash
String bleName = "HQT-KhoaXe";
String blePin = "123456";
uint32_t starterTimeMs = 1500;
// Nhip dieu bao TIM XE (chinh tren web): chon 1 trong cac mau nhip + so lan lap lai
// Moi mau la day thoi gian (ms): keu, nghi, keu, nghi, ... (bat dau va ket thuc bang "keu")
// THU TU PHAI TRUNG VOI DANH SACH TREN TRANG WEB (index.html)
static const uint16_t FIND_P0[] = {150};   // Bíp ngắn
static const uint16_t FIND_P1[] = {150,100,150};   // Bíp bíp
static const uint16_t FIND_P2[] = {150,100,150,100,150};   // Bíp bíp bíp
static const uint16_t FIND_P3[] = {800};   // Bíp dài
static const uint16_t FIND_P4[] = {600,150,150};   // Dài – ngắn
static const uint16_t FIND_P5[] = {150,150,600};   // Ngắn – dài
static const uint16_t FIND_P6[] = {80,60,80,60,80,60,80,60,80,60,80};   // Dồn dập
static const uint16_t FIND_P7[] = {400,400,400,400,400};   // Chậm đều
static const uint16_t FIND_P8[] = {100,100,100,100,100,300,300,100,300,100,300,300,100,100,100,100,100};   // SOS
static const uint16_t FIND_P9[] = {120,80,120,400,120,80,120};   // Cảnh báo kép
static const uint16_t FIND_P10[] = {80,200,150,200,250,200,400};   // Tăng dần
static const uint16_t FIND_P11[] = {400,200,250,200,150,200,80};   // Giảm dần
static const uint16_t FIND_P12[] = {120,100,200,500,120,100,200};   // Nhịp tim
static const uint16_t FIND_P13[] = {100,100,100,100,100,100,100};   // Morse H (····)
static const uint16_t FIND_P14[] = {300,100,100,100,300};   // Morse K (–·–)
static const uint16_t FIND_P15[] = {100,100,300,100,100};   // Morse R (·–·)
static const uint16_t FIND_P16[] = {100,100,100,100,100,100,300};   // Morse V (···–)
static const uint16_t FIND_P17[] = {300,100,100,100,100,100,100};   // Morse B (–···)
static const uint16_t FIND_P18[] = {500,100,500,100,150,100,150};   // Còi hụ: dài dài ngắn ngắn
static const uint16_t FIND_P19[] = {60,60,60,60,300,150,300};   // Lạch cạch
static const uint16_t FIND_P20[] = {80,70,80,70,80,150,600};   // Ba nhanh một dài
static const uint16_t FIND_P21[] = {150,900,150};   // Hai tiếng cách xa
static const uint16_t FIND_P22[] = {150,100,150,100,150,100,150};   // Bốn tiếng đều
static const uint16_t FIND_P23[] = {400,200,100,100,400,200,100};   // Chậm nhanh xen kẽ
struct FindPat { const uint16_t *t; uint8_t n; };
#define FPAT(arr) { arr, (uint8_t)(sizeof(arr) / sizeof(arr[0])) }
static const FindPat FIND_PATTERNS[] = {
  FPAT(FIND_P0),
  FPAT(FIND_P1),
  FPAT(FIND_P2),
  FPAT(FIND_P3),
  FPAT(FIND_P4),
  FPAT(FIND_P5),
  FPAT(FIND_P6),
  FPAT(FIND_P7),
  FPAT(FIND_P8),
  FPAT(FIND_P9),
  FPAT(FIND_P10),
  FPAT(FIND_P11),
  FPAT(FIND_P12),
  FPAT(FIND_P13),
  FPAT(FIND_P14),
  FPAT(FIND_P15),
  FPAT(FIND_P16),
  FPAT(FIND_P17),
  FPAT(FIND_P18),
  FPAT(FIND_P19),
  FPAT(FIND_P20),
  FPAT(FIND_P21),
  FPAT(FIND_P22),
  FPAT(FIND_P23)
};
#define FIND_PATTERN_COUNT ((uint8_t)(sizeof(FIND_PATTERNS) / sizeof(FIND_PATTERNS[0])))
#define FIND_REPEAT_MAX    10
#define FIND_REPEAT_GAP_MS 500
#define FIND_TOTAL_MAX_MS  30000UL   // gioi han an toan cho ca lan bao
// He so hieu chuan dien ap (chinh tren web: Cai dat -> Hieu chuan dien ap)
#define VCAL_DEFAULT 1.2343f
float vCalFactor = VCAL_DEFAULT;
uint8_t findPattern = 2;   // mac dinh: "Bip bip bip" (so thu tu bat dau tu 0)
uint8_t findRepeat  = 1;
uint32_t tapGapMs = FP_TAP_GAP_MS;          // khoang cach toi da giua 2 lan cham (chinh tren web)
uint32_t holdStartMs = FP_HOLD_START_MS;   // thoi gian giu van tay truoc khi de
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
bool holdDenied = false;
bool holdStarted = false;
bool touchActive = false;        // dang co ngon tay tren cam bien
bool touchOwner = false;         // lan cham nay la van tay da dang ky
bool touchWaitRelease = false;   // da xu ly xong (vd vua mo khoa) -> cho nha tay roi moi tinh tiep
bool touchPoor = false;          // anh kem -> thu doi chieu lai trong cung lan cham
uint8_t touchTries = 0;
unsigned long touchStart = 0;
unsigned long holdBase = 0;      // moc tinh thoi gian giu: SAU khi doi chieu van tay xong (doi chieu mat ~0.3-0.5s)
unsigned long lastVerifyMs = 0;
bool fpNeedRelease = false;      // sau khi dang ky xong: cho nhac tay ra roi moi nhan cham

// --- Den LED van tay ---
unsigned long ledOffAt = 0;
uint8_t ledLastCode = 0xFE;
uint8_t ledTestCode = 0;
unsigned long ledTestNext = 0;

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
uint32_t sleepSec = 60;           // 0 = tat tu dong ngu, con lai 10-100 giay
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
int8_t fpIdentifyCurrent();
void fpOnTouchStart();
void fpSyncFromModule();
uint8_t zwLed(uint8_t mode, uint8_t startColor, uint8_t endColor, uint8_t loops);
void ledOff();
void ledSolid(uint8_t color, uint16_t ms);
void ledBlink(uint8_t color, uint8_t times, uint16_t onMs = 180, uint16_t offMs = 180, uint16_t beepFreq = 0);
void ledService();
#if USE_RFID
String uidToString(const uint8_t *uid, uint8_t len);
bool pn532Init(uint8_t tries = 1);
#endif
void printSystemStatusToSerial();
void resetFingerLED();
void hardResetRFID();
void checkAndResetRfidModule();
float readBatteryVolt();
float readBatteryVoltRaw();
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
void rfidSleep();
void rfidWake();
void startWifiOta();
void scanWifiList();
void otaSetResult(const String &r);

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
// ============== CHAN DOAN KHOI DONG LAI + WATCHDOG ================
// Neu ESP32 dung (treo), watchdog se tu khoi dong lai sau WDT_TIMEOUT_S giay de xe khong bi liet.
// BROWNOUT_DISABLE = 1: tat bo phat hien sut nguon (KHONG khuyen dung, sut nguon se lam ESP32 treo thay vi khoi dong lai)
#define BROWNOUT_DISABLE 0
#define WDT_TIMEOUT_S    20
#ifndef ESP_ARDUINO_VERSION_MAJOR
  #define ESP_ARDUINO_VERSION_MAJOR 2
#endif

static const char* resetReasonStr(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_POWERON:   return "POWERON (cap dien lai)";
    case ESP_RST_SW:        return "SW (ESP.restart)";
    case ESP_RST_PANIC:     return "PANIC (loi chuong trinh)";
    case ESP_RST_INT_WDT:   return "INT_WDT (treo ngat)";
    case ESP_RST_TASK_WDT:  return "TASK_WDT (treo, watchdog khoi dong lai)";
    case ESP_RST_WDT:       return "WDT (treo)";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT (SUT NGUON)";
    case ESP_RST_EXT:       return "EXT (nut EN)";
    default:                return "KHAC";
  }
}

static void wdtStart() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t c = {};
  c.timeout_ms = WDT_TIMEOUT_S * 1000;
  c.idle_core_mask = 0;
  c.trigger_panic = true;
  esp_task_wdt_reconfigure(&c);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);   // theo doi tac vu loop()
}
static inline void wdtFeed() { esp_task_wdt_reset(); }

// Bien chan doan (in ra moi 5 giay trong Serial Monitor)
static volatile uint32_t dbgLoops = 0;      // so vong loop() trong 5 giay
static volatile uint8_t  dbgLastP = 0xEE;   // ma tra loi cuoi cua finger.getImage() (0x00 co tay, 0x02 khong co tay, khac = loi)
static volatile uint32_t dbgFpErr = 0;
static volatile uint32_t dbgLoopTotal = 0;  // tong so vong loop (de tinh toc do vong lap tren web)

// Bo dem su co luu trong flash (hien tren web: muc Kiem tra loi)
static uint8_t  diagResetCode = 9;          // 0 bat nguon,1 SW,2 panic,3 int_wdt,4 task_wdt,5 wdt,6 deepsleep,7 brownout,8 EN,9 khac
static uint32_t diagBo = 0, diagWdt = 0, diagPan = 0, diagFpRec = 0, diagGhost = 0;
static void diagSave(const char *k, uint32_t v) {
  Preferences dp;
  if (dp.begin("diag", false)) { dp.putUInt(k, v); dp.end(); }
}
static void diagBump(const char *k, uint32_t &v) { v++; diagSave(k, v); }
static void diagLoad(esp_reset_reason_t r) {
  Preferences dp;
  if (dp.begin("diag", false)) {
    diagBo = dp.getUInt("bo", 0); diagWdt = dp.getUInt("wdt", 0); diagPan = dp.getUInt("pan", 0);
    diagFpRec = dp.getUInt("fprec", 0); diagGhost = dp.getUInt("ghost", 0);
    dp.end();
  }
  switch (r) {
    case ESP_RST_POWERON:   diagResetCode = 0; break;
    case ESP_RST_SW:        diagResetCode = 1; break;
    case ESP_RST_PANIC:     diagResetCode = 2; diagBump("pan", diagPan); break;
    case ESP_RST_INT_WDT:   diagResetCode = 3; diagBump("wdt", diagWdt); break;
    case ESP_RST_TASK_WDT:  diagResetCode = 4; diagBump("wdt", diagWdt); break;
    case ESP_RST_WDT:       diagResetCode = 5; diagBump("wdt", diagWdt); break;
    case ESP_RST_DEEPSLEEP: diagResetCode = 6; break;
    case ESP_RST_BROWNOUT:  diagResetCode = 7; diagBump("bo", diagBo); break;
    case ESP_RST_EXT:       diagResetCode = 8; break;
    default:                diagResetCode = 9; break;
  }
}
static void diagClear() {
  diagBo = diagWdt = diagPan = diagFpRec = diagGhost = 0;
  dbgFpErr = 0;
  Preferences dp;
  if (dp.begin("diag", false)) { dp.clear(); dp.end(); }
}      // so lan module van tay tra loi loi/khong tra loi

void setup() {
#if BROWNOUT_DISABLE
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
#endif
  esp_reset_reason_t rstReason = esp_reset_reason();

  Serial.begin(921600);
  delay(200);

  Serial.println("\n\n==========================================");
  Serial.println("[SYSTEM] Khoi dong he thong Khoa xe thong minh HQT v6 (BLE)...");
  Serial.println("==========================================");
  Serial.printf("[RESET] Ly do khoi dong: %s\n", resetReasonStr(rstReason));
  diagLoad(rstReason);

  // Dat mode truoc roi moi ghi muc logic, tranh relay bi nhay luc khoi dong
  pinMode(RELAY_POWER, OUTPUT);
  pinMode(RELAY_STARTER, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_D2_PIN, OUTPUT);
  pinMode(SIDESTAND_PIN, INPUT_PULLUP);
  pinMode(FP_WAKE_PIN, INPUT_PULLDOWN);

  digitalWrite(RELAY_POWER, RELAY_OFF);
  digitalWrite(RELAY_STARTER, RELAY_OFF);
  buzzOff();
  digitalWrite(LED_D2_PIN, LOW);

  cmdQueue = xQueueCreate(8, CMD_LEN);
  lastActivityMs = millis();
  normalCpuMhz = getCpuFrequencyMhz();

  loadAllSettings();
  Serial.printf("[FW] Phien ban firmware: %d\n", FW_VERSION);

  // Ket qua lan cap nhat OTA truoc do (neu co)
  {
    prefs.begin("bike_cfg", false);
    String otaRes = prefs.getString("otaRes", "");
    if (otaRes.length()) prefs.remove("otaRes");
    prefs.end();
    if (otaRes == "OK") lastStatusMsg = "Da cap nhat firmware xong, dang chay phien ban " + String(FW_VERSION);
    else if (otaRes.length()) lastStatusMsg = otaRes;
  }

  Serial.println("\n[INIT-FP] Dang ket noi cam bien van tay ZW101...");
  finger.begin(57600);
  delay(50);
  // Mo UART2 sau finger.begin() de chac chan dung dung chan RX/TX da chon
  fingerSerial.begin(57600, SERIAL_8N1, FINGER_RX, FINGER_TX);
  delay(200);

  // Cam bien van tay can thoi gian khoi dong sau khi cap nguon -> thu toi da 5 lan
  bool fpOkBoot = false;
  for (uint8_t i = 0; i < 5 && !fpOkBoot; i++) {
    fpOkBoot = finger.verifyPassword();
    if (!fpOkBoot) delay(300);
  }

  if (fpOkBoot) {
    fpConnected = true;
    finger.getParameters();   // can de tim kiem van tay (fingerSearch) biet dung luong thu vien
    if (finger.capacity == 0 || finger.capacity > 1000) finger.capacity = MAX_FP + 1;
    Serial.printf("[FP] dung luong thu vien tim kiem = %d\n", finger.capacity);
    Serial.println("[HW] OK: Cam bien van tay ZW101 san sang!");
    ledBlink(LEDC_BLUE, 2, 120, 120);
    fpSyncFromModule();       // module la nguon dung: ESP32 nho dung cac ID dang co trong module
  } else {
    fpConnected = false;
    Serial.println("[HW-ERR] CANH BAO: Khong the ket noi cam bien van tay ZW101!");
  }

#if USE_RFID
#if PN532_USE_SPI
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, RFID_SS);
#else
  Wire.begin(PN532_SDA, PN532_SCL);
  Wire.setClock(100000);
  delay(20);
  // Quet bus I2C de chan doan: PN532 phai hien dia chi 0x24
  Serial.print("[I2C] Quet bus:");
  uint8_t i2cFound = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { Serial.printf(" 0x%02X", addr); i2cFound++; }
  }
  Serial.println(i2cFound ? "" : " (khong thay thiet bi nao: kiem tra day SDA/SCL/GND chung/nguon 3.3V/cong tat DIP)");
#endif
  delay(20);
  if (pn532Init(3)) Serial.println("[HW] OK: PN532 san sang!");
  else Serial.println("[HW-ERR] CANH BAO: Khong tim thay PN532 (kiem tra day, cong tat DIP I2C/SPI)");
#else
  Serial.println("[RFID] Dang TAT (USE_RFID = 0), mo khoa bang nut tren app");
#endif

  startBLE();
  beepBuzzer(2000, 80, 2);
  // Khoi dong lai do loi (sut nguon / treo / panic): them 3 tieng bip tram dai de de nhan biet
  if (rstReason == ESP_RST_BROWNOUT || rstReason == ESP_RST_TASK_WDT || rstReason == ESP_RST_INT_WDT ||
      rstReason == ESP_RST_WDT || rstReason == ESP_RST_PANIC) {
    delay(300);
    beepBuzzer(800, 350, 3);
  }
  wdtStart();
  Serial.println("[SYSTEM] He thong da san sang!\n");
}

void loop() {
  wdtFeed();
  dbgLoops++;
  dbgLoopTotal++;
#if USE_RFID
  // Vua cham van tay de thuc day -> doi nha tay xong moi khoi dong lai RFID (khong lam lech nhip giu de)
  if (rfidWakePending && !touchActive) {
    rfidWakePending = false;
    if (pn532Init(2)) Serial.println("[RFID] Da khoi dong lai RFID (cham van tay)");
  }
#endif
  handleRFIDReader();

  // Thuc thi lenh nhan tu BLE (chay o loop de khong chan tac vu BLE)
  char cbuf[CMD_LEN];
  while (xQueueReceive(cmdQueue, cbuf, 0) == pdTRUE) {
    processCommand(String(cbuf));
    configDirty = true;
  }

  // Dinh ky nhac BLE quang ba lai khi khong co ket noi (phong khi quang ba bi dung sau thoi gian dai)
  {
    static unsigned long lastAdvKick = 0;
    if (!bleConnected && pServer != nullptr && millis() - lastAdvKick > 60000UL) {
      lastAdvKick = millis();
      BLEDevice::startAdvertising();
    }
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
    if (wifiListUntil && (long)(millis() - wifiListUntil) > 0) { wifiListUntil = 0; wifiListJson = ""; configDirty = true; }
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

  ledService();

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

// Dien ap tho (chua hieu chuan): trung binh 16 lan doc ADC cho do nhieu
float readBatteryVoltRaw() {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < 16; i++) sum += analogRead(BATTERY_ADC_PIN);
  float rawAdc = sum / 16.0f;
  return (rawAdc / 4095.0f) * 3.3f * 4.3f;
}

float readBatteryVolt() {
  float v = readBatteryVoltRaw() * vCalFactor;
  if (v < 2.0f) v = 12.6f;
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
    j += ",\"vBatt\":" + String(readBatteryVolt(), 2);
    j += ",\"standDown\":" + String(digitalRead(SIDESTAND_PIN) == LOW ? "true" : "false");
    j += ",\"fp\":" + String(fpConnected ? "true" : "false");
    j += ",\"rfid\":" + String((rfidOk || rfidSleeping) ? "true" : "false");
    j += ",\"adding\":" + String(isAddingRfid ? "true" : "false");
    j += ",\"fpAdd\":" + String(fpEnrolling ? "true" : "false");
    j += ",\"fpStep\":" + String(fpStep);
    {
      static uint32_t lastTot = 0; static unsigned long lastMs = 0; static uint32_t lps = 0;
      unsigned long nowD = millis();
      if (nowD - lastMs >= 1000UL) {   // toc do vong lap, cap nhat moi giay
        lps = (uint32_t)((uint64_t)(dbgLoopTotal - lastTot) * 1000ULL / (nowD - lastMs));
        lastTot = dbgLoopTotal; lastMs = nowD;
      }
      j += ",\"dg\":{\"up\":" + String(nowD / 1000UL);
      j += ",\"hp\":" + String((unsigned)ESP.getFreeHeap());
      j += ",\"mh\":" + String((unsigned)ESP.getMinFreeHeap());
      j += ",\"lp\":" + String((unsigned)lps);
      j += ",\"fe\":" + String((unsigned long)dbgFpErr);
      j += ",\"fc\":" + String((unsigned)dbgLastP);
      j += ",\"rs\":" + String((unsigned)diagResetCode);
      j += ",\"bo\":" + String((unsigned long)diagBo);
      j += ",\"wd\":" + String((unsigned long)diagWdt);
      j += ",\"pn\":" + String((unsigned long)diagPan);
      j += ",\"fr\":" + String((unsigned long)diagFpRec);
      j += ",\"gh\":" + String((unsigned long)diagGhost) + "}";
    }
    j += ",\"msg\":\"" + jsonEscape(lastStatusMsg) + "\"}";
  }
  chStatus->setValue((uint8_t*)j.c_str(), j.length());
}

void updateConfigChar() {
  String j;
  if (!bleAuthed) {
    j = "{\"auth\":false}";
  } else if (wifiListUntil && wifiListJson.length()) {
    j = "{\"auth\":true,\"wl\":" + wifiListJson + "}";   // tam thoi: chi gui danh sach WiFi
  } else {
    j = "{\"auth\":true";
    j += ",\"name\":\"" + jsonEscape(bleName) + "\"";
    j += ",\"fw\":" + String(FW_VERSION);
    j += ",\"wifi\":\"" + jsonEscape(wifiSsid) + "\"";
    j += ",\"fwSet\":" + String(fwUrl.length() ? 1 : 0);   // khong gui ca link (CONFIG toi da 512 byte)
    j += ",\"starterMs\":" + String(starterTimeMs);
    j += ",\"holdMs\":" + String(holdStartMs);
    j += ",\"tapGapMs\":" + String(tapGapMs);
    j += ",\"vCal\":" + String(vCalFactor, 4);
    j += ",\"findPat\":" + String(findPattern);
    j += ",\"findRep\":" + String(findRepeat);
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
  } else if (verb == "diagclr") {
    diagClear();
    lastStatusMsg = "Da xoa bo dem loi";
  } else if (verb == "tapgap") {
    long ms = arg.toInt();
    if (ms < FP_TAP_GAP_MIN_MS) ms = FP_TAP_GAP_MIN_MS;
    if (ms > FP_TAP_GAP_MAX_MS) ms = FP_TAP_GAP_MAX_MS;
    tapGapMs = ms;
    saveAllSettings();
    lastStatusMsg = "Da luu thoi gian giua cac lan cham: " + String(tapGapMs) + "ms";
  } else if (verb == "hold") {
    long ms = arg.toInt();
    if (ms < FP_HOLD_MIN_MS) ms = FP_HOLD_MIN_MS;
    if (ms > FP_HOLD_MAX_CFG_MS) ms = FP_HOLD_MAX_CFG_MS;
    holdStartMs = ms;
    saveAllSettings();
    lastStatusMsg = "Da luu thoi gian giu de: " + String(holdStartMs) + "ms";
  } else if (verb == "vcal") {
    // vcal:<dien ap thuc te do bang dong ho, vd 12.96> -> he so = thuc te / dien ap tho
    float actual = arg.toFloat();
    float raw = readBatteryVoltRaw();
    if (actual < 5.0f || actual > 20.0f) {
      lastStatusMsg = "Dien ap thuc te khong hop le (5-20V)";
    } else if (raw < 2.0f) {
      lastStatusMsg = "Khong doc duoc dien ap, kiem tra module do ap";
    } else {
      float f = actual / raw;
      if (f < 0.5f || f > 2.5f) {
        lastStatusMsg = "He so hieu chuan bat thuong, kiem tra lai so do";
      } else {
        vCalFactor = f;
        saveAllSettings();
        lastStatusMsg = "Da hieu chuan dien ap, he so " + String(vCalFactor, 4);
      }
    }
  } else if (verb == "vcalreset") {
    vCalFactor = VCAL_DEFAULT;
    saveAllSettings();
    lastStatusMsg = "Da dat lai hieu chuan dien ap (he so " + String(vCalFactor, 4) + ")";
  } else if (verb == "findcfg") {
    // findcfg:<so thu tu mau nhip>,<so lan lap>
    int c1 = arg.indexOf(',');
    if (c1 > 0) {
      long pat = arg.substring(0, c1).toInt();
      long rep = arg.substring(c1 + 1).toInt();
      if (pat < 0) pat = 0;
      if (pat >= FIND_PATTERN_COUNT) pat = FIND_PATTERN_COUNT - 1;
      if (rep < 1) rep = 1;
      if (rep > FIND_REPEAT_MAX) rep = FIND_REPEAT_MAX;
      findPattern = (uint8_t)pat;
      findRepeat = (uint8_t)rep;
      saveAllSettings();
      lastStatusMsg = "Da luu nhip bao tim xe";
    }
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
  } else if (verb == "ledtest") {
    ledTestCode = 1;
    ledTestNext = 0;
    lastStatusMsg = "Thu mau LED: xem den van tay";
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
  } else if (verb == "wifi") {
    // wifi:SSID|MATKHAU  (tach o dau '|' dau tien)
    int bar = arg.indexOf('|');
    String ss = (bar < 0) ? arg : arg.substring(0, bar);
    String pw = (bar < 0) ? "" : arg.substring(bar + 1);
    if (ss.length() < 1 || ss.length() > 32 || pw.length() > 63 || (pw.length() > 0 && pw.length() < 8)) {
      lastStatusMsg = "WiFi khong hop le (ten 1-32, mat khau 8-63 ky tu)";
    } else {
      wifiSsid = ss;
      wifiPass = pw;
      saveAllSettings();
      lastStatusMsg = "Da luu WiFi: " + wifiSsid;
    }
  } else if (verb == "fwurl") {
    if (arg.length() == 0) {
      fwUrl = "";
      saveAllSettings();
      lastStatusMsg = "Da xoa link firmware";
    } else if (arg.length() > 150 || !(arg.startsWith("http://") || arg.startsWith("https://"))) {
      lastStatusMsg = "Link firmware phai bat dau bang http:// hoac https://";
    } else {
      fwUrl = arg;
      saveAllSettings();
      lastStatusMsg = "Da luu link firmware";
    }
  } else if (verb == "wscan") {
    scanWifiList();
  } else if (verb == "ota") {
    startWifiOta();
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
bool pn532Init(uint8_t tries) {
  uint32_t ver = 0;
  for (uint8_t i = 0; i < tries && !ver; i++) {
    nfc.begin();
    ver = nfc.getFirmwareVersion();
    if (!ver && i + 1 < tries) delay(150);
  }
  if (!ver) {
    rfidOk = false;
    return false;
  }
  Serial.printf("[RFID] PN5%02X firmware %d.%d\n", (ver >> 24) & 0xFF, (ver >> 16) & 0xFF, (ver >> 8) & 0xFF);
  nfc.SAMConfig();                       // che do doc the binh thuong
  nfc.setPassiveActivationRetries(0x01); // khong cho lau, de khong lam cham vong quet van tay
  rfidOk = true;
  return true;
}

// Vo hieu hoa RFID khi vao che do ngu: ngung doc the (module van duoc cap nguon, khong dung den chan reset)
void rfidSleep() {
  if (rfidSleeping) return;
  rfidOk = false;
  rfidSleeping = true;
  rfidWakePending = false;
  Serial.println("[RFID] Tat RFID (che do ngu)");
}

// Danh dau can bat lai RFID; loop() se khoi dong lai ngay khi nha tay khoi van tay
void rfidWake() {
  if (!rfidSleeping) return;
  rfidSleeping = false;
  rfidWakePending = true;
}

// Goi sau khi khoa / de: chi cau hinh lai nhe (khong reset cung, tranh dung 0.4 giay)
void hardResetRFID() {
  if (rfidOk) nfc.SAMConfig();
}

void checkAndResetRfidModule() {
  if (rfidSleeping || rfidWakePending) return;   // RFID dang tat / cho bat lai
  if (touchActive) return;                       // dang xu ly van tay thi khong dong vao I2C/SPI
  if (!rfidOk) {
    // Dang loi: thu lai thua (15 giay/lan) de khong chan vong lap quet van tay
    static unsigned long lastRetry = 0;
    if (millis() - lastRetry < 15000UL) return;
    lastRetry = millis();
    pn532Init(1);
    return;
  }
  if (nfc.getFirmwareVersion() == 0) {
    Serial.println("[RFID-WARN] PN532 khong tra loi");
    rfidOk = false;
  }
}
#else
void hardResetRFID() {}
void checkAndResetRfidModule() {}
void rfidSleep() {}
void rfidWake() {}
#endif

// ============== RELAY / KHOA ================
void resetFingerLED() {
  ledOff();
}

void unlockBike() {
  isUnlocked = true;
  tapCount = 0;
  digitalWrite(RELAY_POWER, RELAY_ON);
  digitalWrite(LED_D2_PIN, HIGH);

  Serial.println("[RELAY] RELAY_POWER -> BAT (Mo khoa xe thanh cong)");
  ledBlink(LEDC_GREEN, 2, 150, 100, 2500);
}

void lockBike() {
  isUnlocked = false;
  isEngineRunning = false;
  tapCount = 0;
  digitalWrite(RELAY_STARTER, RELAY_OFF);
  digitalWrite(RELAY_POWER, RELAY_OFF);
  digitalWrite(LED_D2_PIN, LOW);

  Serial.println("[RELAY] RELAY_POWER & RELAY_STARTER -> TAT (Da khoa/tat may xe)");
  ledBlink(LEDC_RED, 2, 150, 100, 1000);

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
  unsigned long startMs = millis();
  const FindPat &pt = FIND_PATTERNS[findPattern < FIND_PATTERN_COUNT ? findPattern : 0];

  zwLed(FINGERPRINT_LED_ON, LEDC_BLUE, LEDC_BLUE, 0);   // den van tay xanh duong suot luc bao (khong lam lech nhip)
  for (uint8_t r = 0; r < findRepeat; r++) {
    for (uint8_t i = 0; i < pt.n; i++) {
      bool on = (i % 2 == 0);
      if (on) { buzzOn(2800); digitalWrite(LED_D2_PIN, HIGH); }
      else    { buzzOff();    digitalWrite(LED_D2_PIN, LOW); }
      delay(pt.t[i]);
      wdtFeed();
    }
    buzzOff();
    digitalWrite(LED_D2_PIN, LOW);
    if (r + 1 < findRepeat) delay(FIND_REPEAT_GAP_MS);
    wdtFeed();
    if (millis() - startMs > FIND_TOTAL_MAX_MS) break;
  }
  isAlarmActive = false;
  digitalWrite(LED_D2_PIN, isUnlocked ? HIGH : LOW);
  ledOff();
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
String uidToString(const uint8_t *uid, uint8_t len) {
  String s = "";
  for (uint8_t i = 0; i < len; i++) {
    if (uid[i] < 0x10) s += "0";
    s += String(uid[i], HEX);
  }
  s.toUpperCase();
  return s;
}

void handleRFIDReader() {
  if (!rfidOk) return;
  if (touchActive || fpEnrolling) return;   // uu tien van tay

  uint8_t uid[7] = {0};
  uint8_t uidLen = 0;
  if (!nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLen, 30)) return;

  // PN532 doc lien tuc the dang nam tren dau doc -> chi tinh la quet moi khi the da roi ra >= 800ms
  static unsigned long lastCardSeen = 0;
  unsigned long nowMs = millis();
  bool isNewScan = (nowMs - lastCardSeen > 800);
  lastCardSeen = nowMs;
  if (!isNewScan) return;

  String scannedUID = uidToString(uid, uidLen);
  Serial.printf("[RFID] Quet duoc the UID: %s\n", scannedUID.c_str());

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

// Doi chieu anh van tay dang nam trong bo nho cam bien (vua getImage xong) voi thu vien.
// Tra ve: 1 = khop van tay da dang ky, 0 = khong khop, -1 = anh kem hoac loi giao tiep.
int8_t fpIdentifyCurrent() {
  uint8_t r = finger.image2Tz(1);
  if (r != FINGERPRINT_OK) {
    Serial.printf("[FP] image2Tz loi ma=%d (anh kem?)\n", r);
    return -1;
  }

  finger.fingerID = 0xFFFF;
  finger.confidence = 0;
  uint8_t p = finger.fingerSearch();   // neu bao loi bien dich khong co ham nay, doi thanh fingerFastSearch()
  Serial.printf("[FP] Search ma=%d ID=%d tin cay=%d | ESP32 luu %d van tay\n",
                p, finger.fingerID, finger.confidence, (int)fpIds.size());

  // Module la nguon dung: van tay khop trong module la van tay da dang ky.
  // Neu ID chua co trong danh sach ESP32 (vd dong bo bi thieu) thi tu bo sung.
  if (p == FINGERPRINT_OK && finger.confidence > 0 && finger.fingerID < 1000) {
    if (!fpHasId(finger.fingerID)) {
      fpIds.push_back(finger.fingerID);
      saveAllSettings();
      configDirty = true;
      Serial.printf("[FP] Tu bo sung ID %d vao danh sach\n", finger.fingerID);
    }
    return 1;
  }
  if (p == FINGERPRINT_OK || p == FINGERPRINT_NOTFOUND) return 0;
  return -1;
}

// Dong bo danh sach ID cua ESP32 voi cac van tay thuc su co trong module
// (vi du van tay dang ky tu chuong trinh thu cu van con trong module).
void fpSyncFromModule() {
  if (!fpConnected) return;
  if (finger.getTemplateCount() != FINGERPRINT_OK) {
    Serial.println("[FP] Khong doc duoc so van tay trong module, bo qua dong bo");
    return;
  }
  uint16_t total = finger.templateCount;
  std::vector<uint16_t> found;
  uint8_t lastErr = 0;

  // Quet toi da 2 lan: neu lan dau thieu ID (loi doc tam thoi) thi quet lai
  for (uint8_t pass = 0; pass < 2 && found.size() < total; pass++) {
    found.clear();
    for (uint16_t id = 1; id <= MAX_FP; id++) {
      uint8_t r = finger.loadModel(id);
      if (r == FINGERPRINT_OK) found.push_back(id);
      else lastErr = r;
    }
  }
  Serial.printf("[FP] Module co %d van tay, quet thay %d ID trong 1..%d, ESP32 dang luu %d (ma loi cuoi=%d)\n",
                total, (int)found.size(), MAX_FP, (int)fpIds.size(), lastErr);

  if (found.size() > total) {
    Serial.println("[FP] Ket qua quet khong dang tin, giu nguyen danh sach cu");
    return;
  }

  if (found.size() < total) {
    // Module co nhieu van tay hon so quet duoc -> KHONG thu nho danh sach, chi bo sung ID quet duoc.
    // (Van tay ngoai danh sach van duoc nhan khi cham: xem fpIdentifyCurrent)
    bool changed = false;
    for (size_t i = 0; i < found.size(); i++) {
      if (!fpHasId(found[i])) { fpIds.push_back(found[i]); changed = true; }
    }
    if (changed) { saveAllSettings(); configDirty = true; }
    Serial.println("[FP] Quet thieu ID -> giu danh sach da luu, khong xoa van tay nao");
    return;
  }

  std::vector<uint16_t> cur = fpIds;
  std::sort(cur.begin(), cur.end());
  if (cur != found) {
    fpIds = found;
    saveAllSettings();
    configDirty = true;
    Serial.println("[FP] Da dong bo danh sach van tay ESP32 theo module");
  }
}

// ============== VAN TAY: XU LY MOI LAN CHAM ================
// Goi mot lan khi vua cham: doi chieu van tay, bao den xanh la (dung) / do (sai),
// va neu xe dang khoa + van tay dung thi mo khoa.
void fpOnTouchStart() {
  unsigned long now = millis();
  touchOwner = false;
  touchPoor = false;
  lastVerifyMs = now;

  // Chua dang ky van tay nao
  if (fpIds.empty()) {
    if (!isUnlocked) {
      lastStatusMsg = "Chua co van tay nao, hay mo khoa bang app";
      ledSolid(LEDC_RED, 600);
      beepBuzzer(1000, 100, 2);
      touchWaitRelease = true;
    } else {
      touchOwner = true;            // du phong: xe da mo khoa tu app, khong co van tay de doi chieu
      ledSolid(LEDC_BLUE, 300);
    }
    return;
  }

  int8_t r = fpIdentifyCurrent();
  if (r == 1) {
    touchOwner = true;
    if (!isUnlocked) {
      lastStatusMsg = "Van tay dung -> Da mo khoa xe";
      unlockBike();                 // den xanh la + bip
      touchWaitRelease = true;      // lan cham nay chi dung de mo khoa
    } else {
      lastStatusMsg = "Van tay dung";
      ledSolid(LEDC_GREEN, 700);
    }
  } else if (r == 0) {
    holdDenied = true;
    lastStatusMsg = "Van tay khong dung";
    ledSolid(LEDC_RED, 900);
    beepBuzzer(1000, 100, 2);
  } else {
    // Anh kem: cho phep thu lai toi da 3 lan trong cung mot lan dat tay
    ledSolid(LEDC_RED, 300);
    if (touchTries >= 2) {
      holdDenied = true;
      lastStatusMsg = "Anh van tay kem, nhac tay ra va dat lai";
    } else {
      touchPoor = true;
      lastStatusMsg = "Anh van tay kem, giu yen ngon tay...";
    }
  }
}

// ============== VAN TAY: CU CHI (mo khoa / giu = de / cham 3 lan = khoa) ================
// Module van tay khong tra loi (treo/mat UART) -> mo lai UART va ket noi lai; that bai nhieu lan -> khoi dong lai ESP32
static unsigned long fpLastReply = 0;     // lan cuoi module tra loi hop le
static unsigned long fpDownSince = 0;     // luc bat dau mat ket noi
static uint8_t fpRecoverCount = 0;
static bool fpWasUp = false;
// Chong ket 'co tay' (touchActive bi giu mai): cam bien bao co tay lien tuc -> bo qua cho den khi bao NOFINGER on dinh
#define FP_STUCK_TOUCH_MS   15000UL   // giu 1 lan cham qua lau (hold de toi da 10s) -> coi la ket
#define FP_GHOST_CLEAR_MS   1000UL    // can bao NOFINGER lien tuc bay nhieu ms moi coi la da nha
#define FP_GHOST_RESET_MS   60000UL   // bi ket qua lau -> mo lai UART van tay
static bool fpGhostBlock = false;
static unsigned long fpGhostSince = 0, fpNoFingerSince = 0;              // da tung ket noi duoc: chi khi do moi tu khoi dong lai (tranh lap khoi dong neu chua cam module)
static void fpRecoverUart(const char *why) {
  Serial.printf("[FP-WARN] %s -> mo lai UART van tay\n", why);
  diagBump("fprec", diagFpRec);
  fpConnected = false;
  touchActive = false;
  fpDownSince = millis();
  fingerSerial.end();
  delay(50);
  fingerSerial.begin(57600, SERIAL_8N1, FINGER_RX, FINGER_TX);
  delay(50);
  while (fingerSerial.available()) fingerSerial.read();
  if (++fpRecoverCount >= 4) {
    Serial.println("[FP-ERR] Module van tay khong phuc hoi, khoi dong lai ESP32");
    delay(100);
    ESP.restart();
  }
}

void handleFingerprintLogic() {
  // Module mat ket noi luc khoi dong -> thu noi lai moi 5 giay
  if (!fpConnected) {
    static unsigned long lastTry = 0;
    if (fpDownSince == 0) fpDownSince = millis();
    if (fpWasUp && millis() - fpDownSince > 90000UL) {   // mat ket noi qua 90 giay: khoi dong lai
      Serial.println("[FP-ERR] Mat ket noi van tay qua lau, khoi dong lai ESP32");
      delay(100);
      ESP.restart();
    }
    if (millis() - lastTry > 5000) {
      lastTry = millis();
      if (finger.verifyPassword()) {
        fpConnected = true;
        fpDownSince = 0;
        fpRecoverCount = 0;
        fpLastReply = millis();
        finger.getParameters();
        if (finger.capacity == 0 || finger.capacity > 1000) finger.capacity = MAX_FP + 1;
        fpSyncFromModule();
        Serial.println("[HW] Cam bien van tay da ket noi lai");
      }
    }
    return;
  }

  fpWasUp = true;
  unsigned long now = millis();
  uint8_t p = finger.getImage();
  dbgLastP = p;
  if (p != FINGERPRINT_OK && p != FINGERPRINT_NOFINGER && p != FINGERPRINT_IMAGEFAIL) dbgFpErr++;
  now = millis();   // getImage co the cho lau khi module khong tra loi
  if (fpLastReply == 0) fpLastReply = now;
  if (p == FINGERPRINT_OK || p == FINGERPRINT_NOFINGER || p == FINGERPRINT_IMAGEFAIL) {
    fpLastReply = now;
    fpRecoverCount = 0;
  } else if (now - fpLastReply > 6000UL) {
    fpLastReply = now;
    fpRecoverUart("Module van tay khong tra loi");
    return;
  }

  // Sau khi dang ky: doi nhac tay ra roi moi nhan cham moi
  if (fpNeedRelease) {
    if (p == FINGERPRINT_NOFINGER) fpNeedRelease = false;
    lastFingerState = false;
    return;
  }

  bool seen = (p == FINGERPRINT_OK);

  // Dang bi chan do 'co tay' ket: bo qua tin hieu co tay cho den khi cam bien bao khong co tay on dinh
  if (fpGhostBlock) {
    if (p == FINGERPRINT_NOFINGER) {
      if (fpNoFingerSince == 0) fpNoFingerSince = now;
      if (now - fpNoFingerSince > FP_GHOST_CLEAR_MS) {
        fpGhostBlock = false;
        fpNoFingerSince = 0;
        Serial.println("[FP] Cam bien het bao co tay, hoat dong lai binh thuong");
      }
    } else {
      fpNoFingerSince = 0;
    }
    if (fpGhostBlock) {
      if (now - fpGhostSince > FP_GHOST_RESET_MS) {
        fpGhostSince = now;
        fpGhostBlock = false;
        fpRecoverUart("Cam bien bao co tay lien tuc qua lau");
        return;
      }
      seen = false;
      lastFingerState = false;
    }
  }
  if (seen) lastFingerSeen = now;

  // Chi coi la nha tay khi cam bien bao NOFINGER du lau; loi doc tam thoi thi giu nguyen trang thai
  bool pressed;
  if (touchActive) {
    unsigned long grace = (p == FINGERPRINT_NOFINGER) ? FP_RELEASE_DEBOUNCE : FP_ERR_GRACE;
    pressed = seen || (now - lastFingerSeen < grace);
  } else {
    pressed = seen;
  }

  // Mot lan cham keo dai qua lau (cam bien ket / nhieu): nha ra va chan cho den khi cam bien bao het co tay.
  // Khong lam nhu vay thi touchActive ket mai: van tay khong nhan lan cham moi va RFID bi chan.
  if (touchActive && (now - touchStart > FP_STUCK_TOUCH_MS)) {
    Serial.println("[FP-WARN] Cham qua lau (cam bien ket?), tu nha ra de RFID va van tay hoat dong lai");
    diagBump("ghost", diagGhost);
    digitalWrite(RELAY_STARTER, RELAY_OFF);
    isEngineRunning = false;
    touchActive = false;
    holdStarted = false;
    touchWaitRelease = false;
    touchPoor = false;
    tapCount = 0;
    ledOff();
    fpGhostBlock = true;
    fpGhostSince = now;
    fpNoFingerSince = 0;
    lastFingerState = false;
    lastStatusMsg = "Cam bien van tay bao co tay qua lau, da bo qua";
    return;
  }

  // Het cua so cham -> reset bo dem
  if (tapCount > 0 && !touchActive && (now - lastTapRelease > tapGapMs)) {
    tapCount = 0;
  }

  // ---- Bat dau cham ----
  if (pressed && !touchActive) {
    touchActive = true;
    touchStart = now;
    touchTries = 0;
    holdStarted = false;
    holdDenied = false;
    touchWaitRelease = false;
    fpOnTouchStart();
    lastFingerSeen = millis();      // viec doi chieu mat thoi gian
    holdBase = lastFingerSeen;      // thoi gian giu tinh tu luc doi chieu xong
  }

  // ---- Anh kem: thu doi chieu lai khi van con tay tren cam bien ----
  if (touchActive && pressed && seen && touchPoor && touchTries < 3 && (millis() - lastVerifyMs > 300)) {
    touchTries++;
    fpOnTouchStart();
    lastFingerSeen = millis();
    holdBase = lastFingerSeen;
  }

  // ---- Dang giu: relay de ----
  if (touchActive && pressed && !touchWaitRelease && isUnlocked && touchOwner && !holdDenied) {
    unsigned long holdDuration = millis() - holdBase;
    if (holdDuration > holdStartMs) {
      holdStarted = true;
      tapCount = 0;   // da giu thi khong tinh la cham

      if (digitalRead(SIDESTAND_PIN) == LOW) {
        holdDenied = true;
        digitalWrite(RELAY_STARTER, RELAY_OFF);
        isEngineRunning = false;
        lastStatusMsg = "Canh bao: Chan chong dang ha!";
        ledSolid(LEDC_RED, 600);
        beepBuzzer(3000, 100, 3);
      } else if (holdDuration > FP_HOLD_MAX_MS) {
        // Gioi han an toan: khong de qua 10 giay lien tuc
        holdDenied = true;
        digitalWrite(RELAY_STARTER, RELAY_OFF);
        isEngineRunning = false;
        lastStatusMsg = "De qua lau, da ngat relay de";
        ledSolid(LEDC_RED, 600);
        beepBuzzer(3000, 100, 2);
      } else {
        if (!isEngineRunning) {
          Serial.println("[FP] Giu van tay -> BAT relay de");
          zwLed(FINGERPRINT_LED_ON, LEDC_GREEN, LEDC_GREEN, 0);   // den xanh la sang suot luc de
          ledOffAt = 0;
        }
        digitalWrite(RELAY_STARTER, RELAY_ON);
        isEngineRunning = true;
        lastStatusMsg = "Dang giu van tay -> De no...";
      }
    }
  }

  // ---- Nha tay ----
  if (touchActive && !pressed) {
    touchActive = false;
    unsigned long dur = lastFingerSeen - touchStart;

    digitalWrite(RELAY_STARTER, RELAY_OFF);
    isEngineRunning = false;

    if (holdStarted) {
      ledOff();
      tapCount = 0;
      if (!holdDenied) lastStatusMsg = "Da nha van tay, ngat de";
    } else if (!touchWaitRelease && isUnlocked && touchOwner && dur >= FP_TAP_MIN_MS) {
      tapCount++;
      lastTapRelease = millis();
      Serial.printf("[FP] Cham %d/%d\n", tapCount, FP_TAPS_TO_LOCK);

      if (tapCount >= FP_TAPS_TO_LOCK) {
        tapCount = 0;
        lockBike();
        lastStatusMsg = "Cham 3 lan -> Da khoa xe";
      } else {
        lastStatusMsg = "Cham " + String(tapCount) + "/" + String(FP_TAPS_TO_LOCK) + " de khoa xe";
        beepBuzzer(2200, 25, 1);
      }
    }
    holdStarted = false;
    touchWaitRelease = false;
    touchPoor = false;
  }

  lastFingerState = touchActive;
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
  touchActive = false;
  touchWaitRelease = false;
  ledOffAt = 0;

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
  touchActive = false;
  fpNeedRelease = true;
  ledSolid(LEDC_RED, 600);
  beepBuzzer(1000, 100, 2);
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
    ledSolid(LEDC_RED, 500);
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

  // Quet thanh cong -> den xanh la sang ro 0,6 giay + bip
  fpStep++;
  fpFailCount = 0;
  ledBlink(LEDC_GREEN, 2, 150, 100, 2500);   // moi lan quet thanh cong: nhay xanh la 2 cai + bip
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
      delay(250);
      ledBlink(LEDC_GREEN, 5, 200, 200, 2500);   // them xong: nhay xanh la 5 lan
    } else {
      lastStatusMsg = "Luu van tay that bai (ma " + String(r) + ")";
      ledSolid(LEDC_RED, 800);
      beepBuzzer(1000, 150, 2);
    }
    lastFingerState = false;
    touchActive = false;
    fpNeedRelease = true;
    fpStep = 0;
    return;
  }

  lastStatusMsg = "Tot! Nhac tay ra, quet lan " + String(fpStep + 1) + "/" + String(FP_ENROLL_SCANS);
  fpPhase = 1;
  fpStepTimer = millis();
}

// ============== CAP NHAT FIRMWARE QUA WIFI (OTA) ================
void otaSetResult(const String &r) {
  prefs.begin("bike_cfg", false);
  prefs.putString("otaRes", r);
  prefs.end();
}

static void otaProgress(int cur, int total) {
  static int lastStep = -1;
  wdtFeed();
  if (total <= 0) return;
  int step = (int)((int64_t)cur * 20 / total);   // moi 5%
  if (step != lastStep) {
    lastStep = step;
    digitalWrite(LED_D2_PIN, !digitalRead(LED_D2_PIN));
    Serial.printf("[OTA] %d%%\n", step * 5);
  }
}

static void otaFail(const String &msg) {
  Serial.printf("[OTA] %s\n", msg.c_str());
  otaSetResult(msg);
  ledBlink(LEDC_RED, 3, 150, 100, 1000);
  delay(500);
  ESP.restart();   // BLE da bi giai phong de nhuong RAM cho WiFi -> khoi dong lai de BLE hoat dong lai
}

void startWifiOta() {
  if (isUnlocked) { lastStatusMsg = "Hay khoa xe truoc khi cap nhat firmware"; return; }
  if (fpEnrolling || isAddingRfid) { lastStatusMsg = "Dang dang ky the/van tay, hay doi"; return; }
  if (wifiSsid.length() == 0) { lastStatusMsg = "Chua luu WiFi, hay nhap WiFi truoc"; return; }
  if (!(fwUrl.startsWith("http://") || fwUrl.startsWith("https://"))) { lastStatusMsg = "Chua co link firmware"; return; }

  lastStatusMsg = "Dang cap nhat firmware, BLE se ngat (khoang 1 phut)";
  updateStatusChar();
  delay(800);   // de web kip doc thong bao

  // An toan: tat 2 relay
  digitalWrite(RELAY_STARTER, RELAY_OFF);
  digitalWrite(RELAY_POWER, RELAY_OFF);

  // Giai phong BLE de co du RAM cho WiFi + TLS
  BLEDevice::getAdvertising()->stop();
  BLEDevice::deinit(true);
  pServer = nullptr; chCmd = nullptr; chStatus = nullptr; chConfig = nullptr;
  bleConnected = false;
  bleAuthed = false;

  beepBuzzer(2000, 80, 2);
  zwLed(FINGERPRINT_LED_BREATHING, LEDC_BLUE, LEDC_BLUE, 0);   // den van tay xanh duong = dang cap nhat

  Serial.printf("[OTA] Noi WiFi: %s\n", wifiSsid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 25000UL) {
    delay(250);
    wdtFeed();
  }
  if (WiFi.status() != WL_CONNECTED) {
    otaFail("Khong noi duoc WiFi \"" + wifiSsid + "\", kiem tra ten/mat khau/phat WiFi 2.4GHz");
    return;
  }
  Serial.printf("[OTA] Da noi WiFi, IP %s. Tai: %s\n", WiFi.localIP().toString().c_str(), fwUrl.c_str());

  WiFiClientSecure secure;
  WiFiClient plain;
  secure.setInsecure();   // khong kiem tra chung chi (muon an toan hon: secure.setCACert(...))
  WiFiClient &cl = fwUrl.startsWith("https://") ? static_cast<WiFiClient&>(secure) : plain;

  httpUpdate.rebootOnUpdate(false);
  httpUpdate.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
  httpUpdate.onProgress(otaProgress);

  t_httpUpdate_return ret = httpUpdate.update(cl, fwUrl);
  switch (ret) {
    case HTTP_UPDATE_OK:
      otaSetResult("OK");
      Serial.println("[OTA] Thanh cong, khoi dong lai...");
      ledBlink(LEDC_GREEN, 3, 150, 100, 2500);
      delay(300);
      ESP.restart();
      break;
    case HTTP_UPDATE_NO_UPDATES:
      otaFail("Server bao khong co ban cap nhat");
      break;
    default:
      otaFail("Loi tai firmware: " + httpUpdate.getLastErrorString());
      break;
  }
}

// ============== QUET WIFI (de web chon ten WiFi) ================
void scanWifiList() {
  if (isUnlocked || fpEnrolling || isAddingRfid) { lastStatusMsg = "Hay khoa xe va khong dang ky van tay/the roi moi quet WiFi"; return; }
  Serial.println("[WIFI] Dang quet...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  wdtFeed();
  int n = WiFi.scanNetworks(false, false);   // chan ~2-4 giay
  wdtFeed();
  String out = "[";
  int cnt = 0;
  if (n > 0) {
    // chon theo song manh nhat, bo trung ten / ten rong
    bool used[40] = {false};
    int lim = n < 40 ? n : 40;
    for (int k = 0; k < 8; k++) {
      int best = -1;
      for (int i = 0; i < lim; i++) {
        if (used[i]) continue;
        if (best < 0 || WiFi.RSSI(i) > WiFi.RSSI(best)) best = i;
      }
      if (best < 0) break;
      used[best] = true;
      String ss = WiFi.SSID(best);
      if (ss.length() == 0 || ss.length() > 32) { k--; continue; }
      bool dup = false;
      for (int i = 0; i < lim; i++) if (used[i] && i != best && WiFi.SSID(i) == ss) dup = true;
      if (dup) { k--; continue; }
      String item = (cnt ? "," : "") + String("[\"") + jsonEscape(ss) + "\"," + String(WiFi.RSSI(best)) + "," + String(WiFi.encryptionType(best) == WIFI_AUTH_OPEN ? 0 : 1) + "]";
      if (out.length() + item.length() > 440) break;
      out += item;
      cnt++;
    }
  }
  out += "]";
  WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);
  wifiListJson = out;
  wifiListUntil = millis() + 15000UL;
  configDirty = true;
  lastStatusMsg = "Quet WiFi xong: " + String(cnt) + " mang";
  Serial.printf("[WIFI] Tim thay %d mang\n", cnt);
}


// ============== NGU NONG (LIGHT SLEEP) ================
void checkAutoSleep() {
  if (sleepSec == 0) {
    if (ecoActive) exitEco();
    return;
  }
  if (isUnlocked || bleConnected || fpEnrolling || isAddingRfid) return;
  if (millis() - lastActivityMs < sleepSec * 1000UL) return;

  // sleepMode 1 = tiet kiem (van phat BLE), 0 = ngu hang (tat BLE). Ca hai deu tat RFID.
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
#if SLEEP_BEEP
  beepBuzzer(2000, 80, 3);  // bao vao ngu: 3 tieng
#endif
  setCpuFrequencyMhz(80);   // BLE can toi thieu 80MHz
  setAdvInterval(true);
  rfidSleep();              // RFID tat, cham van tay se bat lai
  Serial.println("[ECO] Vao che do tiet kiem (CPU 80MHz, BLE phat thua)");
}

void exitEco() {
  if (!ecoActive) return;
  ecoActive = false;
  setCpuFrequencyMhz(normalCpuMhz);
  setAdvInterval(false);
  rfidWake();               // bat lai RFID sau khi nha tay
  Serial.println("[ECO] Thoat che do tiet kiem");
}

void enterLightSleep() {
#if SLEEP_ENABLED
  Serial.println("[SLEEP] Vao ngu nong. Cham van tay de danh thuc.");
#if SLEEP_BEEP
  beepBuzzer(2000, 80, 3);    // bao vao ngu: 3 tieng
#endif
  rfidSleep();                // tat RFID truoc khi ngu (thuc day se khoi dong lai va init lai RFID)
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

  esp_task_wdt_delete(NULL);   // ngu nong co the keo dai: tam bo theo doi watchdog
  // Ngu nhe, cu 250ms tu day hoi module van tay qua UART (phong khi chan TOUCH OUT khong danh thuc duoc)
  esp_sleep_enable_timer_wakeup((uint64_t)FP_POLL_SLEEP_MS * 1000ULL);
  for (;;) {
    esp_light_sleep_start();
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_EXT0) { Serial.println("[SLEEP] Thuc day: chan TOUCH OUT"); break; }
    while (fingerSerial.available()) fingerSerial.read();
    if (finger.getImage() == FINGERPRINT_OK) { Serial.println("[SLEEP] Thuc day: module van tay bao co tay (UART)"); break; }
  }
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_TIMER);

  // ---- DA THUC DAY ----
#if SLEEP_WAKE_RESTART
  ESP.restart();
#else
  esp_task_wdt_add(NULL);
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
  setFingerLED(FINGERPRINT_LED_FLASHING, 0x05, LEDC_BLUE, 1);
  Serial.println("[SLEEP] Da thuc day.");
#endif
#endif
}

// ============== TIEN ICH ================
// Den LED ZW101: lenh PS_ControlBLN (0x3C) = [che do][mau dau][mau cuoi][so vong].
// Thu vien Adafruit chi gui lenh 0x35 (kieu R503) nen ZW101 khong doi mau -> tu gui goi tin rieng.
// che do: 1 tho, 2 nhay, 3 sang lien tuc, 4 tat (trung ma FINGERPRINT_LED_*)
uint8_t zwLed(uint8_t mode, uint8_t startColor, uint8_t endColor, uint8_t loops) {
  if (!fpConnected) return 0xFF;

  uint8_t pkt[16];
  uint8_t n = 0;
  pkt[n++] = 0xEF; pkt[n++] = 0x01;                                   // header
  pkt[n++] = 0xFF; pkt[n++] = 0xFF; pkt[n++] = 0xFF; pkt[n++] = 0xFF; // dia chi mac dinh
  pkt[n++] = 0x01;                                                    // loai goi: lenh
  pkt[n++] = 0x00; pkt[n++] = 0x07;                                   // do dai = 1 lenh + 4 tham so + 2 checksum
  pkt[n++] = 0x3C;                                                    // PS_ControlBLN
  pkt[n++] = mode; pkt[n++] = startColor; pkt[n++] = endColor; pkt[n++] = loops;
  uint16_t sum = 0;
  for (uint8_t i = 6; i < n; i++) sum += pkt[i];
  pkt[n++] = (uint8_t)(sum >> 8);
  pkt[n++] = (uint8_t)(sum & 0xFF);

  while (fingerSerial.available()) fingerSerial.read();
  fingerSerial.write(pkt, n);
  fingerSerial.flush();

  uint8_t rx[12];
  uint8_t got = 0;
  unsigned long t0 = millis();
  while (got < 12 && millis() - t0 < 80) {
    if (fingerSerial.available()) rx[got++] = (uint8_t)fingerSerial.read();
  }
  uint8_t code = (got >= 10) ? rx[9] : 0xFF;
  if (code != ledLastCode) {
    ledLastCode = code;
    Serial.printf("[LED] ma tra ve cua module = 0x%02X (0x00 = OK, 0xFF = khong tra loi)\n", code);
  }
  return code;
}

void setFingerLED(uint8_t control, uint8_t speed, uint8_t color, uint8_t count) {
  (void)speed;
  zwLed(control, color, color, count);
}

void ledOff() {
  ledOffAt = 0;
  zwLed(FINGERPRINT_LED_OFF, 0, 0, 0);
}

// Den sang lien tuc `ms` mili giay roi tu tat (khong chan chuong trinh)
void ledSolid(uint8_t color, uint16_t ms) {
  zwLed(FINGERPRINT_LED_ON, color, color, 0);
  ledOffAt = millis() + ms;
  if (ledOffAt == 0) ledOffAt = 1;
}

// Nhay `times` lan (chan chuong trinh), tuy chon bip theo moi lan nhay
void ledBlink(uint8_t color, uint8_t times, uint16_t onMs, uint16_t offMs, uint16_t beepFreq) {
  ledOffAt = 0;
  for (uint8_t i = 0; i < times; i++) {
    zwLed(FINGERPRINT_LED_ON, color, color, 0);
    if (beepFreq) buzzOn(beepFreq);
    delay(onMs);
    if (beepFreq) buzzOff();
    zwLed(FINGERPRINT_LED_OFF, 0, 0, 0);
    delay(offMs);
  }
}

// Goi trong loop(): tu tat den sau khi het gio + chay che do thu mau den
void ledService() {
  if (ledTestCode) {
    if ((long)(millis() - ledTestNext) >= 0) {
      if (ledTestCode > 7) {
        ledTestCode = 0;
        ledOff();
        lastStatusMsg = "Het thu mau LED";
      } else {
        zwLed(FINGERPRINT_LED_ON, ledTestCode, ledTestCode, 0);
        Serial.printf("[LED-TEST] ma mau %d\n", ledTestCode);
        lastStatusMsg = "Thu LED: ma mau " + String(ledTestCode);
        ledTestCode++;
        ledTestNext = millis() + 2500;
      }
    }
    return;
  }
  if (ledOffAt && (long)(millis() - ledOffAt) >= 0) {
    ledOffAt = 0;
    ledOff();
  }

  // --- 1. KHI XE ĐANG KHÓA (RẢNH RỖI): Tắt hoàn toàn LED ---
  if (!isUnlocked && !fpEnrolling && !touchActive && !ledOffAt) {
    static bool lastLockState = false;
    if (!lastLockState) {
      ledOff();
      lastLockState = true;
    }
    return;
  } else {
    static bool lastLockState = false;
    lastLockState = false;
  }

  // --- 2. KHI XE ĐÃ MỞ KHÓA: Hiệu ứng thở đổi màu chậm đều qua các màu (1 -> 7) ---
  if (isUnlocked && !fpEnrolling && !touchActive && !ledOffAt) {
    static unsigned long unlockColorTimer = 0;
    static uint8_t currentUnlockColor = 1;
    
    // Cứ mỗi 3 giây chuyển sang một màu thở tiếp theo
    if (millis() - unlockColorTimer > 4500) {
      unlockColorTimer = millis();
      
      // Gọi chế độ thở (FINGERPRINT_LED_BREATHING = 1), Sử dụng chế độ nhấp nháy (FINGERPRINT_LED_FLASHING = 2) với màu hiện tại / loops = 0 để hiệu ứng thở tự chạy lặp mượt mà trong khoảng thời gian chờ
      
      zwLed(FINGERPRINT_LED_BREATHING, currentUnlockColor, currentUnlockColor, 1);
      
      // Tăng mã màu lên cho lần đổi tiếp theo (vòng lặp từ 1 đến 7)
      currentUnlockColor++;
      if (currentUnlockColor > 7) {
        currentUnlockColor = 1;
      }
    }
  }
}

void beepBuzzer(uint16_t freq, uint16_t duration, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    buzzOn(freq);
    delay(duration);
    buzzOff();
    delay(10);
  }
  buzzOff();
}

void printSystemStatusToSerial() {
  Serial.println("\n==================== [ REAL-TIME MONITOR ] ====================");
  Serial.printf("Trang thai: %s | Dong co: %s\n", isUnlocked ? "MO KHOA" : "DA KHOA", isEngineRunning ? "DANG DE" : "TAT");
  Serial.printf("Dien ap: %.1fV | Heap trong: %u (thap nhat %u) | Chay: %lus\n", readBatteryVolt(),
                (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMinFreeHeap(), millis() / 1000UL);
  Serial.printf("RFID: %s | ZW101: %s | Van tay da dang ky: %d | BLE: %s%s\n", rfidOk ? "OK" : "ERR", fpConnected ? "OK" : "ERR",
                (int)fpIds.size(), bleConnected ? "da ket noi" : "dang quang ba", bleAuthed ? " (da xac thuc)" : "");
  Serial.printf("Thong diep: %s\n", lastStatusMsg.c_str());
  Serial.printf("[DBG] vong/5s=%lu | fpP=0x%02X fpLoi=%lu | cham=%d ghost=%d tap=%d | eco=%d rfidNgu=%d rfidOk=%d | fpNoi=%d | mo=%d\n",
                (unsigned long)dbgLoops, (unsigned)dbgLastP, (unsigned long)dbgFpErr, (int)touchActive, (int)fpGhostBlock, (int)tapCount,
                (int)ecoActive, (int)rfidSleeping, (int)rfidOk, (int)fpConnected, (int)isUnlocked);
  dbgLoops = 0;
  Serial.println("===============================================================\n");
}

// ============== LUU / DOC CAI DAT ================
void loadAllSettings() {
  prefs.begin("bike_cfg", true);
  bleName = prefs.getString("bleName", "HQT-KhoaXe");
  blePin = prefs.getString("blePin", "123456");
  starterTimeMs = prefs.getUInt("starterTime", 1500);
  holdStartMs = prefs.getUInt("holdMs", FP_HOLD_START_MS);
  tapGapMs = prefs.getUInt("tapGap", FP_TAP_GAP_MS);
  if (tapGapMs < FP_TAP_GAP_MIN_MS) tapGapMs = FP_TAP_GAP_MIN_MS;
  if (tapGapMs > FP_TAP_GAP_MAX_MS) tapGapMs = FP_TAP_GAP_MAX_MS;
  vCalFactor = prefs.getFloat("vCal", VCAL_DEFAULT);
  if (vCalFactor < 0.5f || vCalFactor > 2.5f) vCalFactor = VCAL_DEFAULT;
  findPattern = prefs.getUChar("findPat", 2);
  findRepeat  = prefs.getUChar("findRep", 1);
  if (findPattern >= FIND_PATTERN_COUNT) findPattern = 2;
  if (findRepeat < 1 || findRepeat > FIND_REPEAT_MAX) findRepeat = 1;
  if (holdStartMs < FP_HOLD_MIN_MS) holdStartMs = FP_HOLD_MIN_MS;
  if (holdStartMs > FP_HOLD_MAX_CFG_MS) holdStartMs = FP_HOLD_MAX_CFG_MS;
  delaySec = prefs.getUInt("delaySec", 3);
  sleepSec = prefs.getUInt("sleepSec", 60);
  // Gia tri cu ngoai khoang 10-100 giay (vd 120 tu ban truoc) thi dua ve gioi han; 0 = tat
  if (sleepSec != 0 && sleepSec < SLEEP_MIN_SEC) sleepSec = SLEEP_MIN_SEC;
  if (sleepSec > SLEEP_MAX_SEC) sleepSec = SLEEP_MAX_SEC;
  sleepMode = prefs.getUChar("sleepMode", 1);
  odometerKm = prefs.getUInt("odo", 0);
  oilLimitKm = prefs.getUInt("oilKm", 1500);
  oilLimitDays = prefs.getUInt("oilDays", 90);
  lastOilOdo = prefs.getUInt("lastOilOdo", 0);
  oilDate = prefs.getString("oilDate", "");
  servDate = prefs.getString("servDate", "");
  airDate = prefs.getString("airDate", "");
  wifiSsid = prefs.getString("wifiSsid", "");
  wifiPass = prefs.getString("wifiPass", "");
  fwUrl = prefs.getString("fwUrl", "");

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
  prefs.putUInt("holdMs", holdStartMs);
  prefs.putUInt("tapGap", tapGapMs);
  prefs.putFloat("vCal", vCalFactor);
  prefs.putUChar("findPat", findPattern);
  prefs.putUChar("findRep", findRepeat);
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
  prefs.putString("wifiSsid", wifiSsid);
  prefs.putString("wifiPass", wifiPass);
  prefs.putString("fwUrl", fwUrl);

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
