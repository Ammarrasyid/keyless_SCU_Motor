#include <Arduino.h>
#include <NimBLEDevice.h>
#include <cstring>  // untuk memcmp manufacturer data

// ======================================================================
//  OPSI MODE / DEBUG
// ======================================================================

// Uncomment kalau mau mode scan-only (lihat MAC, services, MFG):
// #define ScanForGetMac

// Uncomment kalau mau print payload BLE detail (HEX, battery, klik dll):
// #define ReadMessage

// Master switch untuk log verbose (state internal, discover, dsb.)
#define DEBUG_VERBOSE 0

#if DEBUG_VERBOSE
  #define DBG(...)    Serial.printf(__VA_ARGS__)
  #define DBGLN(x)    Serial.println(x)
#else
  #define DBG(...)
  #define DBGLN(x)
#endif

// Banyak ESP32-C3 Super Mini: LED builtin di GPIO8, aktif LOW
#define LED_BUILTIN 8

// ======================================================================
//  MODE SCAN-ONLY: ScanForGetMac
// ======================================================================
#ifdef ScanForGetMac

class ScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* dev) override {
        String addr = dev->getAddress().toString().c_str();
        String name = dev->haveName() ? dev->getName().c_str() : String("<no name>");
        int rssi = dev->getRSSI();

        Serial.printf("Device: %-20s  MAC: %s  RSSI: %d dBm\n",
                      name.c_str(), addr.c_str(), rssi);

        // Service UUID yang di-adv
        if (dev->haveServiceUUID()) {
            Serial.print("  Services: ");
            uint8_t count = dev->getServiceUUIDCount();
            for (uint8_t i = 0; i < count; ++i) {
                NimBLEUUID u = dev->getServiceUUID(i);
                Serial.print(u.toString().c_str());
                if (i < count - 1) Serial.print(", ");
            }
            Serial.println();
        }

        // Manufacturer Data
        std::string mfg = dev->getManufacturerData();
        if (!mfg.empty()) {
            Serial.print("  MFG: ");
            for (size_t i = 0; i < mfg.size(); ++i) {
                Serial.printf("%02X ", (uint8_t)mfg[i]);
            }
            Serial.println();
        }

        Serial.println();
    }

    void onScanEnd(const NimBLEScanResults& results, int reason) override {
        Serial.println("Scan ended, restarting...");
        // start(durationSeconds, isContinue, restart)
        NimBLEDevice::getScan()->start(0, true, false); // 0 = tanpa timeout
    }
} scanCallbacks;

void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32-C3 SCAN FOR GET MAC / SERVICE / MFG ===");

    pinMode(LED_BUILTIN, OUTPUT);

    NimBLEDevice::init("ScanMAC-C3");
    NimBLEDevice::setPower(3);  // +3 dB

    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(&scanCallbacks);
    scan->setInterval(45);
    scan->setWindow(30);
    scan->setActiveScan(true);

    scan->start(0, true, false);
}

void loop() {
    static unsigned long lastBlink = 0;
    static bool led = false;
    unsigned long now = millis();

    if (now - lastBlink >= 500) {
        lastBlink = now;
        led = !led;
        digitalWrite(LED_BUILTIN, led ? LOW : HIGH); // aktif LOW
    }

    delay(50);
}

#else   // ======================= MODE NORMAL (CONTROL ITAG) =======================


// ======================================================================
//  KONFIGURASI BLE / ITAG
// ======================================================================

// MAC iTAG dari hasil scan
static const char* TARGET_MAC  = "f4:a9:05:54:53:48";

// Service / Char iTAG untuk tombol
static const uint16_t ITAG_SERVICE_UUID = 0xFFE0;
static const uint16_t ITAG_CHAR_UUID    = 0xFFE1;

// Battery Service
static const uint16_t BATTERY_SERVICE_UUID = 0x180F;
static const uint16_t BATTERY_CHAR_UUID    = 0x2A19;

// Manufacturer Data prefix dari iTAG
// Hasil scan: 05 01 F4 A9 05 54 53 48 66 02 01 03 00
// Pakai 8 byte pertama buat anti-spoof MAC clone
static const uint8_t ITAG_MFG_PREFIX[] = {
    0x05, 0x01, 0xF4, 0xA9, 0x05, 0x54, 0x53, 0x48
};
static const size_t ITAG_MFG_PREFIX_LEN =
    sizeof(ITAG_MFG_PREFIX) / sizeof(ITAG_MFG_PREFIX[0]);

// ======================================================================
//  PIN ESP32-C3 SUPER MINI (sesuaikan dengan wiring lo)
// ======================================================================
#define CONTACT_RELAY    0      // Kontak utama
#define SEIN_RELAY       1      // Sein (single click iTAG)
#define HORN_RELAY       4      // Horn (double/multi click iTAG)
#define INDICATOR_LED    3      // LED indikator (manual / low batt / dimming)
#define CONTACT_TRIGGER  10     // Tombol trigger ke GND (INPUT_PULLUP)

// ======================================================================
//  JARAK (RSSI) & CONTACT
// ======================================================================
#define RSSI_NEAR_THRESHOLD -70
#define RSSI_FAR_THRESHOLD  -72

float rssiAvg = -100.0f;
unsigned long lastRssiUpdate = 0;

bool bleConnected = false;
bool isNear       = false;
uint8_t nearFalseCount = 0;   // hitung berapa kali status isNear = false pada update RSSI

bool contactActive          = false;
unsigned long contactOnStartMs = 0;
// Durasi kontak: auto 3 detik, manual 5 detik
const unsigned long CONTACT_AUTO_ON_MS   = 3UL * 1000UL;
const unsigned long CONTACT_MANUAL_ON_MS = 7UL * 1000UL;
unsigned long contactDurationMs = CONTACT_AUTO_ON_MS;
// Tambahan: penanda bahwa di sesi BLE ini kontak PERNAH ON
bool sessionHadContact = false;

// ======================================================================
//  BATTERY STATE
// ======================================================================
int  batteryPercent      = -1;
bool batteryLow          = false;
unsigned long lastBattPollMs = 0;
const unsigned long BATTERY_POLL_MS = 60000; // 60 detik

// ======================================================================
//  TOMBOL TRIGGER FISIK
// ======================================================================
bool lastPhysicalState = HIGH;
bool stableState       = HIGH;
unsigned long lastChangeMs = 0;
const unsigned long DEBOUNCE_MS = 30;

// ======================================================================
//  KLIK ITAG → SINGLE / MULTI
// ======================================================================
volatile uint8_t  clickCount    = 0;
volatile unsigned long lastClickMs = 0;
unsigned long lastBtnDedupMs    = 0;
const unsigned long BTN_DEBOUNCE_MS  = 150;
const unsigned long CLICK_WINDOW_MS  = 400;

// ======================================================================
//  MODE MANUAL: TRIPLE TRIGGER + PIN 2-3-1-0
// ======================================================================
enum ManualState {
    MANUAL_IDLE,
    MANUAL_CODE
};
ManualState manualState = MANUAL_IDLE;

uint8_t       activationCount    = 0;
unsigned long activationStartMs  = 0;
const unsigned long ACTIVATION_WINDOW_MS = 2000;

// PIN: 2-3-1-0
const uint8_t CODE_LEN               = 4;
const uint8_t CODE_PATTERN[CODE_LEN] = {2, 3, 1, 0};

uint8_t       manualIndex     = 0;
uint8_t       digitPressCount = 0;
unsigned long digitStartMs    = 0;
const unsigned long DIGIT_WINDOW_MS = 5000;

// ======================================================================
//  KARAKTERISTIK BLE (BUTTON & BATTERY)
// ======================================================================
NimBLERemoteCharacteristic* gButtonChar = nullptr;
NimBLERemoteCharacteristic* gBattChar   = nullptr;

// ======================================================================
//  PWM / DIMMING UNTUK INDICATOR_LED (pakai analogWrite)
// ======================================================================

// Dimming “nafas”: level 0–255 (0 = OFF, 255 = full brightness, secara logis)
// Karena LED aktif LOW, nanti dibalik di indicatorSet().
uint8_t       indicatorLevel         = 0;
bool          indicatorDimmingActive = false;
bool          indicatorDimmingUp     = true;
unsigned long lastDimStepMs          = 0;
const uint8_t       DIM_MIN               = 30;
const uint8_t       DIM_MAX               = 200;
const uint8_t       DIM_STEP              = 2;
const unsigned long DIM_STEP_INTERVAL_MS  = 10;

// state untuk low battery blinking
unsigned long lastBattBlinkMs = 0;
bool          battBlinkState  = false;

// helper: set indikator via analogWrite (0..255 logical)
// LED aktif LOW → duty fisik = 255 - level
inline void indicatorSet(uint8_t level) {
    analogWrite(INDICATOR_LED, 255 - level);
}

// ======================================================================
//  UTILITAS: klasifikasi jarak (buat log doang)
// ======================================================================
const char* classifyDistance(float rssi) {
    if (rssi >= -60)  return "VERY_NEAR (~0.5 m)";
    if (rssi >= -70)  return "NEAR (~1-2 m)";
    if (rssi >= -80)  return "MID (~2-4 m)";
    if (rssi >= -90)  return "FAR (~4-8 m)";
    return "VERY_FAR (>8 m)";
}

// ======================================================================
//  UTILITAS: blink LED indikator (blocking, dipakai event penting)
// ======================================================================
void ledBlink(uint8_t times, int onMs, int offMs) {
    for (uint8_t i = 0; i < times; i++) {
        indicatorSet(255);   // ON
        delay(onMs);
        indicatorSet(0);     // OFF
        if (i < times - 1) delay(offMs);
    }
}

// ======================================================================
//  NOTIFY CALLBACK
// ======================================================================
void notifyCallback(NimBLERemoteCharacteristic* chr,
                    uint8_t* data,
                    size_t len,
                    bool isNotify)
{
    if (len == 0) return;

    NimBLEUUID chrId = chr->getUUID();

    // Battery characteristic (2A19)
    if (chrId.equals(NimBLEUUID(BATTERY_CHAR_UUID))) {
        uint8_t level = data[0];
        batteryPercent = level;
        batteryLow     = (level < 20);

#ifdef ReadMessage
        Serial.printf("[BATT-NOTIFY] level=%u%%  low=%d\n", level, batteryLow);
#endif
        return;
    }

    // Button iTAG (FFE1)
    if (!chrId.equals(NimBLEUUID(ITAG_CHAR_UUID))) {
        return; // bukan dari karakteristik tombol
    }

    uint8_t val       = data[0];
    unsigned long now = millis();

#ifdef ReadMessage
    Serial.println();
    Serial.println("=== Incoming iTAG Data ===");
    Serial.print("HEX  : ");
    for (size_t i = 0; i < len; ++i) {
        Serial.printf("%02X ", (uint8_t)data[i]);
    }
    Serial.println();
    Serial.printf("VAL  : %u\n", val);
    Serial.println("==========================");
#endif

    // iTAG: biasanya kirim 0x01 untuk klik (kadang 2 notif/klik)
    if (val == 0x01) {
        if (now - lastBtnDedupMs < BTN_DEBOUNCE_MS) {
            // notif duplikat klik yang sama → diabaikan
            return;
        }
        lastBtnDedupMs = now;

        clickCount++;
        lastClickMs = now;
    }
}

// ======================================================================
//  CLIENT CALLBACKS
// ======================================================================
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) override {
        Serial.printf(">> CONNECTED to %s\n",
                      pClient->getPeerAddress().toString().c_str());
        bleConnected = true;
    }

    void onDisconnect(NimBLEClient* pClient, int reason) override {
        Serial.printf(">> DISCONNECTED (reason=%d). Restart scan.\n", reason);

        bleConnected      = false;
        isNear            = false;
        nearFalseCount    = 0;
        contactActive     = false;
        sessionHadContact = false;          // <-- reset flag sesi
        digitalWrite(CONTACT_RELAY, LOW);

        manualState       = MANUAL_IDLE;
        activationCount   = 0;
        manualIndex       = 0;
        digitPressCount   = 0;

        gButtonChar = nullptr;
        gBattChar   = nullptr;

        // pastikan indikator juga kembali normal
        indicatorDimmingActive = false;
        battBlinkState         = false;
        indicatorSet(0);

        NimBLEDevice::getScan()->start(5000);
    }
} clientCallbacks;

// ======================================================================
//  SCAN CALLBACKS (FILTER MAC + SERVICE + MFG PREFIX)
// ======================================================================
class ScanCallbacks : public NimBLEScanCallbacks {
    bool matchManufacturer(const NimBLEAdvertisedDevice* dev) {
        if (ITAG_MFG_PREFIX_LEN == 0) return true;

        std::string mfg = dev->getManufacturerData();
        if (mfg.size() < ITAG_MFG_PREFIX_LEN) return false;

        return (memcmp(mfg.data(), ITAG_MFG_PREFIX, ITAG_MFG_PREFIX_LEN) == 0);
    }

    void onResult(const NimBLEAdvertisedDevice* dev) override {

        String mac = dev->getAddress().toString().c_str();
        if (mac != TARGET_MAC) {
            return; // MAC beda
        }

        // Harus advertise service FFE0
        if (!(dev->haveServiceUUID() &&
              dev->isAdvertisingService(NimBLEUUID(ITAG_SERVICE_UUID)))) {
            DBGLN(">> MATCH MAC tapi service FFE0 tidak ada → ignore");
            return;
        }

        // Filter manufacturer prefix
        if (!matchManufacturer(dev)) {
            DBGLN(">> MATCH MAC + service, MFG beda → ignore");
            return;
        }

        Serial.println(">> MATCH: TARGET DEVICE FOUND");

        NimBLEScan* scan = NimBLEDevice::getScan();
        scan->stop();

        NimBLEClient* client = NimBLEDevice::getDisconnectedClient();
        if (!client) {
            client = NimBLEDevice::createClient(dev->getAddress());
        }

        if (!client) {
            Serial.println("!! Cannot create BLE client");
            return;
        }

        client->setClientCallbacks(&clientCallbacks, false);

        if (!client->connect(true, true, false)) {
            Serial.println("!! Async connect failed");
            NimBLEDevice::deleteClient(client);
        }
    }

    void onScanEnd(const NimBLEScanResults& results, int reason) override {
        DBGLN("Scan ended, restarting...");
        NimBLEDevice::getScan()->start(5000);
    }
} scanCallbacks;

// ======================================================================
//  DISCOVER SERVICES: ambil langsung service/char FFE0/FFE1 & 180F/2A19
// ======================================================================
void discoverServices(NimBLEClient* client)
{
    DBGLN(">> Discovering services...");

    // Service tombol iTAG (FFE0)
    NimBLERemoteService* svcButton =
        client->getService(NimBLEUUID(ITAG_SERVICE_UUID));
    if (svcButton) {
        DBGLN("  SERVICE FFE0 found");
        NimBLERemoteCharacteristic* chrButton =
            svcButton->getCharacteristic(NimBLEUUID(ITAG_CHAR_UUID));
        if (chrButton && (chrButton->canNotify() || chrButton->canIndicate())) {
            DBGLN("  >> Subscribing BUTTON FFE1");
            if (chrButton->subscribe(true, notifyCallback, true)) {
                gButtonChar = chrButton;
                DBGLN("  >> BUTTON subscribed OK");
            } else {
                Serial.println("  !! BUTTON subscribe FAILED");
            }
        } else {
            Serial.println("  !! BUTTON char FFE1 tidak ditemukan / tidak bisa notify");
        }
    } else {
        Serial.println("!! SERVICE FFE0 (iTAG) tidak ditemukan");
    }

    // Battery service (180F) / 2A19
    NimBLERemoteService* svcBatt =
        client->getService(NimBLEUUID(BATTERY_SERVICE_UUID));
    if (svcBatt) {
        DBGLN("  SERVICE 180F (Battery) found");
        NimBLERemoteCharacteristic* chrBatt =
            svcBatt->getCharacteristic(NimBLEUUID(BATTERY_CHAR_UUID));
        if (chrBatt) {
            gBattChar = chrBatt;
            if (chrBatt->canNotify() || chrBatt->canIndicate()) {
                DBGLN("  >> Subscribing BATTERY 2A19");
                chrBatt->subscribe(true, notifyCallback, true);
            } else {
                DBGLN("  >> BATTERY 2A19 READ-ONLY");
            }
        } else {
            Serial.println("  !! BATTERY char 2A19 tidak ditemukan");
        }
    } else {
        Serial.println("!! SERVICE 180F (Battery) tidak ditemukan");
    }
}

// ======================================================================
//  MODE MANUAL: triple trigger + PIN 2-3-1-0
// ======================================================================
void resetManual(bool errorBlink) {
    manualState     = MANUAL_IDLE;
    activationCount = 0;
    manualIndex     = 0;
    digitPressCount = 0;

    if (errorBlink) {
        Serial.println("[MANUAL] Kode salah, reset");
        ledBlink(3, 100, 80);
    }
}

void startManualCode(unsigned long nowMs) {
    manualState     = MANUAL_CODE;
    manualIndex     = 0;
    digitPressCount = 0;
    digitStartMs    = nowMs;

    Serial.println("[MANUAL] Mode manual aktif, masukkan kode 2-3-1-0");
    ledBlink(3, 150, 150);
}

void processDigitTimeout(unsigned long nowMs) {
    if (manualState != MANUAL_CODE) return;
    if (nowMs - digitStartMs <= DIGIT_WINDOW_MS) return;

    uint8_t expected = CODE_PATTERN[manualIndex];
    uint8_t actual   = digitPressCount;

    DBG("[MANUAL] Digit %u: input=%u, expected=%u\n",
        manualIndex, actual, expected);

    if (actual != expected) {
        resetManual(true);
        return;
    }

    // digit benar → blink pemisah
    ledBlink(1, 150, 0);

    manualIndex++;
    if (manualIndex >= CODE_LEN) {
        Serial.println("[MANUAL] KODE BENAR, CONTACT ON 7 DETIK");
        ledBlink(3, 200, 150);

        contactActive     = true;
        contactDurationMs = CONTACT_MANUAL_ON_MS;
        contactOnStartMs  = nowMs;
        sessionHadContact = true;
        digitalWrite(CONTACT_RELAY, HIGH);

        resetManual(false);
    } else {
        digitPressCount = 0;
        digitStartMs    = nowMs;
    }
}

// Dipanggil saat trigger fisik ditekan (rising edge)
void handleTriggerPress(unsigned long nowMs) {
    // Kalau lagi input kode manual → hitung sebagai digit
    if (manualState == MANUAL_CODE) {
        digitPressCount++;
        DBG("[MANUAL] digitPressCount = %u\n", digitPressCount);
        return;
    }

    // triple click untuk masuk mode manual
    if (activationCount == 0) {
        activationStartMs = nowMs;
    }

    if (nowMs - activationStartMs > ACTIVATION_WINDOW_MS) {
        activationCount   = 0;
        activationStartMs = nowMs;
    }

    activationCount++;
    DBG("[MANUAL] activationCount = %u\n", activationCount);

    if (activationCount >= 3) {
        startManualCode(nowMs);
        activationCount = 0;
        return;
    }

    // MODE AUTO: sekali tekan + BLE connect + NEAR + kontak belum aktif
    if (bleConnected && isNear && !contactActive) {
        contactActive     = true;
        contactDurationMs = CONTACT_AUTO_ON_MS;
        contactOnStartMs  = nowMs;
        sessionHadContact = true;
        digitalWrite(CONTACT_RELAY, HIGH);
        Serial.println("[CONTACT] AUTO ON (BLE+near+trigger, 3 detik)");
    }
}

// ======================================================================
//  INDICATOR LED STATE MACHINE (LOW BATTERY + DIMMING CONTACT)
// ======================================================================
void updateIndicatorLed(unsigned long nowMs) {
    // Saat manual mode aktif, indikator dipakai oleh ledBlink → jangan ganggu
    if (manualState != MANUAL_IDLE) {
        indicatorDimmingActive = false;
        battBlinkState         = false;
        return;
    }

    // Indikator hanya aktif jika jarak valid (< threshold 2 m)
    if (!isNear) {
        indicatorDimmingActive = false;
        battBlinkState         = false;
        indicatorSet(0);
        return;
    }

    // PRIORITAS 1: low battery blink
    if (batteryLow) {
        indicatorDimmingActive = false;

        if (nowMs - lastBattBlinkMs >= 400) {
            lastBattBlinkMs = nowMs;
            battBlinkState  = !battBlinkState;
            indicatorSet(battBlinkState ? 255 : 0);
        }
        return;
    }

    // PRIORITAS 2: selama BLE connect dan di sesi ini CONTACT pernah ON → dimming
    if (bleConnected && sessionHadContact) {
        battBlinkState = false;

        if (!indicatorDimmingActive) {
            indicatorDimmingActive = true;
            indicatorDimmingUp     = true;
            indicatorLevel         = DIM_MIN;
            lastDimStepMs          = nowMs;
            indicatorSet(indicatorLevel);
        } else if (nowMs - lastDimStepMs >= DIM_STEP_INTERVAL_MS) {
            lastDimStepMs = nowMs;

            if (indicatorDimmingUp) {
                if (indicatorLevel + DIM_STEP >= DIM_MAX) {
                    indicatorLevel     = DIM_MAX;
                    indicatorDimmingUp = false;
                } else {
                    indicatorLevel += DIM_STEP;
                }
            } else {
                if (indicatorLevel <= DIM_MIN + DIM_STEP) {
                    indicatorLevel     = DIM_MIN;
                    indicatorDimmingUp = true;
                } else {
                    indicatorLevel -= DIM_STEP;
                }
            }
            indicatorSet(indicatorLevel);
        }
        return;
    }

    // PRIORITAS 3: default OFF
    indicatorDimmingActive = false;
    battBlinkState         = false;
    indicatorSet(0);
}

// ======================================================================
//  SETUP
// ======================================================================
unsigned long lastHBMs   = 0;
bool          hbLedState = false;

void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32-C3 SUPER MINI — iTAG CONTROL ===");

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CONTACT_RELAY, OUTPUT);
    pinMode(HORN_RELAY, OUTPUT);
    pinMode(SEIN_RELAY, OUTPUT);
    pinMode(CONTACT_TRIGGER, INPUT_PULLUP); // tombol → GND
    pinMode(INDICATOR_LED, OUTPUT);

    digitalWrite(CONTACT_RELAY, LOW);
    digitalWrite(HORN_RELAY, LOW);
    digitalWrite(SEIN_RELAY, LOW);

    // indikator OFF awal
    indicatorSet(0);

    NimBLEDevice::init("Async-Client-C3");
    NimBLEDevice::setPower(3);

    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(&scanCallbacks);
    scan->setInterval(45);
    scan->setWindow(45);
    scan->setActiveScan(true);
    scan->start(5000);
}

// ======================================================================
//  LOOP
// ======================================================================
void loop() {
    unsigned long nowMs = millis();
    bool rssiUpdated = false;

    // Heartbeat LED builtin (indikasi MCU hidup)
    if (nowMs - lastHBMs >= 500) {
        lastHBMs = nowMs;
        hbLedState = !hbLedState;
        digitalWrite(LED_BUILTIN, hbLedState ? LOW : HIGH); // aktif LOW
    }

    // ==================== LOGIKA YANG TIDAK BERGANTUNG BLE ====================

    // Tombol trigger fisik (debounce + rising edge)
    int reading = digitalRead(CONTACT_TRIGGER);  // HIGH idle, LOW pressed

    if (reading != lastPhysicalState) {
        lastChangeMs = nowMs;
        lastPhysicalState = reading;
    }

    if ((nowMs - lastChangeMs) > DEBOUNCE_MS && reading != stableState) {
        stableState = reading;
        if (stableState == LOW) {
            handleTriggerPress(nowMs);
        }
    }

    // Timeout digit PIN manual
    processDigitTimeout(nowMs);

    // CONTACT relay timeout (auto 3 detik, manual 5 detik)
    if (contactActive) {
        if (nowMs - contactOnStartMs >= contactDurationMs) {
            contactActive = false;
            digitalWrite(CONTACT_RELAY, LOW);
            Serial.println("[CONTACT] OFF (timeout)");
        }
    }

    // Update indikator (low battery / dimming contact)
    updateIndicatorLed(nowMs);

    // ===================== BAGIAN YANG BUTUH BLE CONNECT ======================

    auto clients = NimBLEDevice::getConnectedClients();
    if (clients.size() == 0) {
        // Manual mode tetap jalan walau BLE belum connect
        delay(5);
        return;
    }

    NimBLEClient* client = clients[0];

    // sekali discover sampai dapat button & battery char
    if (!gButtonChar && !gBattChar) {
        discoverServices(client);
        delay(50);
        return;
    }

    // EVALUASI single / multi click iTAG
    if (clickCount > 0 && (nowMs - lastClickMs > CLICK_WINDOW_MS)) {
        uint8_t count = clickCount;
        clickCount = 0;

        if (count == 1) {
            Serial.println("[ACTION] iTAG SINGLE CLICK → SEIN BLINK 2x");
            for (int i = 0; i < 2; i++) {
                digitalWrite(SEIN_RELAY, HIGH);
                delay(120);
                digitalWrite(SEIN_RELAY, LOW);
                delay(120);
            }
        } else {
            Serial.printf("[ACTION] iTAG MULTI (%u) → HORN BLINK 2x\n", count);
            digitalWrite(HORN_RELAY, HIGH);
            delay(300);
            digitalWrite(HORN_RELAY, LOW);
            delay(200);
            digitalWrite(HORN_RELAY, HIGH);
            delay(300);
            digitalWrite(HORN_RELAY, LOW);
        }
    }

    // RSSI smoothing
    if (nowMs - lastRssiUpdate >= 400) {
        lastRssiUpdate = nowMs;
        int rssi = client->getRssi();
        rssiUpdated = true;

        const float alpha = 0.2f;
        rssiAvg = alpha * rssi + (1.0f - alpha) * rssiAvg;

        static const char* lastZone = nullptr;
        const char* zone = classifyDistance(rssiAvg);
        if (zone != lastZone) {
            DBG("[DIST] RSSI avg=%.1f dBm → %s\n", rssiAvg, zone);
            lastZone = zone;
        }
    }

    // update isNear (hysteresis)
    if (!isNear && rssiAvg >= RSSI_NEAR_THRESHOLD) {
        isNear = true;
        nearFalseCount = 0;
        Serial.println("[DIST] <2m → NEAR = true");
    } else if (isNear && rssiAvg <= RSSI_FAR_THRESHOLD) {
        isNear = false;
        Serial.println("[DIST] >2m → NEAR = false");
    }

    // Reset sessionHadContact setelah 5x status isNear = false (mengacu pada update RSSI)
    if (rssiUpdated) {
        if (!isNear) {
            if (nearFalseCount < 20) {
                nearFalseCount++;
            }
            if (nearFalseCount == 20 && sessionHadContact) {
                sessionHadContact = false;
                Serial.println("[DIST] NEAR false x20 → sessionHadContact reset");
            }
        } else {
            nearFalseCount = 0;
        }
    }

    // polling battery (fallback kalau tidak pakai notify)
    if (bleConnected && gBattChar && (nowMs - lastBattPollMs >= BATTERY_POLL_MS)) {
        lastBattPollMs = nowMs;
        std::string val = gBattChar->readValue();
        if (!val.empty()) {
            uint8_t level = (uint8_t)val[0];
            batteryPercent = level;
            batteryLow     = (level < 20);
#ifdef ReadMessage
            Serial.printf("[BATT-POLL] level=%u%%  low=%d\n", level, batteryLow);
#endif
        }
    }

    delay(5);
}

#endif  // !ScanForGetMac
