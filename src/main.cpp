#include <Arduino.h>
#include <NimBLEDevice.h>
#include "esp_task_wdt.h"

// ===================== CONFIG ==========================
// BLE
static const char* TARGET_MAC  = "2a:07:98:31:df:e9";      // MAC BLE TNW-T26
static const char* TARGET_NAME = "TNW-T26";                // optional

// Relay & input pin
#define CONTACT_RELAY    18    // Kontak utama
#define HORN_RELAY       4     // Horn (double click)
#define SEIN_RELAY       5     // Sein (single click)
#define CONTACT_TRIGGER  15    // Input trigger kontak (tombol ke GND, INPUT_PULLUP)
#define INDICATOR_LED    19    // LED indikator mode manual / pemisah / error

// Distance threshold
#define RSSI_NEAR_THRESHOLD -105     // approx < 2 meter
#define RSSI_FAR_THRESHOLD  -120     // hysteresis

// RSSI smoothing
float rssiAvg = -100;
unsigned long lastRssiUpdate = 0;

// Notify characteristic pointer
NimBLERemoteCharacteristic* notifyChar = nullptr;

// ====== STATE DETEKSI CLICK BLE (REMOTE) ======
const unsigned long DOUBLE_CLICK_WINDOW = 700; // ms
uint8_t clickFirst  = 0;
uint8_t clickSecond = 0;
bool waitingClickEval = false;
unsigned long clickStartTime = 0;

// ====== STATE BLE & JARAK ======
bool bleConnected = false;
bool isNear       = false;

// ====== STATE CONTACT RELAY (AUTO & MANUAL) ======
bool contactActive = false;
unsigned long contactOnStartMs = 0;
const unsigned long CONTACT_MAX_ON_MS = 3UL * 60UL * 1000UL;  // 3 menit

// ====== WATCHDOG CONFIG ======
#define WDT_TIMEOUT_SEC 10

// ====== MANUAL MODE STATE (TRIGGER 3x + KODE 2-8-1-0) ======
enum ManualState {
    MANUAL_IDLE,
    MANUAL_CODE
};

ManualState manualState = MANUAL_IDLE;

// triple-click activation
uint8_t       activationCount    = 0;
unsigned long activationStartMs  = 0;
const unsigned long ACTIVATION_WINDOW_MS = 2000; // ms

// kode 2-8-1-0
const uint8_t CODE_LEN = 4;
const uint8_t CODE_PATTERN[CODE_LEN] = {2, 8, 1, 0};
uint8_t       manualIndex     = 0;  // index digit yang sedang diisi
uint8_t       digitPressCount = 0;  // jumlah tekan di digit saat ini
unsigned long digitStartMs    = 0;
const unsigned long DIGIT_WINDOW_MS = 3000; // ms per digit

// ====== BUTTON DEBOUNCE STATE ======
bool lastPhysicalState = HIGH;
bool stableState       = HIGH;
unsigned long lastChangeMs = 0;
const unsigned long DEBOUNCE_MS = 30;

// ========================================================
// ====== KLASIFIKASI JARAK DARI RSSI (MIRIP SCANNER HP) ======
const char* classifyDistance(float rssi) {
    if (rssi >= -60)  return "VERY_NEAR (~0.5 m)";
    if (rssi >= -70)  return "NEAR (~1-2 m)";
    if (rssi >= -80)  return "MID (~2-4 m)";
    if (rssi >= -90)  return "FAR (~4-8 m)";
    return "VERY_FAR (>8 m)";
}

// ===================== INDICATOR HELPERS ==================
void ledBlink(uint8_t times, int onMs, int offMs) {
    for (uint8_t i = 0; i < times; i++) {
        digitalWrite(INDICATOR_LED, HIGH);
        delay(onMs);
        digitalWrite(INDICATOR_LED, LOW);
        if (i < times - 1) delay(offMs);
    }
}

// ===================== Notify Callback (BLE remote) ======
void notifyCallback(NimBLERemoteCharacteristic* chr,
                    uint8_t* data,
                    size_t len,
                    bool isNotify)
{
    if (len < 2) return;

    uint16_t value = data[0] | (data[1] << 8);
    unsigned long now = millis();

    Serial.printf("[BLE] Value = %u\n", value);

    // Hanya gunakan 1 dan 2 sebagai klik
    if (value == 1 || value == 2) {
        if (!waitingClickEval) {
            waitingClickEval = true;
            clickStartTime   = now;
            clickFirst       = (uint8_t)value;
            clickSecond      = 0;
            Serial.printf("[EVENT] First click = %u\n", value);
        } else if (clickSecond == 0) {
            clickSecond = (uint8_t)value;
            Serial.printf("[EVENT] Second click = %u\n", value);
        }
    }
    // value == 0 (release) → diabaikan untuk logika
}

// ===================== Client Callbacks =================
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) override {
        Serial.printf(">> CONNECTED to %s\n",
                      pClient->getPeerAddress().toString().c_str());
        bleConnected = true;
    }

    void onDisconnect(NimBLEClient* pClient, int reason) override {
        Serial.printf(">> DISCONNECTED (reason=%d). Restart scan.\n", reason);

        bleConnected  = false;
        isNear        = false;
        contactActive = false;
        digitalWrite(CONTACT_RELAY, LOW);

        manualState       = MANUAL_IDLE;
        activationCount   = 0;
        manualIndex       = 0;
        digitPressCount   = 0;

        notifyChar = nullptr; // reset characteristic
        NimBLEDevice::getScan()->start(5000);
    }
} clientCallbacks;

// ===================== Scan Callback ====================
class ScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* dev) override {

        if (dev->getAddress().toString() == TARGET_MAC) {
            Serial.println(">> MATCH: TNW-T26 FOUND");

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
    }

    void onScanEnd(const NimBLEScanResults& results, int reason) override {
        Serial.println("Scan ended, restarting...");
        NimBLEDevice::getScan()->start(5000);
    }
} scanCallbacks;

// ===================== DISCOVER SERVICES =================
void discoverServices(NimBLEClient* client)
{
    Serial.println(">> Discovering services...");

    auto services = client->getServices(true);

    for (auto* svc : services) {
        Serial.printf("SERVICE: %s\n", svc->getUUID().toString().c_str());

        auto chars = svc->getCharacteristics(true);
        for (auto* chr : chars) {
            Serial.printf("  CHAR: %s [",
                          chr->getUUID().toString().c_str());

            if (chr->canNotify())   Serial.print(" NOTIFY");
            if (chr->canIndicate()) Serial.print(" INDICATE");
            if (chr->canRead())     Serial.print(" READ");
            if (chr->canWrite())    Serial.print(" WRITE");
            Serial.println(" ]");

            if (chr->canNotify() || chr->canIndicate()) {
                Serial.printf("  >> Subscribing: %s\n",
                              chr->getUUID().toString().c_str());

                if (chr->subscribe(true, notifyCallback, true)) {
                    notifyChar = chr;
                    // cukup satu notify
                }
            }
        }
    }
}

// ===================== MANUAL MODE HELPERS ==================
void resetManual(bool errorBlink) {
    manualState     = MANUAL_IDLE;
    activationCount = 0;
    manualIndex     = 0;
    digitPressCount = 0;

    if (errorBlink) {
        Serial.println("[MANUAL] Kode salah, reset");
        ledBlink(3, 100, 80); // blink cepat 3x
    }
}

// dipanggil saat triple-click sudah terdeteksi
void startManualCode(unsigned long nowMs) {
    manualState     = MANUAL_CODE;
    manualIndex     = 0;
    digitPressCount = 0;
    digitStartMs    = nowMs;

    Serial.println("[MANUAL] Mode manual aktif, masukkan kode 2-8-1-0");
    ledBlink(3, 150, 150); // indikator masuk manual
}

// proses digit saat window habis
void processDigitTimeout(unsigned long nowMs) {
    if (manualState != MANUAL_CODE) return;
    if (nowMs - digitStartMs <= DIGIT_WINDOW_MS) return;

    uint8_t expected = CODE_PATTERN[manualIndex];
    uint8_t actual   = digitPressCount;

    Serial.printf("[MANUAL] Digit %u: input=%u, expected=%u\n",
                  manualIndex, actual, expected);

    if (actual != expected) {
        // salah → reset total
        resetManual(true);
        return;
    }

    // digit benar → blink pemisah
    ledBlink(1, 150, 0);

    manualIndex++;
    if (manualIndex >= CODE_LEN) {
        // semua digit benar → sukses
        Serial.println("[MANUAL] KODE BENAR, CONTACT ON 3 MENIT");
        ledBlink(3, 200, 150);

        contactActive    = true;
        contactOnStartMs = nowMs;
        digitalWrite(CONTACT_RELAY, HIGH);

        resetManual(false); // keluar dari mode manual (tapi contact tetap ON)
    } else {
        // lanjut ke digit berikutnya
        digitPressCount = 0;
        digitStartMs    = nowMs;
    }
}

// dipanggil saat terdeteksi 1x tekan (rising edge)
void handleButtonPress(unsigned long nowMs) {
    // jika lagi input kode manual → hitung sebagai digit
    if (manualState == MANUAL_CODE) {
        digitPressCount++;
        Serial.printf("[MANUAL] digitPressCount = %u\n", digitPressCount);
        return;
    }

    // ====== TRIPLE-CLICK DETECTION (aktifkan manual mode) ======
    if (activationCount == 0) {
        activationStartMs = nowMs;
    }

    // kalau jeda terlalu lama, mulai hitungan baru
    if (nowMs - activationStartMs > ACTIVATION_WINDOW_MS) {
        activationCount   = 0;
        activationStartMs = nowMs;
    }

    activationCount++;

    Serial.printf("[MANUAL] activationCount = %u\n", activationCount);

    if (activationCount >= 3) {
        // masuk manual mode
        startManualCode(nowMs);
        activationCount = 0;
        return;
    }

    // ====== AUTO CONTACT MODE (tekan sekali) ======
    // Hanya kalau BLE connect + near + contact belum aktif
    if (bleConnected && isNear && !contactActive) {
        contactActive    = true;
        contactOnStartMs = nowMs;
        digitalWrite(CONTACT_RELAY, HIGH);
        Serial.println("[CONTACT] AUTO ON (BLE+near+trigger)");
    }
}

// ===================== SETUP =============================
void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32 BLE Async Client (TNW-T26 CONTROL) ===");

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(INDICATOR_LED, OUTPUT);
    pinMode(CONTACT_RELAY, OUTPUT);
    pinMode(HORN_RELAY, OUTPUT);
    pinMode(SEIN_RELAY, OUTPUT);
    pinMode(CONTACT_TRIGGER, INPUT_PULLUP); // tombol ke GND

    digitalWrite(CONTACT_RELAY, LOW);
    digitalWrite(HORN_RELAY, LOW);
    digitalWrite(SEIN_RELAY, LOW);
    digitalWrite(INDICATOR_LED, LOW);

    // Watchdog init untuk task utama (loop)
    esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
    esp_task_wdt_add(NULL);

    NimBLEDevice::init("Async-Client");
    NimBLEDevice::setPower(3);

    NimBLEScan* scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(&scanCallbacks);
    scan->setInterval(45);
    scan->setWindow(45);
    scan->setActiveScan(true);
    scan->start(5000);
}

// ===================== LOOP ==============================
unsigned long lastBlinkMs = 0;
bool ledState = false;

void loop() {
    // Feed watchdog
    esp_task_wdt_reset();

    unsigned long nowMs = millis();

    // Heartbeat LED_BUILTIN
    if (nowMs - lastBlinkMs >= 500) {
        lastBlinkMs = nowMs;
        ledState = !ledState;
        digitalWrite(LED_BUILTIN, ledState);
    }

    auto clients = NimBLEDevice::getConnectedClients();
    if (!clients.size()) {
        delay(20);
        return;
    }

    NimBLEClient* client = clients[0];

    // ---- DISCOVERY ----
    if (!notifyChar) {
        discoverServices(client);
        delay(50);
        return;
    }

    // ---- EVALUASI SINGLE / DOUBLE CLICK (REMOTE BLE) ----
    if (waitingClickEval) {
        if (nowMs - clickStartTime > DOUBLE_CLICK_WINDOW) {
            waitingClickEval = false;

            uint8_t first  = clickFirst;
            uint8_t second = clickSecond;

            clickFirst  = 0;
            clickSecond = 0;

            if ((first == 1 && second == 2) ||
                (first == 2 && second == 1))
            {
                Serial.println("[ACTION] DOUBLE CLICK → HORN BLINK 2x");
                digitalWrite(HORN_RELAY, HIGH);
                delay(300);
                digitalWrite(HORN_RELAY, LOW);
                delay(200);
                digitalWrite(HORN_RELAY, HIGH);
                delay(300);
                digitalWrite(HORN_RELAY, LOW);
            }
            else if (first != 0 && second == 0) {
                Serial.println("[ACTION] SINGLE CLICK → SEIN BLINK 2x");
                for (int i = 0; i < 2; i++) {
                    digitalWrite(SEIN_RELAY, HIGH);
                    delay(120);
                    digitalWrite(SEIN_RELAY, LOW);
                    delay(120);
                }
            }
        }
    }

    // ---- RSSI SMOOTHING & ZONA JARAK ----
    if (nowMs - lastRssiUpdate >= 700) {
        lastRssiUpdate = nowMs;
        int rssi = client->getRssi();

        const float alpha = 0.2f;
        rssiAvg = alpha * rssi + (1.0f - alpha) * rssiAvg;

        static const char* lastZone = nullptr;
        const char* zone = classifyDistance(rssiAvg);
        if (zone != lastZone) {
            Serial.printf("[DIST] RSSI avg=%.1f dBm → %s\n", rssiAvg, zone);
            lastZone = zone;
        }
    }

    // ---- UPDATE isNear (HYSTERESIS) ----
    if (!isNear && rssiAvg >= RSSI_NEAR_THRESHOLD) {
        isNear = true;
        Serial.println("[DIST] <2m → NEAR = true");
    } else if (isNear && rssiAvg <= RSSI_FAR_THRESHOLD) {
        isNear = false;
        Serial.println("[DIST] >2m → NEAR = false");
    }

    // ---- READ BUTTON + DEBOUNCE + RISING EDGE ----
    int reading = digitalRead(CONTACT_TRIGGER);  // HIGH idle, LOW pressed

    if (reading != lastPhysicalState) {
        lastChangeMs = nowMs;
        lastPhysicalState = reading;
    }

    if ((nowMs - lastChangeMs) > DEBOUNCE_MS && reading != stableState) {
        stableState = reading;
        if (stableState == LOW) {
            // tombol baru saja ditekan (rising edge logis)
            handleButtonPress(nowMs);
        }
    }

    // ---- PROSES TIMEOUT DIGIT KODE MANUAL ----
    processDigitTimeout(nowMs);

    // ---- CONTACT RELAY TIMEOUT (AUTO & MANUAL) ----
    if (contactActive) {
        if (nowMs - contactOnStartMs >= CONTACT_MAX_ON_MS) {
            contactActive = false;
            digitalWrite(CONTACT_RELAY, LOW);
            Serial.println("[CONTACT] OFF (timeout 3 menit)");
        }
    }

    delay(5);
}
