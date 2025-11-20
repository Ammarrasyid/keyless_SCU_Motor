#include <Arduino.h>
#include <NimBLEDevice.h>
#include "esp_task_wdt.h"

// ===================== CONFIG ==========================
// BLE
static const char* TARGET_MAC  = "2a:07:98:31:df:e9";      // MAC BLE TNW-T26
static const char* TARGET_NAME = "TNW-T26";                // optional

// Relay & input pin
#define CONTACT_RELAY    18    // Kontak utama
#define HORN_RELAY       4    // Horn (double click)
#define SEIN_RELAY       5    // Sein (single click)
#define CONTACT_TRIGGER 35    // Input trigger kontak

// Distance threshold
#define RSSI_NEAR_THRESHOLD -105     // approx < 2 meter
#define RSSI_FAR_THRESHOLD  -120     // hysteresis

// RSSI smoothing
float rssiAvg = -100;
unsigned long lastRssiUpdate = 0;

// Notify characteristic pointer
NimBLERemoteCharacteristic* notifyChar = nullptr;

// ====== STATE DETEKSI CLICK ======
const unsigned long DOUBLE_CLICK_WINDOW = 700; // ms
uint8_t clickFirst  = 0;
uint8_t clickSecond = 0;
bool waitingClickEval = false;
unsigned long clickStartTime = 0;

// ====== STATE BLE & JARAK ======
bool bleConnected = false;
bool isNear       = false;

// ====== STATE CONTACT RELAY ======
bool contactActive = false;
unsigned long contactOnStartMs = 0;
const unsigned long CONTACT_MAX_ON_MS = 3UL * 60UL * 1000UL;  // 3 menit

// ====== WATCHDOG CONFIG ======
#define WDT_TIMEOUT_SEC 10

// ========================================================
// ====== KLASIFIKASI JARAK DARI RSSI (MIRIP SCANNER HP) ======
const char* classifyDistance(float rssi) {
    if (rssi >= -60)  return "VERY_NEAR (~0.5 m)";
    if (rssi >= -70)  return "NEAR (~1-2 m)";
    if (rssi >= -80)  return "MID (~2-4 m)";
    if (rssi >= -90)  return "FAR (~4-8 m)";
    return "VERY_FAR (>8 m)";
}

// ===================== Notify Callback ==================
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
        // isNear akan di-update oleh RSSI
    }

    void onDisconnect(NimBLEClient* pClient, int reason) override {
        Serial.printf(">> DISCONNECTED (reason=%d). Restart scan.\n", reason);

        bleConnected  = false;
        isNear        = false;
        contactActive = false;
        digitalWrite(CONTACT_RELAY, LOW);

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
                    // Bisa break di sini kalau cukup satu notify
                }
            }
        }
    }
}

// ===================== SETUP =============================
void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32 BLE Async Client (TNW-T26 CONTROL) ===");

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(CONTACT_RELAY, OUTPUT);
    pinMode(HORN_RELAY, OUTPUT);
    pinMode(SEIN_RELAY, OUTPUT);
    pinMode(CONTACT_TRIGGER, INPUT_PULLUP); // kalau pakai pull-up, ubah ke INPUT_PULLUP // perlu dikasih resistor eksternal

    digitalWrite(CONTACT_RELAY, LOW);
    digitalWrite(HORN_RELAY, LOW);
    digitalWrite(SEIN_RELAY, LOW);

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
unsigned long lastTime;
bool triggerState = false;
void loop() {
    // Feed watchdog
    esp_task_wdt_reset();

    // LED built-in toggle
    static bool loop = true;
    digitalWrite(LED_BUILTIN, loop);
    unsigned long timeNow = millis();
    if (timeNow - lastTime >= 500) {
        lastTime = timeNow;
        loop = !loop;
    }

    auto clients = NimBLEDevice::getConnectedClients();
    if (!clients.size()) {
        delay(50);
        return;
    }

    NimBLEClient* client = clients[0];

    // ---- DISCOVERY ----
    if (!notifyChar) {
        discoverServices(client);
        delay(50);
        return;
    }

    // ---- EVALUASI SINGLE / DOUBLE CLICK SETELAH WINDOW ----
    if (waitingClickEval) {
        unsigned long now = millis();
        if (now - clickStartTime > DOUBLE_CLICK_WINDOW) {
            waitingClickEval = false;

            uint8_t first  = clickFirst;
            uint8_t second = clickSecond;

            // reset buffer
            clickFirst  = 0;
            clickSecond = 0;

            // Double click: 1→2 atau 2→1
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
            // Single click
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
    if (millis() - lastRssiUpdate >= 700) {
        lastRssiUpdate = millis();
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

    // ---- CONTACT RELAY LOGIC ----
    triggerState = digitalRead(CONTACT_TRIGGER);
    // Serial.printf("[CONTACT] Trigger state = %d\n", triggerState);
    // if (digitalRead(CONTACT_TRIGGER) == 1) triggerState = true; // sesuaikan dengan wiring
    // static bool lastTriggerState = false;
    unsigned long nowMs = millis();

    if (contactActive) {
        bool timeout = (nowMs - contactOnStartMs >= CONTACT_MAX_ON_MS);

        if (timeout) {
            contactActive = false;
            digitalWrite(CONTACT_RELAY, LOW);
            Serial.println("[CONTACT] OFF (timeout)");
        }
    } else {
        // Aktifkan kontak jika semua kondisi terpenuhi
        if (bleConnected && isNear && triggerState) {
            contactActive     = true;
            contactOnStartMs  = nowMs;
            digitalWrite(CONTACT_RELAY, HIGH);
            Serial.println("[CONTACT] ON (trigger + BLE + near)");
        }
    }
    delay(10);
}
