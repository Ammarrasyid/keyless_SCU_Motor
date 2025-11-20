#include <Arduino.h>
#include <NimBLEDevice.h>

// ===================== CONFIG ==========================
static const char* TARGET_MAC  = "2a:07:98:31:df:e9";      // MAC BLE TNW-T26
static const char* TARGET_NAME = "TNW-T26";                // optional

// Relay pin
#define RELAY1_PIN 2     // <2 meter
#define RELAY2_PIN 4     // double-click
#define RELAY3_PIN 5     // single-click

// Distance threshold
#define RSSI_NEAR_THRESHOLD -105     // approx < 2 meter
#define RSSI_FAR_THRESHOLD  -120     // hysteresis (lebih kecil = lebih jauh)

// RSSI smoothing
float rssiAvg = -100;
unsigned long lastRssiUpdate = 0;

// Notify characteristic pointer
NimBLERemoteCharacteristic* notifyChar = nullptr;

// ====== STATE DETEKSI CLICK ======
const unsigned long DOUBLE_CLICK_WINDOW = 700; // ms

// Menyimpan maksimal dua klik non-zero dalam satu window
uint8_t clickFirst  = 0;
uint8_t clickSecond = 0;
bool waitingClickEval = false;
unsigned long clickStartTime = 0;

// State jarak (Relay 1)
bool isNear = false;

// ========================================================


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

    // Kita cuma pakai 1 dan 2 sebagai event click
    if (value == 1 || value == 2) {
        // Klik pertama: mulai window
        if (!waitingClickEval) {
            waitingClickEval = true;
            clickStartTime   = now;
            clickFirst       = (uint8_t)value;
            clickSecond      = 0;
            Serial.printf("[EVENT] First click = %u\n", value);
        }
        // Klik kedua: simpan kalau slot kedua masih kosong
        else if (clickSecond == 0) {
            clickSecond = (uint8_t)value;
            Serial.printf("[EVENT] Second click = %u\n", value);
        }
    }

    // value == 0 (release) → tidak dipakai untuk logika, cukup diabaikan
}


// ===================== Client Callbacks =================
class ClientCallbacks : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pClient) override {
        Serial.printf(">> CONNECTED to %s\n",
                      pClient->getPeerAddress().toString().c_str());
        // Saat reconnect, biarkan isNear tetap false, nanti RSSI logic yang nyalakan Relay1
    }

    void onDisconnect(NimBLEClient* pClient, int reason) override {
        Serial.printf(">> DISCONNECTED (reason=%d). Restart scan.\n", reason);

        // Pastikan Relay1 mati saat disconnect
        isNear = false;
        digitalWrite(RELAY1_PIN, LOW);

        notifyChar = nullptr; // reset characteristic
        NimBLEDevice::getScan()->start(5000);
    }
} clientCallbacks;


// ===================== Scan Callback ====================
class ScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* dev) override {

        // Serial.printf("SCAN: %s\n", dev->toString().c_str());

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
                    // cukup pakai 1 notify char, boleh return di sini kalau mau
                }
            }
        }
    }
}


// ===================== SETUP =============================
void setup() {
    Serial.begin(115200);
    Serial.println("=== ESP32 BLE Async Client (TNW-T26 CONTROL) ===");

    pinMode(RELAY1_PIN, OUTPUT);
    pinMode(RELAY2_PIN, OUTPUT);
    pinMode(RELAY3_PIN, OUTPUT);
    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);
    digitalWrite(RELAY3_PIN, LOW);

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
void loop() {
    auto clients = NimBLEDevice::getConnectedClients();
    if (!clients.size()) return;

    NimBLEClient* client = clients[0];

    // ---- DISCOVERY ----
    if (!notifyChar) {
        discoverServices(client);
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
                Serial.println("[ACTION] DOUBLE CLICK → Relay 2 BLINK");
                // Blink Relay 2 dua kali
                digitalWrite(RELAY2_PIN, HIGH);
                delay(300);
                digitalWrite(RELAY2_PIN, LOW);
                delay(200);
                digitalWrite(RELAY2_PIN, HIGH);
                delay(300);
                digitalWrite(RELAY2_PIN, LOW);
            }
            // Single click: cuma klik pertama yang ada (second = 0)
            else if (first != 0 && second == 0) {
                Serial.println("[ACTION] SINGLE CLICK → Relay 3 BLINK 2x");
                for (int i = 0; i < 2; i++) {
                    digitalWrite(RELAY3_PIN, HIGH);
                    delay(120);
                    digitalWrite(RELAY3_PIN, LOW);
                    delay(120);
                }
            }
        }
    }

    // ---- RSSI SMOOTHING ----
    if (millis() - lastRssiUpdate >= 300) {
        lastRssiUpdate = millis();
        int rssi = client->getRssi();

        const float alpha = 0.2;
        rssiAvg = alpha * rssi + (1 - alpha) * rssiAvg;

        Serial.printf("[RSSI] raw=%d  avg=%.1f\n", rssi, rssiAvg);
    }

    // ---- DISTANCE ACTION (Relay1) ----
    if (!isNear && rssiAvg >= RSSI_NEAR_THRESHOLD) {
        isNear = true;
        digitalWrite(RELAY1_PIN, HIGH);
        Serial.println("[DIST] <2m → Relay1 ON");
    }
    else if (isNear && rssiAvg <= RSSI_FAR_THRESHOLD) {
        isNear = false;
        digitalWrite(RELAY1_PIN, LOW);
        Serial.println("[DIST] >2m → Relay1 OFF");
    }

    delay(10);
}
