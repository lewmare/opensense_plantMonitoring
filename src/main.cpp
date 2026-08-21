/*
 * ESP32 Environmental & Soil Monitoring System — 24/7 Optimized
 * -----------------------------------------------------------
 * Sensors : DHT11 (fallback temp/humidity), BMP280 (pressure/temp/alt),
 *           BH1750 (lux), capacitive soil moisture (analog), DS18B20 (soil temp)
 * Actuator: Relay-driven water pump
 * Network : WiFi + HTTP POST (JSON) to backend API
 *
 * Changes vs. previous version, specifically for unattended 24/7 operation:
 *  - Task watchdog (esp_task_wdt) added: if loop() ever hangs, the chip
 *    self-resets instead of staying dead forever.
 *  - Removed all String-based JSON building (was allocating temporary
 *    String objects every 5s -> heap fragmentation over days/weeks).
 *    ArduinoJson now writes floats natively.
 *  - checkPumpCommand() now uses ArduinoJson deserialization instead of
 *    manual indexOf/substring parsing (fragile and could misbehave if the
 *    response format or whitespace ever changes).
 *  - Firmware-side hard cap (MAX_PUMP_DURATION_MS) on relay-on time,
 *    independent of whatever the server says — the physical device should
 *    never fully trust the network for something that can flood a plant.
 *  - Irrigation cooldown lockout added: after a watering cycle, the
 *    auto-irrigation logic will not retrigger for IRRIGATION_COOLDOWN_MS,
 *    preventing relay chatter while the soil is still absorbing water.
 *  - Soil history buffer is cleared after a triggered watering, so stale
 *    "dry" samples can't cause an instant retrigger.
 *  - Explicit HTTP timeouts added to checkPumpCommand() (previously only
 *    sendDataToServer() had them).
 *  - Periodic free-heap logging + scheduled safe self-restart every
 *    SAFE_RESTART_INTERVAL_MS as a safety net against slow leaks that
 *    are otherwise invisible until the device eventually crashes.
 *  - Minor comment/constant fixes (CHECK_INTERVAL comment now matches
 *    the actual 2000ms value).
 * -----------------------------------------------------------
 */

#include <Arduino.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>        // Install via Library Manager: "ArduinoJson" by Benoit Blanchon
#include <OneWire.h>            // Install via Library Manager: "OneWire" by Paul Stoffregen
#include <DallasTemperature.h>  // Install via Library Manager: "DallasTemperature" by Miles Burton
#include <esp_task_wdt.h>       // Built into ESP32 core

#include "secrets.h"  // defines WIFI_SSID, WIFI_PASSWORD, SERVER_URL, Pump_API_URL

// ---------------------------------------------------------------------
// Pin & hardware configuration
// ---------------------------------------------------------------------
namespace Pins {
  constexpr int SOIL_ANALOG  = 34;
  constexpr int RELAY        = 25;
  constexpr int DHT_DATA     = 15;
  constexpr int I2C_SDA      = 21;
  constexpr int I2C_SCL      = 22;
  constexpr int ONE_WIRE_BUS = 32; // DS18B20
}

constexpr uint8_t DHT_TYPE        = DHT11;
constexpr uint8_t BMP280_I2C_ADDR = 0x76;

// ---------------------------------------------------------------------
// Timing configuration (all in ms)
// ---------------------------------------------------------------------
namespace Interval {
  constexpr unsigned long SEND_DATA  = 5000;
  constexpr unsigned long SOIL_CHECK = 1000;
  constexpr unsigned long ENV_CHECK  = 4000;
  constexpr unsigned long RELAY_ON   = 2000;
  constexpr unsigned long WIFI_RETRY = 10000; // how often to retry a dropped connection
}

// ---------------------------------------------------------------------
// Soil / irrigation configuration
// ---------------------------------------------------------------------
namespace SoilCfg {
  constexpr int   ADC_DRY            = 3100; // raw reading in dry air
  constexpr int   ADC_WET            = 1000; // raw reading fully submerged
  constexpr int   MOISTURE_THRESHOLD = 30;   // % below which irrigation may trigger
  constexpr int   SAMPLE_COUNT       = 20;   // averaged analogRead samples per reading
  constexpr int   HISTORY_SIZE       = 3;    // samples used for the moving average
}

constexpr float SEA_LEVEL_HPA = 1013.25f;

// ---------------------------------------------------------------------
// Safety configuration (24/7 unattended operation)
// ---------------------------------------------------------------------
namespace Safety {
  // Firmware-side hard ceiling on any single watering cycle, regardless of
  // what a server/command requests. Protects against a compromised or
  // buggy backend flooding the plant.
  constexpr unsigned long MAX_PUMP_DURATION_MS = 30000; // 30s

  // Minimum time between two auto-irrigation triggers. Soil moisture takes
  // minutes to rise after watering, so without this the relay could
  // chatter on/off repeatedly right after a cycle finishes.
  constexpr unsigned long IRRIGATION_COOLDOWN_MS = 10UL * 60UL * 1000UL; // 10 min

  // Self-restart on a schedule as a safety net against slow, hard-to-spot
  // heap leaks — common practice for devices that run unattended for
  // weeks/months. Restart is only performed when the relay is off, so a
  // watering cycle in progress is never interrupted.
  constexpr unsigned long SAFE_RESTART_INTERVAL_MS = 48UL * 60UL * 60UL * 1000UL; // 48h

  // Below this, we start logging warnings; below half of this, we force a
  // restart at the next safe opportunity (relay off).
  constexpr uint32_t LOW_HEAP_WARN_BYTES  = 20000;
  constexpr uint32_t LOW_HEAP_FORCE_BYTES = 10000;

  constexpr unsigned long HEAP_LOG_INTERVAL_MS = 60000; // 1 min

  constexpr int WDT_TIMEOUT_S = 15; // reset chip if loop() doesn't feed the watchdog in time
}

// ---------------------------------------------------------------------
// Global objects
// ---------------------------------------------------------------------
DHT dht(Pins::DHT_DATA, DHT_TYPE);
Adafruit_BMP280 bmp;
BH1750 lightMeter;
OneWire oneWire(Pins::ONE_WIRE_BUS);
DallasTemperature dallas(&oneWire);

bool bmpAvailable    = false;
bool bh1750Available = false;

struct SensorData {
  int   soilRaw            = 0;
  int   moisturePercentage = 0;
  float humidity           = 0;
  float temperature        = 0;
  float pressure            = 0;
  float lightIntensity       = 0;
  float altitude              = 0;
  float soilTemperature       = 0;
} s_data;

// Circular buffer for soil moving average
int  soilHistory[SoilCfg::HISTORY_SIZE] = {0};
int  soilHistoryIndex = 0;
bool soilHistoryFull  = false;

bool relayOn = false;
unsigned long relayStartTime = 0;
unsigned long vlt_pumpInterval = 0;

// When the current cooldown window ends. 0 means "no cooldown active".
unsigned long irrigationCooldownUntil = 0;

unsigned long lastSendTime    = 0;
unsigned long lastSoilCheck   = 0;
unsigned long lastEnvCheck    = 0;
unsigned long lastWifiAttempt = 0;
unsigned long lastHeapLog     = 0;
unsigned long bootTime        = 0;

bool restartRequested = false; // set when heap is critically low; applied at next safe moment

namespace PumpAPI {
  unsigned long lastCheckTime = 0;
  constexpr unsigned long CHECK_INTERVAL = 2000; // Check pump API every 2 seconds
}

// ---------------------------------------------------------------------
// WiFi (non-blocking connect/reconnect)
// ---------------------------------------------------------------------
void beginWifiConnection() {
  Serial.printf("Connecting to WiFi \"%s\"...\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false); // lower latency for HTTP requests
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) return;

  unsigned long now = millis();
  if (now - lastWifiAttempt < Interval::WIFI_RETRY) return; // don't spam reconnects

  lastWifiAttempt = now;
  Serial.println("WiFi disconnected, retrying...");
  WiFi.disconnect();
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

// ---------------------------------------------------------------------
// Sensor helpers
// ---------------------------------------------------------------------
int readSoilRaw() {
  long sum = 0;
  for (int i = 0; i < SoilCfg::SAMPLE_COUNT; i++) {
    sum += analogRead(Pins::SOIL_ANALOG);
    delay(5);
  }
  return sum / SoilCfg::SAMPLE_COUNT;
}

void pushSoilHistory(int value) {
  soilHistory[soilHistoryIndex] = value;
  soilHistoryIndex = (soilHistoryIndex + 1) % SoilCfg::HISTORY_SIZE;
  if (soilHistoryIndex == 0) soilHistoryFull = true;
}

void resetSoilHistory() {
  for (int i = 0; i < SoilCfg::HISTORY_SIZE; i++) soilHistory[i] = 0;
  soilHistoryIndex = 0;
  soilHistoryFull  = false;
}

// Turns the relay on for `requestedMs`, clamped to Safety::MAX_PUMP_DURATION_MS
// regardless of the caller (manual command or auto-irrigation).
void TurnRelay(unsigned long requestedMs) {
  if (relayOn) {
    Serial.println("Pump is still on, ignoring new request.");
    return;
  }

  unsigned long safeMs = requestedMs;
  if (safeMs > Safety::MAX_PUMP_DURATION_MS) {
    Serial.printf("Requested pump duration %lums exceeds safety cap, clamping to %lums.\n",
                  requestedMs, Safety::MAX_PUMP_DURATION_MS);
    safeMs = Safety::MAX_PUMP_DURATION_MS;
  }

  vlt_pumpInterval = safeMs;
  digitalWrite(Pins::RELAY, HIGH);
  relayOn = true;
  relayStartTime = millis();
}

void readEnvironmentSensors() {
  float dhtHumidity    = dht.readHumidity();
  float dhtTemperature = dht.readTemperature();

  float bmpPressure = bmpAvailable ? bmp.readPressure() / 100.0F : NAN;
  float bmpTemp     = bmpAvailable ? bmp.readTemperature() : NAN;
  float bmpAltitude = bmpAvailable ? bmp.readAltitude(SEA_LEVEL_HPA) : NAN;
  float lux          = bh1750Available ? lightMeter.readLightLevel() : NAN;

  dallas.requestTemperatures();
  float soiltemp = dallas.getTempCByIndex(0);

  s_data.humidity        = isnan(dhtHumidity) ? 0.0f : dhtHumidity;
  s_data.temperature     = !isnan(bmpTemp) ? bmpTemp : (isnan(dhtTemperature) ? 0.0f : dhtTemperature);
  s_data.pressure        = isnan(bmpPressure) ? 0.0f : bmpPressure;
  s_data.altitude        = isnan(bmpAltitude) ? 0.0f : bmpAltitude;
  s_data.lightIntensity  = (isnan(lux) || lux < 0) ? 0.0f : lux;
  s_data.soilTemperature = (soiltemp == DEVICE_DISCONNECTED_C) ? 0.0f : soiltemp;

  Serial.printf("DHT11  -> Hum: %.1f%%, Temp: %.1f C\n", s_data.humidity, dhtTemperature);
  Serial.printf("BMP280 -> Press: %.1f hPa, Temp: %.1f C, Alt: %.1f m\n",
                s_data.pressure, s_data.temperature, s_data.altitude);
  Serial.printf("BH1750 -> Light: %.1f lux\n", s_data.lightIntensity);
  Serial.printf("DS18B20 -> Soil Temp: %.1f C\n", s_data.soilTemperature);
  Serial.println("------------------------------------");
}

void handleSoilAndIrrigation() {
  int raw = readSoilRaw();
  int pct = map(raw, SoilCfg::ADC_WET, SoilCfg::ADC_DRY, 100, 0);
  pct = constrain(pct, 0, 100);

  s_data.soilRaw = raw;
  s_data.moisturePercentage = pct;
  Serial.printf("Soil Raw: %d, Moisture: %d%%\n", raw, pct);

  pushSoilHistory(pct);

  bool consistentlyDry = soilHistoryFull;
  for (int i = 0; i < SoilCfg::HISTORY_SIZE && consistentlyDry; i++) {
    if (soilHistory[i] >= SoilCfg::MOISTURE_THRESHOLD) consistentlyDry = false;
  }

  bool cooldownActive = (long)(millis() - irrigationCooldownUntil) < 0;

  if (consistentlyDry && !relayOn && !cooldownActive) {
    Serial.println("Soil is dry -> turning relay ON (auto-irrigation).");
    TurnRelay(Interval::RELAY_ON);

    // Clear stale "dry" samples so the next few checks don't instantly
    // see a full-dry buffer again, and start the cooldown window.
    resetSoilHistory();
    irrigationCooldownUntil = millis() + Safety::IRRIGATION_COOLDOWN_MS;
  } else if (cooldownActive) {
    Serial.println("Soil condition dry, but irrigation cooldown active — skipping.");
  } else {
    Serial.println("Soil condition normal.");
  }
}

void updateRelayTimeout() {
  if (relayOn && millis() - relayStartTime >= vlt_pumpInterval) {
    digitalWrite(Pins::RELAY, LOW);
    relayOn = false;
    Serial.println("Relay turned off after duration.");
  }
}

// ---------------------------------------------------------------------
// Networking: send sensor data as JSON
// ---------------------------------------------------------------------
void sendDataToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected. Skipping send.");
    return;
  }

  // Native numeric fields — no temporary String objects, avoids heap
  // fragmentation from building/discarding Strings every 5 seconds for
  // days/weeks at a time.
  JsonDocument doc;
  doc["soil_raw"]             = s_data.soilRaw;
  doc["moisture_percentage"]  = s_data.moisturePercentage;
  doc["humidity"]             = s_data.humidity;
  doc["temperature"]          = s_data.temperature;
  doc["pressure"]             = s_data.pressure;
  doc["lightIntensity"]       = s_data.lightIntensity;
  doc["altitude"]             = s_data.altitude;
  doc["soilTemp"]             = s_data.soilTemperature;

  char payload[256];
  size_t len = serializeJson(doc, payload, sizeof(payload));

  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  http.setConnectTimeout(3000);
  http.setTimeout(3000);

  int code = http.POST((uint8_t*)payload, len);
  Serial.printf("Sent: %s | HTTP %d\n", payload, code);
  http.end();
}

// =========================
// CheckPumpWebsiteAPI
// =========================
void checkPumpCommand() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(Pump_API_URL);
  http.setConnectTimeout(3000);
  http.setTimeout(3000);

  int httpCode = http.GET();

  if (httpCode == 200) {
    // Small fixed-size document — avoids String allocation and is safe
    // against malformed/partial responses (deserializeJson reports an
    // error instead of reading past a missing "}").
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, http.getStream());

    if (err) {
      Serial.printf("Pump API JSON parse failed: %s\n", err.c_str());
    } else if (doc["active"] == true) {
      unsigned long duration = doc["duration"] | 0UL;
      Serial.printf("Server command: water for %lu ms\n", duration);
      TurnRelay(duration); // TurnRelay() itself enforces MAX_PUMP_DURATION_MS
    }
  } else {
    Serial.printf("HTTP error: %d\n", httpCode);
  }

  http.end();
}

// ---------------------------------------------------------------------
// 24/7 health: watchdog feed, heap monitoring, scheduled safe restart
// ---------------------------------------------------------------------
void maintainHealth() {
  esp_task_wdt_reset(); // feed the watchdog every loop() — if this stops
                         // happening (loop hung), the chip resets itself.

  unsigned long now = millis();

  if (now - lastHeapLog >= Safety::HEAP_LOG_INTERVAL_MS) {
    lastHeapLog = now;
    uint32_t freeHeap = ESP.getFreeHeap();
    Serial.printf("[health] free heap: %u bytes | uptime: %lus\n",
                  freeHeap, (now - bootTime) / 1000);

    if (freeHeap < Safety::LOW_HEAP_FORCE_BYTES) {
      Serial.println("[health] Heap critically low — restart requested.");
      restartRequested = true;
    } else if (freeHeap < Safety::LOW_HEAP_WARN_BYTES) {
      Serial.println("[health] Warning: heap getting low.");
    }
  }

  bool scheduledRestartDue = (now - bootTime) >= Safety::SAFE_RESTART_INTERVAL_MS;

  // Only restart when the relay is off, so we never cut power to an
  // in-progress watering cycle.
  if ((restartRequested || scheduledRestartDue) && !relayOn) {
    Serial.println("[health] Performing scheduled/safety restart...");
    Serial.flush();
    delay(100);
    ESP.restart();
  }
}

// ---------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin(Pins::I2C_SDA, Pins::I2C_SCL);

  pinMode(Pins::RELAY, OUTPUT);
  digitalWrite(Pins::RELAY, LOW);
  pinMode(Pins::DHT_DATA, INPUT);

  dht.begin();

  bmpAvailable = bmp.begin(BMP280_I2C_ADDR);
  if (bmpAvailable) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                     Adafruit_BMP280::SAMPLING_X2,
                     Adafruit_BMP280::SAMPLING_X16,
                     Adafruit_BMP280::FILTER_X16,
                     Adafruit_BMP280::STANDBY_MS_500);
    Serial.println("BMP280 OK");
  } else {
    Serial.println("BMP280 not found - readings will fall back to DHT temperature.");
  }

  dallas.begin();

  bh1750Available = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  Serial.println(bh1750Available ? "BH1750 OK" : "BH1750 not found - light readings will read 0.");

  // Watchdog: if loop() stops feeding it for WDT_TIMEOUT_S seconds, the
  // chip resets itself instead of hanging forever unattended.
  esp_task_wdt_init(Safety::WDT_TIMEOUT_S, true);
esp_task_wdt_add(NULL);

  bootTime = millis();

  beginWifiConnection();
}

void loop() {
  maintainHealth();

  ensureWifiConnected();
  updateRelayTimeout();

  unsigned long now = millis();

  if (now - lastEnvCheck >= Interval::ENV_CHECK) {
    lastEnvCheck = now;
    readEnvironmentSensors();
  }

  if (now - lastSoilCheck >= Interval::SOIL_CHECK) {
    lastSoilCheck = now;
    handleSoilAndIrrigation();
  }

  if (now - lastSendTime >= Interval::SEND_DATA) {
    lastSendTime = now;
    sendDataToServer();
  }

  if (now - PumpAPI::lastCheckTime >= PumpAPI::CHECK_INTERVAL) {
    PumpAPI::lastCheckTime = now;
    checkPumpCommand();
  }
}
