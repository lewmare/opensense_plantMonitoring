/*
 * ESP32 Environmental & Soil Monitoring System
 * -----------------------------------------------------------
 * Sensors : DHT11 (fallback temp/humidity), BMP280 (pressure/temp/alt),
 *           BH1750 (lux), capacitive soil moisture (analog)
 * Actuator: Relay-driven water pump
 * Network : WiFi + HTTP POST (JSON) to backend API
 *
 * Improvements over the original sketch:
 *  - secrets.h separates credentials from source code
 *  - ArduinoJson used for safe, correctly-escaped JSON payloads
 *  - non-blocking WiFi reconnection (loop no longer stalls)
 *  - HTTP timeouts set so a slow/dead server can't block the loop
 *  - moving-average soil reading via circular buffer (was a buggy
 *    "fill 3 then reset" pattern that assumed 0 == "empty slot")
 *  - constants/magic numbers named and moved to config section
 *  - code split into single-purpose functions
 * -----------------------------------------------------------
 */

#include <Arduino.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>
#include <BH1750.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>   // Install via Library Manager: "ArduinoJson" by Benoit Blanchon
#include <OneWire.h>      // Install via Library Manager: "OneWire" by Paul Stoffregen
#include <DallasTemperature.h> // Install via Library Manager: "DallasTemperature" by Miles Burton

#include "secrets.h"       // defines WIFI_SSID, WIFI_PASSWORD, SERVER_URL (see note at bottom)

// ---------------------------------------------------------------------
// Pin & hardware configuration
// ---------------------------------------------------------------------
namespace Pins {
  constexpr int SOIL_ANALOG = 33;
  constexpr int RELAY       = 25;
  constexpr int DHT_DATA    = 15;
  constexpr int I2C_SDA     = 21;
  constexpr int I2C_SCL     = 22;
  constexpr int ONE_WIRE_BUS = 32; // For DS18B20 temperature sensor (if used)
}



constexpr uint8_t DHT_TYPE        = DHT11;
constexpr uint8_t BMP280_I2C_ADDR = 0x76;

// ---------------------------------------------------------------------
// Timing configuration (all in ms)
// ---------------------------------------------------------------------
namespace Interval {
  constexpr unsigned long SEND_DATA   = 5000;
  constexpr unsigned long SOIL_CHECK  = 1000;
  constexpr unsigned long ENV_CHECK   = 4000;
  constexpr unsigned long RELAY_ON    = 200;
  constexpr unsigned long WIFI_RETRY  = 10000; // how often to retry a dropped connection
}

// ---------------------------------------------------------------------
// Soil / irrigation configuration
// ---------------------------------------------------------------------
namespace SoilCfg {
  constexpr int   ADC_DRY            = 4095; // raw reading in dry air
  constexpr int   ADC_WET            = 1600; // raw reading fully submerged
  constexpr int   MOISTURE_THRESHOLD = 30;   // % below which irrigation may trigger
  constexpr int   SAMPLE_COUNT       = 20;   // averaged analogRead samples per reading
  constexpr int   HISTORY_SIZE       = 3;    // samples used for the moving average
}

constexpr float SEA_LEVEL_HPA = 1013.25f;

// ---------------------------------------------------------------------
// Global objects
// ---------------------------------------------------------------------
DHT dht(Pins::DHT_DATA, DHT_TYPE);
Adafruit_BMP280 bmp;
BH1750 lightMeter;
OneWire oneWire(Pins::ONE_WIRE_BUS); // For DS18B20 temperature sensor (if used)
DallasTemperature dallas(&oneWire);

bool bmpAvailable   = false;
bool bh1750Available = false;

struct SensorData {
  int   soilRaw            = 0;
  int   moisturePercentage = 0;
  float humidity            = 0;
  float temperature         = 0;
  float pressure             = 0;
  float lightIntensity        = 0;
  float altitude               = 0;
  float soilTemperature          = 0; // For DS18B20 temperature sensor (if used)
} s_data;

// Circular buffer for soil moving average (replaces the old "0 = empty slot" hack)
int  soilHistory[SoilCfg::HISTORY_SIZE] = {0};
int  soilHistoryIndex = 0;
bool soilHistoryFull  = false;

bool relayOn = false;
unsigned long relayStartTime = 0;

unsigned long lastSendTime     = 0;
unsigned long lastSoilCheck    = 0;
unsigned long lastEnvCheck     = 0;
unsigned long lastWifiAttempt  = 0;

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

int soilHistoryAverage() {
  int count = soilHistoryFull ? SoilCfg::HISTORY_SIZE : soilHistoryIndex;
  if (count == 0) return 0;
  long sum = 0;
  for (int i = 0; i < count; i++) sum += soilHistory[i];
  return sum / count;
}

void readEnvironmentSensors() {
  float dhtHumidity    = dht.readHumidity();
  float dhtTemperature = dht.readTemperature();

  float bmpPressure = bmpAvailable ? bmp.readPressure() / 100.0F : NAN;
  float bmpTemp      = bmpAvailable ? bmp.readTemperature() : NAN;
  float bmpAltitude   = bmpAvailable ? bmp.readAltitude(SEA_LEVEL_HPA) : NAN;
  float lux            = bh1750Available ? lightMeter.readLightLevel() : NAN;

  dallas.requestTemperatures(); // For DS18B20 temperature sensor (if used)
  float soiltemp = dallas.getTempCByIndex(0); // Get temperature from the first DS18B20 sensor

  s_data.humidity      = isnan(dhtHumidity) ? 0.0f : dhtHumidity;
  s_data.temperature    = !isnan(bmpTemp) ? bmpTemp : (isnan(dhtTemperature) ? 0.0f : dhtTemperature);
  s_data.pressure         = isnan(bmpPressure) ? 0.0f : bmpPressure;
  s_data.altitude          = isnan(bmpAltitude) ? 0.0f : bmpAltitude;
  s_data.lightIntensity      = (isnan(lux) || lux < 0) ? 0.0f : lux;
  s_data.soilTemperature      = (soiltemp == DEVICE_DISCONNECTED_C) ? 0.0f : soiltemp;

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

  // Store the *percentage* history (not raw) so the dryness check below
  // is directly comparable to MOISTURE_THRESHOLD.
  pushSoilHistory(pct);

  // Trigger irrigation once moisture has stayed below the threshold across
  // the whole history window. This debounces a single noisy/outlier reading
  // without ever permanently blocking irrigation the old "raw < average"
  // check used to (that check could fail forever once the soil settled into
  // a steady dry state, since raw would then hover right around its own
  // moving average and rarely dip strictly below it).
  bool consistentlyDry = soilHistoryFull;
  for (int i = 0; i < SoilCfg::HISTORY_SIZE && consistentlyDry; i++) {
    if (soilHistory[i] >= SoilCfg::MOISTURE_THRESHOLD) consistentlyDry = false;
  }

  if (consistentlyDry && !relayOn) {
    Serial.println("Soil is dry -> turning relay ON.");
    digitalWrite(Pins::RELAY, HIGH);
    relayOn = true;
    relayStartTime = millis();
  } else {
    Serial.println("Soil condition normal.");
  }
}

void updateRelayTimeout() {
  if (relayOn && millis() - relayStartTime >= Interval::RELAY_ON) {
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

  JsonDocument doc; // ArduinoJson v7 auto-sized document
  doc["soil_raw"]              = s_data.soilRaw;
  doc["moisture_percentage"]  = s_data.moisturePercentage;
  doc["humidity"]               = serialized(String(s_data.humidity, 1));
  doc["temperature"]             = serialized(String(s_data.temperature, 1));
  doc["pressure"]                 = serialized(String(s_data.pressure, 1));
  doc["lightIntensity"]            = serialized(String(s_data.lightIntensity, 1));
  doc["altitude"]                   = serialized(String(s_data.altitude, 1));
  doc["soilTemp"]          = serialized(String(s_data.soilTemperature, 1)); // For DS18B20 temperature sensor (if used)

  String payload;
  serializeJson(doc, payload);
  HTTPClient http;
  http.begin(SERVER_URL);
  http.addHeader("Content-Type", "application/json");
  http.setConnectTimeout(3000); // ms - avoid blocking loop() for long on a dead server
  http.setTimeout(3000);

  int code = http.POST(payload);
  Serial.printf("Sent: %s | HTTP %d\n", payload.c_str(), code);
  http.end();
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

  dallas.begin(); // Initialize DallasTemperature library for DS18B20 (if used)

  bh1750Available = lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);
  Serial.println(bh1750Available ? "BH1750 OK" : "BH1750 not found - light readings will read 0.");

  beginWifiConnection();
}

void loop() {
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
}
