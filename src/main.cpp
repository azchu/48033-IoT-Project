#include <Arduino.h>

#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif

#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>

#define PIN_ANALOG_IN_TEMP 32
#define PIN_ANALOG_IN_LIGHT 34

// WiFi AP SSID
#define WIFI_SSID "noot"
// WiFi password
#define WIFI_PASSWORD "ChuChuyoo"

#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "3ug-Egxdzj7ui9w2qw4W8M4NTv1dGs3yD6EIImgOELHe0IDvmWNQU9COlo9JUXyBqip__lbjkFaqXgVr_0vlTQ=="
#define INFLUXDB_ORG "857f6e8ce39c5194"
#define INFLUXDB_BUCKET "ESP32"

// Time zone info
// #define TZ_INFO "UTC11"
#define TZ_INFO "AEST-10AEDT,M10.1.0,M4.1.0/3"

// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Declare Data point
Point sensor("alicesesp32");

float get_temp();
float get_light();

void setup() {
  Serial.begin(115200);
  analogReadResolution(11);

  // Setup wifi
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to wifi");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  // Accurate time is necessary for certificate validation and writing in batches
  // We use the NTP servers in your area as provided by: https://www.pool.ntp.org/zone/
  // Syncing progress and the time will be printed to Serial.
  timeSync(TZ_INFO, "pool.ntp.org", "time.nist.gov");

  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  // Add tags to the data point
  sensor.addTag("device", DEVICE);
  sensor.addTag("location", "unilab");
  sensor.addTag("esp32_CUI_13927557", String(WiFi.BSSIDstr().c_str()));

}
void loop() {
  // Clear fields for reusing the point. Tags will remain
  sensor.clearFields();

  // Store measured value into point
  // Report RSSI of currently connected network
  sensor.addField("temp", get_temp());
  sensor.addField("light", get_light());

  // Print what are we exactly writing
  Serial.print("Writing: ");
  Serial.println(sensor.toLineProtocol());

  // Check WiFi connection and reconnect if needed
  if (wifiMulti.run() != WL_CONNECTED) {
    Serial.println("Wifi connection lost");
  }

  // Write point
  if (!client.writePoint(sensor)) {
    Serial.print("InfluxDB write failed: ");
    Serial.println(client.getLastErrorMessage());
  }

  Serial.println("Waiting 1 second");
  delay(1000);
}

float get_temp() {
  int adcValue = analogRead(PIN_ANALOG_IN_TEMP);
  double voltage = (float)adcValue / 2047.0 * 3.3;
  double Rt = 10 * voltage / (3.3 - voltage);
  double tempK = 1 / (1 / (273.15 + 25) + log(Rt / 10) /3950.0);
  double tempC = tempK - 273.15;
  return tempC;
}

float get_light() {
  int adcValue = analogRead(PIN_ANALOG_IN_LIGHT);
  return adcValue;
}