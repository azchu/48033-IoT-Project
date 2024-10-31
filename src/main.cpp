#include <Arduino.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <stdlib.h>
#include <math.h>

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

#define LED_RED 13
#define LED_BLUE 14
#define LED_GREEN 4

#define LED_YELLOW 33
#define CHN 1
#define PWM_BIT 11
#define FRQ 1000

#define ADC_LIGHT_MAX 1360
#define ADC_LIGHT_MIN 735


// WiFi AP SSID
#define WIFI_SSID ""
// WiFi password
#define WIFI_PASSWORD ""

#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN ""
#define INFLUXDB_ORG ""
#define INFLUXDB_BUCKET "ESP32"

// Time zone info
// #define TZ_INFO "UTC11"
#define TZ_INFO "AEST-10AEDT,M10.1.0,M4.1.0/3"

#define WRITE_PRECISION WritePrecision::S
#define MAX_BATCH_SIZE 20
#define WRITE_BUFFER_SIZE MAX_BATCH_SIZE * 3

// Declare InfluxDB client instance with preconfigured InfluxCloud certificate
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);

// Declare Data point
Point sensor("alicesesp32");

const char *mqtt_broker = "broker.emqx.io";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

String clientID;
float tempUpperThreshold = 35; //could we make this a temp threshold a float?
float tempLowerThreshold = 25;
int lightUpperThreshold;//i dont think we need a light threshold, just a temperature threshold - NJ
int lightLowerThreshold;

void callback(char *topic, byte *payload, unsigned int length);
void reconnect();

float get_temp();
float get_light();

void setup() {
  Serial.begin(115200);
  analogReadResolution(11);

  //initalise LEDS as digital output
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  ledcSetup(CHN, FRQ, PWM_BIT);
  ledcAttachPin(LED_YELLOW, CHN);

  // Setup wifi
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to wifi");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();

  clientID = "kaisesp32-";
  clientID += WiFi.macAddress();
  Serial.println(clientID);

  mqttClient.setServer(mqtt_broker, mqtt_port);
  mqttClient.setCallback(callback);
  while (!mqttClient.connected()) {
    Serial.printf("The client %s connects to the public MQTT broker\n", clientID);
    if (mqttClient.connect(clientID.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Public EMQX MQTT broker connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(mqttClient.state());
      delay(2000);
    }
  }
  mqttClient.subscribe("kaisesp32/msgIn/tempControl");

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

  client.setWriteOptions(
    WriteOptions().writePrecision(WRITE_PRECISION).
    batchSize(MAX_BATCH_SIZE).bufferSize(WRITE_BUFFER_SIZE)
  );

  // Add tags to the data point
  sensor.addTag("device", DEVICE);
  sensor.addTag("location", "unilab");
  sensor.addTag("esp32_CUI_13927557", String(WiFi.BSSIDstr().c_str()));

}
void loop() {
  if (!mqttClient.connected()) {
    reconnect();
  }

  mqttClient.loop();

  // Clear fields for reusing the point. Tags will remain
  sensor.clearFields();

  String rssi = String(WiFi.RSSI());
  const char *rssiTopic = "kaisesp32/wifiMeta/rssi";
  const char *rssiPayload = rssi.c_str();
  if (mqttClient.publish(rssiTopic, rssiPayload)) {
    Serial.println("publish okay");
  } else {
    Serial.println("publish failed");
  }

  // Store measured value into point
  // Report RSSI of currently connected network
  float temp_reading = get_temp();
  float light_reading = get_light();
  sensor.addField("temp", temp_reading);
  sensor.addField("light", light_reading);

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

  //turn on LEDS based on the temperature level
  if(temp_reading < tempLowerThreshold)
  { 

    //turn on the blue LED if current temp < then the lower threshold
    digitalWrite(LED_BLUE, HIGH);
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_RED, LOW);
  }

  else if(temp_reading > tempUpperThreshold) {
    //turn on the red LED if current temp is higher then the lower threshold
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_GREEN, LOW);
  }
  else 
  {
    //turn on the green led, if the temperature is within the threshold
    digitalWrite(LED_GREEN, HIGH);
    digitalWrite(LED_RED, LOW);
    digitalWrite(LED_RED, LOW);
  }

  //breathing light 
  if(light_reading <= ADC_LIGHT_MIN)
  {
    light_reading = ADC_LIGHT_MIN;
  }
  else if(light_reading >= ADC_LIGHT_MAX)
  {
    light_reading = ADC_LIGHT_MIN;
  }

  int adc_value = map(light_reading, ADC_LIGHT_MIN, ADC_LIGHT_MAX, 0, (pow(2, PWM_BIT) -1));
  ledcWrite(CHN, adc_value);

  Serial.println("Waiting 1 second");
  delay(1000);
 
}

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.println();
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");

  char message[length+1];
  memcpy(message, payload, length);
  message[length] = '\0';
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);

  if (!error) {
    const char *temp_upper_thres = doc["temp_upper_thres"];
    const char *temp_lower_thres = doc["temp_lower_thres"];
    const char *light_upper_thres = doc["light_upper_thres"];
    const char *light_lower_thres = doc["light_lower_thres"];

    Serial.println(temp_upper_thres);
    Serial.println(temp_lower_thres);
    Serial.println(light_upper_thres);
    Serial.println(light_lower_thres);
    Serial.println();

    tempUpperThreshold = atof(temp_upper_thres);
    tempLowerThreshold = atof(temp_lower_thres);
    lightUpperThreshold = atoi(light_upper_thres);
    lightLowerThreshold = atoi(light_lower_thres);
  }
}

void reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (mqttClient.connect(clientID.c_str())) {
      Serial.println("connected");
      mqttClient.subscribe("kaisesp32/msgIn/tempControl");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
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
