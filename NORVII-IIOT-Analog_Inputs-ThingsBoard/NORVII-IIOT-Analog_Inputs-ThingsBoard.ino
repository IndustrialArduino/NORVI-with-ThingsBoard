#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define SDA 16
#define SCL 17

// WiFi credentials
const char* ssid = "SLT-Fiber-2.4G";
const char* password = "UrAT4254";

// ThingsBoard Cloud settings
const char* thingsboardServer = "thingsboard.cloud";
const int mqttPort = 1883;
const char* token = "7oqqozxjzmbaszdyjhda"; // Get this from device!


Adafruit_ADS1115 ads1;
#define VOLTAGE_DIVIDER_RATIO  0.2065

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastSend = 0;

void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500); Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
}

void connectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to ThingsBoard MQTT...");
    if (client.connect("ESP32Client", token, NULL)) {
      Serial.println("connected!");
    } else {
      Serial.print("failed with state ");
      Serial.println(client.state());
      delay(2000);
    }
  }
}

void sendTelemetry() {
  int16_t adc0, adc1, adc2, adc3;

  adc0 = ads1.readADC_SingleEnded(0);
  adc1 = ads1.readADC_SingleEnded(1);
  adc2 = ads1.readADC_SingleEnded(2);
  adc3 = ads1.readADC_SingleEnded(3);

  float voltage0 = adc0 * 0.125 / 1000.0 / VOLTAGE_DIVIDER_RATIO;  
  float voltage1 = adc1 * 0.125 / 1000.0 / VOLTAGE_DIVIDER_RATIO; 
  float voltage2 = adc2 * 0.125 / 1000.0 / VOLTAGE_DIVIDER_RATIO;  
  float voltage3 = adc3 * 0.125 / 1000.0 / VOLTAGE_DIVIDER_RATIO;
  
  Serial.print("V 0:   "); Serial.print(voltage0); Serial.println(" V");
  Serial.print("V 1:   "); Serial.print(voltage1); Serial.println(" V");
  Serial.print("V 2:   "); Serial.print(voltage2); Serial.println(" V");
  Serial.print("V 3:   "); Serial.print(voltage3); Serial.println(" V");

  String payload = "{\"A0\":";
  payload += voltage0;
  payload += ", \"A1\":";
  payload += voltage1;
  payload += ", \"A2\":";
  payload += voltage2;
  payload += ", \"A3\":";
  payload += voltage3;
  payload += "}";

  Serial.print("Publishing: ");
  Serial.println(payload);

  client.publish("v1/devices/me/telemetry", payload.c_str());
}

void setup() {
  Serial.begin(115200);
  delay(100);
  connectWiFi();
  delay(1000);
  client.setServer(thingsboardServer, mqttPort);
  delay(1000);
  
  Wire.begin(SDA, SCL); 
  if (!ads1.begin(0x48)) {
    Serial.println("Failed to initialize ADS 1 .");
     while (1);
  }

  
}

void loop() {
  if (!client.connected()) {
    connectMQTT();
  }

  client.loop();

  if (millis() - lastSend > 5000) {
    lastSend = millis();
    sendTelemetry();
  }
}
