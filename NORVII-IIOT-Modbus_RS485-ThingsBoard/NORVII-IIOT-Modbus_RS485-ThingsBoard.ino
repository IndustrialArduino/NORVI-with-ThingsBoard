#include <WiFi.h>
#include <PubSubClient.h>
#include <ModbusMaster.h>

#define RXD 13
#define TXD 2
#define FC  4

// WiFi credentials
const char* ssid = "SLT-Fiber-2.4G";
const char* password = "UrAT4254";

// ThingsBoard Cloud settings
const char* thingsboardServer = "thingsboard.cloud";
const int mqttPort = 1883;
const char* token = "634scpxcfjs9u6kuqriv"; // Get this from device!


WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastSend = 0;

ModbusMaster node;

void preTransmission()
{
  digitalWrite(FC, 1);
}

void postTransmission()
{
  digitalWrite(FC, 0);
}

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

  uint8_t value;
  uint8_t IN1, IN2, IN3, IN4;
  value = node.readHoldingRegisters(0x40001, 3);

  IN1 = node.getResponseBuffer(0x00);
  IN2 = node.getResponseBuffer(0x01);
  IN3 = node.getResponseBuffer(0x02);
  
  Serial.print("\n");
  Serial.print(" ANIN1 : ");
  Serial.print(IN1);
  Serial.print(" ANIN2 : ");
  Serial.print(IN2);
  Serial.print(" ANIN3 : ");
  Serial.print(IN3);
  Serial.print("\n");
  delay(500);


  String payload = "{\"ANIN1\":";
  payload += IN1;
  payload += ", \"ANIN2\":";
  payload += IN2;
  payload += ", \"ANIN3\":";
  payload += IN3;
  payload += "}";

  Serial.print("Publishing: ");
  Serial.println(payload);

  client.publish("v1/devices/me/telemetry", payload.c_str());
}

void setup() {
  Serial.begin(9600);
  delay(100);
  connectWiFi();
  delay(1000);
  client.setServer(thingsboardServer, mqttPort);
  delay(1000);

  pinMode(FC, OUTPUT);
  digitalWrite(FC, 0);

  Serial1.begin(9600, SERIAL_8N1, RXD, TXD);

  node.begin(1, Serial1);                           //Slave ID as 1
  node.preTransmission(preTransmission);
  delay(10);
  node.postTransmission(postTransmission);


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
