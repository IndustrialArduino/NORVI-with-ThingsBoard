#include <Arduino.h>
#include <Wire.h>
#include <ModbusMaster.h>

#define SerialMon Serial
#define SerialAT Serial1

#define MODEM_TX 32
#define MODEM_RX 33
#define GSM_RESET 21
#define UART_BAUD 115200

#define RXD 25
#define TXD 26
#define FC  22

// GPRS Settings
const char apn[] = "dialogbb";

// ThingsBoard Cloud MQTT settings
const char* mqttServer = "mqtt.thingsboard.cloud";
const int mqttPort = 1883;
const char* accessToken = "EEqkBEnWAU9o41FFW3Mb";  // Device token from ThingsBoard

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

void setup() {
  SerialMon.begin(115200);
  delay(1000);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  pinMode(GSM_RESET, OUTPUT);
  digitalWrite(GSM_RESET, HIGH);

  pinMode(FC, OUTPUT);
  digitalWrite(FC, 0);
  
  Serial2.begin(115200, SERIAL_8N1, RXD, TXD);

  node.begin(1, Serial2);                           //Slave ID as 1
  node.preTransmission(preTransmission);
  delay(10);
  node.postTransmission(postTransmission);

  InitModem();
  connectToGPRS();
  connectToMQTT();
}

void loop() {
  if (millis() - lastSend > 10000) {
    lastSend = millis();
    sendTelemetry();
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

 publishMQTTMessage("v1/devices/me/telemetry", payload);
}

void publishMQTTMessage(String topic, String message) {
  int msgLen = message.length();
  String cmd = "AT+QMTPUBEX=0,0,0,0,\"" + topic + "\"," + String(msgLen);
  gsm_send_serial(cmd, 1000);
  gsm_send_serial(message + "\x1A", 3000);  // End with Ctrl+Z
}

void InitModem() {
  delay(3000);
  gsm_send_serial("AT+CFUN=1", 2000);
  gsm_send_serial("AT+CPIN?", 1000);
  gsm_send_serial("AT+CSQ", 1000);
  gsm_send_serial("AT+CREG?", 1000);
  gsm_send_serial("AT+CGATT?", 1000);
  gsm_send_serial("AT+COPS?", 1000);
  gsm_send_serial("AT+CPSI?", 1000);
  gsm_send_serial("AT+QIFGCNT=0", 500);
  gsm_send_serial("AT+QICSGP=1,1,\"" + String(apn) + "\",\"\",\"\",1", 1000);
}

void connectToGPRS() {
  gsm_send_serial("AT+QIDEACT=1", 1000);
  gsm_send_serial("AT+QIACT=1", 2000);
  gsm_send_serial("AT+QIACT?", 1000);
}

void connectToMQTT() {
  gsm_send_serial("AT+QMTCFG=\"recv/mode\",0,0,1", 1000);
  gsm_send_serial("AT+QMTOPEN=0,\"" + String(mqttServer) + "\"," + String(mqttPort), 3000);
  delay(3000);  // Wait for +QMTOPEN: 0,0
  gsm_send_serial("AT+QMTCONN=0,\"esp32client\",\"" + String(accessToken) + "\"", 3000);
}

String gsm_send_serial(String command, int timeout) {
  String response = "";
  SerialMon.println("Send ->: " + command);
  SerialAT.println(command);

  unsigned long startMillis = millis();
  while (millis() - startMillis < timeout) {
    while (SerialAT.available()) {
      char c = SerialAT.read();
      response += c;
    }
    delay(10);
  }

  SerialMon.println("Response ->: " + response);
  return response;
}
