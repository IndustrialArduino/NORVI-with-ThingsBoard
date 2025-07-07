#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>

#define SerialMon Serial
#define SerialAT Serial1

#define MODEM_TX 32
#define MODEM_RX 33
#define GSM_RESET 21
#define UART_BAUD 115200

#define SDA 16
#define SCL 17

Adafruit_ADS1115 ads;
const float mA_Factor = 4.096 / 3269.826;

// GPRS Settings
const char apn[] = "dialogbb";

// ThingsBoard Cloud MQTT settings
const char* mqttServer = "mqtt.thingsboard.cloud";
const int mqttPort = 1883;
const char* accessToken = "qRHmLcB7gHkW1hZTv1dp";  // Device token from ThingsBoard

unsigned long lastSend = 0;

void setup() {
  SerialMon.begin(115200);
  delay(1000);
  SerialAT.begin(UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);
  pinMode(GSM_RESET, OUTPUT);
  digitalWrite(GSM_RESET, HIGH);

  Wire.begin(SDA, SCL);
  
  if (!ads.begin(0x48)) {
    Serial.println("ADS1115 (0x48) initialization failed");
    while (1);
  }

  ads.setGain(GAIN_ONE);

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
  int16_t adc0 = ads.readADC_SingleEnded(0);
  int16_t adc1 = ads.readADC_SingleEnded(1);
  int16_t adc2 = ads.readADC_SingleEnded(2);
  int16_t adc3 = ads.readADC_SingleEnded(3);

  float current0 = adc0 * mA_Factor;
  float current1 = adc1 * mA_Factor;
  float current2 = adc2 * mA_Factor;
  float current3 = adc3 * mA_Factor;

  String payload = "{\"A0\":";
  payload += current0;
  payload += ",\"A1\":";
  payload += current1;
  payload += ",\"A2\":";
  payload += current2;
  payload += ",\"A3\":";
  payload += current3;
  payload += "}";

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
