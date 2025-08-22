#define BLYNK_TEMPLATE_ID "TMPL6t1fjuz1X"
#define BLYNK_TEMPLATE_NAME "T4G2025"
#define BLYNK_AUTH_TOKEN "lTCAxzczJZa-nlCr1HjVLc9Cr5YALxzF"
#define BLYNK_PRINT Serial

#include <Arduino.h>
#include <map>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <BlynkSimpleEsp32.h>
#include <ESP_Google_Sheet_Client.h>
#include <time.h>

#define PROJECT_ID "core-respect-339704"
#define CLIENT_EMAIL "durianesp32@core-respect-339704.iam.gserviceaccount.com"
const char PRIVATE_KEY[] PROGMEM = "-----BEGIN PRIVATE KEY-----\nMIIEvQIBADANBgkqhkiG9w0BAQEFAASCBKcwggSjAgEAAoIBAQDH8pbSBH6Ll+Dm\nf5o20HnAn5014QnYhurMxkH3uDlClJms0/8ai45rJDOwwXe8EweHmqZXr4PkRO3y\n/UvaL7IrsWJNxDZA5SiecaIqORWOmTDB/38dAblDG7c12WvMX4pf7gv61EFP6wN2\nIVISWw7MMP4LZiCty8SQ+Pmbm0gwt1LEnqVCwXWJ9tywAGFSKyrA/RUxJF2B737C\nHt+l+QsyrYO172LJv09hYLGSesaGurQmwhY2ImwTqhbvmDFAo79gDrKaFqr5GQ8E\n8YOmiJoFIfq4GodiMwv7QT4ORJZ8NCVF3+tpeEvsq5vQOujGEGEq8QtcxPrd9sfd\nKdNWfsgpAgMBAAECggEACfSDG9vLbSnXscJCrF3+4d9QlYEp21UEVcO9Pxc2CqGZ\ncShw+CJgqeb4h+FqE7wi0NabV5xOhcwEyOdhonBjf5dqXJLbXKnMNu9TcqS69Qmd\nitssugX+8l9r5Mt5eYh8PEB+6jMtOUxrNLN+AZn1B6ecznZ8RAr0M3+h9eRqWLqB\nVVDaR3a/IsIAsjbWWcpZTRZDuPajnB5V2rnvQx1XIdXtwW6/4xXpa9XGGl+PzPrJ\nrqiHqPG81ZCCtJEpAOlm4ZN+5mlCGT5AaKE/tVrTd+3LNfuBOZDOmjtwttVr5G+V\nwGwano5HbpXm0uvHEE/eNWpdwaniR8aVlOZkFxQ3/QKBgQD0HjxsDgoJUsn3I376\nQgCvv21hQ+TT7lPOFgml6MqUbnwi5VIpd2Qm6iqwyEmWSBxI0KAQlz84Jmtm9FDQ\nLeAEnDSO6KN0nVtSJYYHfC9l8/1WEkJsJq7hMWyNwm53QjeIICqNW87TQdxHOeX+\negiSA+YPfLuEZd+IVQAbkxvlmwKBgQDRrfqxsEggOyptzxKAMrW7Tir7JXjr+ovE\nC7ySY8YFOkN0epZfitwZV/hronEJmiFPNuQYfb+F45FbtuBSjawhdwPDCxy26E5H\nq25Ianuov9e+8qygNXkDZpTUhWl7lGiNxPv5V+OHgYQjVMY0voCW9MlNn5MoCleh\nzGvbREOniwKBgEvQDBG+tm040dI3qtqLnuz1tUFstWb/7/TjsTFeP3OjcFfiAgcw\nteDD/mgSe4/5axdN+zfL0O1eOSKrI2HTrWuhG016gDDaIZ88Wgh/D7VI9ddCKnCt\n/see8sh/ppDQ9rAG1VA7P9sp7AAOwheqzHiUT2Fl6lJu1OHhC/yETjibAoGBAJQ0\nyDUeZ+y4RUY2jlok/cU1/DVmiDEr9+yLay+B7G63fwRuvaksqmQThRj3SbTCx/aI\n0vLotx44+v57pdVUJC3HRNZxHUu/qc0IxDvVYDCn4SQrvY7EHRQlRt9sqWFbRgen\nC39z+vRFvJOIkUyotpV80mjWeRpdLRHFPxtOBg//AoGAbfOWHOc/t+Ay29w+fj3J\nENFzNDxrrHLIt9QGW12r3F68iFh9g2jFnfjYJal82ainHyHhu1perwqvzr4BdLja\nXWdy/UM1AgHtBPvmaiegZ1wPK52o+vPzm6156tf1//12XMga3qkd7vQnlKihW3ur\nyi5MKhrCPreSIw+kUTlb+s4=\n-----END PRIVATE KEY-----\n";

String spreadsheetId = "1gTe0lQhJFadmvHZqngsLbZ26OqpKYsfOA2MFDK6Pvak";

#define M1_PWM 20
#define M1_DIR 19
#define M2_PWM 13
#define M2_DIR 12
#define actuatorIN1 2
#define actuatorIN2 14

bool motorEnabled = false;

#define RS485RXD1 17
#define RS485TXD1 18

#define sensorFrameSize 19
#define sensorWaitingTime 1000
#define sensorID 0x01
#define sensorFunction 0x03
#define sensorByteResponse 0x0E

unsigned char byteRequest[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x07, 0x04, 0x08};
unsigned char byteResponse[19] = {};

float moisture, temperature, ph, nitrogen, phosphorus, potassium;
int ec;

unsigned long long sensorLastTime = 0;
unsigned long long motorStartTime = 0;
int motorRunDuration = 0;
bool restricted = true;

#define WIFI_SSID "PaiPai"
#define WIFI_PASS "Junyi3329*"

BLYNK_CONNECTED() {
  Blynk.syncVirtual(V0);
  Blynk.syncVirtual(V1);
  Blynk.syncVirtual(V10);
}

BLYNK_WRITE(V0) {
  if (param.asInt() == 1) {
    motorBrake();
  } else if (param.asInt() == 0) {
    restricted ? motorReverse(90, 90, 2000) : motorReverse(90, 90, 999999999);
  } else if (param.asInt() == 2) {
    restricted ? motorForward(90, 90, 2000) : motorForward(90, 90, 999999999);
  }
}

BLYNK_WRITE(V1) {
  if (param.asInt() == 0) {
    actuatorReverse();
  } else if (param.asInt() == 1) {
    actuatorBreak();
  } else if (param.asInt() == 2) {
    actuatorForward();
  }
}

BLYNK_WRITE(V9) {
  if (param.asInt() == 1) {
    sendData();
    uploadSoilData();
  }
}

BLYNK_WRITE(V10) {
  restricted = param.asInt() == 1;
}

void tokenStatusCallback(TokenInfo info) {
  if (info.status == token_status_error) {
    GSheet.printf("Token error: %s\n", GSheet.getTokenError(info).c_str());
  } else {
    GSheet.printf("Token info: %s - %s\n", GSheet.getTokenType(info).c_str(), GSheet.getTokenStatus(info).c_str());
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, RS485RXD1, RS485TXD1);

  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(actuatorIN1, OUTPUT);
  pinMode(actuatorIN2, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts++ < 20) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✅ WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n❌ Failed to connect to WiFi!");
    return;
  }

  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);
  motorBrake();

  configTime(8 * 3600, 0, "pool.ntp.org", "time.nist.gov");

  GSheet.setTokenCallback(tokenStatusCallback);
  GSheet.setPrerefreshSeconds(600);
  GSheet.begin(CLIENT_EMAIL, PROJECT_ID, PRIVATE_KEY);
}

void loop() {
  Blynk.run();

  if (millis() - sensorLastTime >= 2000) {
    if (getSoilSensorData()) {
      printSensorData();
    } else {
      Serial.println("Sensor Read Fail!");
    }
    sensorLastTime = millis();
  }

  if (motorEnabled && (millis() - motorStartTime >= motorRunDuration)) {
    motorBrake();
  }
}

void motorForward(uint8_t leftSpeed, uint8_t rightSpeed, long long int time) {
  analogWrite(M1_PWM, leftSpeed);
  digitalWrite(M1_DIR, LOW);
  analogWrite(M2_PWM, rightSpeed);
  digitalWrite(M2_DIR, LOW);
  motorStartTime = millis();
  motorRunDuration = time;
  motorEnabled = true;
}

void motorReverse(uint8_t leftSpeed, uint8_t rightSpeed, long long int time) {
  analogWrite(M1_PWM, leftSpeed);
  digitalWrite(M1_DIR, HIGH);
  analogWrite(M2_PWM, rightSpeed);
  digitalWrite(M2_DIR, HIGH);
  motorStartTime = millis();
  motorRunDuration = time;
  motorEnabled = true;
}

void motorBrake() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  motorEnabled = false;
}

bool getSoilSensorData() {
  Serial1.write(byteRequest, 8);
  unsigned long resptime = millis();
  while ((Serial1.available() < sensorFrameSize) && ((millis() - resptime) < sensorWaitingTime)) {
    delay(1);
  }

  if (Serial1.available() < sensorFrameSize) {
    Serial.println("Timeout: Incomplete sensor response.");
    return false;
  }

  for (int n = 0; n < sensorFrameSize; n++) {
    byteResponse[n] = Serial1.read();
  }

  if (byteResponse[0] != sensorID || byteResponse[1] != sensorFunction || byteResponse[2] != sensorByteResponse) {
    Serial.println("Sensor response invalid.");
    return false;
  }

  temperature = sensorValue(byteResponse[3], byteResponse[4]) * 0.1;
  moisture = sensorValue(byteResponse[5], byteResponse[6]) * 0.1;
  ec = sensorValue(byteResponse[7], byteResponse[8]);
  ph = sensorValue(byteResponse[9], byteResponse[10]) * 0.01;
  nitrogen = sensorValue(byteResponse[11], byteResponse[12]);
  phosphorus = sensorValue(byteResponse[13], byteResponse[14]);
  potassium = sensorValue(byteResponse[15], byteResponse[16]);
  return true;
}

int sensorValue(int x, int y) {
  return x * 256 + y;
}

void printSensorData() {
  Serial.println("===== SOIL DATA SENSING =====");
  Serial.printf("Moisture: %.1f %%\n", moisture);
  Serial.printf("Temperature: %.1f °C\n", temperature);
  Serial.printf("pH: %.2f\n", ph);
  Serial.printf("EC: %d uS/cm\n", ec);
  Serial.printf("Nitrogen (N): %.1f mg/kg\n", nitrogen);
  Serial.printf("Phosphorus (P): %.1f mg/kg\n", phosphorus);
  Serial.printf("Potassium (K): %.1f mg/kg\n", potassium);
  Serial.println("=============================");
}

void actuatorReverse() {
  analogWrite(actuatorIN1, 254);
  analogWrite(actuatorIN2, 254);
}

void actuatorForward() {
  analogWrite(actuatorIN1, 254);
  analogWrite(actuatorIN2, 0);
}

void actuatorBreak() {
  analogWrite(actuatorIN1, 0);
  analogWrite(actuatorIN2, 0);
}

void sendData() {
  Blynk.virtualWrite(V2, moisture);
  Blynk.virtualWrite(V3, temperature);
  Blynk.virtualWrite(V4, ec);
  Blynk.virtualWrite(V5, ph);
  Blynk.virtualWrite(V6, nitrogen);
  Blynk.virtualWrite(V7, phosphorus);
  Blynk.virtualWrite(V8, potassium);
}

void uploadSoilData() {
  if (!GSheet.ready()) return;

  FirebaseJson response;
  FirebaseJson valueRange;

  valueRange.add("majorDimension", "ROWS");

  // 生成当前时间戳
  String timestamp = getCurrentTimeString(); // 例如 "2025-07-12 14:30:01"

  // 设置每列数据（加上时间戳作为第一个值）
  valueRange.set("values/[0]/[0]", timestamp);
  valueRange.set("values/[0]/[1]", moisture);
  valueRange.set("values/[0]/[2]", temperature);
  valueRange.set("values/[0]/[3]", ec);
  valueRange.set("values/[0]/[4]", ph);
  valueRange.set("values/[0]/[5]", nitrogen);
  valueRange.set("values/[0]/[6]", phosphorus);
  valueRange.set("values/[0]/[7]", potassium);

  // 使用 append 来追加数据
  bool success = GSheet.values.append(&response,
                                      spreadsheetId,
                                      "Sheet1", // 不指定范围，让它自动换行追加
                                      &valueRange);

  if (success) {
    Serial.println("✅ 数据已追加到 Google Sheet");
    response.toString(Serial, true);
  } else {
    Serial.print("❌ 上传失败: ");
    Serial.println(GSheet.errorReason());
  }
}

String getCurrentTimeString() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return "N/A";
  }
  char buf[25];
  strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(buf);
}


