#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <utility/wifi_drv.h>
#include <ArduinoMotorCarrier.h>

// -------------------- WIFI --------------------
char ssid[] = "HUAWEI-B535-BFC2";
char pass[] = "L6DJ5N4F385";

WiFiUDP Udp;
const IPAddress ip(192, 168, 8, 88);
const unsigned int localPort = 8888;

OSCErrorCode error;

// -------------------- OUTPUT STATE --------------------
uint8_t ledR = 0;
uint8_t ledG = 0;
uint8_t ledB = 0;

int motor1 = 0;
int motor2 = 0;
int motor3 = 0;
int motor4 = 0;

uint8_t ser1 = 90;
uint8_t ser2 = 90;
uint8_t ser3 = 90;
uint8_t ser4 = 90;

// -------------------- BATTERY --------------------
// 3S LiPo / Li-ion recommended thresholds
const float BAT_WARN_V     = 10.5f; // ~3.5V/cell
const float BAT_CRITICAL_V = 9.9f;  // ~3.3V/cell
const float BAT_CUTOFF_V   = 9.6f;  // ~3.2V/cell

unsigned long lastBatteryRead = 0;
const unsigned long BATTERY_READ_INTERVAL = 500; // ms

float batteryVoltage = 0.0f;
bool batteryCritical = false;

// -------------------- HELPERS --------------------
int clampInt(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  ledR = r;
  ledG = g;
  ledB = b;
  WiFiDrv::analogWrite(25, ledR);
  WiFiDrv::analogWrite(26, ledG);
  WiFiDrv::analogWrite(27, ledB);
}

void stopAllMotors() {
  motor1 = motor2 = motor3 = motor4 = 0;
  M1.setDuty(0);
  M2.setDuty(0);
  M3.setDuty(0);
  M4.setDuty(0);
}

float readBatteryVoltageAveraged(uint8_t samples = 8) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += battery.getRaw();
    delay(2);
  }
  float avgRaw = (float)sum / samples;

  // Based on current MKR Motor Carrier community finding:
  // raw / 77 is closer to real voltage than getConverted()/getFiltered()
  return avgRaw / 77.0f;
}

void updateBatteryStatus() {
  if (millis() - lastBatteryRead < BATTERY_READ_INTERVAL) return;
  lastBatteryRead = millis();

  batteryVoltage = readBatteryVoltageAveraged();

  Serial.print("Battery: ");
  Serial.print(batteryVoltage, 2);
  Serial.println(" V");

  if (batteryVoltage <= BAT_CUTOFF_V) {
    batteryCritical = true;
    stopAllMotors();

    // Red = battery cutoff
    setRGB(255, 0, 0);

    Serial.println("BATTERY CUTOFF! Motors stopped.");
  } 
  else if (batteryVoltage <= BAT_CRITICAL_V) {
    batteryCritical = false;

    // Orange = critical
    setRGB(255, 80, 0);

    Serial.println("Battery critical!");
  } 
  else if (batteryVoltage <= BAT_WARN_V) {
    batteryCritical = false;

    // Yellow = warning
    setRGB(180, 120, 0);

    Serial.println("Battery warning.");
  }
}

// -------------------- OSC CALLBACKS --------------------
void led1(OSCMessage &msg) {
  int v = clampInt(msg.getInt(0), 0, 255);
  ledR = (uint8_t)v;
  WiFiDrv::analogWrite(25, ledR);
  Serial.print("/ledR: ");
  Serial.println(ledR);
}

void led2(OSCMessage &msg) {
  int v = clampInt(msg.getInt(0), 0, 255);
  ledG = (uint8_t)v;
  WiFiDrv::analogWrite(26, ledG);
  Serial.print("/ledG: ");
  Serial.println(ledG);
}

void led3(OSCMessage &msg) {
  int v = clampInt(msg.getInt(0), 0, 255);
  ledB = (uint8_t)v;
  WiFiDrv::analogWrite(27, ledB);
  Serial.print("/ledB: ");
  Serial.println(ledB);
}

void motorA(OSCMessage &msg) {
  if (batteryCritical) return;
  motor1 = clampInt(msg.getInt(0), -100, 100);
  M1.setDuty(motor1);
  Serial.print("/motor1: ");
  Serial.println(motor1);
}

void motorB(OSCMessage &msg) {
  if (batteryCritical) return;
  motor2 = clampInt(msg.getInt(0), -100, 100);
  M2.setDuty(motor2);
  Serial.print("/motor2: ");
  Serial.println(motor2);
}

void motorC(OSCMessage &msg) {
  if (batteryCritical) return;
  motor3 = clampInt(msg.getInt(0), -100, 100);
  M3.setDuty(motor3);
  Serial.print("/motor3: ");
  Serial.println(motor3);
}

void motorD(OSCMessage &msg) {
  if (batteryCritical) return;
  motor4 = clampInt(msg.getInt(0), -100, 100);
  M4.setDuty(motor4);
  Serial.print("/motor4: ");
  Serial.println(motor4);
}

void servoA(OSCMessage &msg) {
  ser1 = (uint8_t)clampInt(msg.getInt(0), 0, 180);
  servo1.setAngle(ser1);
  Serial.print("/ser1: ");
  Serial.println(ser1);
}

void servoB(OSCMessage &msg) {
  ser2 = (uint8_t)clampInt(msg.getInt(0), 0, 180);
  servo2.setAngle(ser2);
  Serial.print("/ser2: ");
  Serial.println(ser2);
}

void servoC(OSCMessage &msg) {
  ser3 = (uint8_t)clampInt(msg.getInt(0), 0, 180);
  servo3.setAngle(ser3);
  Serial.print("/ser3: ");
  Serial.println(ser3);
}

void servoD(OSCMessage &msg) {
  ser4 = (uint8_t)clampInt(msg.getInt(0), 0, 180);
  servo4.setAngle(ser4);
  Serial.print("/ser4: ");
  Serial.println(ser4);
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  WiFi.config(ip);

  if (controller.begin()) {
    Serial.print("MKR Motor Carrier connected, firmware version ");
    Serial.println(controller.getFWVersion());
    controller.reboot();
    delay(500);

    stopAllMotors();
  } else {
    Serial.println("Couldn't connect motor shield!");
    while (1) {
      setRGB(255, 0, 0);
      delay(200);
      setRGB(0, 0, 0);
      delay(200);
    }
  }

  WiFiDrv::pinMode(25, OUTPUT); // red
  WiFiDrv::pinMode(26, OUTPUT); // green
  WiFiDrv::pinMode(27, OUTPUT); // blue
  setRGB(0, 0, 0);

  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    setRGB(0, 0, 40);
    delay(200);
    setRGB(0, 0, 0);
    delay(300);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Udp.begin(localPort);
  Serial.print("UDP listening on port: ");
  Serial.println(localPort);

  // startup blink
  setRGB(0, 180, 0); delay(200);
  setRGB(0, 0, 0);   delay(200);
  setRGB(0, 180, 0); delay(200);
  setRGB(0, 0, 0);   delay(200);
}

// -------------------- LOOP --------------------
void loop() {
  OSCBundle bundle;

  int size = Udp.parsePacket();
  if (size > 0) {
    while (size--) {
      bundle.fill(Udp.read());
    }

    if (!bundle.hasError()) {
      bundle.dispatch("/ledR",   led1);
      bundle.dispatch("/ledG",   led2);
      bundle.dispatch("/ledB",   led3);
      bundle.dispatch("/motor1", motorA);
      bundle.dispatch("/motor2", motorB);
      bundle.dispatch("/motor3", motorC);
      bundle.dispatch("/motor4", motorD);
      bundle.dispatch("/ser1",   servoA);
      bundle.dispatch("/ser2",   servoB);
      bundle.dispatch("/ser3",   servoC);
      bundle.dispatch("/ser4",   servoD);
    } else {
      error = bundle.getError();
      Serial.print("OSC error: ");
      Serial.println(error);
    }

    bundle.empty();
  }

  updateBatteryStatus();
  controller.ping();
}#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <utility/wifi_drv.h>
#include <ArduinoMotorCarrier.h>

// -------------------- WIFI --------------------
char ssid[] = "HUAWEI-B535-BFC2";
char pass[] = "L6DJ5N4F385";

WiFiUDP Udp;
const IPAddress ip(192, 168, 6, 88);
const unsigned int localPort = 8888;

OSCErrorCode error;

// -------------------- OUTPUT STATE --------------------
uint8_t ledR = 0;
uint8_t ledG = 0;
uint8_t ledB = 0;

int motor1 = 0;
int motor2 = 0;
int motor3 = 0;
int motor4 = 0;

uint8_t ser1 = 90;
uint8_t ser2 = 90;
uint8_t ser3 = 90;
uint8_t ser4 = 90;

// -------------------- BATTERY --------------------
// 3S LiPo / Li-ion recommended thresholds
const float BAT_WARN_V     = 10.5f; // ~3.5V/cell
const float BAT_CRITICAL_V = 9.9f;  // ~3.3V/cell
const float BAT_CUTOFF_V   = 9.6f;  // ~3.2V/cell

unsigned long lastBatteryRead = 0;
const unsigned long BATTERY_READ_INTERVAL = 500; // ms

float batteryVoltage = 0.0f;
bool batteryCritical = false;

// -------------------- HELPERS --------------------
int clampInt(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  ledR = r;
  ledG = g;
  ledB = b;
  WiFiDrv::analogWrite(25, ledR);
  WiFiDrv::analogWrite(26, ledG);
  WiFiDrv::analogWrite(27, ledB);
}

void stopAllMotors() {
  motor1 = motor2 = motor3 = motor4 = 0;
  M1.setDuty(0);
  M2.setDuty(0);
  M3.setDuty(0);
  M4.setDuty(0);
}

float readBatteryVoltageAveraged(uint8_t samples = 8) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += battery.getRaw();
    delay(2);
  }
  float avgRaw = (float)sum / samples;

  // Based on current MKR Motor Carrier community finding:
  // raw / 77 is closer to real voltage than getConverted()/getFiltered()
  return avgRaw / 77.0f;
}

void updateBatteryStatus() {
  if (millis() - lastBatteryRead < BATTERY_READ_INTERVAL) return;
  lastBatteryRead = millis();

  batteryVoltage = readBatteryVoltageAveraged();

  Serial.print("Battery: ");
  Serial.print(batteryVoltage, 2);
  Serial.println(" V");

  if (batteryVoltage <= BAT_CUTOFF_V) {
    batteryCritical = true;
    stopAllMotors();

    // Red = battery cutoff
    setRGB(255, 0, 0);

    Serial.println("BATTERY CUTOFF! Motors stopped.");
  } 
  else if (batteryVoltage <= BAT_CRITICAL_V) {
    batteryCritical = false;

    // Orange = critical
    setRGB(255, 80, 0);

    Serial.println("Battery critical!");
  } 
  else if (batteryVoltage <= BAT_WARN_V) {
    batteryCritical = false;

    // Yellow = warning
    setRGB(180, 120, 0);

    Serial.println("Battery warning.");
  }
}

// -------------------- OSC CALLBACKS --------------------
void led1(OSCMessage &msg) {
  int v = clampInt(msg.getInt(0), 0, 255);
  ledR = (uint8_t)v;
  WiFiDrv::analogWrite(25, ledR);
  Serial.print("/ledR: ");
  Serial.println(ledR);
}

void led2(OSCMessage &msg) {
  int v = clampInt(msg.getInt(0), 0, 255);
  ledG = (uint8_t)v;
  WiFiDrv::analogWrite(26, ledG);
  Serial.print("/ledG: ");
  Serial.println(ledG);
}

void led3(OSCMessage &msg) {
  int v = clampInt(msg.getInt(0), 0, 255);
  ledB = (uint8_t)v;
  WiFiDrv::analogWrite(27, ledB);
  Serial.print("/ledB: ");
  Serial.println(ledB);
}

void motorA(OSCMessage &msg) {
  if (batteryCritical) return;
  motor1 = clampInt(msg.getInt(0), -100, 100);
  M1.setDuty(motor1);
  Serial.print("/motor1: ");
  Serial.println(motor1);
}

void motorB(OSCMessage &msg) {
  if (batteryCritical) return;
  motor2 = clampInt(msg.getInt(0), -100, 100);
  M2.setDuty(motor2);
  Serial.print("/motor2: ");
  Serial.println(motor2);
}

void motorC(OSCMessage &msg) {
  if (batteryCritical) return;
  motor3 = clampInt(msg.getInt(0), -100, 100);
  M3.setDuty(motor3);
  Serial.print("/motor3: ");
  Serial.println(motor3);
}

void motorD(OSCMessage &msg) {
  if (batteryCritical) return;
  motor4 = clampInt(msg.getInt(0), -100, 100);
  M4.setDuty(motor4);
  Serial.print("/motor4: ");
  Serial.println(motor4);
}

void servoA(OSCMessage &msg) {
  ser1 = (uint8_t)clampInt(msg.getInt(0), 0, 180);
  servo1.setAngle(ser1);
  Serial.print("/ser1: ");
  Serial.println(ser1);
}

void servoB(OSCMessage &msg) {
  ser2 = (uint8_t)clampInt(msg.getInt(0), 0, 180);
  servo2.setAngle(ser2);
  Serial.print("/ser2: ");
  Serial.println(ser2);
}

void servoC(OSCMessage &msg) {
  ser3 = (uint8_t)clampInt(msg.getInt(0), 0, 180);
  servo3.setAngle(ser3);
  Serial.print("/ser3: ");
  Serial.println(ser3);
}

void servoD(OSCMessage &msg) {
  ser4 = (uint8_t)clampInt(msg.getInt(0), 0, 180);
  servo4.setAngle(ser4);
  Serial.print("/ser4: ");
  Serial.println(ser4);
}

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);
  WiFi.config(ip);

  if (controller.begin()) {
    Serial.print("MKR Motor Carrier connected, firmware version ");
    Serial.println(controller.getFWVersion());
    controller.reboot();
    delay(500);

    stopAllMotors();
  } else {
    Serial.println("Couldn't connect motor shield!");
    while (1) {
      setRGB(255, 0, 0);
      delay(200);
      setRGB(0, 0, 0);
      delay(200);
    }
  }

  WiFiDrv::pinMode(25, OUTPUT); // red
  WiFiDrv::pinMode(26, OUTPUT); // green
  WiFiDrv::pinMode(27, OUTPUT); // blue
  setRGB(0, 0, 0);

  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    setRGB(0, 0, 40);
    delay(200);
    setRGB(0, 0, 0);
    delay(300);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Udp.begin(localPort);
  Serial.print("UDP listening on port: ");
  Serial.println(localPort);

  // startup blink
  setRGB(0, 180, 0); delay(200);
  setRGB(0, 0, 0);   delay(200);
  setRGB(0, 180, 0); delay(200);
  setRGB(0, 0, 0);   delay(200);
}

// -------------------- LOOP --------------------
void loop() {
  OSCBundle bundle;

  int size = Udp.parsePacket();
  if (size > 0) {
    while (size--) {
      bundle.fill(Udp.read());
    }

    if (!bundle.hasError()) {
      bundle.dispatch("/ledR",   led1);
      bundle.dispatch("/ledG",   led2);
      bundle.dispatch("/ledB",   led3);
      bundle.dispatch("/motor1", motorA);
      bundle.dispatch("/motor2", motorB);
      bundle.dispatch("/motor3", motorC);
      bundle.dispatch("/motor4", motorD);
      bundle.dispatch("/ser1",   servoA);
      bundle.dispatch("/ser2",   servoB);
      bundle.dispatch("/ser3",   servoC);
      bundle.dispatch("/ser4",   servoD);
    } else {
      error = bundle.getError();
      Serial.print("OSC error: ");
      Serial.println(error);
    }

    bundle.empty();
  }

  updateBatteryStatus();
  controller.ping();
}
