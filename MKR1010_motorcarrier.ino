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
const IPAddress ip(192, 168, 6, 88);
const unsigned int localPort = 8888;

OSCErrorCode error;

// -------------------- STATE --------------------
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

// -------------------- SIGNAL TIMEOUT --------------------
unsigned long lastPacketTime = 0;
const unsigned long SIGNAL_TIMEOUT = 150; // ms
bool signalActive = false;
bool timeoutHandled = false;

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

void setAllServosToStoredValues() {
  servo1.setAngle(ser1);
  servo2.setAngle(ser2);
  servo3.setAngle(ser3);
  servo4.setAngle(ser4);
}

void handleSignalTimeout() {
  if (signalActive && !timeoutHandled && (millis() - lastPacketTime > SIGNAL_TIMEOUT)) {
    timeoutHandled = true;
    signalActive = false;

    // Stop DC motors, but keep servos at their last received positions
    stopAllMotors();
    setAllServosToStoredValues();

    // Red LED indicates signal timeout
    setRGB(120, 0, 0);

    Serial.println("Signal timeout -> motors stopped, servos holding last position");
  }
}

// -------------------- OSC CALLBACKS --------------------
void led1(OSCMessage &msg) {
  ledR = (uint8_t)clampInt(msg.getInt(0), 0, 255);
  WiFiDrv::analogWrite(25, ledR);
}

void led2(OSCMessage &msg) {
  ledG = (uint8_t)clampInt(msg.getInt(0), 0, 255);
  WiFiDrv::analogWrite(26, ledG);
}

void led3(OSCMessage &msg) {
  ledB = (uint8_t)clampInt(msg.getInt(0), 0, 255);
  WiFiDrv::analogWrite(27, ledB);
}

void motorA(OSCMessage &msg) {
  motor1 = clampInt(msg.getInt(0), -100, 100);
  M1.setDuty(motor1);
}

void motorB(OSCMessage &msg) {
  motor2 = clampInt(msg.getInt(0), -100, 100);
  M2.setDuty(motor2);
}

void motorC(OSCMessage &msg) {
  motor3 = clampInt(msg.getInt(0), -100, 100);
  M3.setDuty(motor3);
}

void motorD(OSCMessage &msg) {
  motor4 = clampInt(msg.getInt(0), -100, 100);
  M4.setDuty(motor4);
}

void servoA(OSCMessage &msg) {
  ser1 = (uint8_t)clampInt(msg.getInt(0), 0, 180);
  servo1.setAngle(ser1);
}

void servoB(OSCMessage &msg) {
  ser2 = (uint8_t)clampInt(msg.getInt(0), 0, 180);
  servo2.setAngle(ser2);
}

void servoC(OSCMessage &msg) {
  ser3 = (uint8_t)clampInt(msg.getInt(0), 0, 180);
  servo3.setAngle(ser3);
}

void servoD(OSCMessage &msg) {
  ser4 = (uint8_t)clampInt(msg.getInt(0), 0, 180);
  servo4.setAngle(ser4);
}

// -------------------- PACKET PROCESSING --------------------
void processIncomingOSC() {
  int packetSize = Udp.parsePacket();

  while (packetSize > 0) {
    OSCBundle bundle;

    while (packetSize--) {
      bundle.fill(Udp.read());
    }

    if (!bundle.hasError()) {
      lastPacketTime = millis();
      signalActive = true;
      timeoutHandled = false;

      bundle.dispatch("/ledR", led1);
      bundle.dispatch("/ledG", led2);
      bundle.dispatch("/ledB", led3);
      bundle.dispatch("/motor1", motorA);
      bundle.dispatch("/motor2", motorB);
      bundle.dispatch("/motor3", motorC);
      bundle.dispatch("/motor4", motorD);
      bundle.dispatch("/ser1", servoA);
      bundle.dispatch("/ser2", servoB);
      bundle.dispatch("/ser3", servoC);
      bundle.dispatch("/ser4", servoD);
    } else {
      error = bundle.getError();
      Serial.print("OSC error: ");
      Serial.println(error);
    }

    bundle.empty();

    // Check if more packets are waiting in the UDP buffer
    packetSize = Udp.parsePacket();
  }
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
      delay(100);
    }
  }

  WiFiDrv::pinMode(25, OUTPUT);
  WiFiDrv::pinMode(26, OUTPUT);
  WiFiDrv::pinMode(27, OUTPUT);
  setRGB(0, 0, 0);

  // Set initial servo positions
  setAllServosToStoredValues();

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    setRGB(0, 0, 40);
    delay(150);
    setRGB(0, 0, 0);
    delay(150);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Udp.begin(localPort);
  Serial.print("UDP listening on port: ");
  Serial.println(localPort);

  setRGB(0, 180, 0);
  delay(200);
  setRGB(0, 0, 0);

  lastPacketTime = millis();
}

// -------------------- LOOP --------------------
void loop() {
  processIncomingOSC();
  handleSignalTimeout();
  controller.ping();
}
