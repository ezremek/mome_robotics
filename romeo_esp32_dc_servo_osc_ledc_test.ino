#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCBundle.h>
#include <OSCMessage.h>
#include <OSCData.h>
#include <ESP32Servo.h>
#include <math.h>

// Important: this combined test uses LEDC for DC motors.
// On this ESP32-S3 setup, the ESP32Servo library uses fixed-frequency PWM
// internally, which can conflict with our previous MCPWM-based motor test.

const char *ssid = "goodrobot";
const char *pass = "badr0b0t";

WiFiUDP Udp;
IPAddress localIp(192, 168, 8, 42);
const unsigned int localPort = 8888;

constexpr bool ENABLE_SERIAL_DEBUG = false;

// -------------------- MOTOR PINS (PH/EN MODE) --------------------
constexpr int M1_EN_PIN = 12;
constexpr int M1_PH_PIN = 13;
constexpr int M2_EN_PIN = 14;
constexpr int M2_PH_PIN = 21;
constexpr int M3_EN_PIN = 9;
constexpr int M3_PH_PIN = 10;
constexpr int M4_EN_PIN = 47;
constexpr int M4_PH_PIN = 11;

constexpr int M1_PWM_CHANNEL = 0;
constexpr int M2_PWM_CHANNEL = 1;
constexpr int M3_PWM_CHANNEL = 2;
constexpr int M4_PWM_CHANNEL = 3;

constexpr int MOTOR_PWM_FREQ = 1000;
constexpr int MOTOR_PWM_RESOLUTION = 8;
constexpr int MOTOR_PWM_MAX_DUTY = (1 << MOTOR_PWM_RESOLUTION) - 1;

// -------------------- SERVO PINS --------------------
constexpr int SERVO1_PIN = 3;
constexpr int SERVO2_PIN = 38;
constexpr int SERVO3_PIN = 43;
constexpr int SERVO4_PIN = 44;

constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2400;
constexpr int SERVO_FREQ_HZ = 50;

// -------------------- NETWORK / SIGNAL --------------------
constexpr unsigned long SIGNAL_TIMEOUT = 300;

OSCErrorCode error;

// Compatibility only: keep these OSC values so the TD patch can stay the same.
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

Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

bool servo1Attached = false;
bool servo2Attached = false;
bool servo3Attached = false;
bool servo4Attached = false;

unsigned long lastPacketTime = 0;
bool signalActive = false;
bool timeoutHandled = false;

int clampInt(int value, int minVal, int maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

int readOscInt(OSCMessage &msg, int index, int fallbackValue = 0) {
  if (msg.isInt(index)) {
    return msg.getInt(index);
  }
  if (msg.isFloat(index)) {
    return (int)lroundf(msg.getFloat(index));
  }
  return fallbackValue;
}

void logLine(const char *label) {
  if (!ENABLE_SERIAL_DEBUG) return;
  Serial.println(label);
}

void initMotorChannel(int enPin, int phPin, int pwmChannel) {
  pinMode(phPin, OUTPUT);
  digitalWrite(phPin, LOW);
  ledcAttachChannel(enPin, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION, pwmChannel);
  ledcWrite(enPin, 0);
}

void initMotorPwm() {
  initMotorChannel(M1_EN_PIN, M1_PH_PIN, M1_PWM_CHANNEL);
  initMotorChannel(M2_EN_PIN, M2_PH_PIN, M2_PWM_CHANNEL);
  initMotorChannel(M3_EN_PIN, M3_PH_PIN, M3_PWM_CHANNEL);
  initMotorChannel(M4_EN_PIN, M4_PH_PIN, M4_PWM_CHANNEL);
}

void setMotorRaw(int enPin, int phPin, int value) {
  int speed = clampInt(abs(value), 0, 100);
  int duty = map(speed, 0, 100, 0, MOTOR_PWM_MAX_DUTY);

  if (value > 0) {
    digitalWrite(phPin, HIGH);
    ledcWrite(enPin, duty);
  } else if (value < 0) {
    digitalWrite(phPin, LOW);
    ledcWrite(enPin, duty);
  } else {
    ledcWrite(enPin, 0);
  }
}

void applyMotor1() { setMotorRaw(M1_EN_PIN, M1_PH_PIN, motor1); }
void applyMotor2() { setMotorRaw(M2_EN_PIN, M2_PH_PIN, motor2); }
void applyMotor3() { setMotorRaw(M3_EN_PIN, M3_PH_PIN, motor3); }
void applyMotor4() { setMotorRaw(M4_EN_PIN, M4_PH_PIN, motor4); }

void stopAllMotors() {
  motor1 = 0;
  motor2 = 0;
  motor3 = 0;
  motor4 = 0;
  applyMotor1();
  applyMotor2();
  applyMotor3();
  applyMotor4();
}

bool attachServoChecked(Servo &servo, int pin) {
  servo.setPeriodHertz(SERVO_FREQ_HZ);
  return servo.attach(pin, SERVO_MIN_US, SERVO_MAX_US) > 0;
}

void writeServoIfAttached(Servo &servo, bool attached, uint8_t angle) {
  if (attached) {
    servo.write(angle);
  }
}

void applyAllServos() {
  writeServoIfAttached(servo1, servo1Attached, ser1);
  writeServoIfAttached(servo2, servo2Attached, ser2);
  writeServoIfAttached(servo3, servo3Attached, ser3);
  writeServoIfAttached(servo4, servo4Attached, ser4);
}

void handleSignalTimeout() {
  if (signalActive && !timeoutHandled && (millis() - lastPacketTime > SIGNAL_TIMEOUT)) {
    timeoutHandled = true;
    signalActive = false;
    stopAllMotors();
    applyAllServos();
    logLine("Signal timeout");
  }
}

// -------------------- OSC CALLBACKS --------------------
void led1(OSCMessage &msg) {
  ledR = (uint8_t)clampInt(readOscInt(msg, 0, ledR), 0, 255);
}

void led2(OSCMessage &msg) {
  ledG = (uint8_t)clampInt(readOscInt(msg, 0, ledG), 0, 255);
}

void led3(OSCMessage &msg) {
  ledB = (uint8_t)clampInt(readOscInt(msg, 0, ledB), 0, 255);
}

void motorA(OSCMessage &msg) {
  motor1 = clampInt(readOscInt(msg, 0, motor1), -100, 100);
  applyMotor1();
}

void motorB(OSCMessage &msg) {
  motor2 = clampInt(readOscInt(msg, 0, motor2), -100, 100);
  applyMotor2();
}

void motorC(OSCMessage &msg) {
  motor3 = clampInt(readOscInt(msg, 0, motor3), -100, 100);
  applyMotor3();
}

void motorD(OSCMessage &msg) {
  motor4 = clampInt(readOscInt(msg, 0, motor4), -100, 100);
  applyMotor4();
}

void servoA(OSCMessage &msg) {
  ser1 = (uint8_t)clampInt(readOscInt(msg, 0, ser1), 0, 180);
  writeServoIfAttached(servo1, servo1Attached, ser1);
}

void servoB(OSCMessage &msg) {
  ser2 = (uint8_t)clampInt(readOscInt(msg, 0, ser2), 0, 180);
  writeServoIfAttached(servo2, servo2Attached, ser2);
}

void servoC(OSCMessage &msg) {
  ser3 = (uint8_t)clampInt(readOscInt(msg, 0, ser3), 0, 180);
  writeServoIfAttached(servo3, servo3Attached, ser3);
}

void servoD(OSCMessage &msg) {
  ser4 = (uint8_t)clampInt(readOscInt(msg, 0, ser4), 0, 180);
  writeServoIfAttached(servo4, servo4Attached, ser4);
}

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
      if (ENABLE_SERIAL_DEBUG) {
        Serial.print("OSC error: ");
        Serial.println(error);
      }
    }

    bundle.empty();
    packetSize = Udp.parsePacket();
  }
}

void setup() {
  if (ENABLE_SERIAL_DEBUG) {
    Serial.begin(115200);
    delay(200);
  }

  initMotorPwm();
  stopAllMotors();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  servo1Attached = attachServoChecked(servo1, SERVO1_PIN);
  servo2Attached = attachServoChecked(servo2, SERVO2_PIN);
  servo3Attached = attachServoChecked(servo3, SERVO3_PIN);
  servo4Attached = attachServoChecked(servo4, SERVO4_PIN);

  applyAllServos();

  WiFi.config(localIp, IPAddress(192, 168, 8, 1), IPAddress(255, 255, 255, 0));
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
  }

  Udp.begin(localPort);
  lastPacketTime = millis();
}

void loop() {
  processIncomingOSC();
  handleSignalTimeout();
  delay(2);
}
