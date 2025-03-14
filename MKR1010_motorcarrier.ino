
//#include <SPI.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OSCMessage.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <utility/wifi_drv.h>
#include <ArduinoMotorCarrier.h>  // Nano Motor Carrier library
//#define INTERRUPT_PIN 6

//Variable to store the battery voltage
static float batteryVoltage;
//low battery limit (discharged)
static float batteryLimit;

char ssid[] = "dancehack";  // your network SSID (name)
char pass[] = "hackhack";  // your network password

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;
//const IPAddress outIp(192, 168, 8, 100);  // remote IP (not needed for receive)
const IPAddress ip(192, 168, 6, 77);  //Set the board ip

const unsigned int outPort = 9999;    // remote port (not needed for receive)
const unsigned int localPort = 8888;  // local port to listen for UDP packets (here's where we send the packets)

OSCErrorCode error;

unsigned int ledR = 0;  // 0-255
unsigned int ledG = 0;  // 0-255
unsigned int ledB = 0;  // 0-255
int motor1 = 0;         // -100-100
int motor2 = 0;         // -100-100
int motor3 = 0;         // -100-100
int motor4 = 0;         // -100-100
unsigned int ser1 = 0;  // 0-180
unsigned int ser2 = 0;  // 0-180
unsigned int ser3 = 0;  // 0-180
unsigned int ser4 = 0;  // 0-180

void setup() {
  Serial.begin(115200);
  WiFi.config(ip);

  if (controller.begin()) {
    Serial.print("MKR Motor Shield connected, firmware version ");
    Serial.println(controller.getFWVersion());
    controller.reboot();
    delay(500);

    M1.setDuty(0);
    M2.setDuty(0);
    M3.setDuty(0);
    M4.setDuty(0);
  } else {
    Serial.println("Couldn't connect motor shield!");
  }

  WiFiDrv::pinMode(25, OUTPUT);  //define red
  WiFiDrv::pinMode(26, OUTPUT);  //define green
  WiFiDrv::pinMode(27, OUTPUT);  //define blue

  WiFiDrv::analogWrite(25, ledR);
  WiFiDrv::analogWrite(26, ledG);
  WiFiDrv::analogWrite(27, ledB);

  // Connect to WiFi network
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("");
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("Starting UDP");
  Serial.print("Local port: ");
  Serial.println(localPort);
  Udp.begin(localPort);
  WiFiDrv::analogWrite(26, 250);
  delay(500);
  WiFiDrv::analogWrite(26, 0);
  delay(500);
  WiFiDrv::analogWrite(26, 250);
  delay(500);
  WiFiDrv::analogWrite(26, 0);
  delay(500);
}

void led1(OSCMessage &msg) {
  ledR = msg.getInt(0);
  WiFiDrv::analogWrite(25, ledR);
  Serial.print("/ledR: ");
  Serial.println(ledR);
}
void led2(OSCMessage &msg) {
  ledG = msg.getInt(0);
  WiFiDrv::analogWrite(26, ledG);
  Serial.print("/ledG: ");
  Serial.println(ledG);
}
void led3(OSCMessage &msg) {
  ledB = msg.getInt(0);
  WiFiDrv::analogWrite(27, ledB);
  Serial.print("/ledB: ");
  Serial.println(ledB);
}
void motorA(OSCMessage &msg) {
  motor1 = msg.getInt(0);
  M1.setDuty(motor1);
  Serial.print("/motor1: ");
  Serial.println(motor1);
}
void motorB(OSCMessage &msg) {
  motor2 = msg.getInt(0);
  M2.setDuty(motor2);
  Serial.print("/motor2: ");
  Serial.println(motor2);
}
void motorC(OSCMessage &msg) {
  motor3 = msg.getInt(0);
  M3.setDuty(motor3);
  Serial.print("/motor3: ");
  Serial.println(motor3);
}
void motorD(OSCMessage &msg) {
  motor4 = msg.getInt(0);
  M4.setDuty(motor4);
  Serial.print("/motor4: ");
  Serial.println(motor4);
}

void servoA(OSCMessage &msg) {
  ser1 = msg.getInt(0);
  servo1.setAngle(ser1);
  Serial.print("/servo1: ");
  Serial.println(ser1);
}
void servoB(OSCMessage &msg) {
  ser2 = msg.getInt(0);
  servo2.setAngle(ser2);
  Serial.print("/servo2: ");
  Serial.println(ser2);
}
void servoC(OSCMessage &msg) {
  ser3 = msg.getInt(0);
  servo3.setAngle(ser3);
  Serial.print("/ser3: ");
  Serial.println(ser3);
}
void servoD(OSCMessage &msg) {
  ser4 = msg.getInt(0);
  servo4.setAngle(ser4);
  Serial.print("/servo4: ");
  Serial.println(ser4);
}

void loop() {

  OSCBundle bundle;
  OSCMessage msg;

  int size = Udp.parsePacket();
  //Serial.println(size);
  if (size > 0) {
    while (size--) {
      bundle.fill(Udp.read());
    }
    if (!bundle.hasError()) {
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
      bundle.empty();
    } else {
      error = bundle.getError();
      Serial.print("error: ");
      Serial.println(error);
      bundle.empty();
    }
  }
  controller.ping();
}
