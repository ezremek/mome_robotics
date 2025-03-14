#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <OSCBundle.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

// WiFi credentials
const char* ssid = "dancehack";
const char* password = "hackhack";

// OSC settings
const IPAddress outIp(192, 168, 6, 152);  // Replace with your receiver's IP
const unsigned int outPort = 7777;        // Replace with the OSC receiver port
WiFiUDP Udp;

// BNO085 settings
Adafruit_BNO08x bno085;
sh2_SensorValue_t sensorValue;

unsigned long lastSendTime = 0;
const int updateRate = 10;  // 100Hz = 1000ms / 100 = 10ms

// LED blink interval
unsigned long lastBlinkTime = 0;
const int blinkInterval = 500;  // 500 ms interval for LED blink
const int led = 10;             //led brightness
bool ledState = false;          // Keeps track of LED on/off state
bool errorState = false;        // Tracks if we are in error mode (WiFi or sensor failure)
bool sensorError = false;       // Tracks if the error is due to the sensor

// Struct to store Euler angles
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr, customYpr;

// Function declarations
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false);
void quaternionToCustomEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false);
void setLED(int red, int green, int blue);


void setup() {
  delay(500);

  // Configure static IP for stability
  //IPAddress local_IP(192, 168, 0, 123);
  //IPAddress gateway(192, 168, 0, 1);
  // IPAddress subnet(255, 255, 255, 0);
  // WiFi.config(local_IP, gateway, subnet);

  // Begin WiFi connection
  WiFi.begin(ssid, password);

  // Wait for WiFi connection
  while (WiFi.status() != WL_CONNECTED) {
    setLED(led, 0, 0);  // 50% Red for WiFi connection attempt
    delay(500);
  }

  // Initialize UDP
  Udp.begin(8888);

  // Initialize BNO085 sensor
  if (!bno085.begin_I2C()) {
    sensorError = true;
    while (1) {
      setLED(led, 0, led);  // 50% Purple (Red + Blue) for sensor failure
      delay(500);
    }
  }

  // Set both linear acceleration and gyro integrated quaternion to 100Hz
  bno085.enableReport(SH2_LINEAR_ACCELERATION, 100);
  bno085.enableReport(SH2_GYRO_INTEGRATED_RV, 100);
}

void loop() {
  unsigned long currentTime = millis();

  // Check WiFi status
  if (WiFi.status() != WL_CONNECTED) {
    errorState = true;
    sensorError = false;  // Only WiFi error
  } else {
    errorState = false;
  }

  // Check sensor status (if the sensor fails to get data, trigger sensor error)
  if (!bno085.getSensorEvent(&sensorValue)) {
    sensorError = true;
  } else {
    sensorError = false;
  }

  // Blinking logic for LED (Red if WiFi error, Purple if sensor error, Green if working)
  if (currentTime - lastBlinkTime >= blinkInterval) {
    lastBlinkTime = currentTime;
    ledState = !ledState;  // Toggle LED state

    if (errorState) {
      // Blink red at 50% brightness for WiFi error
      if (ledState) {
        setLED(led, 0, 0);  // 50% Red LED on
      } else {
        setLED(0, 0, 0);  // LED off
      }
    } else if (sensorError) {
      // Blink purple (red + blue) at 50% brightness for sensor error
      if (ledState) {
        setLED(led, 0, led);  // 50% Purple LED on (Red + Blue)
      } else {
        setLED(0, 0, 0);  // LED off
      }
    } else {
      // Blink green at 50% brightness if everything is working
      if (ledState) {
        setLED(0, led, 0);  // 50% Green LED on
      } else {
        setLED(0, 0, 0);  // LED off
      }
    }
  }

  // Limit the loop to run at the desired rate (100Hz)
  if (currentTime - lastSendTime >= updateRate) {
    lastSendTime = currentTime;

    OSCBundle bundle;

    // Linear acceleration data (without gravity)
    if (sensorValue.sensorId == SH2_LINEAR_ACCELERATION) {
      float accelX = sensorValue.un.linearAcceleration.x;
      float accelY = sensorValue.un.linearAcceleration.y;
      float accelZ = sensorValue.un.linearAcceleration.z;

      // Add linear acceleration data to the bundle
      bundle.add("/imu/acceloX").add(accelX);
      bundle.add("/imu/acceloY").add(accelY);
      bundle.add("/imu/acceloZ").add(accelZ);
    }

    // Quaternion data from gyro integrated rotation vector
    if (sensorValue.sensorId == SH2_GYRO_INTEGRATED_RV) {
      float quatX = sensorValue.un.gyroIntegratedRV.i;
      float quatY = sensorValue.un.gyroIntegratedRV.j;
      float quatZ = sensorValue.un.gyroIntegratedRV.k;
      float quatW = sensorValue.un.gyroIntegratedRV.real;

      // Add quaternion data to the bundle
      bundle.add("/imu/quaternionX").add(quatX);
      bundle.add("/imu/quaternionY").add(quatY);
      bundle.add("/imu/quaternionZ").add(quatZ);
      bundle.add("/imu/quaternionW").add(quatW);

      // Convert quaternion to Euler angles
      quaternionToEuler(quatW, quatX, quatY, quatZ, &ypr, true);
      quaternionToCustomEuler(quatW, quatX, quatY, quatZ, &customYpr, true);

      // Add standard Euler angles (yaw, pitch, roll) to the bundle
      bundle.add("/imu/yaw").add(ypr.yaw);
      bundle.add("/imu/pitch").add(ypr.pitch);
      bundle.add("/imu/roll").add(ypr.roll);

      // Add custom Euler angles with -90 to 90 range
      bundle.add("/imu/customYaw").add(customYpr.yaw);
      bundle.add("/imu/customPitch").add(customYpr.pitch);
      bundle.add("/imu/customRoll").add(customYpr.roll);
    }

    // Send the OSC bundle via UDP
    Udp.beginPacket(outIp, outPort);
    bundle.send(Udp);
    Udp.endPacket();
    bundle.empty();  // Clear the bundle for reuse
  }
}

// Function to control the RGB LED on the MKR1010
void setLED(int red, int green, int blue) {
  WiFiDrv::analogWrite(25, red);    // Red pin
  WiFiDrv::analogWrite(26, green);  // Green pin
  WiFiDrv::analogWrite(27, blue);   // Blue pin
}

// Function to convert quaternions to standard Euler angles (yaw, pitch, roll)
void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}

// Function to convert quaternions to custom Euler angles within -90 to 90 range
void quaternionToCustomEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  ypr->yaw = asin(2.0 * (qi * qj + qk * qr) / (sqi + sqj + sqk + sqr));
  ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  ypr->roll = asin(2.0 * (qj * qk + qi * qr) / (sqi + sqj + sqk + sqr));

  if (degrees) {
    ypr->yaw *= RAD_TO_DEG;
    ypr->pitch *= RAD_TO_DEG;
    ypr->roll *= RAD_TO_DEG;
  }
}
