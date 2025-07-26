#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050.h>

MPU6050 mpu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

unsigned long lastTime = 0;  // For timing if you need it

// Servo limits (pulse width out of 4096)
#define SERVOMIN 150  // ~500us
#define SERVOMAX 600  // ~2500us

// Servo channels on PCA9685
#define SERVO_HAND_CHANNEL 0
#define SERVO_WRIST_CHANNEL 1
#define SERVO_ELBOW_CHANNEL 2
#define SERVO_UPPER_ELBOW_CHANNEL 3

// Current servo angles for smooth movement
int flexSensorHandAngle = 90;
int flexSensorWristAngle = 120;
int flexSensorElbowAngle = 90;
int GYROUpperElbowAngle = 90;

// Flex Sensor Pins
const int flexSensorHandPin = A0;
const int flexSensorWristPin = A1;
const int flexSensorElbowPin = A2;

// MPU6050 Complementary Filter Variables
float pitch = 0;       // Filtered pitch angle
float accelPitch = 0;  // Raw pitch from accelerometer
float gyroY = 0;       // Raw gyro Y-axis data
float alpha = 0.96;    // Complementary filter coefficient (0.96 = 96% gyro, 4% accel)
unsigned long lastTimeMPU = 0;

// Movement smoothing
const int stepSize = 5;       // Servo movement step (degrees)
const int delayPerStep = 10;  // Delay between steps (ms)


// Helper: Smooth sensor reading (average over samples)
int smoothFlexRead(int pin, int samples = 10) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  return sum / samples;
}

// Helper: Convert 0-180 degrees to PCA9685 pulse width
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1)
      ;
  }
  Serial.println("MPU6050 connected.");

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50Hz

  lastTime = millis();
  lastTimeMPU = micros();  // For gyro integration
  delay(100);
}

void loop() {
  // Read MPU6050 data
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // --- Complementary Filter for Pitch ---
  // 1. Calculate raw accelerometer pitch (degrees)
  accelPitch = atan2(-ax, sqrt((long)ay * ay + (long)az * az)) * 180.0 / PI;

  // 2. Get gyro Y-axis data (convert to degrees/sec)
  float dt = (micros() - lastTimeMPU) / 1000000.0;  // Delta time in seconds
  lastTimeMPU = micros();
  gyroY = gy / 131.0;  // MPU6050 sensitivity factor for +/-250deg/s range

  // 3. Complementary filter: Combine gyro (short-term) + accelerometer (long-term)
  pitch = alpha * (pitch + gyroY * dt) + (1 - alpha) * accelPitch;

  // --- Flex Sensor Readings ---
  int flexSensorHandValue = smoothFlexRead(flexSensorHandPin);
  int flexSensorWristValue = smoothFlexRead(flexSensorWristPin);
  int flexSensorElbowValue = smoothFlexRead(flexSensorElbowPin);

  // Map flex sensor values to servo angles
  int targetHandAngle = map(flexSensorHandValue, 600, 900, 0, 180);
  int targetWristAngle = map(flexSensorWristValue, 600, 900, 100, 140);
  int targetElbowAngle = map(flexSensorElbowValue, 600, 900, 30, 150);

  // Map filtered pitch to upper elbow angle (adjusted range for natural movement)
  int targetUpperElbowAngle = map(pitch, -45, 45, 60, 120);  // Reduced range for stability

  // Constrain all angles
  targetHandAngle = constrain(targetHandAngle, 0, 180);
  targetWristAngle = constrain(targetWristAngle, 0, 180);
  targetElbowAngle = constrain(targetElbowAngle, 0, 180);
  targetUpperElbowAngle = constrain(targetUpperElbowAngle, 0, 180);

  // Smoothly move servos toward targets
  flexSensorHandAngle += constrain(targetHandAngle - flexSensorHandAngle, -stepSize, stepSize);
  flexSensorWristAngle += constrain(targetWristAngle - flexSensorWristAngle, -stepSize, stepSize);
  flexSensorElbowAngle += constrain(targetElbowAngle - flexSensorElbowAngle, -stepSize, stepSize);
  GYROUpperElbowAngle += constrain(targetUpperElbowAngle - GYROUpperElbowAngle, -stepSize, stepSize);

  // Clamp angles
  flexSensorHandAngle = constrain(flexSensorHandAngle, 0, 180);
  flexSensorWristAngle = constrain(flexSensorWristAngle, 0, 180);
  flexSensorElbowAngle = constrain(flexSensorElbowAngle, 0, 180);
  GYROUpperElbowAngle = constrain(GYROUpperElbowAngle, 0, 180);

  // Drive servos
  pwm.setPWM(SERVO_HAND_CHANNEL, 0, angleToPulse(flexSensorHandAngle));
  pwm.setPWM(SERVO_WRIST_CHANNEL, 0, angleToPulse(flexSensorWristAngle));
  pwm.setPWM(SERVO_ELBOW_CHANNEL, 0, angleToPulse(flexSensorElbowAngle));
  pwm.setPWM(SERVO_UPPER_ELBOW_CHANNEL, 0, angleToPulse(GYROUpperElbowAngle));

  delay(delayPerStep);
}