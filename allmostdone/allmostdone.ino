#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
MPU6050 mpu;

// Flex Sensor Pins
int flexSensorHandPin = A0;
int flexSensorWristPin = A1;
int flexSensorElbowPin = A2;

// Complementary filter variables
float pitch = 0.0;  // orientation angle estimate (degrees)
float alpha = 0.96; // complementary filter coefficient
unsigned long lastTime;

// Servo limits (pulse width out of 4096)
#define SERVOMIN 150 
#define SERVOMAX 600 

// Servo channels on PCA9685
#define SERVO_HAND_CHANNEL 0
#define SERVO_WRIST_CHANNEL 1
#define SERVO_ELBOW_CHANNEL 2
#define SERVO_UPPER_ELBOW_CHANNEL 3

// Current servo angles for smooth movement
int flexSensorHandAngle = 90; 
int flexSensorWristAngle = 120;
int flexSensorElbowAngle = 90;
int flexSensorUpperElbowAngle = 90;

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

  pwm.begin();
  pwm.setPWMFreq(50);  // Standard servo freq
  delay(10);

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1);
  }
  Serial.println("MPU6050 connected.");

  lastTime = millis();
}

void loop() {
  const int stepSize = 5;      // Servo movement smoothing step
  const int delayPerStep = 10; // Delay between steps for smoothness (ms)

  // Read flex sensors with smoothing
  int flexSensorHandValue = smoothFlexRead(flexSensorHandPin);
  int flexSensorWristValue = smoothFlexRead(flexSensorWristPin);
  int flexSensorElbowValue = smoothFlexRead(flexSensorElbowPin);

  // Read raw gyro & accel data from MPU6050
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Calculate delta time for filter
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // Calculate pitch from accelerometer (degrees)
  float pitchAcc = atan2(ay, az) * 180.0 / PI;

  // Gyro rate around X axis (degrees/s)
  float gyroRateX = gx / 131.0;

  // Complementary filter fusion
  pitch = alpha * (pitch + gyroRateX * dt) + (1 - alpha) * pitchAcc;

  // Map pitch (-90 to 90) to servo angle range (60 to 130) for upper elbow
  int targetUpperElbowAngle = map(pitch, -90, 90, 60, 130);
  targetUpperElbowAngle = constrain(targetUpperElbowAngle, 0, 180);

  // Map flex sensor values to servo angles
  int targetHandAngle = map(flexSensorHandValue, 600, 900, 0, 180);
  int targetWristAngle = map(flexSensorWristValue, 600, 900, 100, 140);
  int targetElbowAngle = map(flexSensorElbowValue, 600, 900, 30, 150);

  // Constrain all angles
  targetHandAngle = constrain(targetHandAngle, 0, 180);
  targetWristAngle = constrain(targetWristAngle, 0, 180);
  targetElbowAngle = constrain(targetElbowAngle, 0, 180);

  // Smoothly move current angles towards targets
  if (targetHandAngle != flexSensorHandAngle)
    flexSensorHandAngle += (targetHandAngle > flexSensorHandAngle) ? stepSize : -stepSize;

  if (targetWristAngle != flexSensorWristAngle)
    flexSensorWristAngle += (targetWristAngle > flexSensorWristAngle) ? stepSize : -stepSize;

  if (targetElbowAngle != flexSensorElbowAngle)
    flexSensorElbowAngle += (targetElbowAngle > flexSensorElbowAngle) ? stepSize : -stepSize;

  if (targetUpperElbowAngle != flexSensorUpperElbowAngle)
    flexSensorUpperElbowAngle += (targetUpperElbowAngle > flexSensorUpperElbowAngle) ? stepSize : -stepSize;

  // Clamp current angles
  flexSensorHandAngle = constrain(flexSensorHandAngle, 0, 180);
  flexSensorWristAngle = constrain(flexSensorWristAngle, 0, 180);
  flexSensorElbowAngle = constrain(flexSensorElbowAngle, 0, 180);
  flexSensorUpperElbowAngle = constrain(flexSensorUpperElbowAngle, 0, 180);

  // Write angles to servos via PCA9685
  pwm.setPWM(SERVO_HAND_CHANNEL, 0, angleToPulse(flexSensorHandAngle));
  pwm.setPWM(SERVO_WRIST_CHANNEL, 0, angleToPulse(flexSensorWristAngle));
  pwm.setPWM(SERVO_ELBOW_CHANNEL, 0, angleToPulse(flexSensorElbowAngle));
  pwm.setPWM(SERVO_UPPER_ELBOW_CHANNEL, 0, angleToPulse(flexSensorUpperElbowAngle));

  // Debug printout
  Serial.print("Hand: "); Serial.print(flexSensorHandAngle);
  Serial.print(" | Wrist: "); Serial.print(flexSensorWristAngle);
  Serial.print(" | Elbow: "); Serial.print(flexSensorElbowAngle);
  Serial.print(" | UpperElbow (Pitch): "); Serial.println(flexSensorUpperElbowAngle);

  delay(delayPerStep);
}
