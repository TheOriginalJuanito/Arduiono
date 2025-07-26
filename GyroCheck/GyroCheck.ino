#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo servo;

float pitch = 0.0;
float alpha = 0.96;
unsigned long lastTime;

int16_t gx_offset = 0;

void calibrateGyro() {
  const int calibrationSamples = 1000;
  long gx_sum = 0;

  Serial.println("Calibrating gyro... Keep sensor still.");

  for (int i = 0; i < calibrationSamples; i++) {
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    gx_sum += gx;
    delay(2);
  }

  gx_offset = gx_sum / calibrationSamples;

  Serial.print("Gyro X offset: ");
  Serial.println(gx_offset);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  servo.attach(9);  // Servo signal connected to Arduino pin 9

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1);
  } else {
    Serial.println("MPU6050 connected");
  }

  calibrateGyro();

  lastTime = millis();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  float pitchAcc = atan2(ay, az) * 180.0 / PI;
  float gyroRateX = (gx - gx_offset) / 131.0;

  pitch = alpha * (pitch + gyroRateX * dt) + (1 - alpha) * pitchAcc;

  // Map pitch (-90 to 90) to servo angle (0 to 180)
  int servoAngle = map(pitch, -90, 90, 0, 180);
  servoAngle = constrain(servoAngle, 0, 180);

  servo.write(servoAngle);

  Serial.print("Pitch (acc): ");
  Serial.print(pitchAcc);
  Serial.print(" | Gyro rate X: ");
  Serial.print(gyroRateX);
  Serial.print(" | Filtered pitch: ");
  Serial.print(pitch);
  Serial.print(" | Servo angle: ");
  Serial.println(servoAngle);

  delay(10);
}
