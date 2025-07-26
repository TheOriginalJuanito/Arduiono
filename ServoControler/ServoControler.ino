#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

int sensor1Pin = A0;
int sensor2Pin = A1;
int sensor3Pin = A2;  // NEW sensor 3

#define SERVOMIN 150  // minimum pulse length count
#define SERVOMAX 600  // maximum pulse length count

#define SERVO1_CHANNEL 0
#define SERVO2_CHANNEL 1
#define SERVO3_CHANNEL 2  // NEW
#define SERVO4_CHANNEL 3  // NEW


int currentAngle1 = 90; 
int currentAngle3 = 90; 
int currentAngle2 = 120;
int currentAngle4 = 90;



void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(50);  // Standard for servos: 50 Hz
  delay(10);
}

int smoothFlexRead(int pin, int samples = 10) {
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  return sum / samples;
}

// Converts degrees (0-180) to PCA9685 pulse
int angleToPulse(int angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

void loop() {
  int stepSize = 2;       // Smaller = slower movement
  int delayPerStep = 10;  // Delay in ms between steps

  int flexValue1 = smoothFlexRead(sensor1Pin);
  int flexValue2 = smoothFlexRead(sensor2Pin);
  int flexValue3 = smoothFlexRead(sensor3Pin);  // NEW

  Serial.print("Sensor 1: ");
  Serial.print(flexValue1);
  Serial.print(" | Sensor 2: ");
  Serial.print(flexValue2);
  Serial.print(" | Sensor 3: ");
  Serial.println(flexValue3);

  int angle1 = map(flexValue1, 600, 900, 0, 180);
  int angle2 = map(flexValue2, 600, 900, 100, 140);
  int angle3 = map(flexValue3, 600, 900, 0, 180);  // NEW

  angle1 = constrain(angle1, 0, 180);
  angle2 = constrain(angle2, 0, 180);
  angle3 = constrain(angle3, 0, 90);
  int angle4 = 180 - angle3;  // Mirror (opposite) motion

  
  if (angle1 != currentAngle1) {
    if (angle1 > currentAngle1) currentAngle1 += stepSize;
    else currentAngle1 -= stepSize;
  }

  if (angle2 != currentAngle2) {
    if (angle2 > currentAngle2) currentAngle2 += stepSize;
    else currentAngle2 -= stepSize;
  }

  if (angle3 != currentAngle3) {
    if (angle3 > currentAngle3) currentAngle3 += stepSize;
    else currentAngle3 -= stepSize;
  }

  currentAngle1 = constrain(currentAngle1, 0, 180);
  currentAngle2 = constrain(currentAngle2, 0, 180);
  currentAngle3 = constrain(currentAngle3, 0, 90);
  currentAngle4 = 170 - currentAngle3;
  currentAngle4 = constrain(currentAngle4, 0, 170);


  pwm.setPWM(SERVO1_CHANNEL, 0, angleToPulse(currentAngle1));
  pwm.setPWM(SERVO2_CHANNEL, 0, angleToPulse(currentAngle2));
  pwm.setPWM(SERVO3_CHANNEL, 0, angleToPulse(currentAngle3));
  pwm.setPWM(SERVO4_CHANNEL, 0, angleToPulse(currentAngle4));
  delay(delayPerStep);

Serial.print("Servo 1: ");
Serial.print(currentAngle1);
Serial.print(" | Servo 2: ");
Serial.print(currentAngle2);
Serial.print(" | Servo 3: ");
Serial.print(currentAngle3);
Serial.print(" | Servo 4: ");
Serial.println(currentAngle4);


  delay(20);
}
