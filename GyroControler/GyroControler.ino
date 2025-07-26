#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

MPU6050 mpu;
Servo myServo;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // I2C Scanner
  Serial.println("I2C Scanner");
  byte error, address;
  int nDevices = 0;

  for (address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println(" !");
      nDevices++;
    }
  }

  if (nDevices == 0) {
    Serial.println("No I2C devices found");
  } else {
    Serial.println("done");
  }
  delay(2000);

  // Initialize MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  } else {
    Serial.println("MPU6050 connected.");
  }

  // Attach servo to pin 9
  myServo.attach(9);
}

void loop() {
  int16_t gx, gy, gz;
  int16_t ax, ay, az; // not used here

  // Get gyro data
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Print gyro data
  Serial.print("Gyro: ");
  Serial.print("X="); Serial.print(gx);
  Serial.print(" Y="); Serial.print(gy);
  Serial.print(" Z="); Serial.println(gz);

  // Map gyro X to servo angle
  // gx ranges roughly from -17000 to +17000
  int angle = map(gx, -17000, 17000, 0, 180);
  angle = constrain(angle, 0, 180); // make sure it's in range

  myServo.write(angle);

  delay(100);
}
