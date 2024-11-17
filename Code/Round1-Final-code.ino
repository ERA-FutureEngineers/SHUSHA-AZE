#include <Servo.h>
#include <Pixy2.h>
#include <Wire.h>
#include <MPU6050.h>

Servo servo_9;
Pixy2 pixy;
MPU6050 gyro;

float frontdist = 0, rightdist = 0, leftdist = 0;
int objectWidth = 0;
float knownWidth = 5.0, focalLength = 218, distance = 0;
bool colorDetected = false;
int targetAngle = 0;  
float currentAngle = 0;
float offsetZ = 0; 
int angle = 20;
float previousRotationZ = 0.0;

int L = 0, R = 0, j = 0, old = 0, count = 0;

long readUltrasonicDistance(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  return pulseIn(echoPin, HIGH);
}

unsigned long previousMillis = 0;
unsigned long previousTime;
unsigned long previousPixyMillis = 0;
unsigned long previousGyroMillis = 0;
const unsigned long pixyInterval = 100;
const unsigned long gyroInterval = 10;   

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setClock(400000);
  gyro.initialize();
  servo_9.attach(2);
  pixy.init();

  if (!gyro.testConnection()) {
    Serial.println("MPU6050 connection failed.");
    while (1);
  }

  servo_9.write(90);
  calibrateGyro();
  
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(14, OUTPUT);
  analogWrite(5, 75);
  analogWrite(6, 75);
  digitalWrite(14, HIGH);
  digitalWrite(16, HIGH);
  digitalWrite(15, LOW);
  digitalWrite(17, LOW);

  previousTime = millis();
}

void loop() {
  if (L == 0 && R == 0) {
    MeasureDistance();
    if (rightdist > 80) {
      R = 1;
    } else if (leftdist > 80) {
      L = 1;
    }
  unsigned long currentMillis = millis();
  if (currentMillis - previousGyroMillis >= gyroInterval) {
    previousGyroMillis = currentMillis;
    Gyro();
    }
  }

  while (L == 1 || R == 1) {
    MeasureDistance();

  unsigned long currentMillis = millis();
  if (currentMillis - previousGyroMillis >= gyroInterval) {
    previousGyroMillis = currentMillis;
    Gyro();
  }

    if (currentMillis - previousMillis >= 3000 && leftdist >= 100 && rightdist < 100) { 
      previousMillis = currentMillis;
      targetAngle += 90;
      angle = 35;
      analogWrite(5, 60);
      analogWrite(6, 60);
      count++;
    }

    if (currentMillis - previousMillis >= 3000 && rightdist >= 100 && leftdist < 100) { 
      previousMillis = currentMillis;
      targetAngle -= 90;
      angle = 35;
      analogWrite(5, 60);
      analogWrite(6, 60);
      count++;
    }

    if(count == 12 and currentMillis - previousMillis >= 1500){
      Stop();
      delay(100000);
    }

    if (currentAngle >= targetAngle - 10 && currentAngle <= targetAngle + 10) {
      angle = 20;
      analogWrite(5, 75);
      analogWrite(6, 75);
    }
      if (rightdist <= 5) {
        servo_9.write(115);
        delay(200);
      }
      if (leftdist <= 5) {
        servo_9.write(65);
        delay(200);
    }
  }
}

void Forward() {
  analogWrite(6, 65);
  analogWrite(5, 65);
  digitalWrite(14, HIGH);
  digitalWrite(16, HIGH);
  digitalWrite(15, LOW);
  digitalWrite(17, LOW);
}

void Backward() {
  analogWrite(6, 65);
  analogWrite(5, 65);
  digitalWrite(15, HIGH);
  digitalWrite(17, HIGH);
  digitalWrite(14, LOW);
  digitalWrite(16, LOW);
}

void Stop() {
  analogWrite(6, 0);
  analogWrite(5, 0);
}

void Move() {
  analogWrite(6, 65);
  analogWrite(5, 65);
}

void MeasureDistance() {
  leftdist = 0.01723 * readUltrasonicDistance(A6, A7);
  delay(10);
  rightdist = 0.01723 * readUltrasonicDistance(A0, A1);
  delay(10);
}


void updateGyroAngle() {
  int16_t gz, ax, ay, az;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0; 
  previousTime = currentTime;

  gz = gyro.getRotationZ();
  ax = gyro.getAccelerationX();
  ay = gyro.getAccelerationY();
  az = gyro.getAccelerationZ();

  float rotationZ = (gz / 131.0) - offsetZ;
  currentAngle += rotationZ * deltaTime;

}

void Gyro() {
  if (currentAngle - 5 > targetAngle) {
    servo_9.write(90 - angle);
  } else if (currentAngle + 5 < targetAngle) {
    servo_9.write(90 + angle);
  } else {
    servo_9.write(90);
  }
  updateGyroAngle();
  delay(10);
}

void calibrateGyro() {
  int numReadings = 200;
  long totalZ = 0;

  Serial.println("Calibrating gyroscope... Keep the robot stationary.");
  for (int i = 0; i < numReadings; i++) {
    gyro.getRotationZ();
    delay(10);
  }

  for (int i = 0; i < numReadings; i++) {
    int16_t gz = gyro.getRotationZ();
    totalZ += gz;
    delay(10);
  }

  offsetZ = totalZ / (float)numReadings / 131.0; 
  Serial.print("Gyroscope Z-axis offset: ");
  Serial.println(offsetZ);
}