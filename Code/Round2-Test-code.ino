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

int Kp = 1.5, Kd = 0.3, errold = 0, err = 0, U = 0;
int target = 30, L = 0, R = 0, r = 0, l = 0, mx = 0, k = 0, old = 0, j = 0, X = 0, c = 0, oldc = 0;

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

void setup() {
  Wire.begin();
  Serial.begin(9600);
  gyro.initialize();
  servo_9.attach(2);
  pixy.init();
  offsetZ = gyro.getRotationZ() / 131.0;
  servo_9.write(87);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(14, OUTPUT);
  analogWrite(5, 70);
  analogWrite(6, 70);
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
    Gyro();
  }

  while (L == 1 || R == 1) {
    pixy.ccc.getBlocks();
    bool greenDetected = false, redDetected = false;
    int colorX = 0, k = 0, j = 0, old = 0, mx = 0;
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if (pixy.ccc.blocks[i].m_signature == 1) {
        greenDetected = true;
        colorX = pixy.ccc.blocks[i].m_x;
        objectWidth = pixy.ccc.blocks[i].m_width;
      }
      if (pixy.ccc.blocks[i].m_signature == 2) {
        redDetected = true;
        colorX = pixy.ccc.blocks[i].m_x;
        objectWidth = pixy.ccc.blocks[i].m_width;
      }
    }

    MeasureDistance();
    distance = (knownWidth * focalLength) / objectWidth;

    if (redDetected) {
      updateGyroAngle();
      if (colorX > 60)
        servo_9.write(70);
      else
        servo_9.write(90);
    } else if (greenDetected) {
      updateGyroAngle();
      if (colorX < 220)
        servo_9.write(110);
      else
        servo_9.write(90);
    } else {
      MeasureDistance();
      Gyro();
      if (rightdist <= 5) {
        servo_9.write(115);
        delay(300);
      }
      if (leftdist <= 5) {
        servo_9.write(65);
        delay(300);
      }
      unsigned long currentMillis = millis();
      if (currentMillis - previousMillis >= 3000 && leftdist >= 100 && rightdist < 100) { 
        previousMillis = currentMillis;
        targetAngle += 85;
        angle = 30;
      }
      if (currentAngle >= targetAngle - 10 && currentAngle <= targetAngle + 10) {
        angle = 20;
      }
    }
  }
}

void Forward() {
  analogWrite(6, 70);
  analogWrite(5, 70);
  digitalWrite(14, HIGH);
  digitalWrite(16, HIGH);
  digitalWrite(15, LOW);
  digitalWrite(17, LOW);
}

void Backward() {
  analogWrite(6, 70);
  analogWrite(5, 70);
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
  analogWrite(6, 70);
  analogWrite(5, 70);
}

void MeasureDistance() {
  leftdist = 0.01723 * readUltrasonicDistance(A6, A7);
  delay(10);
  rightdist = 0.01723 * readUltrasonicDistance(A0, A1);
  delay(10);
}

float accelAngle, filteredAngle;
const float alpha = 0.98;

void updateGyroAngle() {
  int16_t gz, ax, ay;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;
  gz = gyro.getRotationZ();
  float rotationZ = (gz / 131.0) - offsetZ;
  if (rotationZ * deltaTime >= 0.25 || rotationZ * deltaTime <= -0.25) { 
    currentAngle += rotationZ * deltaTime;
  }
  ax = gyro.getAccelerationX();
  ay = gyro.getAccelerationY();
  accelAngle = atan2(ay, ax) * (180.0 / PI);
  filteredAngle = alpha * (filteredAngle + rotationZ * deltaTime) + (1 - alpha) * accelAngle;
  if (filteredAngle >= 0.25 && filteredAngle <= -0.25)
    currentAngle = filteredAngle;
}

void Gyro() {
  if (currentAngle - 10 > targetAngle) {
    servo_9.write(90 - angle);
  } else if (currentAngle + 10 < targetAngle) {
    servo_9.write(90 + angle);
  } else {
    servo_9.write(90);
  }
  updateGyroAngle();
}
