#include <Servo.h>
#include <Pixy2.h>
#include <Wire.h>
#include <MPU6050.h>

Servo servo_9;
Pixy2 pixy;
MPU6050 gyro;

float frontdist = 0, rightdist = 0, leftdist = 0;
int objectWidth = 0;
float knownWidth = 5.0, focalLength = 200, distance = 0;
bool colorDetected = false;
float targetAngle = 0;
float currentAngle = 0;
float offsetZ = 0;
int angle = 20;
float previousRotationZ = 0.0;
int colorX = 0;
int target = 30, L = 0, R = 0, old = 0, j = 0, count = 0, last_color = 0, old_last_color = 0, reverse = 0, height = 0, old1 = 1, old2 = 1, old3 = 1, secondcount = 0, second = 3000, gyro_count = 0;
   bool greenDetected = false, redDetected = false;

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
unsigned long CautionMillis = 0;
unsigned long PreviousCautionMillis = 0;

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
  analogWrite(5, 65);
  analogWrite(6, 65);
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

  if(R == 1 or L == 1){
    servo_9.write(90);
    Stop();
    delay(500);
    Backward();    
    CautionMillis = millis();
    PreviousCautionMillis = millis();
    while(CautionMillis - PreviousCautionMillis <= 1000){
    CautionMillis = millis();
    updateGyroAngle();
    }
    Forward();
  }

  while (L == 1 || R == 1) {
    pixy.ccc.getBlocks();
    bool parkingDetected = false;  
    greenDetected = false, redDetected = false;
    int k = 0, j = 0, old = 0, mx = 0;
    colorX = 0;
    height = 0;
    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if(old <= (pixy.ccc.blocks[i].m_height)){
        old = (pixy.ccc.blocks[i].m_height);
        j = i;
      }
    }
      if(count >= 4 and pixy.ccc.blocks[j].m_signature == 3){
      parkingDetected = true;
      Parking();
      }

      if (pixy.ccc.blocks[j].m_signature == 1) {
        greenDetected = true;
        colorX = pixy.ccc.blocks[j].m_x;
        objectWidth = pixy.ccc.blocks[j].m_width;
        height = pixy.ccc.blocks[j].m_height;
      }
      if (pixy.ccc.blocks[j].m_signature == 2) {
        redDetected = true;
        colorX = pixy.ccc.blocks[j].m_x;
        objectWidth = pixy.ccc.blocks[j].m_width;
        height = pixy.ccc.blocks[j].m_height;
      }

    MeasureDistance();
    distance = (knownWidth * focalLength) / objectWidth;
      if (rightdist <= 5) {
        while(rightdist <= 10){
        servo_9.write(115);
        rightdist = 0.01723 * readUltrasonicDistance(A0, A1);
        delay(10);
        }
      }
      if (leftdist <= 5) {
        while(leftdist <= 10){
        servo_9.write(65);
        leftdist = 0.01723 * readUltrasonicDistance(A6, A7);
        delay(10);
        }
      }

    unsigned long currentMillis = millis(); 
    if (currentMillis - previousMillis >= 3000 && leftdist >= 100 && rightdist < 100 and L == 1) { 
      previousMillis = currentMillis;
      targetAngle += 89.5;
      angle = 30;
      count++;
    }

    if (currentMillis - previousMillis >= 3000 && rightdist >= 100 && leftdist < 100 and R == 1) { 
      previousMillis = currentMillis;
      targetAngle -= 90.5;
      angle = 30;
      count++;
    }

    if (abs(currentAngle - targetAngle) <= 10) {
      angle = 20;
    }
    if(greenDetected or redDetected)
    ColorSearching();
    else 
    Gyro();
  }
}

void Forward() {
  analogWrite(6, 65);
  analogWrite(5, 65);
  digitalWrite(14, HIGH);
  digitalWrite(16, HIGH);
  digitalWrite(15, LOW);
  digitalWrite(18, LOW);
}

void Backward() {
  analogWrite(6, 75);
  analogWrite(5, 75);
  digitalWrite(15, HIGH);
  digitalWrite(18, HIGH);
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
  // frontdist = 0.01723 * readUltrasonicDistance(A4, A5);
  // delay(10);  
}

void updateGyroAngle() {
  int16_t gz, ax, ay, az;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  gz = gyro.getRotationZ();

  float rotationZ = (gz / 131.0) - offsetZ;
  currentAngle += rotationZ * deltaTime;

  Serial.print("Filtered Angle (Yaw): ");
  Serial.println(currentAngle);
}

void Gyro() {
  int deadzone = 5;
  if (currentAngle > targetAngle + deadzone) {
    servo_9.write(90 - angle);
  } else if (currentAngle < targetAngle - deadzone) {
    servo_9.write(90 + angle);
  } else {
    servo_9.write(90);
  }
  updateGyroAngle();
}


void calibrateGyro() {
  int numReadings = 200;
  long totalZ = 0;

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

void ColorSearching() {
  if (redDetected) {
    if ((R == 1 && abs(currentAngle - targetAngle) <= 35) || L == 1) {
        updateGyroAngle();
        last_color = 2;

        if (colorX > 50) {
            servo_9.write(65);
        } else if (R == 1 && colorX < 10 && abs(currentAngle - targetAngle) >= 10) {
            servo_9.write(115);
        } else {
            servo_9.write(90);
      }
    }
    else
    Gyro();
  }
  else if (greenDetected) {
    if ((L == 1 && abs(currentAngle - targetAngle) <= 35) || R == 1) {
        updateGyroAngle();
        last_color = 1;
        if (colorX < 230) {
            servo_9.write(115);
        } else if (L == 1 && colorX > 260 && abs(currentAngle - targetAngle) >= 10) {
            servo_9.write(65);
        } else {
            servo_9.write(90);
        }
      }
      else
      Gyro();
    }
}

// void ReverseTurning() {
//     if(last_color == 2 and count == 8 and reverse == 0){
//       while(abs(currentAngle - targetAngle) >= 5){
//         Gyro();
//       }
//       angle = 25;
//       Backward();
//       targetAngle += 90;
//       analogWrite(5, 70);
//       analogWrite(6, 70);
//       while(abs(currentAngle - targetAngle) >= 5){
//         servo_9.write(60);
//         updateGyroAngle();
//       }
//       servo_9.write(90);
//       delay(500);
//       Forward();      
//       analogWrite(5, 65);
//       analogWrite(6, 65);
//       angle = 20;
//       if(L == 1){R = 1; L = 0;}
//       else if(R == 1){L = 1; R = 0;}
//       count = 100;
//       targetAngle += 90;
//     }
//     else if(count == 8)
//     reverse = 1;
// }

void Parking() {
  bool parkingDetected = true;
  secondcount = millis();
  
    unsigned long lastDetectionTime = millis();
    int target = 42, Kp = 1.5, Kd = 0.3, err = 0, errold = 0, U = 0;

    while (parkingDetected) {
      pixy.ccc.getBlocks();
      MeasureDistance();
      int colorX = 0;
      redDetected = false, greenDetected = false;

    for (int i = 0; i < pixy.ccc.numBlocks; i++) {
      if(old <= (pixy.ccc.blocks[i].m_height)){
        old = (pixy.ccc.blocks[i].m_height);
        j = i;
      }
    }
      if (pixy.ccc.blocks[j].m_signature == 1) {
        greenDetected = true;
        colorX = pixy.ccc.blocks[j].m_x;
      }
      if (pixy.ccc.blocks[j].m_signature == 2) {
        redDetected = true;
        colorX = pixy.ccc.blocks[j].m_x;
      }
      if (pixy.ccc.blocks[j].m_signature == 3) {
        lastDetectionTime = millis();
        colorX = pixy.ccc.blocks[j].m_x;
      }

      if (millis() - lastDetectionTime > 1000) {
        parkingDetected = false;
      }

    if(abs(currentAngle - targetAngle <= 20)){
    if(greenDetected or redDetected)
    ColorSearching();
    else if(pixy.ccc.blocks[j].m_signature == 3){
      // if(leftdist >= 50){
      
      // if(R == 1){
      //     if (colorX > 40) {
      //       servo_9.write(65);
      //   } else if (colorX < 10 && abs(currentAngle - targetAngle) >= 10) {
      //       servo_9.write(115);
      //   } else {
      //       servo_9.write(90);
      //   }    
      // }
      // else if(L == 1){
      //     if (colorX < 270) {
      //       servo_9.write(115);
      //   } else if (L == 1 && colorX > 300 && abs(currentAngle - targetAngle) >= 10) {
      //       servo_9.write(70);
      //   } else {
      //       servo_9.write(90);
      //       }      
      //     }
        // }
        // else{
        if(L == 1){
          err = leftdist - target;
          U = err * Kp + Kd * (err - errold);
          if (90 + U > 120) {
          servo_9.write(120);
          }
           else {
          servo_9.write(90 + U);
          }
          errold = err;  
         } 
        }
      }
    else {
      Gyro();
    }

    unsigned long currentMillis = millis(); 
    if(currentMillis - secondcount < 5000){
    if (currentMillis - previousMillis >= 3000 && leftdist >= 100 && rightdist < 100 and L == 1) { 
      previousMillis = currentMillis;
      targetAngle += 90;
      angle = 35;
      count++;
    }

    if (currentMillis - previousMillis >= 3000 && rightdist >= 100 && leftdist < 100 and R == 1) { 
      previousMillis = currentMillis;
      targetAngle -= 90;
      angle = 35;
      count++;
      }
    }
  }
    currentAngle = 0;
    analogWrite(5, 0);
    analogWrite(6, 0);
    delay(500);
    Backward();
    delay(500);
    analogWrite(5, 85);
    analogWrite(6, 85);

    CautionMillis = millis();
    PreviousCautionMillis = millis();
    while(CautionMillis - PreviousCautionMillis <= 1500){
    CautionMillis = millis();
      if(L == 1)
      servo_9.write(125);
      if(R == 1)
      servo_9.write(55);
    }
    servo_9.write(90);
    MeasureDistance();
    Forward();
    analogWrite(5, 60);
    analogWrite(6, 60);

    while (leftdist >= 15 || rightdist >= 15) {
      MeasureDistance();
      pixy.ccc.getBlocks();
      int colorX1 = 0;
      int colorX2 = 0;
      int X0 = 0;
      int X1 = 0;
      if(pixy.ccc.numBlocks == 1){
        servo_9.write(90);
      }
      else if(pixy.ccc.numBlocks == 2){
          colorX1 = pixy.ccc.blocks[0].m_x;
          colorX2 = pixy.ccc.blocks[1].m_x;
          if(colorX1 < colorX2){
            X0 = colorX1;
            X1 = colorX2; 
          }
          else {
            X0 = colorX2;
            X1 = colorX1;             
          }
      }

      if(abs(158 - X0) > abs(158 - X1) + 10)
      servo_9.write(110);
      else if(abs(158 - X0) + 10 < abs(158 - X1))
      servo_9.write(70);
      else 
      servo_9.write(90);
    }

    while (L == 1 or R == 1) {
      servo_9.write(90);
      Stop();
  }
}
