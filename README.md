# SHUSHA-AZE 2024
 ### movzular

* [Electronic-Components](#components)
* [Robot Photos](#robotphotos)
* [Power Managament](#power)
* [Scheme](#scheme)
* [TeamPhoto](#tphoto)
* [Codes](#code)
  * [First Raund](#code)
  * [Second Raund]
* Design

### Electronic-Components <a class="anchor" id="components"></a>
* Pixy2.1 cam  
   
![pixy222](https://github.com/user-attachments/assets/ea5298ed-464a-4901-aa05-9aa7c3aeb38f)

Our main goal in choosing the pixy was its high performance and no lag. At first we were thinking of using esp cam, but after we found out that its image processing is weak, we switched to pixabay. It is very easy to connect the pixy to arduino or other microcontroller.

Biz bu kodda pixyle kub arasindaki mesafeni olcuruk

```ino
#include <Pixy2.h>
#include <Servo.h>

Servo servo_9;
long duration;
int distance;
 float knownWidth = 5.0;
 float focalLength = 218; 
 int colorX = 0;
Pixy2 pixy;

void setup() {
  servo_9.attach(2);
  Serial.begin(9600);
  pixy.init();
}

void loop() {
  pixy.ccc.getBlocks();
  
  if (pixy.ccc.numBlocks) {
    int objectWidth = pixy.ccc.blocks[0].m_width; 
    int colorX = pixy.ccc.blocks[0].m_x;
    
    float distance = (knownWidth * focalLength) / objectWidth;
    Serial.println(distance);
    delay(50);
  }
}
```
* MPU6050

![mpu222](https://github.com/user-attachments/assets/70847a97-fe15-469f-8533-9a55b9ff14db)

Bu kod gyro ile derece olcmek ucundur.
```ino
void updateGyroAngle() {
  int16_t gz;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  gz = gyro.getRotationZ();

  float rotationZ = (gz / 131.0) - offsetZ;
  currentAngle += rotationZ * deltaTime;

  Serial.print("Filtered Angle (Yaw): ");
  Serial.println(currentAngle);

```

 Robotu merkezde saxlamaq robotu duzeltmek ucun kod
```ino
void Gyro() {
  if (currentAngle - 5 > targetAngle) {
    servo_9.write(90 - angle);
  } else if (currentAngle + 5 < targetAngle) {
    servo_9.write(90 + angle);
  } else {
    servo_9.write(90);
  }
  updateGyroAngle();
}
```


To use the gyro sensor, we need to include the MPU6050.h library. We have given 5 seconds time to measure the error of the sensor while starting. This helps the robot to work more accurately and correctly

* Driver L298N and Dc motors

![driverl298n-222](https://github.com/user-attachments/assets/b9b312d4-a93b-4e89-94e5-a0e05b13fd90)
![dc-motors222](https://github.com/user-attachments/assets/86ba6389-0e62-4d51-9de1-91c468af46a2)

bu motorlari xodlamaq ucun koddur
```ino
#define LEFT_MOTOR_PWM 5
#define RIGHT_MOTOR_PWM 6
#define LEFT_MOTOR_DIR1 14
#define LEFT_MOTOR_DIR2 15
#define RIGHT_MOTOR_DIR1 16
#define RIGHT_MOTOR_DIR2 17

void setup() {
  pinMode(LEFT_MOTOR_PWM, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM, OUTPUT);
  pinMode(LEFT_MOTOR_DIR1, OUTPUT);
  pinMode(LEFT_MOTOR_DIR2, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR1, OUTPUT);
  pinMode(RIGHT_MOTOR_DIR2, OUTPUT);

  moveForward(70);
}

void loop() {
}

void moveForward(int speed) {
  digitalWrite(LEFT_MOTOR_DIR1, HIGH);
  digitalWrite(LEFT_MOTOR_DIR2, LOW);
  digitalWrite(RIGHT_MOTOR_DIR1, HIGH);
  digitalWrite(RIGHT_MOTOR_DIR2, LOW);
  analogWrite(LEFT_MOTOR_PWM, speed);
  analogWrite(RIGHT_MOTOR_PWM, speed);
}
```
We used the L298N and 2 Dc gear motors driver in the project. This driver works uniquely with dc motors and doesn't cause any problems for us.
* Servo

![servo222](https://github.com/user-attachments/assets/75496dab-44af-484e-960c-b52d35b29692)

We have done the freezing with servo means. We have configured the servo and gyro and ensured the robot works more accurately.

* Ultrasonic sensors
  
![ultra-s222](https://github.com/user-attachments/assets/b5a9c8c8-3723-4640-bd6a-d485d4359abd)

biz bu kod vasitesile mesacfeni olcuruk.
```ino
const int leftTrigPin = A6;
const int leftEchoPin = A7;
const int rightTrigPin = A0;
const int rightEchoPin = A1;

long getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration * 0.034 / 2;

  return distance;
}

void setup() {
  pinMode(leftTrigPin, OUTPUT);
  pinMode(leftEchoPin, INPUT);
  pinMode(rightTrigPin, OUTPUT);
  pinMode(rightEchoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  long leftDistance = getDistance(leftTrigPin, leftEchoPin);
  long rightDistance = getDistance(rightTrigPin, rightEchoPin);
  Serial.print(leftDistance);
  Serial.print(" cm, Right: ");
  Serial.print(rightDistance); 
  Serial.println(" cm");
  delay(10);
}
```

* Arduino Mega 2560 Rev3

![mega222](https://github.com/user-attachments/assets/acd3ee99-1cf3-49d2-a23e-a3e3defd6a00)

At first we were using arduino uno, but as we developed the robot, the sensors increased and there was no place for the pins for the sensors, and we decided to switch to the arduino mega. Arduino mega is far ahead of uno in terms of many features.

* E18-D80NK Ir sensors <a class="anchor" id="irsensors"></a>

![ir222](https://github.com/user-attachments/assets/ee5489d9-d60a-482e-b7aa-b3b7e78ee29f)

As you know, ultrasonics measure the distance with sound waves. because the sound wave form is less than the light form, it caused us many problems. we used it accordingly. now we use both ultrasonic sensor and E18-D80NK Ir sensors in the robot. These sensors provide data faster and work without delay.

### Robot Photos <a class="anchor" id="robotphotos"></a>

| <img src="./Robot-photos/Robot_Left.jpg" width="90%" /> | <img src="./Robot-photos/Robot_Right.jpg" width="85%" /> | 
| :--: | :--: | 
| *Left* | *Right* |
| <img src="./Robot-photos/Robot_Front.jpg" width="90%" /> | <img src="./Robot-photos/Robot_Back.jpg" width="85%" /> | 
| *Front* | *Back* |
| <img src="./Robot-photos/Robot_Up.jpg" width="90%" /> | <img src="./Robot-photos/Robot_Downjpg.jpg" width="85%" /> | 
| *Up* | *Down* |

### Power Managament <a class="anchor" id="power"></a>



We used Cylindrical Lithium Polymer Battery 3.7V 18650 Rechargeable Lipo Battery as a battery. We use 3 units of this battery and it gives us approximately 11v energy. We connect the 11v driver and get 5v energy from the output of the driver. This energy is enough for the sensors to work properly. Also, since these batteries are not very heavy, they help the robot to work faster

### Scheme <a class="anchor" id="scheme"></a>

| <img src="./Scheme/scheme-robot.jpg" width="90%" /> | <img src="./Scheme/scheme-robot2.jpg" width="85%" /> | 
| :--: | :--: | 
| *Scheme* | *Circuit diagram* |

### TeamPhoto <a class="anchor" id="tphoto"></a>

| <img src="./Team-photos/team-photo1.jpg" width="90%" /> |
| :--: | 
| *Normal Team Photo* | 

## Codes <a class="anchor" id="code"></a>

### First raund code <a class="anchor" id="code1st"></a>
```ino
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

int L = 0, R = 0, j = 0, old = 0;

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

  pixy.ccc.getBlocks();
  bool greenDetected = false, redDetected = false;
  int colorX = 0;

  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if(pixy.ccc.blocks[i].m_width > old){
      old = pixy.ccc.blocks[i].m_width;
      j = i;
    }
  }
    if (pixy.ccc.blocks[j].m_signature == 1) {
      greenDetected = true;
      colorX = pixy.ccc.blocks[j].m_x;
      objectWidth = pixy.ccc.blocks[j].m_width;
    }
    if (pixy.ccc.blocks[j].m_signature == 2) {
      redDetected = true;
      colorX = pixy.ccc.blocks[j].m_x;
      objectWidth = pixy.ccc.blocks[j].m_width;
    }
  }

  distance = (knownWidth * focalLength) / objectWidth;

  if (redDetected) {
    if (colorX > 70)
      servo_9.write(70);
    else
      servo_9.write(90);
  } else if (greenDetected) {
    if (colorX < 230)
      servo_9.write(110);
    else
      servo_9.write(90);
  }

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
    }

    if (currentMillis - previousMillis >= 3000 && rightdist >= 100 && leftdist < 100) { 
      previousMillis = currentMillis;
      targetAngle -= 90;
      angle = 35;
      analogWrite(5, 60);
      analogWrite(6, 60);
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
}
```
Immediately after connecting the robot, we turn on the gyro in the code and the gyro begins to measure the degree. First of all, we need to know in which direction the robot is going. Accordingly, we wrote in the code that if one of the ultrasonic sensors sees a distance of more than 80 cm, it means that it has reached the dong. We call this a decision, and the decision is repeated only once after the robot is connected. For example, if the right ultrasonic distance is more than 80 cm, it means that we are going in a clockwise direction. Otherwise, if the left sensor measures the distance above 80 cm, it means that we are going counter-clockwise. The ultrasonic sees above 80cm and the servo freezes until the gyro is 90 degrees. this counter is necessary so that we know that we have finished the 3rd round. If the gyro freezes 12 times at 90 degrees, it means that we have already finished the 3rd round and the robot goes and stops for a second. This is the logic of the first round.
