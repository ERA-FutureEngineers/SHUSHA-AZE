# SHUSHA-AZE 2024
 ### movzular
* [Codes]
*  [First Raund]
*  [Second Raund]
* [Components](#components)
* Design
* [Robot Photos](#robotphotos)
* [Scheme](#scheme)
* TeamPhoto

### Components <a class="anchor" id="components"></a>
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

* Driver L298N

![driverl298n-222](https://github.com/user-attachments/assets/b9b312d4-a93b-4e89-94e5-a0e05b13fd90)

We used the l298n driver in the project. This driver works uniquely with dc motors and doesn't cause any problems for us.
* Servo

![servo222](https://github.com/user-attachments/assets/75496dab-44af-484e-960c-b52d35b29692)

We have done the freezing with servo means. We have configured the servo and gyro and ensured the robot works more accurately.

* Dc motors

![dc-motors222](https://github.com/user-attachments/assets/86ba6389-0e62-4d51-9de1-91c468af46a2)


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

biz ilk once arduino uno istifade edirdik .lakin robotu inkisaf etdirdikce sensorlar coxaldi ve sensorlar ucun pin yeri qalmadi ve bizde arduino megaya kecmek qerarina geldik . Arduino mega bir cox ozelliyine gore unodan cox qabaqdadir. 

* E18-D80NK Ir sensors <a class="anchor" id="irsensors"></a>

![ir222](https://github.com/user-attachments/assets/ee5489d9-d60a-482e-b7aa-b3b7e78ee29f)

bildiyiniz kimi ultrasonicler ses dalgasi ile mesafeni olcur. ses dalgasi sureti isiq suretinden az oldugu ucun bize bezi problemler cixarirdi. bizde ona gore istifade etdik . indi robotda hem ultrasonic sensor hemde E18-D80NK Ir sensorlardan istifade edirik. Bu sensorlar daha tez data verir ve gecikme yasamadan  isleyir.
  
### Robot Photos <a class="anchor" id="robotphotos"></a>

| <img src="./Robot-photos/Robot_Left.jpg" width="90%" /> | <img src="./Robot-photos/Robot_Right.jpg" width="85%" /> | 
| :--: | :--: | 
| *Left* | *Right* |
| <img src="./Robot-photos/Robot_Front.jpg" width="90%" /> | <img src="./Robot-photos/Robot_Back.jpg" width="85%" /> | 
| *Front* | *Back* |
| <img src="./Robot-photos/Robot_Up.jpg" width="90%" /> | <img src="./Robot-photos/Robot_Downjpg.jpg" width="85%" /> | 
| *Up* | *Down* |

### Scheme <a class="anchor" id="scheme"></a>
