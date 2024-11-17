#include <Servo.h>

int Kp = 1.0, Kd = 0.2, errold = 0, err = 0, U = 0, targetR = 30, targetL = 30, target = 45, L = 0, R = 0, count = 0;
const int targetAngle = 90;
float currentAngle = 0;

float leftdist, rightdist, frontdist;

Servo servo_9;

long readUltrasonicDistance(int triggerPin, int echoPin)
{
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
int integral = 2000;

void setup()
{
  Serial.begin(9600);
  servo_9.attach(2);
  servo_9.write(90);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(16, OUTPUT);
  pinMode(15, OUTPUT);
  pinMode(17, OUTPUT);
  pinMode(14, OUTPUT);
  analogWrite(5, 80);
  analogWrite(6, 80);
  digitalWrite(14, HIGH);
  digitalWrite(16, HIGH);
  digitalWrite(15, LOW);
  digitalWrite(17, LOW);
}

void loop()
{

if(L == 0 and R == 0)
  {
  MeasureDistance();

    if(leftdist > 80) L = 1;
    else if(rightdist > 80) R = 1;
    if(R == 1){
    Stop();
    delay(1000);
    servo_9.write(70);
    Move();
    Backward();
    delay(800);
    servo_9.write(110);
    delay(800);
    servo_9.write(90);  
    Forward();
    }

    if(L == 1){
    Stop();
    delay(1000);
    servo_9.write(110);
    Move();
    Backward();
    delay(800);
    servo_9.write(70);
    delay(600);
    servo_9.write(90);  
    Forward();
    }

    err = leftdist - 43;
    U = err * Kp + Kd * (err - errold);
    if(90+U > 110) servo_9.write(110);
    else if(90+U < 70) servo_9.write(70);
    else servo_9.write(90 + U);
    errold = err;
    delay(50);
  }
  
                                                         

  while(L == 1){
      MeasureDistance();
      unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 2000 and leftdist >= 200 and leftdist <= 300) {
      previousMillis = currentMillis;
      count++;
    }
    else
    if(count == 12 and currentMillis - previousMillis >= 1500){
      Stop();
    }

    err = leftdist - targetL;
    U = err * Kp + Kd * (err - errold);
        if (90 + U > 120) {
      servo_9.write(120);
  } else {
     servo_9.write(90 + U);
  }
  errold = err;

    if (frontdist < 5 and frontdist > 1) {
        Backward();
        servo_9.write(115);
        delay(600);
        servo_9.write(65);
        Forward();
        delay(600);
      }
}


    while(R == 1){
      MeasureDistance();

      unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= 2000 and rightdist >= 200 and rightdist <= 300) {
      previousMillis = currentMillis;
      count++;
    }
    else
    if(count == 12 and currentMillis - previousMillis >= 1500){
      Stop();
    }

    err = targetR - rightdist;
    U = err * Kp + Kd * (err - errold);
        if (90 + U < 60) {
      servo_9.write(60);
  } else {
     servo_9.write(90 + U);
  }
  errold = err;

  if (frontdist < 5 and frontdist > 1) {
      servo_9.write(65);
      Backward();
      delay(600);
      servo_9.write(115);
      Forward();
      delay(600);
    }
  }
}

void Forward() {
  analogWrite(6, 90);
  analogWrite(5, 90);
  digitalWrite(14, HIGH);
  digitalWrite(16, HIGH);
  digitalWrite(15, LOW);
  digitalWrite(17, LOW);
}

void Backward() {
  analogWrite(6, 90);
  analogWrite(5, 90);
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
  analogWrite(6, 90);
  analogWrite(5, 90);
}

void MeasureDistance() {
  leftdist = 0.01723 * readUltrasonicDistance(A6, A7);
  delay(10);
  rightdist = 0.01723 * readUltrasonicDistance(A0, A1);
  delay(10);
  frontdist = 0.01723 * readUltrasonicDistance(A2, A3);
  delay(10);
  Serial.print(rightdist);
  Serial.println(" cm");
}
