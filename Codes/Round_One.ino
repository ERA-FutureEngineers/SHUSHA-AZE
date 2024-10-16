#include <Servo.h>

int Kp = 1.5, Kd = 0.4, errold = 0, err = 0, U = 0, targetR = 25, targetL = 25, target = 45, L = 0, R = 0;

Servo servo_9;

long readUltrasonicDistance(int triggerPin, int echoPin)
{
  pinMode(triggerPin, OUTPUT);  // Clear the trigger
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  // Sets the trigger pin to HIGH state for 10 microseconds
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  // Reads the echo pin, and returns the sound wave travel time in microseconds
  return pulseIn(echoPin, HIGH);
}

#define S0 2
#define S1 10
#define S2 12
#define S3 11
#define sensorOut 13

int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;
int k = 0, r = 0, oldr = 0;
int previousMillis = 0;
int interval = 4000;


void setup()
{
  servo_9.attach(9, 500, 2500);
  Serial.begin(9600);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(3, OUTPUT);

  delay(1500);

  
    analogWrite(6, 84);
    analogWrite(5, 77);
    digitalWrite(7, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(8, LOW);
    digitalWrite(3, LOW);  
}

void loop()
{

if(L == 0 and R == 0)
  {
  float leftdist = 0.01723 * readUltrasonicDistance(A4, A5);
  delay(10);
  float rightdist = 0.01723 * readUltrasonicDistance(A1, A0);
  delay(10);
    if(rightdist > 100 or leftdist > 100)
    {
      if(leftdist > 100) L = 1;
    if(rightdist > 100) R = 1;
    }
    if(R == 1){
    Stop();
    delay(1000);
    servo_9.write(70);
    Move();
    Backward();
    delay(1500);
    servo_9.write(110);
    delay(1000);
    servo_9.write(90);  
    Forward();
    }

    if(L == 1){
    Stop();
    delay(1000);
    servo_9.write(110);
    Move();
    Backward();
    delay(1000);
    servo_9.write(70);
    delay(1000);
    servo_9.write(90);  
    Forward();
    }

    err = leftdist - target;
    U = err * Kp + Kd * (err - errold);
    if(90+U > 110) servo_9.write(110);
    else if(90+U < 70) servo_9.write(70);
    else servo_9.write(90 + U);
    errold = err;
    delay(100);
  }
  
                                                         

    while(L == 1){

  float leftdist = 0.01723 * readUltrasonicDistance(A4, A5);
  delay(10);
  float rightdist = 0.01723 * readUltrasonicDistance(A1, A0);
  delay(10);

    err = leftdist - targetL;

    U = err * Kp + Kd * (err - errold);


        if (90 + U > 115) {
      servo_9.write(115);
  } else {
     servo_9.write(90 + U);
  }
  errold = err;
  delay(100);
}




    while(R == 1){
    //Desicion
  float leftdist = 0.01723 * readUltrasonicDistance(A4, A5);
  float rightdist = 0.01723 * readUltrasonicDistance(A1, A0);

    err = targetR - rightdist;

    U = err * Kp + Kd * (err - errold);


        if (90 + U < 65) {
      servo_9.write(65);
  } else {
     servo_9.write(90 + U);
  }
  errold = err;
  delay(100);

    if(leftdist <= 5){
    servo_9.write(70);
    delay(200);
      }
    }
  }

void Forward() {
  digitalWrite(7, HIGH);
  digitalWrite(4, HIGH);
  digitalWrite(8, LOW);
  digitalWrite(3, LOW);
}

void Backward() {
  digitalWrite(8, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(7, LOW);
  digitalWrite(4, LOW);
}

void Stop() {
  analogWrite(6, 0);
  analogWrite(5, 0);
}

void Move() {
  analogWrite(6, 84);
  analogWrite(5, 77);
}
  
