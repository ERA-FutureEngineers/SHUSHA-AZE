#include <Servo.h>

int frontdist = 0;
int leftdist = 0;
int rightdist = 0;
int Kp = 2, Kd = 0.4, Ki = 0.005, integral = 0, errold = 0, err = 0, U = 0, target = 25, L = 0, R = 0;

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

  
    analogWrite(6, 75);
    analogWrite(5, 75);
    digitalWrite(7, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(8, LOW);
    digitalWrite(3, LOW);  
}

void loop()
{
    /// Qərarvermə
    if(L == 0 and R == 0)
  {
  frontdist = 0.01723 * readUltrasonicDistance(11, 10);
  leftdist = 0.01723 * readUltrasonicDistance(13, 12);
  rightdist = 0.01723 * readUltrasonicDistance(A0, A1);
    if(rightdist > 100 or leftdist > 100)
    {
      if(leftdist < 100) R = 1;
    if(rightdist < 100) L = 1;
    }
    err = leftdist - rightdist;
    U = err * Kp + Kd * (err - errold);
    if(90+U > 120) servo_9.write(120);
    else if(90+U < 60) servo_9.write(60);
    else servo_9.write(90 + U);
    errold = err;
    delay(100);
  }

  //// Sol
  while(L == 1){
    frontdist = 0.01723 * readUltrasonicDistance(11, 10);
    leftdist = 0.01723 * readUltrasonicDistance(13, 12);
    rightdist = 0.01723 * readUltrasonicDistance(A0, A1);

    err = leftdist - target;

    integral += err;
    U = err * Kp + Kd * (err - errold) + Ki * integral;

    if (90 + U > 120) {
      servo_9.write(120);
  } 
   else {
     servo_9.write(90 + U);
  }

errold = err;
  
    }

  //// Sağ
  while(R == 1){
    frontdist = 0.01723 * readUltrasonicDistance(11, 10);
    rightdist = 0.01723 * readUltrasonicDistance(A0, A1);
    leftdist = 0.01723 * readUltrasonicDistance(13, 12);

    err = target - rightdist;

    integral += err;
    U = err * Kp + Kd * (err - errold) + Ki * integral;

    if (90 + U < 60) {
      servo_9.write(60);
  } else {
     servo_9.write(90 + U);
  }

errold = err;
    }
} 
