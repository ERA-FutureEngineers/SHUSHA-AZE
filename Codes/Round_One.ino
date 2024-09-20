#include <Servo.h>


int frontdist = 0;
int leftdist = 0;
int rightdist = 0;
int Kp = 1.5, Kd = 0.4, errold = 0, err = 0, U = 0, target = 30, L = 0, R = 0, c = 0;

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

  delay(1500);

  
    analogWrite(6, 75);
    analogWrite(5, 80);
    digitalWrite(8, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(3, LOW);  

}

void loop()
{
    //Desicion
    if(L == 0 and R == 0)
  {
  leftdist = 0.01723 * readUltrasonicDistance(12, 13);
  rightdist = 0.01723 * readUltrasonicDistance(11, 10);

    if(rightdist > 100 or leftdist > 100)
    {
      if(leftdist > 100) L = 1;
    if(rightdist > 100) R = 1;
    }
    if(R == 1){
    servo_9.write(60);
    digitalWrite(7, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(8, LOW);
    digitalWrite(4, LOW);
    delay(2000);
    servo_9.write(100);
    delay(1000);
    servo_9.write(80);  
    digitalWrite(8, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(3, LOW);  
    }

    if(L == 1){
    servo_9.write(100);
    digitalWrite(7, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(8, LOW);
    digitalWrite(4, LOW);
    delay(1500);
    servo_9.write(60);
    delay(500);
    servo_9.write(80);  
    digitalWrite(8, HIGH);
    digitalWrite(4, HIGH);
    digitalWrite(7, LOW);
    digitalWrite(3, LOW);  
    }

    err = leftdist - rightdist;
    U = err * Kp + Kd * (err - errold);
    if(80+U > 107) servo_9.write(107);
    else if(80+U < 55) servo_9.write(55);
    else servo_9.write(80 + U);
    errold = err;
    delay(200);
  
  }
  
  // Left
  while(L == 1){
  leftdist = 0.01723 * readUltrasonicDistance(12, 13);
  rightdist = 0.01723 * readUltrasonicDistance(11, 10);

 
    err = leftdist - target;

    U = err * Kp + Kd * (err - errold);

    if(rightdist <= 10){
      servo_9.write(105);
      delay(500);
  }

  // if(frontdist <= 15){
  //   digitalWrite(7, HIGH);
  //   digitalWrite(3, HIGH);
  //   digitalWrite(8, LOW);
  //   digitalWrite(4, LOW);  
  //     servo_9.write(55);
  //     delay(1500);
  //   digitalWrite(8, HIGH);
  //   digitalWrite(4, HIGH);
  //   digitalWrite(7, LOW);
  //   digitalWrite(3, LOW);  
  //     servo_9.write(105);
  //     delay(500);
  // }


    if (80 + U > 107) {
      servo_9.write(107);
  } 
   else {
     servo_9.write(80 + U);
  }

errold = err;
delay(200); 

}

  //Right
  while(R == 1){
  leftdist = 0.01723 * readUltrasonicDistance(12, 13);
  rightdist = 0.01723 * readUltrasonicDistance(11, 10);

    err = target - rightdist;

    U = err * Kp + Kd * (err - errold);


  //     if(frontdist <= 15){
  //   digitalWrite(7, HIGH);
  //   digitalWrite(3, HIGH);
  //   digitalWrite(8, LOW);
  //   digitalWrite(4, LOW);  
  //     servo_9.write(105);
  //     delay(1500);
  //   digitalWrite(8, HIGH);
  //   digitalWrite(4, HIGH);
  //   digitalWrite(7, LOW);
  //   digitalWrite(3, LOW);  
  //     servo_9.write(55);
  //     delay(500);
  // }


    if (80 + U < 55) {
      servo_9.write(55);
  } else {
     servo_9.write(80 + U);
  }
  delay(200);
errold = err;

  }
}     
