#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>

int Kp = 1.0, Kd = 0.2, errold = 0, err = 0, U = 0, targetR = 43, targetL = 43, target = 45, L = 0, R = 0, count = 0, T = 0, k = 0;
int targetAngle = 90;  // Target turn angle in degrees
float currentAngle = 0;
float offsetZ = 0;  // Offset for calibration
float previousRotationZ = 0.0;


Servo servo_9;
MPU6050 gyro;

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


unsigned long previousMillis = 0;
unsigned long previousTime;
int integral = 2000;



void setup()
{
  Wire.begin();
  gyro.initialize();
  servo_9.attach(2);
  servo_9.write(90);
      Serial.begin(9600);  // Initialize serial communication at 9600 baud


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

  previousTime = millis();
}

void loop()
{

	if(L == 0 and R == 0)
	{
		float leftdist = 0.01723 * readUltrasonicDistance(A6, A7);
		delay(10);
		float frontdist = 0.01723 * readUltrasonicDistance(A2, A3);
		delay(10);
		float rightdist = 0.01723 * readUltrasonicDistance(A0, A1);
		delay(10);

		if(leftdist > 80) L = 1;
		else if(rightdist > 80) R = 1;

    Gyro();
	}



	while(L == 1) {

		float leftdist = 0.01723 * readUltrasonicDistance(A6, A7);
		delay(10);
		float rightdist = 0.01723 * readUltrasonicDistance(A0, A1);
		delay(10);
		float frontdist = 0.01723 * readUltrasonicDistance(A2, A3);
		delay(10);

		unsigned long currentMillis = millis();
		if (currentMillis - previousMillis >= 2000 and leftdist >= 200 and leftdist <= 300) {  // 1.5-second interval
			previousMillis = currentMillis;
			count++;
      T = T + 85;
		}
		else if(count == 12 and currentMillis - previousMillis >= 1700) {
			Stop();
		}
    Gyro();
    if(k == 0){
		err = targetR - rightdist;
		U = err * Kp + Kd * (err - errold);
		if (90 + U < 65) 
			servo_9.write(65);
		  else 
			servo_9.write(90 + U);


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
  }




	while(R == 1) {
		//Desicion
		float leftdist = 0.01723 * readUltrasonicDistance(A6, A7);
		delay(10);
		float rightdist = 0.01723 * readUltrasonicDistance(A0, A1);
		delay(10);
		float frontdist = 0.01723 * readUltrasonicDistance(A2, A3);
		delay(10);

		unsigned long currentMillis = millis();
		if (currentMillis - previousMillis >= 2000 and rightdist >= 200 and rightdist <= 300) {  
			previousMillis = currentMillis;
			count++;
		}
		else if(count == 12 and currentMillis - previousMillis >= 1700) {
			Stop();
		}

		err = targetR - rightdist;
		U = err * Kp + Kd * (err - errold);
		if (90 + U < 65) {
			servo_9.write(65);
		} else {
			servo_9.write(90 + U);
		}
		errold = err;

		if (frontdist < 5 and frontdist > 1) {
			servo_9.write(70);
			Backward();
			delay(600);
			servo_9.write(110);
			Forward();
			delay(600);
		}
	}
}

void Forward() {
	analogWrite(6, 80);
	analogWrite(5, 80);
	digitalWrite(14, HIGH);
	digitalWrite(16, HIGH);
	digitalWrite(15, LOW);
	digitalWrite(17, LOW);
}

void Backward() {
	analogWrite(6, 80);
	analogWrite(5, 80);
	digitalWrite(15, HIGH);
	digitalWrite(17, HIGH);
	digitalWrite(14, LOW);
	digitalWrite(16, LOW);
}

void Stop() {
	analogWrite(6, 0);
	analogWrite(5, 0);
}

void updateGyroAngle() {
  int16_t gz;
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;  // Convert to seconds
  previousTime = currentTime;  // Update previousTime after calculating deltaTime

  // Read and adjust for offset
  gz = gyro.getRotationZ();
  float rotationZ = (gz / 131.0) - offsetZ;  // Apply offset calibration

  if(rotationZ * deltaTime >= 0.25 or rotationZ * deltaTime <= -0.25){ 
  currentAngle += rotationZ * deltaTime;  
  }
  Serial.print("Current Angle: ");
  Serial.println(currentAngle);
}




void Gyro() {
  if (currentAngle - 20 > targetAngle) {
    servo_9.write(115);
    k = 1;
  } else if (currentAngle + 20 < targetAngle) {
    servo_9.write(65);
    k = 1;
  }
  else{
  servo_9.write(90);
  k = 0;
  }
updateGyroAngle();
}


