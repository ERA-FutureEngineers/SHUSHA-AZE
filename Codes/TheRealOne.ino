#include <Servo.h>
#include <Pixy2.h>

Servo servo_9;
Pixy2 pixy;

int g = 0, r = 0;  // Flags to track whether it turned left (green) or right (red)

bool colorDetected = false;
unsigned long lastColorDetectedTime = 0;
const unsigned long detectionTimeout = 500; // Time in milliseconds to confirm object is out of view



long readUltrasonicDistance(int triggerPin, int echoPin) {
  pinMode(triggerPin, OUTPUT);  // Clear the trigger
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  pinMode(echoPin, INPUT);
  return pulseIn(echoPin, HIGH);
}

void setup() {
  servo_9.attach(9);          // Attach servo to pin 9
  Serial.begin(9600);         // Initialize serial port for debugging
  pixy.init();                // Initialize Pixy2

  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(3, OUTPUT);

  analogWrite(6, 75);         // Turn on motors to move straight
  analogWrite(5, 80);
  digitalWrite(8, HIGH);
  digitalWrite(4, HIGH);

  servo_9.write(80);         // Set initial servo position to 80 degrees
}

void loop() {


    float frontdist = 0.01723 * readUltrasonicDistance(A1, A0);
  float leftdist = 0.01723 * readUltrasonicDistance(12, 13);
  float rightdist = 0.01723 * readUltrasonicDistance(11, 10);

  
  pixy.ccc.getBlocks();       // Get data from Pixy2

  bool greenDetected = false;
  bool redDetected = false;
  int colorX = 0;

  // Check for blocks and track the color position
  if(pixy.ccc.numBlocks > 0){
  for (int i = 0; i < pixy.ccc.numBlocks; i++) {
    if (pixy.ccc.blocks[i].m_signature == 1) {  // Green color
      greenDetected = true;
      colorX = pixy.ccc.blocks[i].m_x;
    }
    if (pixy.ccc.blocks[i].m_signature == 2) {  // Red color
      redDetected = true;
      colorX = pixy.ccc.blocks[i].m_x;
    }
  }

  frontdist = 0.01723 * readUltrasonicDistance(A1, A0);  // Convert to cm

  // Move towards the detected color using servo to steer
  if (greenDetected || redDetected) {
    // Adjust servo to turn towards the color based on its position in the frame
    if (colorX < 140) {
      // Color is on the left side of the camera frame
      servo_9.write(100);  // Turn left (increase servo angle)
    } else if (colorX > 180) {
      // Color is on the right side of the camera frame
      servo_9.write(60);   // Turn right (decrease servo angle)
    } else {
      // Color is centered
      servo_9.write(80);   // Keep servo straight
    }

    // Check if the front distance is less than 20 cm
    if (frontdist < 20) {

      // Perform obstacle avoidance using the servo
      if (greenDetected) {

        digitalWrite(8, LOW);
        digitalWrite(4, LOW);
        delay(1000);
        digitalWrite(7, HIGH);
        digitalWrite(3, HIGH);
        
        servo_9.write(55); 
        delay(2000);

        digitalWrite(8, HIGH);
        digitalWrite(4, HIGH);
        digitalWrite(7, LOW);
        digitalWrite(3, LOW);

        servo_9.write(105);  // Turn left (green)
        delay(1000);         // Hold the turn for 1 second
        servo_9.write(55);   // Reverse the turn
        delay(1000);         // Hold the reverse turn for 1 second
      } else if (redDetected) {
        digitalWrite(8, LOW);
        digitalWrite(4, LOW);
        delay(1000);
        digitalWrite(7, HIGH);
        digitalWrite(3, HIGH);
        servo_9.write(105); 
        delay(2000);

        digitalWrite(8, HIGH);
        digitalWrite(4, HIGH);
        digitalWrite(7, LOW);
        digitalWrite(3, LOW);

        servo_9.write(55);  // Turn left (green)
        delay(1000);         // Hold the turn for 1 second
        servo_9.write(105);   // Reverse the turn
        delay(1000);         // Hold the reverse turn for 1 second
      }

      // After avoiding, reset the servo to default position
      servo_9.write(80);
    }



    } 
  }
  else{
      if(rightdist <= 30){
    servo_9.write(105);
    delay(1000);
    servo_9.write(55);
    delay(500);
  }

  if(leftdist <= 30){
      servo_9.write(55);
      delay(1000);
      servo_9.write(105);
      delay(500);
      }

      servo_9.write(80);
      delay(500);
  }
  // Short delay before the next loop iteration
  delay(100);  
}