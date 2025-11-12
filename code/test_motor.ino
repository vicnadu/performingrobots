#include <Servo.h>

Servo myServo;  // Create a servo object

void setup() {
  myServo.attach(20); // Attach servo to digital pin D20
  Serial.begin(9600);
  Serial.println("Servo test started");
}

void loop() {
  // Sweep from 0 to 180 degrees
  for (int angle = 0; angle <= 180; angle++) {
    myServo.write(angle);
    delay(15); // Adjust delay for servo speed (smaller = faster)
  }

  // Sweep back from 180 to 0 degrees
  for (int angle = 180; angle >= 0; angle--) {
    myServo.write(angle);
    delay(15);
  }
}
