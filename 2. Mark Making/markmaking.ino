#include <Servo.h>

Servo myServo;  // Create a servo object


const int buttonPin = 2;
int buttonState = 0;
int BUTTON_PRESSED = 0;

bool STOP_FLAG = false;

int xValue = 0;        // of joystick
int yValue = 0;        // of joystick
int xValue_mapped = 0; // converting joystick motion to servo motion
int yValue_mapped = 0; // converting joystick motion to servo motion

// Servo range
const int minAngle = 0;   // Min servo angle
const int maxAngle = 180; // Max servo angle

void setup() {
    pinMode(buttonPin, INPUT_PULLUP);
    Serial.begin(9600);
    
    myServo.attach(7);

}

void loop() {
  if (STOP_FLAG) return;

  xValue = analogRead(A1); // values range from 0 - 1023
  yValue = analogRead(A0); // values range from 0 - 1023

  xValue_mapped = map(xValue, 0, 1023, 0, 180);
  yValue_mapped = map(yValue, 0, 1023, 0, 180);

  myServo.write(yValue_mapped);


  // Print joystick values to the Serial Monitor for debugging
  Serial.print("Joystick Y: ");
  Serial.print(yValue);
  Serial.print(" -> Servo Angle: ");
  Serial.println(yValue_mapped);

  delay(15);  // Wait for the servo to reach the position

  buttonState = digitalRead(buttonPin);  // Read the button state
  if (buttonState == LOW) {
    Serial.println("STOPPING");
    STOP_FLAG = true;
  }



  
}