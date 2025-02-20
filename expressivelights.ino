# include <Servo.h>                // servo library to make our boy MOVE

const int begin = 2;                     // PIN of first LED
const int end = 4;                       // PIN of last LED
const int buttonPin = 7;                 // PIN of button
const int servoPin = 6;                  // PIN of servo
const int echo = 13;                     // PIN of ultrasonic echo
const int trig = 12;                     // PIN of ultrasonic trig

unsigned long previousMillis = 0;  // keeping track of time since last "check"
unsigned long previousHungerMillis = 0;  // keeping track of time since last hunger "check"
unsigned long previousUSSMillis = 0;  // keeping track of time since last "check" for nearby objects
unsigned long previousLEDMillis = 0;  // keeping track of RGB time

const long interval = 1000;        // delay for blinking
const long LEDinterval = 10;        // delay for RGBLED
int advanceLightID = begin;        // what LED to start at for light
int lightState = 0;                // 0: advance forward ;; 1: advance back
int buttonState;                   // 0: not pressed ;; 1: pressed

/// -- CREATING SERVO --
Servo arduiBot;
int currentAngle = 180;               // Start servo at max happy (180)

/// -- ULTRASONIC SENSOR setup --
int uss_duration = 0;
int uss_distance = 0;

// -- LIL LIGHTY UP THING DESKBUDDY -- 
// the setup routine runs once when you press reset:
int sensorValue = 0;
bool isAwake = false; // to greet you ! 

const int redLEDPin = 9;     // LED connected to pin 9
const int greenLEDPin = 10;  // LED connected to pin 10
const int blueLEDPin = 11;    // LED connected to pin 11

void setup() {
  Serial.begin(9600);  // let it talk to computer

  /// -- Setting up LEDS --
  for (int i = 2; i <= 4; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH);
    digitalWrite(i, LOW);
  }

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);
  arduiBot.attach(servoPin); // connecting pin to servo object

  for (int i = advanceLightID; i <= end; i++){ // starts out w/ full happiness
    advanceLightID = advanceLight(i);          // or whatever you want the LED to mean (hunger?)
  }
}

void loop() {
  unsigned long currentMillis = millis();
  unsigned long currentHungerMillis = millis();
  unsigned long currentUSSMillis = millis();
  unsigned long currentLEDMillis = millis();

  /* reading button - how to make it keep track of state? otherwise discrepancy btwn pressed and being read */
  int buttonState = digitalRead(buttonPin);

  /* fiddling with ultrasonic sensor */
  digitalWrite(trig, HIGH); // turn this low after 1000ms

  // general timer
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    if (buttonState == LOW) { // button is pressed, turn lights on in succession
      Serial.println("button pressed!");
      advanceLightID = advanceLight(advanceLightID);
    }

    if (currentHungerMillis - previousHungerMillis >= (interval*5)) { // setting hunger to execute every 3000 ms
      previousHungerMillis = currentHungerMillis;
      // Serial.println("im hungy"); // check if timer is working
      advanceLightID = hunger(advanceLightID);
    }

    digitalWrite(trig, LOW);
    uss_duration = pulseIn(echo, HIGH); // ??
    uss_distance = (uss_duration/2) / 28.5;
  }

  if (uss_distance = 10 && !isAwake) { // when you get close enough
    Serial.println(uss_distance);
    isAwake = true;
    welcome();
    }
  
  else {
    analogWrite(redLEDPin, 0);
    analogWrite(greenLEDPin, 0);
    analogWrite(blueLEDPin, 0);
  }
}

int advanceLight(int currentAdvance) {          // "feed" function
  // -- time to advance forward --
    Serial.println(currentAdvance);
    digitalWrite(currentAdvance, HIGH);         // turn light on
    moveServo(60);
    if (currentAdvance >= end) {                // reach end?
      return currentAdvance;
      /* lightState = 1;                           // change state to BACKWARD
      return currentAdvance; */                    // return pointer to led
    }
    return currentAdvance+1;
}

int hunger(int currentAdvance) {
  /* turns off "food counter" by 1 in a set interval
     (soon to be randomized, and then also make a beeping noise)
  */
  Serial.println(currentAdvance);
  digitalWrite(currentAdvance, LOW); // set the current pin to LOW
  moveServo(-60);
  if (currentAdvance <= begin) {     // if we're at the end, let the program know to ADVANCE light
    lightState = 0;
    return currentAdvance;           // return same pin #
  }
  return currentAdvance-1;           // return the next (previous) pin #
}

void welcome() {
  partyEffect();
}

void partyEffect() {
  for (int i = 0; i < 20; i++) { // Flash 20 times
    setColor(random(128), random(128), random(128));
    delay(200);
    setColor(0, 0, 0); // Turn off momentarily
    delay(100);
  }
  setColor(0, 0, 0); // Turn off LED completely
}

void setColor(int red, int green, int blue) {
  analogWrite(redLEDPin, red);
  analogWrite(greenLEDPin, green);
  analogWrite(blueLEDPin, blue);
}

void moveServo(int angleChange){
  currentAngle += angleChange; // Update current position
  currentAngle = constrain(currentAngle, 0, 180); // Keep it within valid range
  arduiBot.write(currentAngle); // Move servo
}
  /* for (int pin = begin; pin <= end; pin++) {
    digitalWrite(pin, HIGH);
    delay(100);
  }
  for (int pin = begin; pin <= end; pin++) {
    digitalWrite(pin, LOW);
    delay(100);
  }

  for (int pin = end; pin >= begin; pin--) {
    digitalWrite(pin, HIGH);
    delay(100);
  }
  for (int pin = end; pin >= begin; pin--) {
    digitalWrite(pin, LOW);
    delay(100);
  }
  */

