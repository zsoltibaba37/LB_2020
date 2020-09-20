/* MCP2308 Input and output test

   v0.56  - Analog turn off sense
   v0.55  - Tested with Mega and mcp1, button work only sensState = 0, analogRead A0
   v0.54  - Program Structure
   v0.53  - Measure Mainbord power consumption in Vin pin = 160mA
   v0.52  - 3 Relay And rotary encoder implemented. relay timing setup with millis
   v0.51  - Arduino Mega test,
   v0.5   - Connect Central unit to mcp1 and TFT test
   v0.4   - Two MCP test, 4 + 4 meter cable, Working
   v0.3   - 2m cable, Photo diode test, Self-Holding Sens tested
   v0.2   - 1m Cable without PullUp Resistor, 2m Cable Tested
   v0.1   - Laser Working, Pushbutton working, RGB Led working -> Red: Light barrier=ON Green: Light barrier=OFF
*/

float version = 0.56;

// Analog Smooth
#include "AnalogPin.h"

// -------------------- I2C ----------
#include <Wire.h>
#include <Adafruit_MCP23008.h>

Adafruit_MCP23008 mcp0;         // Create mpc instance
Adafruit_MCP23008 mcp1;         // Create mpc instance

//char *mcpNames[] = {"mcp0", "mcp1", "mcp2"};
//unsigned int mcpNumber = 2;


// -------------------- SPI ----------
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <gfxfont.h>
#include <TFT_ILI9163C.h>


// All wiring required, only 3 defines for hardware SPI on 328P
//                              Mega      Nano
#define __DC 49                 // 49     // 9
#define __CS 53                 // 53     // 10
// MOSI --> (SDA) --> D11       // 51     // 11
#define __RST 48                // 48     // 12
// SCLK --> (SCK) --> D13       // 52     // 13


// Color definitions
#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF

TFT_ILI9163C tft = TFT_ILI9163C(__CS, __DC, __RST);


// ---------- Mega ----------
#define buttonN 2               // PushButton
#define laserN 3               // Laser Diode On
#define redLedNO 4               // Red Led
#define greenLedNO 5             // Green Led
#define redLedNI 6                // Red Led IN
#define greenLedNI 7              // Green Led IN
#define photoDiodeN 8           // Photo Diode
unsigned int laserSensN;       // Photo Diode Value 1 or 0

// Rotary encoder pin numbers PmodENC
const int pinA = 9;  // Connected to A
const int pinB = 10;  // Connected to B
const int pinSW = 11; // Connected to SW
int pinALast;
boolean pinSWLast;
int aVal;
int swVal;
// ---------- Sate pinSW ----------
int stateSW = LOW;           // the current state of the output pin
int readingSW;               // the current reading from the input pin
int previousSW = HIGH;       // the previous reading from the input pin
long timeSW = 0;             // the last time the output pin was toggled

// Error State 0 = No Error  1 = Error
unsigned int error = 1;

// Sens State  0 = Sens Off  1 = Sense
unsigned int sens = 0;
unsigned int sensState = 0;

// Pause Robot -- Relay Control
#define pauseRelay 22                 // In 1sec 200mS On - Pause Signal
#define contRelay 24                  // Allways On except on error - Continue
#define gripperRelay 26                // Allways On except on error - Gripper
unsigned long previousMillis = 0;     // will store last time LED was updated
long OnTime = 200;                    // milliseconds of on-time
long OffTime = 800;                   // milliseconds of off-time
int relayState = LOW;                 // The Pause relay State

// Analog In
//#define anaRobot A0                 // Reads the analog input every 250 milliseconds
AnalogPin INA(A0);
unsigned long prevMillis = 0;     // will store last time LED was updated
long measureTime = 5;           // milliseconds of on-time 10mS
unsigned int valueRobot;          // Raw analog
unsigned int sensOff1 = 0;         // 1 gate turn off
unsigned int sensOff2 = 0;         // 2 gate turn off
unsigned int sensOff3 = 0;         // 3 gate turn off
unsigned int sensOff4 = 0;         // 4 gate turn off

// ---------- mcp0 ----------
#define button 0                // PushButton
#define laser 1                 // Laser Diode On
#define redLedO 2                // Red Led OUT
#define greenLedO 3              // Green Led OUT
#define photoDiode 5            // Photo Diode
#define redLedI 6                // Red Led IN
#define greenLedI 7              // Green Led IN
unsigned int laserSens;        // Photo Diode Value 1 or 0

// ---------- mcp1 ----------

unsigned int laserSens1;        // Photo Diode Value 1 or 0

// ---------- Button Mega ----------
int stateN = LOW;           // the current state of the output pin
int readingN;               // the current reading from the input pin
int previousN = HIGH;       // the previous reading from the input pin
long timeN = 0;             // the last time the output pin was toggled
long debounce = 200;       // old 200


// ---------- Button mcp0 ----------
int state = LOW;           // the current state of the output pin
int reading;               // the current reading from the input pin
int previous = HIGH;       // the previous reading from the input pin
long time = 0;             // the last time the output pin was toggled
//long debounce = 200;       // old 200

// ---------- Button mcp1 ----------
int state1 = LOW;           // the current state of the output pin
int reading1;               // the current reading from the input pin
int previous1 = HIGH;       // the previous reading from the input pin
long time1 = 0;             // the last time the output pin was toggled
//long debounce1 = 200;       // old 200


// ---------- SETUP ----------
void setup() {
  // TFT
  tft.begin();
  tft.fillScreen();
  tft.setTextColor(WHITE);
  tft.setTextSize(1);
  tft.setCursor(28, 50);
  tft.println("Laser Barrier");
  tft.setCursor(33, 62);
  tft.println("Test v" + String(version));

  //  Serial.begin(115200);
  Serial.begin(115200);

  // ---------------- Mega ----------------
  pinMode(buttonN, INPUT_PULLUP);
  pinMode(laserN, OUTPUT);
  pinMode(redLedNO, OUTPUT);
  pinMode(greenLedNO, OUTPUT);
  pinMode(redLedNI, OUTPUT);
  pinMode(greenLedNI, OUTPUT);
  pinMode(photoDiodeN, INPUT);

  // Rotary encoder
  pinMode (pinA, INPUT);
  pinMode (pinB, INPUT);
  pinMode (pinSW, INPUT); // PUSH BUTTON
  pinSWLast = true;
  pinALast = digitalRead(pinA);

  pinMode(pauseRelay, OUTPUT);    // 1mp 200mS
  pinMode(contRelay, OUTPUT);     // Allways On except on error
  //digitalWrite(contRelay, HIGH);
  pinMode(gripperRelay, OUTPUT);   // Allways On except on error
  //digitalWrite(gripperRelay, HIGH);

  //  pinMode(anaRobot, INPUT);       //

  // ------------------------------------------------------------

  // mcp0
  mcp0.begin(0);           // 0 = 0x20
  mcp0.pinMode(button, INPUT);
  mcp0.pullUp(button, HIGH);
  mcp0.pinMode(laser, OUTPUT);
  mcp0.pinMode(redLedO, OUTPUT);
  mcp0.pinMode(greenLedO, OUTPUT);
  mcp0.pinMode(photoDiode, INPUT);
  mcp0.pinMode(redLedI, OUTPUT);
  mcp0.pinMode(greenLedI, OUTPUT);

  // mcp1
  mcp1.begin(1);           // 1 = 0x21
  mcp1.pinMode(button, INPUT);
  mcp1.pullUp(button, HIGH);
  mcp1.pinMode(laser, OUTPUT);
  mcp1.pinMode(redLedO, OUTPUT);
  mcp1.pinMode(greenLedO, OUTPUT);
  mcp1.pinMode(photoDiode, INPUT);
  mcp1.pinMode(redLedI, OUTPUT);
  mcp1.pinMode(greenLedI, OUTPUT);

  // Print Program Version
  Serial.println("Laser Barrier");
  Serial.println("Test v" + String(version));

}
// ------------------------------------------------------------------------------------------------
// ------------------------------ LOOP ------------------------------------------------------------
void loop() {
  // Error State 0 = No Error  1 = Error
  // Sens State  0 = Sens Off  1 = Sense On

  if (error == 0 && sensState == 0) {
    sensState = sensSW();
    changeOutMega(buttonN, laserN);
    // changeOutMcp0(button, laser);
    //changeOutMcp1(button, laser);
  }
  else if (error == 0 && sensState == 1) {
    offSens();

    // If 1 turn off sens
    if (sensOff1 != 1)
    {
      sensMega();
    }
    //    if (sensOff2 != 1)
    //    {
    //      sensMcp0();
    //    }
    //    if (sensOff2 != 1)
    //    {
    //      sensMcp1();
    //    }


    digitalWrite(pauseRelay, LOW);
    digitalWrite(contRelay, HIGH);
    digitalWrite(gripperRelay, HIGH);
  }

  if (error == 1) {
    while (sensState == 0) {
      pauseRobot();
      sensState = sensSW();
      changeOutMega(buttonN, laserN);
      //    changeOutMcp0(button, laser);
      //changeOutMcp1(button, laser);
    }
  }

}
// ------------------------------------------------------------------------------------------------
// ------------------------------------------------------------------------------------------------

// ------------------- VOID Button mcp0 -------------------
void changeOutMcp0(int inPort, int outPort) {
  reading = !mcp0.digitalRead(inPort);

  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    if (state == LOW)
      state = HIGH;
    else
      state = LOW;
    time = millis();
  }
  mcp0.digitalWrite(outPort, state);
  previous = reading;

  if (state == HIGH) {
    mcp0.digitalWrite(greenLedO, LOW);
    mcp0.digitalWrite(redLedO, HIGH);
    mcp1.digitalWrite(greenLedI, LOW);
    mcp1.digitalWrite(redLedI, HIGH);
  }
  if (state == LOW) {
    mcp0.digitalWrite(redLedO, LOW);
    mcp0.digitalWrite(greenLedO, HIGH);
    mcp1.digitalWrite(redLedI, LOW);
    mcp1.digitalWrite(greenLedI, HIGH);

  }

  // Sens of
  //  laserSens = mcp0.digitalRead(photoDiode);
  //  if (laserSens == 1 ) {
  //    mcp0.digitalWrite(laser, LOW);
  //    reading = LOW;
  //    previous = LOW;
  //    state = LOW;
  //    errorState = 1;
  //    error = 1;
  //  }
}

// ------------------- VOID Button mcp1 -------------------
void changeOutMcp1(int inPort, int outPort) {
  reading1 = !mcp1.digitalRead(inPort);

  if (reading1 == HIGH && previous1 == LOW && millis() - time1 > debounce) {
    if (state1 == LOW)
      state1 = HIGH;
    else
      state1 = LOW;
    time1 = millis();
  }
  mcp1.digitalWrite(outPort, state1);
  previous1 = reading1;

  if (state1 == HIGH) {
    mcp1.digitalWrite(greenLedO, LOW);
    mcp1.digitalWrite(redLedO, HIGH);
    digitalWrite(greenLedNI, LOW);
    digitalWrite(redLedNI, HIGH);
  }
  if (state1 == LOW) {
    mcp1.digitalWrite(redLedO, LOW);
    mcp1.digitalWrite(greenLedO, HIGH);
    digitalWrite(redLedNI, LOW);
    digitalWrite(greenLedNI, HIGH);
  }

  // Sens in
  //  laserSens1 = mcp1.digitalRead(photoDiode);
  //  if (laserSens1 == 1 ) {
  //    mcp1.digitalWrite(laser, LOW);
  //    reading1 = LOW;
  //    previous1 = LOW;
  //    state1 = LOW;
  //    errorState = 1;
  //    error = 1;
  //  }
}

// ------------------- VOID Button Mega -------------------
void changeOutMega(int inPort, int outPort) {
  readingN = !digitalRead(inPort);
  if (readingN == HIGH && previousN == LOW && millis() - timeN > debounce) {
    if (stateN == LOW)
      stateN = HIGH;
    else
      stateN = LOW;
    timeN = millis();
  }
  digitalWrite(outPort, stateN);
  previousN = readingN;
  error = 0;

  if (stateN == HIGH) {
    digitalWrite(greenLedNO, LOW);
    digitalWrite(redLedNO, HIGH);
    mcp0.digitalWrite(greenLedI, LOW);
    mcp0.digitalWrite(redLedI, HIGH);
  }
  if (stateN == LOW) {
    digitalWrite(redLedNO, LOW);
    digitalWrite(greenLedNO, HIGH);
    mcp0.digitalWrite(redLedI, LOW);
    mcp0.digitalWrite(greenLedI, HIGH);
  }


  //  laserSensN = digitalRead(photoDiodeN);
  //  if (laserSensN == 1 ) {
  //    digitalWrite(laserN, LOW);
  //    readingN = LOW;
  //    previousN = LOW;
  //    stateN = LOW;
  //    errorState = 1;
  //    error = 1;
  //  }
}

// -------------------- Sens Mega --------------------
void sensMega() {
  laserSensN = digitalRead(photoDiodeN);
  if (laserSensN == 1 ) {
    digitalWrite(laserN, LOW);
    readingN = LOW;
    previousN = LOW;
    stateN = LOW;
    error = 1;
    sensState = 0;
    sens = 0;
    tft.fillScreen();
    tft.setCursor(10, 50);
    tft.println("First line crossed");
  }
}

// -------------------- Sens Mcp0 --------------------
void sensMcp0() {
  laserSens = mcp0.digitalRead(photoDiode);
  if (laserSens == 1 ) {
    mcp0.digitalWrite(laser, LOW);
    reading = LOW;
    previous = LOW;
    state = LOW;
    error = 1;
    sensState = 0;
    sens = 0;
  }
}

// -------------------- Sens Mcp1 --------------------
void sensMcp1() {
  laserSens1 = mcp1.digitalRead(photoDiode);
  if (laserSens1 == 1 ) {
    mcp1.digitalWrite(laser, LOW);
    reading1 = LOW;
    previous1 = LOW;
    state1 = LOW;
    error = 1;
    sensState = 0;
    sens = 0;
    tft.fillScreen();
    tft.setCursor(8, 50);
    tft.println("Second line crossed");
  }
}

// -------------------- Rotary ENC Switch --------------------
int sensSW() {

  swVal = !digitalRead(pinSW);
  if (pinSWLast == true ) {
    if (swVal == LOW) {
      ++sens;
      if (sens >= 1)
        sens = 1;
      pinSWLast = false;
    }
  }
  if (swVal == HIGH) {
    pinSWLast = true;
  }
  return sens;

}

// -------------------- Pause Relay --------------------
void pauseRobot() {
  digitalWrite(contRelay, LOW);
  digitalWrite(gripperRelay, LOW);

  unsigned long currentMillis = millis();
  if ((relayState == HIGH) && (currentMillis - previousMillis >= OnTime))
  {
    relayState = LOW;                       // Turn it off
    previousMillis = currentMillis;       // Remember the time
    digitalWrite(pauseRelay, relayState);   // Update the actual Relay
  }
  else if ((relayState == LOW) && (currentMillis - previousMillis >= OffTime))
  {
    relayState = HIGH;                      // turn it on
    previousMillis = currentMillis;       // Remember the time
    digitalWrite(pauseRelay, relayState);   // Update the actual Relay
  }
}

// -------------------- Analog in From Robot --------------------
int offSens() {
  unsigned long currentMillis = millis();
  if ((currentMillis - prevMillis >= measureTime))
  {
    prevMillis = currentMillis; // Remember the time
    //INA.setPrescaler(5);
    INA.setNoiseThreshold(60);
    valueRobot = INA.read();
    //valueRobot = analogRead(anaRobot);   // Update the actual Relay
    //Serial.println(valueRobot);

    // 102.3
    if ( valueRobot >= 73 && valueRobot <= 143 )
    {
      sensOff1 = 1;
      digitalWrite(greenLedNO, HIGH);
      //Serial.println("sensOff1 State: " + String(sensOff1));
    }

    // 205
    else if ( valueRobot >= 185 && valueRobot <= 225 )
    {
      sensOff2 = 1;
      //Serial.println("sensOff2 State: " + String(sensOff2));
    }

    // 307
    else if ( valueRobot >= 287 && valueRobot <= 327 )
    {
      sensOff3 = 1;
      //Serial.println("sensOff3 State: " + String(sensOff3));
    }

    // 409
    else if ( valueRobot >= 389 && valueRobot <= 429 )
    {
      sensOff4 = 1;
      //Serial.println("sensOff4 State: " + String(sensOff4));
    }

    else
    {
      sensOff1 = 0;
      sensOff2 = 0;
      sensOff3 = 0;
      sensOff4 = 0;
      digitalWrite(greenLedNO, LOW);
      //Serial.println("All State: 0");
    }
  }

}


// -------------------- Rotary ENC A and B ---------------------
//int rotaryENC() {
//  aVal = digitalRead(pinA);
//  if (aVal != pinALast) { // Means the knob is rotating
//    // if the knob is rotating, we need to determine direction
//    if (digitalRead(pinB) != aVal) {       // Means pin A Changed first
//      // dt = dt + 1;
//      ++dt;
//      dt = min(dt, 60);
//      DispalyDelayTime();
//
//    } else {                               // Otherwise B changed first
//      // dt = dt - 1;
//      --dt;
//      dt = max(1, dt);
//      DispalyDelayTime();
//    }
//  }
//  pinALast = aVal;
//  errorState = sensSW();
//}
