/* Laser Barrier - For Cobot - AutoKobot Ltd.

   v0.66  - Output to robot tested, working properly.
   v0.65  - Reed relay included, sensOff1 - sensOff4 working all line
   v0.64  - Off Relay Delay - setup integrated. All menu item translated.
   v0.63  - Language manu integrated. Language select is working. A few menu item translated
   v0.62  - Setup menu integrated. Exit working
   v0.61  - Main menu integrated. Activate
   v.060  - Hardware ready and tested.
   v.059  - Check Mega and mcp0-1-2  PCBs
   v0.58  - Display Design
   v0.57  - Analog Turn Off First and Third Line TEST
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

float version = 0.66;

// Analog Smooth
#include "AnalogPin.h"

// -------------------- I2C ----------
#include <Wire.h>
#include <Adafruit_MCP23008.h>

Adafruit_MCP23008 mcp0;         // Create mpc instance
Adafruit_MCP23008 mcp1;         // Create mpc instance
Adafruit_MCP23008 mcp2;         // Create mpc instance

//char *mcpNames[] = {"mcp0", "mcp1", "mcp2"};

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

// -------- Language --------
#include <EEPROM.h>
unsigned int langAddr = 0;
int langValue;    // 1 - 2 - 3
int newLangValue; // 1 - 2 - 3
String lang;

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
int bVal;
int swVal;

const int reedRelay = 12; // reed relay input
int reedState;

// ---------- Menu ----------
unsigned int menuitem = 1;
//int frame = 1;
unsigned int page = 1;
int lastMenuItem = 1;

// ----- Page 1 -----
String menuItem1Eng = "Activate";
String menuItem2Eng = "Setup";

String menuItem1Hun = "Aktival";
String menuItem2Hun = "Beallit";

String menuItem1Deu = "Aktivie.";
String menuItem2Deu = "Einstel.";

// ----- Page 2 -----
String menuItem3Eng = "Lang";
String menuItem4Eng = "R.Delay";
String menuItem5Eng = " Exit <";

String menuItem3Hun = "Nyelv";
String menuItem4Hun = "R.Kesl.";
String menuItem5Hun = "Kilep <";

String menuItem3Deu = "Sprache";
String menuItem4Deu = "R.Verzo.";
String menuItem5Deu = " Ruck <";

// ----- Page 3 -----

String menuItem6 = " ENG";
String menuItem7 = " HUN";
String menuItem8 = " DEU";

// ---------- Sate pinSW ----------
int stateSW = LOW;           // the current state of the output pin
int readingSW;               // the current reading from the input pin
int previousSW = HIGH;       // the previous reading from the input pin
long timeSW = 0;             // the last time the output pin was toggled

// Error State 0 = No Error  1 = Error
unsigned int error = 0;

// Sens State  0 = Sens Off  1 = Sense
unsigned int sens = 0;
unsigned int sensState = 0;

// Pause Robot -- Relay Control
#define pauseRelay 22                 // In 1sec 200mS On - Pause Signal
#define contRelay 24                  // Allways On except on error - Continue
#define gripperRelay 26               // Allways On except on error - Gripper
unsigned long previousMillis = 0;     // will store last time LED was updated
long OnTime = 200;                    // milliseconds of on-time
long OffTimeValue;
long NewOffTimeValue;
long OffTime;                         // milliseconds of off-time
long BackOnTime = 200;                // Laser barrier On, after robot turn off one of the line
unsigned int timeAddr = 1;            // OffTime Memory Address
//EEPROM.write(timeAddr, 8);          // Write once

int relayState = LOW;                 // The Pause relay State

// Analog In
//#define anaRobot A0                 // Reads the analog input every 250 milliseconds
AnalogPin INA(A0);                    // Analog input
unsigned long prevMillis = 0;         // will store last time
long measureTime = 50;                // milliseconds measure time
unsigned int valueRobot;              // Raw analog
unsigned int sensOff1 = 0;            // 1 gate turn off
unsigned int sensOff2 = 0;            // 2 gate turn off
unsigned int sensOff3 = 0;            // 3 gate turn off
unsigned int sensOff4 = 0;            // 4 gate turn off

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

// ---------- mcp2 ----------

unsigned int laserSens2;        // Photo Diode Value 1 or 0

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

// ---------- Button mcp2 ----------
int state2 = LOW;           // the current state of the output pin
int reading2;               // the current reading from the input pin
int previous2 = HIGH;       // the previous reading from the input pin
long time2 = 0;             // the last time the output pin was toggled
//long debounce1 = 200;       // old 200

// -------------------------------- SETUP --------------------------------
// -------------------------------- SETUP --------------------------------
void setup() {
  // -------- EEPROM Write once lang -------
  //EEPROM.write(langAddr, 1);        // Write only once
  //   1 - English    - ENG
  //   2 - Hungarian  - HUN
  //   3 - Deutsche   - DEU
  langValue = EEPROM.read(langAddr);
  if (langValue == 1)
  {
    lang = "ENG";
  }
  else if (langValue == 2)
  {
    lang = "HUN";
  }
  else if (langValue == 3)
  {
    lang = "DEU";
  }

  // ---- Relay OffTime
  // EEPROM.write(timeAddr, OffTimeValue);         // Write once
  OffTimeValue = EEPROM.read(timeAddr);
  OffTime = OffTimeValue * 100;


  // TFT
  tft.begin();
  tft.setRotation(3);
  tft.clearScreen();
  tft.drawRect(3, 3, 125, 125, WHITE);
  tft.setTextColor(WHITE);
  tft.setTextSize(2);
  // ----- Lang
  if (langValue == 1)
  {
    tft.setCursor(38, 40);
    tft.println("Laser");
    tft.setCursor(27, 60);
    tft.println("Barrier");
  }
  else if (langValue == 2)
  {
    tft.setCursor(38, 40);
    tft.println("Lezer");
    tft.setCursor(27, 60);
    tft.println("Sorompo");
  }
  else if (langValue == 3)
  {
    tft.setCursor(30, 40);
    tft.println("Laser-");
    tft.setCursor(30, 60);
    tft.println("Sperre");
  }
  // ----
  tft.setTextSize(2);
  tft.setTextColor(RED);
  tft.setCursor(14, 13);
  tft.println("AutoKobot");

  tft.setTextSize(1);
  tft.setTextColor(GREEN);
  tft.setCursor(50, 85);
  tft.println("v" + String(version));
  tft.setTextColor(WHITE);
  tft.setCursor(55, 97);
  tft.println(lang);

  tft.setTextSize(1);
  tft.setTextColor(WHITE);
  tft.setCursor(52, 114);
  tft.println("2020");

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

  // Reed relay
  pinMode (reedRelay, INPUT_PULLUP);

  // Relay Outputs
  pinMode(pauseRelay, OUTPUT);    // 1mp 200mS
  pinMode(contRelay, OUTPUT);     // Allways On except on error
  //digitalWrite(contRelay, HIGH);
  pinMode(gripperRelay, OUTPUT);   // Allways On except on error
  //digitalWrite(gripperRelay, HIGH);

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

  // mcp2
  mcp2.begin(2);           // 1 = 0x22
  mcp2.pinMode(button, INPUT);
  mcp2.pullUp(button, HIGH);
  mcp2.pinMode(laser, OUTPUT);
  mcp2.pinMode(redLedO, OUTPUT);
  mcp2.pinMode(greenLedO, OUTPUT);
  mcp2.pinMode(photoDiode, INPUT);
  mcp2.pinMode(redLedI, OUTPUT);
  mcp2.pinMode(greenLedI, OUTPUT);

  // Print Program Version
  Serial.println("Laser Barrier");
  Serial.println("Test v" + String(version));

}
// ------------------------------------------------------------------------------------------------
// ------------------------------ LOOP ------------------------------------------------------------
void loop() {
  // Error State 0 = No Error  1 = Error
  // Sens State  0 = Sens Off  1 = Sense On
  // ############################# When ON the barrier ###############################
  if (error == 0 && sensState == 0) {

    changeOutMega(buttonN, laserN);
    changeOutMcp0(button, laser);
    changeOutMcp1(button, laser);
    changeOutMcp2(button, laser);

    // --- Main menu ---
    reedState = digitalRead(reedRelay);
    if (reedState == LOW) {
      rotaryMenu(1, 2);
      sensState = sensSW();
    }

  }
  // ################################## Sens ON ######################################
  else if (error == 0 && sensState == 1) {

    changeOutMega(buttonN, laserN);
    changeOutMcp0(button, laser);
    changeOutMcp1(button, laser);
    changeOutMcp2(button, laser);


    offSens();
    // If 1 turn off sens
    if (sensOff1 != 1)
    {
      sensMcp0();
    }
    if (sensOff2 != 1)
    {
      sensMcp1();
    }
    if (sensOff3 != 1)
    {
      sensMcp2();
    }
    if (sensOff4 != 1)
    {
      sensMega();
    }

    digitalWrite(pauseRelay, LOW);
    digitalWrite(contRelay, HIGH);
    digitalWrite(gripperRelay, HIGH);

  }
  // ################################## Error State ######################################
  if (error == 1) {
    while (sensState == 0) {
      reedState = digitalRead(reedRelay);
      if (reedState == LOW) {
        sensState = sensSW();
        rotaryMenu(1, 2);
      }
      pauseRobot();
      changeOutMega(buttonN, laserN);
      changeOutMcp0(button, laser);
      changeOutMcp1(button, laser);
      changeOutMcp2(button, laser);
    }
  }

}
// ------------------------------------------------------------------------------------------------
// --------------------------- END LOOP -----------------------------------------------------------

// ------------------- VOID Button Mega -------------------
void changeOutMega(int inPort, int outPort) {
  readingN = !digitalRead(inPort);
  if (readingN == HIGH && previousN == LOW && millis() - timeN > debounce) {
    if (stateN == LOW){
      if (sensOff1 == 1){
        digitalWrite(pauseRelay, HIGH);
        tft.fillRect(60, 5, 8, 8, BLUE);
        delay(BackOnTime);
        digitalWrite(pauseRelay, LOW);
        tft.fillRect(60, 5, 8, 8, BLACK);
      }      
      stateN = HIGH;
      sensOff1 = 0;
    }
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

  //  laserSens = mcp0.digitalRead(photoDiode);
  //  if (laserSens == 1 ) {
  //    digitalWrite(laserN, LOW);
  //    readingN = LOW;
  //    previousN = LOW;
  //    stateN = LOW;
  //    //    errorState = 1;
  //    //    error = 1;
  //  }
}

// ------------------- VOID Button mcp0 -------------------
void changeOutMcp0(int inPort, int outPort) {
  reading = !mcp0.digitalRead(inPort);

  if (reading == HIGH && previous == LOW && millis() - time > debounce) {
    if (state == LOW){
      if (sensOff2 == 1){
        digitalWrite(pauseRelay, HIGH);
        tft.fillRect(60, 5, 8, 8, BLUE);
        delay(BackOnTime);
        digitalWrite(pauseRelay, LOW);
        tft.fillRect(60, 5, 8, 8, BLACK);
      } 
      state = HIGH;
      sensOff2 = 0;
    }
    else
      state = LOW;
    time = millis();
  }
  mcp0.digitalWrite(outPort, state);
  previous = reading;
  error = 0;

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

  //  laserSens1 = mcp1.digitalRead(photoDiode);
  //  if (laserSens1 == 1 ) {
  //    mcp0.digitalWrite(laser, LOW);
  //    reading = LOW;
  //    previous = LOW;
  //    state = LOW;
  //    //    errorState = 1;
  //    //    error = 1;
  //  }
}

// ------------------- VOID Button mcp1 -------------------
void changeOutMcp1(int inPort, int outPort) {
  reading1 = !mcp1.digitalRead(inPort);

  if (reading1 == HIGH && previous1 == LOW && millis() - time1 > debounce) {
    if (state1 == LOW){
      if (sensOff3 == 1){
        digitalWrite(pauseRelay, HIGH);
        tft.fillRect(60, 5, 8, 8, BLUE);
        delay(BackOnTime);
        digitalWrite(pauseRelay, LOW);
        tft.fillRect(60, 5, 8, 8, BLACK);
      }      
      state1 = HIGH;
      sensOff3 = 0;
    }
    else
      state1 = LOW;
    time1 = millis();
  }
  mcp1.digitalWrite(outPort, state1);
  previous1 = reading1;
  error = 0;

  if (state1 == HIGH) {
    mcp1.digitalWrite(greenLedO, LOW);
    mcp1.digitalWrite(redLedO, HIGH);
    mcp2.digitalWrite(greenLedI, LOW);
    mcp2.digitalWrite(redLedI, HIGH);
  }
  if (state1 == LOW) {
    mcp1.digitalWrite(redLedO, LOW);
    mcp1.digitalWrite(greenLedO, HIGH);
    mcp2.digitalWrite(redLedI, LOW);
    mcp2.digitalWrite(greenLedI, HIGH);
  }

  //  laserSens2 = mcp2.digitalRead(photoDiode);
  //  if (laserSens2 == 1 ) {
  //    mcp1.digitalWrite(laser, LOW);
  //    reading1 = LOW;
  //    previous1 = LOW;
  //    state1 = LOW;
  //    //    errorState = 1;
  //    //    error = 1;
  //  }

}

// ------------------- VOID Button mcp2 -------------------
void changeOutMcp2(int inPort, int outPort) {
  reading2 = !mcp2.digitalRead(inPort);

  if (reading2 == HIGH && previous2 == LOW && millis() - time2 > debounce) {
    if (state2 == LOW){
      if (sensOff4 == 1){
        digitalWrite(pauseRelay, HIGH);
        tft.fillRect(60, 5, 8, 8, BLUE);
        delay(BackOnTime);
        digitalWrite(pauseRelay, LOW);
        tft.fillRect(60, 5, 8, 8, BLACK);
      }
      state2 = HIGH;
      sensOff4 = 0;
    }
    else
      state2 = LOW;
    time2 = millis();
  }
  mcp2.digitalWrite(outPort, state2);
  previous2 = reading2;
  error = 0;

  if (state2 == HIGH) {
    mcp2.digitalWrite(greenLedO, LOW);
    mcp2.digitalWrite(redLedO, HIGH);
    digitalWrite(greenLedNI, LOW);
    digitalWrite(redLedNI, HIGH);
  }
  if (state2 == LOW) {
    mcp2.digitalWrite(redLedO, LOW);
    mcp2.digitalWrite(greenLedO, HIGH);
    digitalWrite(redLedNI, LOW);
    digitalWrite(greenLedNI, HIGH);
  }

  //  laserSensN = digitalRead(photoDiodeN);
  //  if (laserSensN == 1 ) {
  //    mcp2.digitalWrite(laser, LOW);
  //    reading2 = LOW;
  //    previous2 = LOW;
  //    state2 = LOW;
  //    //    errorState = 1;
  //    //    error = 1;
  //  }

}
// -------------------- Sens Mega --------------------
void sensMega() {
  laserSensN = digitalRead(photoDiodeN);
  if (laserSensN == 1 ) {
    mcp2.digitalWrite(laser, LOW);
    reading2 = LOW;
    previous2 = LOW;
    state2 = LOW;
    error = 1;
    sensState = 0;
    sens = 0;
    tft.clearScreen();
    //tft.fillScreen();
    tft.drawRect(3, 3, 125, 125, BLUE);
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    // ----- Lang
    if (langValue == 1)
    {
      tft.setCursor(22, 50);
      tft.println("Fourth line was");
      tft.setTextSize(2);
      tft.setTextColor(BLUE);
      tft.setCursor(24, 65);
      tft.println("crossed");
    }
    if (langValue == 2)
    {
      tft.setCursor(30, 50);
      tft.println("Negyes vonal");
      tft.setTextSize(2);
      tft.setTextColor(BLUE);
      tft.setCursor(25, 65);
      tft.println("szakadt");
    }
    if (langValue == 3)
    {
      tft.setCursor(35, 50);
      tft.println("Linie vier");
      tft.setTextSize(2);
      tft.setTextColor(BLUE);
      tft.setCursor(18, 65);
      tft.println("gekreuzt");
    }
    // -----
  }
}

// -------------------- Sens Mcp0 --------------------
void sensMcp0() {
  laserSens = mcp0.digitalRead(photoDiode);
  if (laserSens == 1 ) {
    digitalWrite(laserN, LOW);
    readingN = LOW;
    previousN = LOW;
    stateN = LOW;
    error = 1;
    sensState = 0;
    sens = 0;
    tft.clearScreen();
    //tft.fillScreen();
    tft.drawRect(3, 3, 125, 125, BLUE);
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    // ---- Lang
    if (langValue == 1)
    {
      tft.setCursor(25, 50);
      tft.println("First line was");
      tft.setTextSize(2);
      tft.setTextColor(BLUE);
      tft.setCursor(24, 65);
      tft.println("crossed");
    }
    if (langValue == 2)
    {
      tft.setCursor(34, 50);
      tft.println("Egyes vonal");
      tft.setTextSize(2);
      tft.setTextColor(BLUE);
      tft.setCursor(25, 65);
      tft.println("szakadt");
    }
    if (langValue == 3)
    {
      tft.setCursor(35, 50);
      tft.println("Linie einer");
      tft.setTextSize(2);
      tft.setTextColor(BLUE);
      tft.setCursor(18, 65);
      tft.println("gekreuzt");
    }
    // -----
  }
}

// -------------------- Sens Mcp1 --------------------
void sensMcp1() {
  laserSens1 = mcp1.digitalRead(photoDiode);
  if (laserSens1 == 1 ) {
    mcp0.digitalWrite(laser, LOW);
    reading = LOW;
    previous = LOW;
    state = LOW;
    error = 1;
    sensState = 0;
    sens = 0;
    tft.clearScreen();
    //tft.fillScreen();
    tft.drawRect(3, 3, 125, 125, BLUE);
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    // ---- Lang
    if (langValue == 1)
    {
      tft.setCursor(22, 50);
      tft.println("Second line was");
      tft.setTextSize(2);
      tft.setTextColor(BLUE);
      tft.setCursor(24, 65);
      tft.println("crossed");
    }
    if (langValue == 2)
    {
      tft.setCursor(34, 50);
      tft.println("Kettes vonal");
      tft.setTextSize(2);
      tft.setTextColor(BLUE);
      tft.setCursor(25, 65);
      tft.println("szakadt");
    }
    if (langValue == 3)
    {
      tft.setCursor(32, 50);
      tft.println("Linie zweite");
      tft.setTextSize(2);
      tft.setTextColor(BLUE);
      tft.setCursor(18, 65);
      tft.println("gekreuzt");
    }
    // ----
  }
}

// -------------------- Sens Mcp2 --------------------
void sensMcp2() {
  laserSens2 = mcp2.digitalRead(photoDiode);
  if (laserSens2 == 1 ) {
    mcp1.digitalWrite(laser, LOW);
    reading1 = LOW;
    previous1 = LOW;
    state1 = LOW;
    error = 1;
    sensState = 0;
    sens = 0;
    tft.clearScreen();
    //tft.fillScreen();
    tft.drawRect(3, 3, 125, 125, BLUE);
    tft.setTextSize(1);
    tft.setTextColor(WHITE);
    // ---- Lang
    if (langValue == 1)
    {
      tft.setCursor(25, 50);
      tft.println("Third line was");
      tft.setTextSize(2);
      tft.setTextColor(BLUE);
      tft.setCursor(24, 65);
      tft.println("crossed");
    }
    if (langValue == 2)
    {
      tft.setCursor(34, 50);
      tft.println("Harmas vonal");
      tft.setTextSize(2);
      tft.setTextColor(BLUE);
      tft.setCursor(25, 65);
      tft.println("szakadt");
    }
    if (langValue == 3)
    {
      tft.setCursor(32, 50);
      tft.println("Linie dreite");
      tft.setTextSize(2);
      tft.setTextColor(BLUE);
      tft.setCursor(18, 65);
      tft.println("gekreuzt");
    }
    // ----
  }
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
    tft.fillRect(60, 5, 8, 8, BLACK);
  }
  else if ((relayState == LOW) && (currentMillis - previousMillis >= OffTime))
  {
    relayState = HIGH;                      // turn it on
    previousMillis = currentMillis;       // Remember the time
    digitalWrite(pauseRelay, relayState);   // Update the actual Relay
    tft.fillRect(60, 5, 8, 8, BLUE);
  }
}

// -------------------- Analog in From Robot --------------------
int offSens() {
  unsigned long currentMillis = millis();
  if ( currentMillis - prevMillis >= measureTime )
  {
    prevMillis = currentMillis; // Remember the time

    //INA.setPrescaler(5);
    INA.setNoiseThreshold(60);
    valueRobot = INA.read();
    //Serial.println(valueRobot);


    // 102.3 - personal value = 147
    if ( valueRobot >= 140 && valueRobot <= 154 )
    {
      sensOff1 = 1;
      //      digitalWrite(redLedNO, LOW);
      //      digitalWrite(greenLedNO, HIGH);
      //      mcp0.digitalWrite(redLedI, LOW);
      //      mcp0.digitalWrite(greenLedI, HIGH);
      stateN = LOW;
      tft.fillRect(20, 5, 8, 8, GREEN);
    }

    // 205 - personal value = 255
    else if ( valueRobot >= 248 && valueRobot <= 262 )
    {
      sensOff2 = 1;
      //      mcp0.digitalWrite(redLedO, LOW);
      //      mcp0.digitalWrite(greenLedO, HIGH);
      //      mcp1.digitalWrite(redLedI, LOW);
      //      mcp1.digitalWrite(greenLedI, HIGH);
      state = LOW;
      tft.fillRect(40, 5, 8, 8, GREEN);
    }

    // 307 - personal value = 350
    else if ( valueRobot >= 243 && valueRobot <= 357 )
    {
      sensOff3 = 1;
      //      mcp1.digitalWrite(redLedO, LOW);
      //      mcp1.digitalWrite(greenLedO, HIGH);
      //      mcp2.digitalWrite(redLedI, LOW);
      //      mcp2.digitalWrite(greenLedI, HIGH);
      state1 = LOW;
      tft.fillRect(60, 5, 8, 8, GREEN);
    }

    // 409 - personal value = 455
    else if ( valueRobot >= 447 && valueRobot <= 462 )
    {
      sensOff4 = 1;
      //      mcp2.digitalWrite(redLedO, LOW);
      //      mcp2.digitalWrite(greenLedO, HIGH);
      //      digitalWrite(redLedNI, LOW);
      //      digitalWrite(greenLedNI, HIGH);
      state2 = LOW;
      tft.fillRect(80, 5, 8, 8, GREEN);
    }
//    else if ( valueRobot >= 915 && valueRobot <= 934 ) { // 9V
//      //      sensOff1 = 0;
//      //      sensOff2 = 0;
//      //      sensOff3 = 0;
//      //      sensOff4 = 0;
//    }

    else
    {
      //sensOff1 = 0;
      //      digitalWrite(redLedNO, HIGH);
      //      digitalWrite(greenLedNO, LOW);
      //      mcp0.digitalWrite(redLedI, HIGH);
      //      mcp0.digitalWrite(greenLedI, LOW);

      tft.fillRect(20, 5, 8, 8, BLACK);

      //sensOff2 = 0;
      //      mcp0.digitalWrite(redLedO, HIGH);
      //      mcp0.digitalWrite(greenLedO, LOW);
      //      mcp1.digitalWrite(redLedI, HIGH);
      //      mcp1.digitalWrite(greenLedI, LOW);

      tft.fillRect(40, 5, 8, 8, BLACK);

      //sensOff3 = 0;
      //      mcp1.digitalWrite(redLedO, HIGH);
      //      mcp1.digitalWrite(greenLedO, LOW);
      //      mcp2.digitalWrite(redLedI, HIGH);
      //      mcp2.digitalWrite(greenLedI, LOW);

      tft.fillRect(60, 5, 8, 8, BLACK);

      //sensOff4 = 0;
      //      mcp2.digitalWrite(redLedO, HIGH);
      //      mcp2.digitalWrite(greenLedO, LOW);
      //      digitalWrite(redLedNI, HIGH);
      //      digitalWrite(greenLedNI, LOW);

      tft.fillRect(80, 5, 8, 8, BLACK);
    }
  }
}

// -------------------- Rotary Button --------------------
int sensSW() {
  swVal = !digitalRead(pinSW);
  // ########################### Activate light barrier ################################
  if (pinSWLast == true ) {
    if (swVal == LOW  && page == 1 && menuitem == 1) {
      ++sens;
      tft.clearScreen();
      tft.drawRect(3, 3, 125, 125, RED);
      tft.setTextColor(WHITE);
      // ---- Lang
      if (langValue == 1)
      {
        tft.setTextSize(1);
        tft.setCursor(28, 40);
        tft.println("Laser Barrier");
        tft.setCursor(12, 65);
        tft.setTextSize(2);
        tft.setTextColor(RED);
        tft.println("ACTIVATED");
      }
      if (langValue == 2)
      {
        tft.setTextSize(1);
        tft.setCursor(35, 40);
        tft.println("Fenysorompo");
        tft.setCursor(12, 65);
        tft.setTextSize(2);
        tft.setTextColor(RED);
        tft.println("AKTIVALVA");
      }
      if (langValue == 3)
      {
        tft.setTextSize(1);
        tft.setCursor(35, 40);
        tft.println("Lasersperre");
        tft.setCursor(12, 65);
        tft.setTextSize(2);
        tft.setTextColor(RED);
        tft.println("AKTIVIERT");
      }
      if (sens >= 1)
        sens = 1;
      pinSWLast = false;
    }
  }
  if (swVal == HIGH) {
    pinSWLast = true;
  }
  // ################################## goto page 2 ######################################
  if (pinSWLast == true ) {
    if (swVal == LOW && page == 1 && menuitem == 2 ) { // goto page 2
      ++page;
      if (page > 2)
      {
        page = 2;
      }
      menuitem = 1;
      tft.fillRect(10, 10, 110, 28, BLACK);
      drawMenu();
      pinSWLast = false;
    }
  }
  if (swVal == HIGH) {
    pinSWLast = true;
  }
  // ################################# goto page 3 #####################################
  if (pinSWLast == true ) {
    if (swVal == LOW && page == 2 && menuitem == 1 ) { // goto page 3
      ++page;
      if (page > 3)
      {
        page = 3;
      }
      menuitem = langValue;
      tft.fillRect(10, 10, 110, 28, BLACK);
      drawMenu();
      //delay(500);
      pinSWLast = false;
    }
  }
  if (swVal == HIGH) {
    pinSWLast = true;
  }
  // ################################## goto page 4 ####################################
  if (pinSWLast == true ) {
    if (swVal == LOW && page == 2 && menuitem == 2 ) { // goto page 4
      page = 4;
      tft.fillRect(10, 10, 110, 28, BLACK);
      menuitem = OffTimeValue;
      drawMenu();
      pinSWLast = false;
    }
  }
  if (swVal == HIGH) {
    pinSWLast = true;
  }
  // ################################ Exit to page 1 #######################################
  if (pinSWLast == true ) {
    if (swVal == LOW && page == 2 && menuitem == 3 ) { // Exit to page 1
      --page;
      if (page < 1)
      {
        page = 1;
      }
      menuitem = 2;
      tft.fillRect(10, 10, 110, 28, BLACK);
      drawMenu();
      pinSWLast = false;
    }
  }
  if (swVal == HIGH) {
    pinSWLast = true;
  }
  // ################################## ENG Selected ###################################
  if (pinSWLast == true ) {
    if (swVal == LOW && page == 3 && menuitem == 1 ) { // ENG Selected
      newLangValue = 1;
      if (newLangValue != langValue) {
        EEPROM.write(langAddr, newLangValue);
        langValue = EEPROM.read(langAddr);
      }
      --page;
      if (page < 2)
      {
        page = 2;
      }
      menuitem = 1;
      tft.fillRect(10, 10, 110, 28, BLACK);
      drawMenu();
      pinSWLast = false;
    }
  }
  if (swVal == HIGH) {
    pinSWLast = true;
  }
  // ################################# HUN Selected #####################################
  if (pinSWLast == true ) {
    if (swVal == LOW && page == 3 && menuitem == 2 ) { // HUN Selected
      newLangValue = 2;
      if (newLangValue != langValue) {
        EEPROM.write(langAddr, newLangValue);
        langValue = EEPROM.read(langAddr);
      }
      --page;
      if (page < 2)
      {
        page = 2;
      }
      menuitem = 1;
      tft.fillRect(10, 10, 110, 28, BLACK);
      drawMenu();
      pinSWLast = false;
    }
  }
  if (swVal == HIGH) {
    pinSWLast = true;
  }
  // ############################### DEU Selected ######################################
  if (pinSWLast == true ) {
    if (swVal == LOW && page == 3 && menuitem == 3 ) { // DEU Selected
      newLangValue = 3;
      if (newLangValue != langValue) {
        EEPROM.write(langAddr, newLangValue);
        langValue = EEPROM.read(langAddr);
      }
      --page;
      if (page < 2)
      {
        page = 2;
      }
      menuitem = 1;
      tft.fillRect(10, 10, 110, 28, BLACK);
      drawMenu();
      pinSWLast = false;
    }
  }
  if (swVal == HIGH) {
    pinSWLast = true;
  }
  // ########################## Exit to page 2 from page 4 ################################
  if (pinSWLast == true ) {
    if (swVal == LOW && page == 4  ) {                 // Exit to page 2 from page 4
      NewOffTimeValue = menuitem;
      if (OffTimeValue != NewOffTimeValue) {
        EEPROM.write(timeAddr, NewOffTimeValue);
        OffTimeValue = EEPROM.read(timeAddr);
        OffTime = OffTimeValue * 100;
      }
      page = 2;
      menuitem = 2;
      tft.fillRect(10, 10, 110, 28, BLACK);
      drawMenu();
      pinSWLast = false;
    }
  }
  if (swVal == HIGH) {
    pinSWLast = true;
  }
  // ################################# END ############################################
  return sens;
}

// -------------------- Rotary Menu ---------------------------
void rotaryMenu(int minValue, int maxValue)
{
  if ( page == 2 | page == 3) {
    maxValue = maxValue + 1;  // Because of exit
  }
  if ( page == 4) {
    maxValue = 30;
    minValue = 2;
  }
  aVal = digitalRead(pinA);
  bVal = digitalRead(pinB);
  if (digitalRead(pinA) == LOW && digitalRead(pinB) == LOW)
  {
    if (page == 4)
    {
      ++menuitem;
      if (menuitem > maxValue)
      {
        menuitem = maxValue;
      }
      drawMenu();
    }
    if (page == 1 | page == 2 | page == 3)
    {
      --menuitem;
      if (menuitem < minValue)
      {
        menuitem = minValue;
      }
      drawMenu();
    }
  }

  else if (digitalRead(pinA) == LOW && digitalRead(pinB) == HIGH)
  {
    if (page == 4)
    {
      --menuitem;
      if (menuitem < minValue)
      {
        menuitem = minValue;
      }
      drawMenu();
    }
    if (page == 1 | page == 2 | page == 3)
    {
      ++menuitem;
      if (menuitem > maxValue)
      {
        menuitem = maxValue;
      }
      drawMenu();
    }
  }
}

// -------------------- Draw Menu ---------------------
void drawMenu()
{
  if (page == 1)
  {
    tft.setTextSize(2);
    tft.fillRect(8, 8, 120, 120, BLACK);
    tft.drawRect(3, 3, 125, 125, WHITE);
    //tft.fillRect(10, 10, 110, 28, BLACK);
    tft.setTextColor(YELLOW);
    // ---- Lang
    if (langValue == 1)
    {
      tft.setCursor(13, 17);
      tft.println("MAIN MENU");
    }
    if (langValue == 2)
    {
      tft.setCursor(28, 17);
      tft.println("FOMENU");
    }
    if (langValue == 3)
    {
      tft.setCursor(13, 17);
      tft.println("HAUPTMENU");
    }

    if (menuitem == 1 )
    {
      if (langValue == 1)
      {
        displayMenuItem(menuItem1Eng, 45, true);
        displayMenuItem(menuItem2Eng, 65, false);
      }
      if (langValue == 2)
      {
        displayMenuItem(menuItem1Hun, 45, true);
        displayMenuItem(menuItem2Hun, 65, false);
      }
      if (langValue == 3)
      {
        displayMenuItem(menuItem1Deu, 45, true);
        displayMenuItem(menuItem2Deu, 65, false);
      }
    }
    else if (menuitem == 2 )
    {
      if (langValue == 1)
      {
        displayMenuItem(menuItem1Eng, 45, false);
        displayMenuItem(menuItem2Eng, 65, true);
      }
      if (langValue == 2)
      {
        displayMenuItem(menuItem1Hun, 45, false);
        displayMenuItem(menuItem2Hun, 65, true);
      }
      if (langValue == 3)
      {
        displayMenuItem(menuItem1Deu, 45, false);
        displayMenuItem(menuItem2Deu, 65, true);
      }
    }
  }
  if (page == 2)
  {
    tft.setTextSize(2);
    tft.fillRect(8, 44, 110, 80, BLACK);
    tft.drawRect(3, 3, 125, 125, WHITE);
    //tft.fillRect(10, 10, 110, 28, BLACK);
    tft.setTextColor(YELLOW);
    if (langValue == 1)
    {
      tft.setCursor(34, 17);
      tft.println("SETUP");
    }
    if (langValue == 2)
    {
      tft.setCursor(13, 17);
      tft.println("BEALLITAS");
    }
    if (langValue == 3)
    {
      tft.setCursor(12, 17);
      tft.println("EINSTEL.");
    }

    if (menuitem == 1 )
    {
      if (langValue == 1)
      {
        displayMenuItem(menuItem3Eng, 45, true);
        displayMenuItem(menuItem4Eng, 65, false);
        displayMenuItem(menuItem5Eng, 95, false);
      }
      if (langValue == 2)
      {
        displayMenuItem(menuItem3Hun, 45, true);
        displayMenuItem(menuItem4Hun, 65, false);
        displayMenuItem(menuItem5Hun, 95, false);
      }
      if (langValue == 3)
      {
        displayMenuItem(menuItem3Deu, 45, true);
        displayMenuItem(menuItem4Deu, 65, false);
        displayMenuItem(menuItem5Deu, 95, false);
      }
    }
    else if (menuitem == 2 )
    {
      if (langValue == 1)
      {
        displayMenuItem(menuItem3Eng, 45, false);
        displayMenuItem(menuItem4Eng, 65, true);
        displayMenuItem(menuItem5Eng, 95, false);
      }
      if (langValue == 2)
      {
        displayMenuItem(menuItem3Hun, 45, false);
        displayMenuItem(menuItem4Hun, 65, true);
        displayMenuItem(menuItem5Hun, 95, false);
      }
      if (langValue == 3)
      {
        displayMenuItem(menuItem3Deu, 45, false);
        displayMenuItem(menuItem4Deu, 65, true);
        displayMenuItem(menuItem5Deu, 95, false);
      }
    }
    else if (menuitem == 3 )
    {
      if (langValue == 1)
      {
        displayMenuItem(menuItem3Eng, 45, false);
        displayMenuItem(menuItem4Eng, 65, false);
        displayMenuItem(menuItem5Eng, 95, true);
      }
      if (langValue == 2)
      {
        displayMenuItem(menuItem3Hun, 45, false);
        displayMenuItem(menuItem4Hun, 65, false);
        displayMenuItem(menuItem5Hun, 95, true);
      }
      if (langValue == 3)
      {
        displayMenuItem(menuItem3Deu, 45, false);
        displayMenuItem(menuItem4Deu, 65, false);
        displayMenuItem(menuItem5Deu, 95, true);
      }
    }
  }
  if (page == 3)
  {
    tft.setTextSize(2);
    tft.fillRect(8, 44, 110, 80, BLACK);
    tft.drawRect(3, 3, 125, 125, WHITE);
    // ENG Flag ----
    tft.fillRect(75, 45, 20, 15, WHITE);
    tft.fillRect(84, 45, 2, 15, BLUE);
    tft.fillRect(75, 52, 20, 2, BLUE);
    //tft.fillRect(75,55,20,5, GREEN);
    // HUN Flag ----
    tft.fillRect(75, 65, 20, 5, BLUE);
    tft.fillRect(75, 70, 20, 5, WHITE);
    tft.fillRect(75, 75, 20, 5, GREEN);
    // DEU Flag ----
    tft.drawRect(75, 85, 20, 15, WHITE);
    tft.fillRect(76, 86, 18, 5, BLACK);
    tft.fillRect(76, 90, 18, 5, BLUE);
    tft.fillRect(76, 95, 18, 5, CYAN);
    // -------------------------------
    //tft.fillRect(10, 10, 110, 28, BLACK);
    tft.setTextColor(YELLOW);
    if (langValue == 1)
    {
      tft.setCursor(18, 17);
      tft.println("LANGUAGE");
    }
    else if (langValue == 2)
    {
      tft.setCursor(34, 17);
      tft.println("NYELV");
    }
    else if (langValue == 3)
    {
      tft.setCursor(22, 17);
      tft.println("SPRACHE");
    }
    if (menuitem == 1 )
    {
      displayMenuItem(menuItem6, 45, true);
      displayMenuItem(menuItem7, 65, false);
      displayMenuItem(menuItem8, 85, false);
    }
    else if (menuitem == 2 )
    {
      displayMenuItem(menuItem6, 45, false);
      displayMenuItem(menuItem7, 65, true);
      displayMenuItem(menuItem8, 85, false);
    }
    else if (menuitem == 3 )
    {
      displayMenuItem(menuItem6, 45, false);
      displayMenuItem(menuItem7, 65, false);
      displayMenuItem(menuItem8, 85, true);
    }
  }

  if (page == 4)
  {
    tft.setTextSize(2);
    tft.fillRect(8, 44, 110, 80, BLACK);
    tft.drawRect(3, 3, 125, 125, WHITE);
    tft.setTextColor(YELLOW);
    if (langValue == 1)
    {
      tft.setCursor(26, 17);
      tft.println("R.Delay");
    }
    if (langValue == 2)
    {
      tft.setCursor(24, 17);
      tft.println("R.Kesl.");
    }
    if (langValue == 3)
    {
      tft.setCursor(24, 17);
      tft.println("R.Verzo");
    }
    if (menuitem >= 1 && menuitem <= 29 )
    {
      tft.setTextColor(BLACK, WHITE);
      tft.setTextSize(2);
      tft.setCursor(40, 65);
      tft.print(menuitem * 100);
      tft.setTextColor(WHITE, BLACK);
      tft.setCursor(29, 85);
      tft.print("millis");
    }
  }
}

void displayMenuItem(String item, int position, boolean selected)
{
  if (selected)
  {
    tft.setTextSize(2);
    tft.setTextColor(BLACK, WHITE);
    tft.setCursor(8, position);
    tft.print(">" + item);
  } else
  {
    tft.setTextSize(2);
    tft.setTextColor(WHITE, BLACK);
    tft.setCursor(8, position);
    tft.print(" " + item);
  }
}
