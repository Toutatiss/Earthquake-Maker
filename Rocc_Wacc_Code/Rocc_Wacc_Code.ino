// PI Earthquake Maker
// Written by Tat Kalintsev
// 21/11/2022

// Libraries
#include <LiquidCrystal.h>
#include <BlockNot.h>
#include <SimplyAtomic.h>

// Pin assignments
#define buttons A0
#define motorEnable 3
#define speedControl A1

// Constants
#define updateDur 2000  // in ms
#define rangeScaler 0.2495108 // DELETE THIS?

// Interrupt Stuff
const byte PhotoInterruptPin = 2;
bool needToUpdateFreq = false;

// LCD Definitions
const int RS = 8;
const int EN = 9;
const int d4 = 4;
const int d5 = 5;
const int d6 = 6;
const int d7 = 7;
const int pin_BL = 10; // arduino pin wired to LCD backlight circuit

LiquidCrystal lcd( RS,  EN,  d4,  d5,  d6,  d7);
BlockNot updateTimer(updateDur);

// CHECK THIS VALUE - DOES IT MATCH THE PHYSICAL SETUP?
const int no_cams = 4;

// Speed and Timing Variables
unsigned long currTime;
unsigned long prevTime;
float period, currFreq, frequency; 
float freq = 0;

// PI Control Variables
float targetSpeed = 20.0;
float error, dt;
float eintegral = 0;
const float Kp = 1;
const float Ki = 1;
float controlSignal, pwm;

// UI Variables
int state = 0;

// Motor Control Variables
bool motorIsOn = false;
bool motorWasOn = false;
int powerIncrement = 5;
int motorPower = 0;

void setup() {
  // Initialise Stuff
  lcd.begin(16,2);
  Serial.begin(9600);

  // Assign Pins
  pinMode(PhotoInterruptPin, INPUT);
  pinMode(motorEnable, OUTPUT);
  pinMode(speedControl, INPUT);

  // Setup Interrupt
  attachInterrupt(digitalPinToInterrupt(PhotoInterruptPin), updateIntTime, RISING);

  // Animated Loading Screen
  lcd.setCursor(3,0);
  lcd.print("Starting");  
  for (int i = 0; i < 3; i++) { // increment the power to 200 gradually
    delay(100);
    lcd.print(".");
  }
  
  delay(500);
  lcd.clear();
  displayInfoScreen();
  Serial.println("Setup Done");
}


void loop() {
  readButtons();

  switch (state) {
      case 0:
        updateFrequencyDisplay(); 
        if (motorIsOn) {
          motorIsOn = false;
          powerOffMotor();
        }      
        break;
      case 1: // Info Screen
        if (!motorWasOn) {
          motorIsOn = true;
          powerOnMotor();
        }

        if (updateTimer.TRIGGERED) {
          updateFrequencyDisplay();
        } else if (needToUpdateFreq) { 
          calculateFrequency();
          controlSpeed();
        }
        
        break;
      default:
        // Do nothing
        break;
    }
}

// SECTION: MOTOR CONTROL
void powerOffMotor() {
  motorPower = 0;
  analogWrite(motorEnable, motorPower);
  motorWasOn = false;

  // Reset Timing Variables:
  prevTime = 0;
  currTime = 0;
  
  // Reset PI Variables:
  dt = 0;
  error = 0;
  controlSignal = 0;
  eintegral = 0;
  pwm = 0;

  // Reset Display Variables:
  frequency = 0;
  updateFrequencyDisplay();
  
  delay(1000);
}

void powerOnMotor() {
    pwm = 0;
    for (int i = 0; i < 10; i++) { // increment the power to 200 gradually
      pwm = pwm + powerIncrement;
      analogWrite(motorEnable, pwm);
      delay(100);
    }
    motorWasOn = true;
    prevTime = millis();
}

//void controlSpeed() {
//  dt = 1/frequency; // calculate time interval in seconds
//  error = targetSpeed - frequency; // calculate error between desired and actual speeds
//  eintegral = eintegral + error*dt; // calculate the integral;
//
//  controlSignal = Kp*error + Ki*eintegral; // calculate what the new control signal should be
//
//  pwm = (int) fabs(controlSignal); // convert control signal to a pwm value
//  pwm = (pwm>255 ? 255 : pwm); // cap the pwm to 255
//  pwm = (pwm<1 ? 0 : pwm);
//  
//  analogWrite(motorEnable, pwm); // write new pwm signal to motor
//}

void controlSpeed() {
  dt = 1/frequency; // calculate time interval in seconds
//  Serial.println("dt: ");
//  Serial.print(dt);
//  Serial.println("");
  error = targetSpeed - frequency; // calculate error between desired and actual speeds
//  Serial.println("Error: ");
//  Serial.print(error);
//  Serial.println("");
  eintegral = eintegral + error*dt; // calculate the integral;
//  Serial.println("eintegral: ");
//  Serial.print(eintegral);
//  Serial.println("");
  
  controlSignal = Kp*error + Ki*eintegral; // calculate what the new control signal should be
//  Serial.println("control signal: ");
//  Serial.print(controlSignal);
//  Serial.println("");
  
  pwm = (int) fabs(controlSignal); // convert control signal to a pwm value
  pwm = (pwm>255 ? 150 : pwm); // cap the pwm
  pwm = (pwm<20 ? 20 : pwm); // prevent motor from stalling
//  Serial.println("PWM: ");
//  Serial.print(pwm);
//  Serial.println("");
  
  analogWrite(motorEnable, pwm); // write new pwm signal to motor
}

void calculateFrequency() {
    period = currTime - prevTime; // Get the difference between last millis
    Serial.println(period);
    Serial.println("");
    freq = (1 / period) * 1000; // Calculate the frequency/rpm

    if (freq < 50) {
      frequency = freq*no_cams;
    }
    
    Serial.println(frequency);
//    Serial.print(" Hz");
    Serial.println("");
    
    prevTime = currTime;
    needToUpdateFreq = false;
}


// SECTION: USER INTERFACE FUNCTIONS
void readButtons() {
 int x;
 x = analogRead(buttons);
 lcd.setCursor(0,0);
 if (x < 60) {
   lcd.print ("Right ");
 }
 else if (x < 200) {
  targetSpeed = targetSpeed + 1;
  delay(200);
 }
 else if (x < 400){
  targetSpeed = targetSpeed - 1;
  delay(200);
 }
 else if (x < 600){ // Left was pressed
   lcd.print ("Left  ");
   lcd.clear(); 
//   state = 0; // return to main menu
 }
 else if (x < 800){ // Select was pressed
  state = (motorIsOn ? 0 : 1);
  Serial.println("State: ");
  Serial.println(state);
  Serial.println("");
  
 }
}

// Menus
void displayInfoScreen() {
  // Line 2:
  lcd.setCursor(0,1);
  lcd.print(">START/STOP");
}

void updateFrequencyDisplay() {
  lcd.setCursor(0,0);
  lcd.print("TS: ");
  lcd.print(int(targetSpeed));
  lcd.print(" CS: ");
  lcd.setCursor(11,0);
  lcd.print(int(round(frequency)));
  lcd.print("Hz  ");
}

void updateIntTime() {
  currTime = millis();
  needToUpdateFreq = true;
}
