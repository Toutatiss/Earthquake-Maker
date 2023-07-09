// Speed Controlled Oscillator
// Written by Tat Kalintsev
// 29/08/2022

#include <LiquidCrystal.h>
#include <BlockNot.h>
#include <SimplyAtomic.h>


#define updateDur 2000  // in ms
#define buttons A0
#define motorEnable 3
#define speedControl A1
#define rangeScaler 0.2495108

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

const int no_cams = 4;

int value;
int prevValue = 0;
volatile unsigned long currTime_i;
unsigned long currTime;
unsigned long prevTime;
float period, currFreq, frequency;
float freq = 0; 
float prevFreq = 7;
float freqFilt = 0;
float speedInput = 10; // Probably delete this later
float control;
float verify;

// PI Control Variables
float targetSpeed = 10.0;
//float currentSpeed;
// current speed == frequency
// dt == period in seconds
float error, dt;
float eintegral = 0;
const float Kp = 1;
const float Ki = 1;
float controlSignal, pwm;


// UI Variables
int state = 0;
int TS = 10;
int CS = 0;

// Motor Control
bool motorIsOn = false;
bool motorWasOn = false;
int powerIncrement = 5;
int motorPower = 0;

void setup() {
  // LCD Setup + Title Screen
  lcd.begin(16,2);

  pinMode(PhotoInterruptPin, INPUT);
  pinMode(motorEnable, OUTPUT);
  pinMode(speedControl, INPUT);

  // Setup Interrupt
  attachInterrupt(digitalPinToInterrupt(PhotoInterruptPin), updateIntTime, RISING);

  // Necessary
  Serial.begin(9600);
  //  analogWrite(motorEnable, 250);
  prevTime = millis();

  // Show loading screen
  lcd.setCursor(3,0);
//  lcd.setCursor(1,0);
//  lcd.print("Quartz Whacker");
  lcd.print("Starting");
  delay(100);
  lcd.print(".");
  delay(100);
  lcd.print(".");
  delay(100);
  lcd.print(".");
//  lcd.setCursor(6,1);
//  lcd.print("9000");
  delay(1000);
  
  lcd.clear();
  displayInfoScreen();
}


void loop() {
  readButtons();
  
//  ATOMIC() {
//    currTime = currTime_i;
//  }
  
//  speedInput = speedInput / rangeScaler;

  switch (state) {
      case 0: 
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
        
//        setMotorSpeed();

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
  
  // Reset PI Variables:
  eintegral = 0;
  pwm = 0;

  // Reset Display Variables:
  frequency = 0;
  updateFrequencyDisplay();
  
  delay(1000);
}

void powerOnMotor() {
    for (int i = 0; i < 12; i++) { // increment the power to 200 gradually
      motorPower = motorPower + powerIncrement;
      analogWrite(motorEnable, motorPower);
      delay(30);
    }
    motorWasOn = true;
}

void controlSpeed() {
  dt = period/1000; // calculate time interval in seconds
  error = targetSpeed - frequency; // calculate error between desired and actual speeds
  eintegral = eintegral + error*dt; // calculate the integral;
  
  controlSignal = Kp*error + Ki*eintegral; // calculate what the new control signal should be
  
  pwm = (int) fabs(controlSignal); // convert control signal to a pwm value
  pwm = (pwm>255 ? 255 : pwm); // cap the pwm to 255
//  Serial.println(pwm);
  
  analogWrite(motorEnable, pwm); // write new pwm signal to motor
}

void calculateFrequency() {
    period = currTime - prevTime; // Get the difference between last millis
    verify = (1 / period) * 1000; // Calculate the frequency/rpm

    if (verify < 50) {
      frequency = verify;
    }
    
    Serial.println(frequency);
    Serial.print(" Hz");
    Serial.println("");
    
    prevTime = currTime;
    prevFreq = frequency;
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
   lcd.print ("Up    ");
 }
 else if (x < 400){
   lcd.print ("Down  ");
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
  // Line 1:
  lcd.setCursor(0,0);
  lcd.print("TS: ");
  lcd.print(int(targetSpeed));
  lcd.print(" CS: ");
  updateFrequencyDisplay();

  // Line 2:
  lcd.setCursor(0,1);
  lcd.print(">START/STOP");
}

void updateFrequencyDisplay() {
  lcd.setCursor(11,0);
  lcd.print(int(round(frequency)));
  lcd.print("Hz  ");
}

void updateIntTime() {
  currTime = millis();
  needToUpdateFreq = true;
}
