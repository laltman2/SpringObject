//#include "Arduino.h"
#include "spring2.h"
#include "filter.h"
#include "conversion.h"
#include "PID_v1.h"

double halfLayer = 12;
int PIDout = 10;

float k_p = 50;
float k_i = 0;
float k_d = 0;


//function to set min thresholds for PWM value
double output_limits(double Output){
  if (Output > 0 && Output < PIDout) Output = PIDout;
  if (Output < 0 && Output > -1*PIDout) Output = -1*PIDout;
  return Output;
}

// Outside of class
//spring2 *pointerToClass; // declare a pointer to class

//static void InterruptHandlerA(void) { // define global handler
//  pointerToClass->doEncoderA(); // calls class member handler
//{

//static void InterruptHandlerB(void) { // define global handler
//  pointerToClass->doEncoderB(); // calls class member handler
//}

spring2::spring2(byte encPinA, byte encPinB, byte pwmPin, byte dirPin, byte flexPin){
  this -> flexPin = flexPin;
  this -> encPinA = encPinA;
  this -> encPinB = encPinB;
  this -> pwmPin = pwmPin;
  this -> dirPin = dirPin;

  myPID = new PID(&input, &output, &setpoint, k_p, k_i, k_d, DIRECT);
}

void spring2::init(){
  //setup pin modes
  pinMode(flexPin, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(encPinA, INPUT_PULLUP);
  pinMode(encPinB, INPUT_PULLUP);

  //initialize encoder bits
  PastA = (boolean)digitalRead(encPinA);
  PastB = (boolean)digitalRead(encPinB);

  myPID -> SetMode(AUTOMATIC);
  myPID -> SetOutputLimits(-250, 250);

  //pointerToClass = this; // assign current instance to pointer (IMPORTANT!!!)

  //declare interrupt service routines
  //attachInterrupt(digitalPinToInterrupt(encPinA), InterruptHandlerA, RISING);
  //attachInterrupt(digitalPinToInterrupt(encPinB), InterruptHandlerB, CHANGE);
}

void spring2::doEncoderA(void){
  PastB ? encPos++ :  encPos--; // if pastB then add 1 to encpos else minus 1 to encpos
}

void spring2::doEncoderB(void){
  PastB = !PastB; //flip pastb
}

void spring2::upLayer(){
  setpoint = input + halfLayer;
}

void spring2::downLayer(){
  setpoint = input - halfLayer;
}

void spring2::onLoop(){
  ADCflex = analogRead(flexPin);
  input = encPos;

  cumulative_value = myfilter.sensor_LPF(ADCflex, cumulative_value);
  current_disp = myconversion.flex2disp(cumulative_value);

  //implement PID control (only proportional control is being used)
  if (abs(input - setpoint) < 2){
    myPID -> SetTunings(0, 0, 0); 
  }
  else{
    myPID -> SetTunings(k_p, k_d, k_i); 
  }
  myPID -> Compute();

  output = output_limits(output);

  //choose motor direction : clockwise/anticlockwise
  if (output > 0){
    digitalWrite(dirPin,0);
  }
  else {
    digitalWrite(dirPin,1);
  }
  if (abs(input-setpoint) < 2){
    output = 0;
  }
  if (encPos < 0){
    output = 0;
    input = 0;
    encPos = 0;
    setpoint = 0;
  }
  analogWrite(pwmPin,abs(output));
}

void spring2::mode_reset(){
  measured = false;
  updated = false;
  myfilter.reset_sensor_vals();
  waitCounter = 0;
}

void spring2::takeReading(int LM){
  if (!measured) {
      isSteady = false;
      isSteady = myfilter.steadyflex(cumulative_value, waitCounter);
      if (isSteady) {
	if (LM==1){
	  free_disp = current_disp;
	}
	else if (LM==2){
	  clamp_disp = current_disp;
	}
	measured = true;
      }
      waitCounter ++;
  }
}

void spring2::doUpdate(){
  if (!updated){
      // float dLayer = myupdt.updaterule_binary(free_disp, clamp_disp);
      // setpoint = dLayer*layer_increment + input;

      // binary learning rule: 
      // if clamp_disp > free_disp, weaken the spring (down a half layer)
      // otherwise, stiffen the spring (up a half layer)
      bool clampOverFree = (clamp_disp > free_disp);
      if (clampOverFree){
	downLayer();
      }
      else {
        upLayer();
      }
      updated = true;
    }
}
