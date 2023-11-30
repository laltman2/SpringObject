#include <Arduino.h>
#ifndef spring2_h
#define spring2_h

class spring2{
  
public:

  double input, output, setpoint;

  // physical and learning DOF
  float free_disp;
  float clamp_disp;

  // flex sensor read value after LPF
  float current_disp;

  void upLayer();
  void downLayer();
  void onLoop_beforePID();
  void onLoop_afterPID();
  void mode_reset();
  void takeReading(int LM);
  void doUpdate();
  
  void init();
  void doEncoderA(void);
  void doEncoderB(void);

  spring2(byte encPinA, byte encPinB, byte pwmPin, byte dirPin, byte flexPin);

private:
    // pin definitions
  byte encPinA;
  byte encPinB;
  byte pwmPin;
  byte dirPin;
  byte flexPin;

  volatile int encPos = 0;
  volatile boolean PastA = 0;
  volatile boolean PastB = 0;
  volatile boolean CurrentA = 0;
  volatile boolean CurrentB = 0;

   int cumulative_value;
  int ADCflex;

  // check if flex reading is steady
  bool isSteady = false;
  // check whether a measurement has been taken or not
  volatile boolean measured;
  bool updated = false;
  int waitCounter;
};

#endif
