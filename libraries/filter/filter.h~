#ifndef FILTER_H
#define FILTER_H

#include <Arduino.h>

class filter{

public:
  //sensor read out with LPF
  int sensor_LPF(int flex, int cumulative_value);
  void reset_sensor_vals();
  // function to obtain readout for flexsensor once value is steady
  void steadyflex(int cflex); 

private:
  // check if flex reading is steady
  bool isSteady = false;
  //flex sensor moving average variable definitions
  const int LPF_window_length = 5;
  int sensor_values[5];
  int iter = 0;
  // flex sensor read value after LPF
  // int cumulative_value = 0;

  //flex sensor steady state values
  const int steady_window_length = 30;
  int steady_sensor_values[30];
  int steady_threshold = 5;  
};

#endif
