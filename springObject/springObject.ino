#include <spring2.h>
#include <PID_v1.h>

int initFlag;

// learning mode and hyperparams (0:none, 1:free state, 2:clamped state, 3:update state)
int learning_mode; 
// plotting mode (0:none, 1:flex sensor, 2:encoder position)
int plot_mode; 

spring2 mySprings[] = {spring2(2, 3, 8, 9, A0), spring2(4, 5, 11, 12, A1)};
int numSprings = sizeof(mySprings)/ sizeof(mySprings[0]);

// define PID and interrupt handlers for each instance of spring2 
// (unfortunately I can't get this into the class itself atm. maybe fixable in the future)
void InterruptHandlerA_0(void) { 
  mySprings[0].doEncoderA();
}
void InterruptHandlerB_0(void) { 
  mySprings[0].doEncoderB();
}
void InterruptHandlerA_1(void) { 
  mySprings[1].doEncoderA();
}
void InterruptHandlerB_1(void) { 
  mySprings[1].doEncoderB(); 
}

//function to plot sensor read out
void sensor_plot() {
  Serial.print("d_min:");
  Serial.print(-30);
   for (int si = 0; si < numSprings; si++){
    Serial.print(",");
    Serial.print("disp");
    Serial.print(si);
    Serial.print(":");
    Serial.print(mySprings[si].current_disp);
   }
  Serial.print(",");
  Serial.print("d_max:");
  Serial.println(30);
}

void layer_plot() {
  // for (int tableindex = 0; tableindex < arrLen; tableindex++){
  // }
  Serial.print("encpos_min:");
  Serial.print(0);
  for (int si = 0; si < numSprings; si++){
    Serial.print(",");
    Serial.print("input");
    Serial.print(si);
    Serial.print(":");
    Serial.print(mySprings[si].input);
    Serial.print(",");
    Serial.print("setpoint");
    Serial.print(si);
    Serial.print(":");
    Serial.print(mySprings[si].setpoint);
   }
  Serial.print(",");
  Serial.print("encpos_max:");
  Serial.println(300);
}

void setup() {
  //setup serial communication
  Serial.begin(9600);

  //setup Springs
  for (int si = 0; si < numSprings; si++){
    mySprings[si].init();
  }
  // mySpring.init();
  attachInterrupt(digitalPinToInterrupt(mySprings[0].encPinA), InterruptHandlerA_0, RISING);
  attachInterrupt(digitalPinToInterrupt(mySprings[0].encPinB), InterruptHandlerB_0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(mySprings[1].encPinA), InterruptHandlerA_1, RISING);
  attachInterrupt(digitalPinToInterrupt(mySprings[1].encPinB), InterruptHandlerB_1, CHANGE);

  learning_mode = 0;
  plot_mode = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int si = 0; si < numSprings; si++){
    mySprings[si].onLoop();
  }
  //  mySpring.onLoop();

  if (Serial.available() > 0)  {    //reading the data from Serial port
    initFlag = Serial.read();

      if (initFlag == 77) {  //key press "M" to change the learning mode
      // mode = 0 is just readout, no updates and no saving flex values
      // mode = 1 is free state (store flex in memory)
      // mode = 2 is clamped state (store flex in memory)
      // mode = 3 is update mode (adjust number of layers)
      learning_mode++;
      learning_mode = learning_mode % 4;
      plot_mode=0;
      for (int si = 0; si < numSprings; si++){
        mySprings[si].mode_reset();
      }
      // mySpring.mode_reset();
    }

    else if(initFlag == 85) { // key press "U" to go up by a half layer
        Serial.println("Go up by a half layer");
        int si = Serial.parseFloat(SKIP_ALL);
        mySprings[si].upLayer();
        // mySpring.upLayer();
    } 

    else if (initFlag == 68) { //key press "D" to go down by a half layer
        Serial.println("Go down by a half layer");
        int si = Serial.parseFloat(SKIP_ALL);
        mySprings[si].downLayer();
        // mySpring.downLayer();
    }

    else if (initFlag == 80) { //key press "P" to change plot mode like so: P 1 -> sets flex plot mode    
      // always turn off plotting before setting a new layer! It will mess up the encoder reading
      plot_mode = Serial.parseFloat(SKIP_ALL);
    }    
  }

  if (learning_mode == 0) {
    if (plot_mode > 0){
      Serial.print("Null state: ");
    }
    else{
      Serial.println("Null state");
    }
  }
  else if (learning_mode == 1) { // free state
    plot_mode=1;
    for (int si = 0; si < numSprings; si++){
      mySprings[si].takeReading(1);
    }
    // mySpring.takeReading(1);
  } 
  else if (learning_mode == 2) { // clamp state
    plot_mode=1;
    for (int si = 0; si < numSprings; si++){
      mySprings[si].takeReading(2);
    }
    // mySpring.takeReading(2);
  }
  else if (learning_mode == 3) { // update state
    Serial.print("Update state: ");
    plot_mode = 2;
    // mySpring.doUpdate();
    for (int si = 0; si < numSprings; si++){
      mySprings[si].doUpdate();
    }
  }

  if (plot_mode == 1) {
      sensor_plot();
    // sensor_plot(mySpring.current_disp);
  }
  else if (plot_mode == 2) {
      layer_plot();
    // layer_plot(mySpring.input, mySpring.setpoint);
  } 
}
