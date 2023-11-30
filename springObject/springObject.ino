#include <spring2.h>
#include <PID_v1.h>

int initFlag;

// float k_p = 50;
// float k_i = 0;
// float k_d = 0;

// learning mode and hyperparams (0:none, 1:free state, 2:clamped state, 3:update state)
int learning_mode; 
// plotting mode (0:none, 1:flex sensor, 2:encoder position)
int plot_mode; 

// const int numSprings = 1;
// byte pins[numSprings][5] = {{2, 3, 8, 9, A0}};
spring2 mySpring(2, 3, 8, 9, A0);

//function to plot sensor read out
void sensor_plot(float cflex) {
  Serial.print("d_min:");
  Serial.print(-30);
  Serial.print(",");
  Serial.print("d_max:");
  Serial.print(30);
  Serial.print(",");
  Serial.print("disp:");
  Serial.println(cflex);
}

void layer_plot(int input, int setpoint) {
  // for (int tableindex = 0; tableindex < arrLen; tableindex++){
  // }
  Serial.print("encpos_min:");
  Serial.print(0);
  Serial.print(",");
  Serial.print("encpos_max:");
  Serial.print(300);
  Serial.print(",");
  Serial.print("encoder_position:");
  Serial.print(input);
  Serial.print(",");
  Serial.print("set_point:");
  Serial.println(setpoint);
}

void setup() {
  //setup serial communication
  Serial.begin(9600);

  //setup Springs
  mySpring.init();

  learning_mode = 0;
  plot_mode = 0;
}

void loop() {
  // put your main code here, to run repeatedly:
  mySpring.onLoop();

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
      mySpring.mode_reset();
    }

    else if(initFlag == 85) { // key press "U" to go up by a half layer
        Serial.println("Go up by a half layer");
        mySpring.upLayer();
    } 

    else if (initFlag == 68) { //key press "D" to go down by a half layer
        Serial.println("Go down by a half layer");
        mySpring.downLayer();
    }

    else if (initFlag == 80) { //key press "P" to change plot mode like so: P 1 -> sets flex plot mode    
      // always turn off plotting before setting a new layer! It will mess up the encoder reading
      plot_mode = Serial.parseFloat(SKIP_ALL);
    }    
  }

  if (learning_mode == 0) {
    if ((plot_mode == 1) || (plot_mode == 2)) {
      Serial.print("Null state: ");
    }
    else {
      Serial.println("Null state");
    }
  }
  else if (learning_mode == 1) { // free state
    plot_mode=1;
    mySpring.takeReading(1);
  } 
  else if (learning_mode == 2) { // clamp state
    plot_mode=1;
    mySpring.takeReading(2);
  }
  else if (learning_mode == 3) { // update state
    Serial.print("Update state: ");
    plot_mode = 2;
    mySpring.doUpdate();
  }

  if (plot_mode == 1) {
    sensor_plot(mySpring.current_disp);
  }
  else if (plot_mode == 2) {
    layer_plot(mySpring.input, mySpring.setpoint);
  } 
}
