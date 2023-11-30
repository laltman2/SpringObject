#ifndef conversion_h
#define conversion_h

#include <Arduino.h>

class conversion{
  public:
  // convert flexsensor values to displacements
  float flex2disp(int flex);
  // convert displacements to flexsensor values (reverse)
  int disp2flex(float disp);
  // convert spring stiffness to number of layers
  float stif2layer(float k);
  // convert number of layers to  spring stiffness (reverse)
  float layer2stif(float layers);

  private:
  //float rest_length = -105.8; // equilibrium flex sensor value, converted to [mm]
  //float df_convert = -0.2; // displacement/flex value conversion, [mm]
  float rest_length = -95.65;
  float df_convert = 0.1487;
  float ls_convert = 137.9; // (half) layers/stiffness conversion, [mm/N]
  float firstlayer_k = 0.07; // stiffness of layer 1, [N/mm]

};

#endif
