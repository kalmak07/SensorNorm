#include "Arduino.h"
class Enc {

private:

  int pinEnc;
  long value;
  bool flagEnc;

public:

  Enc() {}

  Enc(int PinEnc) {
    pinEnc = PinEnc;

    flagEnc = digitalRead(pinEnc);
    value = 0;
  }
  
  void tick()
  {
    if (flagEnc != digitalRead(pinEnc))
    {
      flagEnc = digitalRead(pinEnc);
      if (!flagEnc){
        value += 1;
      }
    }
  }

  long getSteps(){
    return value;
  }

  float getDegrees(){
    return ((360.0 / 500.0 ) * value);
  }

  void resetCount(){
    value = 0;
  }

};