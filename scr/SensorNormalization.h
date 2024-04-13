#ifndef Sensor_Normalization_H
#define Sensor_Normalization_H

#include "Arduino.h"

class LineSens
{
public:

  LineSens(char InPin)
  {
    pin = InPin;
    pinMode(pin, INPUT);
  }

  void SetNorm(int MinS, int MaxS)
  {
    minS = MinS;
    maxS = MaxS;
  }

  int GetValue()
  {
    int S = ((analogRead(pin) - minS) * 100) / (maxS - minS);
    return 100 - S;
  }

private:

  char pin;
  float maxS;
  float minS;

};

