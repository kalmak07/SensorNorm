#include "Arduino.h"
#include "Encoder.cpp"
#include "LineSens.cpp"
#include "Servo.h"

#define MotorRightPwm 3
#define MotorRightDir 12
#define MotorLeftPwm 11
#define MotorLeftDir 13

#define MotorDir 0

#define EncoderLeft 7
#define EncoderRight 8

#define BigServoPin 6
#define SmalServoPin 5

#define RightDist A3
#define FrontDist A4
#define LeftDist A5

Enc encL(EncoderLeft);
Enc encR(EncoderRight);

LineSens Lsens(A2);
LineSens Xsens(A1);
LineSens Rsens(A0);

Servo bigServo;
Servo smalServo;

void Init(){
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(RightDist, INPUT);
  pinMode(FrontDist, INPUT);
  pinMode(LeftDist, INPUT);
  pinMode(4, OUTPUT);
  pinMode(MotorRightPwm, OUTPUT);
  pinMode(MotorRightDir, OUTPUT);
  pinMode(MotorLeftPwm, OUTPUT);
  pinMode(MotorLeftDir, OUTPUT);
  bigServo.attach(6);
  smalServo.attach(5);
  Lsens.SetNorm(30, 230);
  Xsens.SetNorm(30, 160);
  Rsens.SetNorm(30, 380);
}

void SensorLineDebug(){
  Serial.print(Lsens.GetValue());
  Serial.print(" ");
  Serial.println(Rsens.GetValue());
}

void Drive(int pwmL = 0, int pwmR = 0){
  if (pwmL < 0){
    pwmL = -pwmL;
    digitalWrite(MotorLeftDir, MotorDir);
  } else {
    digitalWrite(MotorLeftDir, !MotorDir);
  }
  if (pwmR < 0) {
    pwmR = -pwmR;
    digitalWrite(MotorRightDir, !MotorDir);
  } else {
    digitalWrite(MotorRightDir, MotorDir);
  }
  analogWrite(MotorLeftPwm, (pwmL * 0.9));
  analogWrite(MotorRightPwm, (pwmR * 0.9));
}

float sharp(float V)
{
  float v = V / 200;
  float a = 30.5;
  float b = 2.7;
  float S = (a - v*b) / v;
  return S;
}

bool scan(int num){
  switch (num) {
    case 1:
    if (sharp(analogRead(LeftDist)) < 40) {
      return true;
    } else {
      return false;
    }
    break;

    case 2:
    if (sharp(analogRead(FrontDist)) < 40) {
      return true;
    } else {
      return false;
    }
    break;

    case 3:
    if (sharp(analogRead(RightDist)) < 40) {
      return true;
    } else {
      return false;
    }
    break;
  }
}

void SensorIkDebug(){
  Serial.print(sharp(analogRead(A3)));
  Serial.print(" ");
  Serial.print(sharp(analogRead(A4)));
  Serial.print(" ");
  Serial.println(sharp(analogRead(A5)));
}

int servo(int Smal, int Big){
  smalServo.write(Smal);
  bigServo.write(Big);
  //analogWrite(SmalServoPin, Smal);
  //analogWrite(BigServoPin, Big);
}

void Line(int v) {
  float Kp = 2.5, U, Err, ErrOld = 0, Kd = 5, flag = 0, i = 0;

  encL.resetCount();
  encR.resetCount();

  Drive(v, v);

  while ("True") {
    Err = Lsens.GetValue() - Rsens.GetValue();
    U = Err * Kp + (Err - ErrOld) * Kd;
    if (U > 80)
    {
      U = 80;
    }
    else if (U < -80)
    {
      U = -80;
    }
    ErrOld = Err;
    Drive(v + U, v - U);
    delay(10);
  }
  Drive(0, 0);
}

void TurnCircle(int degrees, bool left = 0) {
  float mkp = 210, Dk = 80, Vl, Vr;
  float S = (degrees * mkp) / Dk;

  if (left){
    Drive(-55, 55);
    Vl = 255;
    Vr = -255;
  } else {
    Drive(55, -55);
    Vl = -255;
    Vr = 255;
  }

  encL.resetCount();
  encR.resetCount();

  while (true) {
    encL.tick();
    encR.tick();

    if (((encL.getDegrees() + encR.getDegrees()) /2) > S){
      break;
    }
  }

  Drive(Vl, Vr);
  delay(10);
  Drive(0, 0);

}

void TurnSensor(int v, bool left){
  if (left) {
    Drive(-255, 255);
    delay(10);
    Drive(-v, v);
    while (Lsens.GetValue() > 70) {
      delay(1);
    }
    while (Xsens.GetValue() > 85) {
      delay(1);
    }
    //delay(40);
    Drive(255, -255);
    delay(10);
  } else {
    Drive(255, -255);
    delay(10);
    Drive(v, -v);
    while (Rsens.GetValue() > 70) {
      delay(1);
    }
    while (Xsens.GetValue() > 85) {
      delay(1);
    }
    delay(150);
    Drive(-255, 255);
    delay(10);
  }
  Drive(0, 0);
}

void LineEnc(int S, int v) {
  float Kp = 5, U, Err, ErrOld = 0, Kd = 20, flag = 0, i = 0, degrees = S / 0.7;

  encL.resetCount();
  encR.resetCount();

  Drive(v, v);

  while (((encL.getDegrees() + encR.getDegrees()) / 2) < degrees) {
    Err = Lsens.GetValue() - Xsens.GetValue();
    U = Err * Kp + (Err - ErrOld) * Kd;
    if (U > 80)
    {
      U = 80;
    }
    else if (U < -80)
    {
      U = -80;
    }
    ErrOld = Err;
    Drive(v + U, v - U);
    encL.tick();
    encR.tick();
    delay(10);
  }
  Drive(0, 0);
}

void LinePerek(int n, int v) {
  float Kp = 1.5, U, Err, ErrOld = 0, Kd = 20, flag = 10, i = 0;

  Drive(v, v);

  while (i < n) {
    flag += 1;
    if (Lsens.GetValue() < 70 and Rsens.GetValue() < 70 and flag > 5 or Lsens.GetValue() < 70 and Xsens.GetValue() < 70 and flag > 5 or Xsens.GetValue() < 70 and Rsens.GetValue() < 70 and flag > 5) {
      i += 1;
      flag = 0;
    }
    if (i > 0 and flag < 5) {
      digitalWrite(4, 1);
    } else {
      digitalWrite(4, 0);
    }
    Err = Lsens.GetValue() - Xsens.GetValue();
    U = Err * Kp + (Err - ErrOld) * Kd;
    if (U > 80)
    {
      U = 80;
    }
    else if (U < -80)
    {
      U = -80;
    }
    ErrOld = Err;
    Drive(v + U, v - U);
    delay(5);
  }
  digitalWrite(4, 0);
  LineEnc(55, 40);
}

void LineEncR(int S, int v) {
  float Kp = 5, U, Err, ErrOld = 0, Kd = 20, flag = 0, i = 0, degrees = S / 0.7;

  encL.resetCount();
  encR.resetCount();

  Drive(v, v);

  while (((encL.getDegrees() + encR.getDegrees()) / 2) < degrees) {
    Err = Xsens.GetValue() - Rsens.GetValue();
    U = Err * Kp + (Err - ErrOld) * Kd;
    if (U > 80)
    {
      U = 80;
    }
    else if (U < -80)
    {
      U = -80;
    }
    ErrOld = Err;
    Drive(v + U, v - U);
    encL.tick();
    encR.tick();
    delay(10);
  }
  Drive(0, 0);
}

void LinePerekR(int n, int v) {
  float Kp = 1.5, U, Err, ErrOld = 0, Kd = 20, flag = 10, i = 0;

  Drive(v, v);

  while (i < n) {
    flag += 1;
    if (Lsens.GetValue() < 70 and Rsens.GetValue() < 70 and flag > 5 or Lsens.GetValue() < 70 and Xsens.GetValue() < 70 and flag > 5 or Xsens.GetValue() < 70 and Rsens.GetValue() < 70 and flag > 5) {
      i += 1;
      flag = 0;
    }
    if (i > 0 and flag < 5) {
      digitalWrite(4, 1);
    } else {
      digitalWrite(4, 0);
    }
    Err = Xsens.GetValue() - Rsens.GetValue();
    U = Err * Kp + (Err - ErrOld) * Kd;
    if (U > 80)
    {
      U = 80;
    }
    else if (U < -80)
    {
      U = -80;
    }
    ErrOld = Err;
    Drive(v + U, v - U);
    delay(5);
  }
  digitalWrite(4, 0);
  LineEncR(70, 30);
}

void Stop(){
  Drive(-255, -255);
  delay(10);
  Drive(0, 0);
}

void InvertStop(){
  Drive(255, 255);
  delay(10);
  Drive(0, 0);
}

void Enc(int vL, int vR, int S){
  float degrees = S / 0.7;
  
  encL.resetCount();
  encR.resetCount();

  Drive(vL, vR);
  while (((encL.getDegrees() + encR.getDegrees()) / 2) < degrees) {
    delay(10);
  }

  int VL = 255;
  int VR = 255;

  if (vL > 0){
    VL = -VL;
  }
  if (vR > 0){
    VR = -VR;
  }
  
  Drive(VL, VR);
  delay(10);
  Drive(0, 0);
}

void EncReg(int v, int S){
  float degrees = S / 0.7, Kp = 1.5, U, Err, ErrOld = 0, Kd = 20;
  
  encL.resetCount();
  encR.resetCount();

  Drive(v, v);
  while (((encL.getDegrees() + encR.getDegrees()) / 2) < degrees) {
    Err = encR.getSteps() - encL.getSteps();
    U = Err * Kp + (Err - ErrOld) * Kd;
    if (U > 80)
    {
      U = 80;
    }
    else if (U < -80)
    {
      U = -80;
    }
    ErrOld = Err;
    Drive(v + U, v - U);
    delay(5);
  }
  Drive(0, 0);
}

LineCalibrate(bool Right){
  float Kp = 10, U, Err, ErrOld = 0, Kd = 35, flag = 0, i = 0;
  int Count = 0;
  
  if (Right){
    while (Count < 50) {
        Err = (Xsens.GetValue() + 4) - (Rsens.GetValue());
        U = Err * Kp + (Err - ErrOld) * Kd;
        if (U > 80)
        {
          U = 80;
        }
        else if (U < -80)
        {
          U = -80;
        }
        ErrOld = Err;
        Drive(U, -U);
        Count += 1;
        delay(10);
    }
  } else {
    while (Count < 50) {
      Err = Lsens.GetValue() - (Xsens.GetValue() + 4);
      U = Err * Kp + (Err - ErrOld) * Kd;
      if (U > 80)
      {
        U = 80;
      }
      else if (U < -80)
      {
        U = -80;
      }
      ErrOld = Err;
      Drive(U, -U);
      Count += 1;
      delay(10);
    }
  }
  Drive(0, 0);
}

void backToLine(int v) {
  Drive(-v, -v);
  delay(1000);
  while (Lsens.GetValue() > 40 and Rsens.GetValue() > 40) {
    delay(10);
  }
  InvertStop();
  delay(300);
  LineEnc(40, 60);
}

void LineDuga() {
  Enc(50, 0, 100);
  delay(300);
  Drive(50, 50);
  while (Lsens.GetValue() > 40) {
    delay(10);
  }
  Drive(0, 50);
  while (Xsens.GetValue() > 40) {
    delay(10);
  }
  Drive(-255, 0);
  delay(10);
  Drive(0, 0);
}