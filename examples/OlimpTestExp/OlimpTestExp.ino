#include "Func.h"

int Map[7]{0, 0, 0, 0, 0, 0, 0};

bool leftKube = false, rightKube = false, doorKube = false, underDoorKube = false, leftLastKube = false, rightLastKube = false;

void setup() {
  // put your setup code here, to run once:
  
  
  Init();
  servo(180, 0);

  delay(5000);
  
  LineDuga();
  delay(300);
  Enc(-50, -50, 40);
  delay(300);
  LinePerek(1, 50);
  Stop();

}

void loop() {
  SensorLineDebug();
  delay(500);
}

void yield(){
  encL.tick();
  encR.tick();
}

void moveDoorKube() {
  servo(0, 170);
  delay(500);
  EncReg(50, 100);
  Stop();
  delay(300);
  servo(160, 170);
  delay(500);
  servo(160, 0);
  Enc(-50, -50, 100);
  delay(300);
  TurnSensor(50, false);
  delay(300);
  LineEnc(150, 50);
  Stop();
  delay(300);
  servo(0, 0);
  Enc(-50, -50, 100);
  delay(300);
  TurnCircle(90, true);
  delay(300);
}

void moveLeftKube() {
  servo(0, 170);
  LineEnc(180, 50);
  Stop();
  servo(160, 170);
  delay(500);
  servo(160, 0);
  EncReg(50, 220);
  Stop();
  delay(300);
  if (scan(3)) {
    doorKube = true;
  }
  delay(300);
  Enc(-50, -50, 70);
  delay(300);
  TurnCircle(90, false);
  servo(160, 170);
  delay(300);
  EncReg(50, 420);
  Stop();
  servo(160, 0);
  delay(300);
  TurnCircle(90, false);
  delay(300);
  LineEnc(140, 50);
  Stop();
  servo(50, 0);
  delay(500);
  Enc(-50, -50, 150);
  delay(300);
  TurnCircle(90, true);
}