//#include <MPU6050_light.h>
//#include "Wire.h"
#include "definitions.h"
#include "methods.h"
//#include "MPU9250.h"
//Pot & gripper pot pins
#define J1 A0
#define J2 A1
#define J3 A2
#define J4 A3
#define G A5
//Base Pins
int InA1 = 22;
int InB1 = 23;
int PWM1 = 12;
//Shoulder Pins
int InA2 = 24;
int InB2 = 25;
int PWM2 = 11;
//Elbow Pins
int InB3 = 26;
int InA3 = 27;
int PWM3 = 10;
//Wrist Rotation Pins
int InB4 = 28;
int InA4 = 29;
int PWM4 = 9;
//Wrist Pitch Pins
int InB5 = 30;
int InA5 = 31;
int PWM5 = 8;
//Gripper Pins
int InB6 = 32;
int InA6 = 33;
int PWM6 = 7;
//Solenoid Pin (40)
int Sole = 53;
bool SoleState = false;
char input;
float Q1 = 0, Q1D = 0 * PI / 180, Q1_old, U1 = 5, V1 = 0;
float Q2 = 0, Q2D = 60 * PI / 180, Q2_old, U2 = 5, V2 = 0;
float Q3 = 0, Q3D = -30 * PI / 180, Q3_old, U3 = 0, V3 = 0;
float Q4 = 0, Q4D = 0 * PI / 180, Q4_old, U4 = 0, V4 = 0;
float Q5 = 0, Q5D = 0 * PI / 180, Q5_old, U5 = 0, V5 = 0;
bool J1_done = true, J2_done = true, J3_done = true, J4_done = true, J5_done = true;
double T = 0, T_old = 0, T_delta;
int i = 0;
float Restingtilt = 10;
float tilt = 0;
float x, y, z, Rx, Ry, Rz, FingerAngle, Mass;
bool moveState = false;
bool ShowPrints = false;
//MPU6050 mpu(Wire);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  /*
    Wire.begin();

    byte status = mpu.begin();
    if(ShowPrints){
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    }
    while(status!=0){ } // stop everything if could not connect to MPU6050

    delay(1000);
    // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
    mpu.calcOffsets(); // gyro and accelero
    //Serial.println("Done!\n");
  */
  pinMode(InA1, OUTPUT);
  pinMode(InB1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(InA2, OUTPUT);
  pinMode(InB2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(InA3, OUTPUT);
  pinMode(InB3, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(InA5, OUTPUT);
  pinMode(InB5, OUTPUT);
  pinMode(PWM5, OUTPUT);
  pinMode(Sole, OUTPUT);
  getCurrentAngles(&Q1, &Q2, &Q3, &Q4, &Q5, &FingerAngle);
  joint2point(Q1, Q2, Q3, Q4, Q5, &x, &y, &z);
  Q1D = Q1;
  Q2D = Q2;
  Q3D = Q3;
  Q4D = Q4;
  Q5D = Q5;

  Rx = x * cos(tilt * PI / 180) - y * sin(tilt * PI / 180);
  Ry = -x * sin(tilt * PI / 180) + y * cos(tilt * PI / 180);
  Rz = z;

  if (ShowPrints) Serial.print("X: ");
  Serial.print(Rx);
  Serial.print(",");
  if (ShowPrints) Serial.print("Y: ");
  Serial.print(Ry);
  Serial.print(",");
  if (ShowPrints) Serial.print("Z: ");
  Serial.print(Rz);
  Serial.print(",");
  Serial.print(Q4 * 180 / PI);
  Serial.print(",");
  Serial.print(Q5 * 180 / PI);
  Serial.print(",");
  Serial.print(FingerAngle);
  Serial.print(",");
  Serial.print(0);
  Serial.print(",");
  Serial.print(0);
  Serial.println();
}

void loop() {
  //mpu.update();
  // put your main code here, to run repeatedly:

  if (millis() >  T * 1000 + 100) {
    //tilt = Restingtilt; //Restingtilt+mpu.getAngleX()%1;
    T = millis() / 1000.0;
    T_delta = (T - T_old);
    T_old = T;

    U1 = (Q1 - Q1_old) / T_delta;
    U2 = (Q2 - Q2_old) / T_delta;
    U3 = (Q3 - Q3_old) / T_delta;
    U4 = (Q4 - Q4_old) / T_delta;
    U5 = (Q5 - Q5_old) / T_delta;
    Q1_old = Q1;
    Q2_old = Q2;
    Q3_old = Q3;
    Q4_old = Q4;
    Q5_old = Q5;
    //if (!(isnan(Q1D) && isnan(Q2D) && isnan(Q3D) && isnan(Q5D)) && !(Q1D == 1000 && Q2D == 1000 && Q3D == 1000 && Q5D==1000)) {
    //if (true/*J1_done && J2_done && J3_done && J4_done*/) {
    //if (moveState) {
    // if(ShowPrints) Serial.println("Ready to move!");
    // moveState = !moveState;
    //}
    //Serial.print("Checkitout");
    if ( Serial.available() > 0 ) {
      /*input=Serial.read();
        if(input=='r'){
        return2Rest();
            moveJ4(-20 * PI / 180, Q4, U4);

        }else if(input=='c'){*/
      cartesianCommands(&Q1D, &Q2D, &Q3D, &Q4D, &Q5D, &FingerAngle, &SoleState, &Mass);
      if (ShowPrints) {
        Serial.print("Q1D: ");
        Serial.print(Q1D * 180 / PI);
        Serial.print(", Q2D: ");
        Serial.print(Q2D * 180 / PI);
        Serial.print(", Q3D: ");
        Serial.print(Q3D * 180 / PI);
        Serial.print(", Q4D: ");
        Serial.print(Q4D * 180 / PI);
        Serial.print(", Q5D: ");
        Serial.print(Q5D * 180 / PI);
        Serial.println();
      }
    }
    //} else {
    //  moveState = true;
    //}
    getCurrentAngles(&Q1, &Q2, &Q3, &Q4, &Q5, &FingerAngle);

    if (isPositionPossible(Q1D, Q2D, Q3D, Q4D, Q5D, FingerAngle)) {
      J1_done = moveJ1(Q1D, Q1, U1);
      J2_done = moveJ2(Q2D, Q2, U2);
      J3_done = moveJ3(Q3D, Q3, U3);
      J4_done = moveJ4(Q4D, Q4, U4);
      //J5_done = moveJ5(0*PI/180, Q5, U5);
      //moveG(30,FingerAngle);
    }

  }

}
/*
  void readCommand(float *Q1D, float *Q2D, float *Q3D, float *Q4D, float *Q5D, float *FingerAngle, bool *SoleState, float *Mass) {
  char D = ' ';
  String dataIn = "";
  float x_delta, y_delta, z_delta;
  float Command1, Command2, Command3, Command4, Command5, Command6, Command7, Command8;
  while (Serial.available() > 0) {
    D = Serial.read();
    if (D == 'c' || D == 'j') { // c is for cartesianal, j for joint by joint
      Command1 = Serial.parseFloat();
      Command2 = Serial.parseFloat();
      Command3 = Serial.parseFloat();
      Command4 = Serial.parseFloat();
      Command5 = Serial.parseFloat();
      Command6 = Serial.parseFloat();
      Command7 = Serial.parseFloat();
      Command8 = Serial.parseFloat();

      if (D == 'c') {
        x_delta = Command1;
        y_delta = Command2;
        z_delta = Command3;
         Q4D = Command4; // pitch
         Q5D = Command5; // rotation
         FingerAngle = Command6;
         SoleState = (bool)Command7;
         Mass = Command8;
        x = x + x_delta; //in Meters
        y = y + y_delta;
        z = z + z_delta;

        point2joint(x, y, z, Command4, Command5, Q1D, Q2D, Q3D);
      } else if (D == 'j') {
         Q1D = Command1 * PI / 180.0;
         Q2D = Command2 * PI / 180.0;
         Q3D = Command3 * PI / 180.0;
         Q4D = Command4 * PI / 180.0;
         Q5D = Command5 * PI / 180.0;
         FingerAngle = Command6;
         SoleState = (boolean) Command7;
         Mass = Command8;
      }
    } else {
      dataIn += D;
    }
  }
  }*/
void cartesianCommands(float *Q1D, float *Q2D, float *Q3D, float *Q4D, float *Q5D, float *FingerAngle, bool *SoleState, float *Mass) {
  float x_delta, y_delta, z_delta;
  float Command1, Command2, Command3, Command4, Command5, Command6, Command7, Command8;

  if ( Serial.available() > 0 ) {
    Command1 = Serial.parseFloat();
    Command2 = Serial.parseFloat();
    Command3 = Serial.parseFloat();
    Command4 = Serial.parseFloat();
    Command5 = Serial.parseFloat();
    Command6 = Serial.parseFloat();
    Command7 = Serial.parseFloat();
    Command8 = Serial.parseFloat();
    //Serial.println("GRABBED VALUES");
    if (moveState) {
      Serial.print(Command1);
      Serial.print(" ");
      Serial.print(Command2);
      Serial.print(" ");
      Serial.print(Command3);
      Serial.print(" ");
      Serial.print(Command4);
      Serial.print(" ");
      Serial.print(Command5);
      Serial.print(" ");
      Serial.print(Command6);
      Serial.print(" ");
      Serial.print(Command7);
      Serial.print(" ");
      Serial.print(Command8);
      Serial.println();
    }
    if (Command1 == 114.0 && Command2 == 114.0 && Command3 == 114.0 && Command4 == 114.0 && Command5 == 114.0 && Command6 == 114.0 && Command7 == 114.0 && Command8 == 114.0) {
      return2Rest();
    } else if (Command1 == 100.0 && Command2 == 100.0 && Command3 == 100.0 && Command4 == 100.0 && Command5 == 100.0 && Command6 == 100.0 && Command7 == 100.0 && Command8 == 100.0) {
      deployArm();
    } else {
      x_delta = Command1;
      y_delta = Command2;
      z_delta = Command3;
      *FingerAngle = Command6;
      *SoleState = (bool)Command7;
      *Mass = Command8;
      x = x + x_delta / 39.3701; //in Meters
      y = y + y_delta / 39.3701;
      z = z + z_delta / 39.3701;

      //Rotation Matrix for tilt
      Rx = x * cos(tilt * PI / 180) - y * sin(tilt * PI / 180);
      Ry = -x * sin(tilt * PI / 180) + y * cos(tilt * PI / 180);
      Rz = z;
      if(ShowPrints) {
        Serial.print(x);
        Serial.print(" ");
        Serial.print(y);
        Serial.print(" ");
        Serial.print(z);
        Serial.println();
        }
        
      point2jointV3(Rx, Ry, Rz, (Command4 - tilt) * PI / 180, Command5 * PI / 180, Q1D, Q2D, Q3D, Q4D, Q5D);

      if (Command5 == 1) { //if statements for J5/Wrist Rotation
        if (moveState) Serial.println("J5 Left");
        digitalWrite(InA5, LOW);
        digitalWrite(InB5, HIGH);
        analogWrite(PWM5, 50);
        delay(500);
        digitalWrite(InA5, LOW);
        digitalWrite(InB5, LOW);
        analogWrite(PWM5, 0);
        Command5 == 0;
      } else if (Command5 == -1) {
        if (moveState) Serial.println("J5 Right");
        digitalWrite(InA5, HIGH);
        digitalWrite(InB5, LOW);
        analogWrite(PWM5, 50);
        delay(500);
        digitalWrite(InA5, LOW);
        digitalWrite(InB5, LOW);
        analogWrite(PWM5, 0);
        Command5 == 0;
      } else if (Command5 == 0) {
        if (moveState) Serial.println("J5 STOP");
        digitalWrite(InA5, LOW);
        digitalWrite(InB5, LOW);
        analogWrite(PWM5, 0);
      }

      if (Command6 == -1) { //if statements for Gripper
        if (moveState) Serial.println("Gripper Close");
        digitalWrite(InA6, HIGH);
        digitalWrite(InB6, LOW);
        analogWrite(PWM6, 255);
        delay(1000);
        digitalWrite(InA6, LOW);
        digitalWrite(InB6, LOW);
        analogWrite(PWM6, 0);
        Command6 == 0;
      } else if (Command6 == 1) {
        if (moveState) Serial.println("Gripper Open");
        digitalWrite(InA6, LOW);
        digitalWrite(InB6, HIGH);
        analogWrite(PWM6, 255);
        delay(1000);
        digitalWrite(InA6, LOW);
        digitalWrite(InB6, LOW);
        analogWrite(PWM6, 0);
        Command6 == 0;
      } else if (Command6 == 0) {
        if (moveState) Serial.println("Gripper STOP");
        digitalWrite(InA6, LOW);
        digitalWrite(InB6, LOW);
        analogWrite(PWM6, 0);
      }

    }
  }

}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void getCurrentAngles (float *q1, float *q2, float *q3, float *q4, float *q5, float *FingerAngle) {
  //Pot & gripper analog values
  int Base = analogRead(J1);
  int Shoulder = analogRead(J2);
  int Elbow = analogRead(J3);
  int Pitch = analogRead(J4);
  int Gripper = analogRead(G);
  //Mapped values of pot to radians
  *q1 = ((float)(map(Base, 0, 1024, 0, 3300) / 10.0) - 184.9) * PI / 180;
  *q2 = ((330 - (float)((map(Shoulder, 0, 1024, 0, 3300) / 10.0 ))) - 173.5) * PI / 180;
  *q3 = ((float)(map(Elbow, 0, 1024, 0, 3300) / 10.0) - 143.7) * PI / 180;
  *q4 = ((float)(map(Pitch, 0, 1024, 0, 3300) / 10.0) - 166.6) * PI / 180;
  //*q5 = (float)(map(Rotation, 0, 1024, 0, 3300) / 10.0 - 149.0) * PI / 180;
  *FingerAngle = -(map(Gripper, 0, 1024, 0, 330) - 171);
}

bool moveJ1(float Q1D, float Q1, float U1) {
  bool areWeThereYet = false;
  float TAU = TAU1(Q1, Q2, Q3, Q4, Q5, U1, U2, U3, U4, U5, Q1D, Q2D, Q3D, Q4D, Q5D);
  float V1 = (TAU / kt) * R + ki * U1;
  if (abs(Q1D - Q1) < 1 * PI / 180 ) {
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, LOW);
    areWeThereYet = true;
  } else if (Q1D - Q1 < 0) {
    //Left
    digitalWrite(InA1, HIGH);
    digitalWrite(InB1, LOW);
    areWeThereYet = false;
  } else {
    //Right
    digitalWrite(InA1, LOW);
    digitalWrite(InB1, HIGH);
    areWeThereYet = false;
  }
  V1 = abs(V1);
  V1 = constrain(V1, 0, 12);
  V1 = map(V1, 0, 12, 0, 40);

  analogWrite(PWM1, V1);
  return areWeThereYet;
}

bool moveJ2(float Q2D, float Q2, float U2) {
  bool areWeThereYet = false;
  float TAU = TAU2(Q1, Q2, Q3, Q4, Q5, U1, U2, U3, U4, U5, Q1D, Q2D, Q3D, Q4D, Q5D);
  float V2 = (TAU / kt) * R + ki * U2;
  if (abs(Q2D - Q2) < 0.5 * PI / 180 ) {
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, LOW);
    areWeThereYet = true;
  } else if (Q2D - Q2 < 0) {
    //Down
    digitalWrite(InA2, LOW);
    digitalWrite(InB2, HIGH);
    areWeThereYet = false;
  } else {
    //Up
    digitalWrite(InA2, HIGH);
    digitalWrite(InB2, LOW);
    areWeThereYet = false;
  }
  V2 = abs(V2);
  V2 = constrain(V2, 0, 12);
  V2 = map(V2, 0, 12, 0, 255);

  analogWrite(PWM2, V2);
  return areWeThereYet;
}
bool moveJ3(float Q3D, float Q3, float U3) {
  bool areWeThereYet = false;
  float TAU = TAU3(Q1, Q2, Q3, Q4, Q5, U1, U2, U3, U4, U5, Q1D, Q2D, Q3D, Q4D, Q5D); //Constants are gear ratios
  float V3 = (TAU / kt) * R + ki * U3;
  if (abs(Q3D - Q3) < 0.5 * PI / 180 ) {
    digitalWrite(InA3, LOW);
    digitalWrite(InB3, LOW);
    areWeThereYet = true;
  } else if (Q3D - Q3 < 0) {
    //down
    digitalWrite(InA3, HIGH);
    digitalWrite(InB3, LOW);
    areWeThereYet = false;
  } else {
    //up
    digitalWrite(InA3, LOW);
    digitalWrite(InB3, HIGH);
    areWeThereYet = false;
  }
  V3 = abs(V3);
  V3 = constrain(V3, 0, 12);
  V3 = map(V3, 0, 12, 0, 255);

  analogWrite(PWM3, 0.25 * V3);
  return areWeThereYet;
}
bool moveJ4(float Q4D, float Q4, float U4) {
  bool areWeThereYet = false;
  float TAU = TAU4(Q1, Q2, Q3, Q4, Q5, U1, U2, U3, U4, U5, Q1D, Q2D, Q3D, Q4D, Q5D);
  float V4 = (TAU / kt) * R + ki * U4;
  if (abs(Q4D - Q4) < 0.5 * PI / 180 ) {
    digitalWrite(InA4, LOW);
    digitalWrite(InB4, LOW);
    areWeThereYet = true;
  } else if (Q4D - Q4 < 0) {
    //Left
    digitalWrite(InA4, LOW);
    digitalWrite(InB4, HIGH);
    areWeThereYet = false;
  } else {
    //Right
    digitalWrite(InA4, HIGH);
    digitalWrite(InB4, LOW);
    areWeThereYet = false;
  }
  V4 = abs(V4);
  V4 = constrain(V4, 0, 12);
  V4 = map(V4, 0, 12, 0, 255);

  analogWrite(PWM4, V4);
  return areWeThereYet;
}
bool moveJ5(float Q5D, float Q5, float U5) {
  bool areWeThereYet = false;
  float TAU = TAU5(Q1, Q2, Q3, Q4, Q5, U1, U2, U3, U4, U5, Q1D, Q2D, Q3D, Q4D, Q5D);
  float V5 = (TAU / kt) * R + ki * U5;
  if (abs(Q5D - Q5) < 0.5 ) {
    digitalWrite(InA5, LOW);
    digitalWrite(InB5, LOW);
    areWeThereYet = true;
  } else if (Q5D - Q5 < 0) {
    //Left
    digitalWrite(InA5, HIGH);
    digitalWrite(InB5, LOW);
    areWeThereYet = false;
  } else {
    //Right
    digitalWrite(InA5, LOW);
    digitalWrite(InB5, HIGH);
    areWeThereYet = false;
  }
  V5 = abs(V5);
  V5 = constrain(V5, 0, 12);
  V5 = map(V5, 0, 12, 0, 255);

  analogWrite(PWM5, V5);
  return areWeThereYet;
}

bool moveG(float QGD, float QG) {
  bool areWeThereYet = false;

  float TAU = KPgripper * (QGD - QG);
  float V1 = (TAU / kt) * R + ki * U1;
  if (abs(QGD - QG) < 1 ) {
    digitalWrite(InA6, LOW);
    digitalWrite(InB6, LOW);
    areWeThereYet = true;
  } else if (Q1D - Q1 < 0) {
    //Left
    digitalWrite(InA6, HIGH);
    digitalWrite(InB6, LOW);
    areWeThereYet = false;
  } else {
    //Right
    digitalWrite(InA6, LOW);
    digitalWrite(InB6, HIGH);
    areWeThereYet = false;
  }
  V1 = abs(V1);
  V1 = constrain(V1, 0, 12);
  V1 = map(V1, 0, 12, 0, 255);

  analogWrite(PWM6, V1);
  return areWeThereYet;
}

void return2Rest() {
  Q1D = 0 * PI / 180;
  Q2D = 150 * PI / 180;
  Q3D = -135 * PI / 180;
  Q4D = -40 * PI / 180;
  joint2point(Q1D, Q2D, Q3D, Q4D, Q5D, &x, &y, &z);
  /*while (!(moveJ2(80 * PI / 180, Q2, U2) && moveJ3(-60 * PI / 180, Q3, U3))) {

    }
    while (!(moveJ4(-30 * PI / 180, Q4, U4) && moveJ1(0 * PI / 180, Q1, U1))) {

    }*/
}

void deployArm() {
  Q1D = 0 * PI / 180;
  Q2D = 60 * PI / 180;
  Q3D = -45 * PI / 180;
  Q4D = -20 * PI / 180;
  joint2point(Q1D, Q2D, Q3D, Q4D, Q5D, &x, &y, &z);
  /*while (!(moveJ2(80 * PI / 180, Q2, U2) && moveJ3(-60 * PI / 180, Q3, U3))) {

    }
    while (!(moveJ4(-30 * PI / 180, Q4, U4) && moveJ1(0 * PI / 180, Q1, U1))) {

    }*/
}

bool isPositionPossible(float Q1D, float Q2D, float Q3D, float Q4D, float Q5D, float FingerAngle) {
  if (isnan(Q1D) || isnan(Q2D) || isnan(Q3D) || isnan(Q4D) || isnan(Q5D)) {
    //Serial.println("NAN desired angle detected...");
    return false;
  }
  if (Q1D > (90 * PI / 180) || Q1D < (-90 * PI / 180)) {
    if (ShowPrints) Serial.println("Q1D out of Range");
    return false;
  }
  if (Q2D > (158 * PI / 180) || Q2D < (0 * PI / 180)) {
    if (ShowPrints) Serial.println("Q2D out of Range");
    return false;
  }
  if (Q3D > (130 * PI / 180) || Q3D < (-135 * PI / 180)) {
    if (ShowPrints) Serial.println("Q3D out of Range");
    return false;
  }
  if (Q4D > (100 * PI / 180) || Q4D < (-100 * PI / 180)) {
    if (ShowPrints) Serial.println("Q4D out of Range");
    return false;
  }
  /*if (FingerAngle > (60*PI/180) && Q3D < (0*PI/180)){
    Serial.println("FingerAngle out of Range");
    return false;
    }*/
  float XD, YD, ZD;
  joint2point(Q1D, Q2D, Q3D, Q4D, Q5D, &XD, &YD, &ZD);
  if (sqrt(pow(XD, 2) + pow(YD - La, 2) + pow(ZD, 2)) > (Lb + Lc + Ld)) {
    if(ShowPrints){
      Serial.println("Out of workspace");
    Serial.print("Current: ");
    Serial.println(sqrt(pow(XD,2) + pow(YD-La,2)+ pow(ZD,2)));
    Serial.print("Max: ");
    Serial.println((Lb+Lc+Ld));
    }
    return false;
  }
  if (sqrt(pow(XD, 2) + pow(YD, 2) + pow(ZD, 2)) < 0.10 * (Lb + Lc + Ld)) {
    if(ShowPrints) Serial.println("Out of workspace");
    return false;
  }
  return true;
}
