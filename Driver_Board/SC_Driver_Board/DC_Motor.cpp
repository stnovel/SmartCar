//=========================================================================
#include "Arduino.h"
#include "DC_Motor.h"

#include <Wire.h>

//=====================================================================
//M1左前輪
#define M1IN1 4
#define M1IN2 5
//M2右前輪
#define M2IN1 6
#define M2IN2 7
//M3右後輪
#define M3IN1 44
#define M3IN2 45
//M4左後輪
#define M4IN1 8
#define M4IN2 9
//=====================================================================
//Define M1,M2,M3,M4 Interrupt Pin
#define ENCODER_M1   19
#define ENCODER_M2   18
#define ENCODER_M3    2
#define ENCODER_M4    3
//=====================================================================
// These let us convert ticks-to-RPM
//#define GEARING     168
//#define ENCODERMULT 11
unsigned short GEARING = 34;
float ENCODERMULT = 11.00;

//=====================================================================
//For Interrup use
uint32_t M1_RPM = 0;
uint32_t M2_RPM = 0;
uint32_t M3_RPM = 0;
uint32_t M4_RPM = 0;

volatile float rev1=0; 
volatile float rev2=0;
volatile float rev3=0;
volatile float rev4=0;

volatile uint32_t lastM1 = 0;
volatile uint32_t lastM2 = 0;
volatile uint32_t lastM3 = 0;
volatile uint32_t lastM4 = 0;

//Wheel Encoder Count : one way encoder counter because use one pin to listen the interrup
//unsigned long int ==> 0 ~ 4,294,967,295
//signed long int ==> -2,147,483,648 ~ 2,147,483,647
volatile signed long int Encoder_1 = 0;
volatile signed long int Encoder_2 = 0;
volatile signed long int Encoder_3 = 0;
volatile signed long int Encoder_4 = 0;

volatile unsigned short Wheel_Directory_1;
volatile unsigned short Wheel_Directory_2;
volatile unsigned short Wheel_Directory_3;
volatile unsigned short Wheel_Directory_4;
//======================================================================
#include "PID.h"

PID pPID;

//======================================================================
DC_MOTOR::DC_MOTOR() {}
//======================================================================
DC_MOTOR::~DC_MOTOR() {}
//======================================================================
//Interrupt
//======================================================================
static DC_MOTOR::InterruptM1()
{
  uint32_t currA = micros();
  if (lastM1 < currA)
  {
    rev1 = currA - lastM1;
    
    if (Wheel_Directory_1 == 1)
    {
      if (Encoder_1 >= 2147483646)
      {
        Encoder_1 = 0;
      }
      else
      {
        Encoder_1 ++;
      }
    }
    else if (Wheel_Directory_1 == 2)
    {
      if (Encoder_1 <= -2147483647)
      {
        Encoder_1 = 0;
      }
      else
      {
        Encoder_1 --;
      }
    }
  }
  lastM1 = currA;
}
//======================================================================
static DC_MOTOR::InterruptM2()
{
  uint32_t currA = micros();
  if (lastM2 < currA)
  {
    rev2 = currA - lastM2;
    
    if (Wheel_Directory_2 == 1)
    {
      if (Encoder_2 >= 2147483646)
      {
        Encoder_2 = 0;
      }
      else
      {
        Encoder_2 ++;
      }
    }
    else if (Wheel_Directory_2 == 2)
    {
      if (Encoder_2 <= -2147483647)
      {
        Encoder_2 = 0;
      }
      else
      {
        Encoder_2 --;
      }
    }
  }
  lastM2 = currA;
}
//======================================================================
static DC_MOTOR::InterruptM3()
{
  uint32_t currA = micros();
  if (lastM3 < currA)
  {
    rev3 = currA - lastM3;
    
    if (Wheel_Directory_3 == 1)
    {
      if (Encoder_3 >= 2147483646)
      {
        Encoder_3 = 0;
      }
      else
      {
        Encoder_3 ++;
      }
    }
    else if (Wheel_Directory_3 == 2)
    {
      if (Encoder_3 <= -2147483647)
      {
        Encoder_3 = 0;
      }
      else
      {
        Encoder_3 --;
      }
    }
  }
  lastM3 = currA;
}
//======================================================================
static DC_MOTOR::InterruptM4()
{
  uint32_t currA = micros();
  if (lastM4 < currA)
  {
    rev4 = currA - lastM4;

    if (Wheel_Directory_4 == 1)
    {
      if (Encoder_4 >= 2147483646)
      {
        Encoder_4 = 0;
      }
      else
      {
        Encoder_4 ++;
      }
    }
    else if (Wheel_Directory_4 == 2)
    {
      if (Encoder_4 <= -2147483647)
      {
        Encoder_4 = 0;
      }
      else
      {
        Encoder_4 --;
      }
    }
  }
  lastM4 = currA;
}
//======================================================================
uint16_t DC_MOTOR::GetRPM_Interrupt(unsigned short wheel)
{
    // did not wrap around
    float rev = 0;  // us
    
    switch (wheel)
    {
      case 1:
        rev = rev1; 
        rev1 = 0;
        break;   
      case 2:
        rev = rev2;
        rev2 = 0; 
        break;  
      case 3:
        rev = rev3; 
        rev3 = 0;
        break;  
      case 4:
        rev = rev4;
        rev4 = 0; 
        break;                       
    }       
    rev = 1.0 / rev;            // rev per us
    rev *= 1000000;             // rev per sec
    rev *= 60;                  // rev per min
    rev /= GEARING;             // account for gear ratio =>168
    rev /= ENCODERMULT;         // account for multiple ticks per rotation =>11 (20.63) plus

    //Output RPM
    return (uint16_t)rev;      
}
//======================================================================
void DC_MOTOR::Display_RPM()
{
  M1_RPM = GetRPM_Interrupt(1);
  M2_RPM = GetRPM_Interrupt(2);
  M3_RPM = GetRPM_Interrupt(3);
  M4_RPM = GetRPM_Interrupt(4);
}
//======================================================================
int DC_MOTOR::GetRPM(unsigned short wheel)
{
  switch (wheel)
  {
    case 0:
      break;
    case 1:
      return M1_RPM;
      break;
    case 2:
      return M2_RPM;
      break;
    case 3:
      return M3_RPM;
      break;
    case 4:
      return M4_RPM;
      break;
  }
}
//======================================================================
//======================================================================
void DC_MOTOR::Initinal()
{
  //PWM 控制引腳位置
  pinMode(M1IN1,OUTPUT);
  pinMode(M1IN2,OUTPUT);
  pinMode(M2IN1,OUTPUT);
  pinMode(M2IN2,OUTPUT);
  pinMode(M3IN1,OUTPUT);
  pinMode(M3IN2,OUTPUT);
  pinMode(M4IN1,OUTPUT);
  pinMode(M4IN2,OUTPUT);
  
  pinMode(ENCODER_M1, INPUT_PULLUP);
  pinMode(ENCODER_M2, INPUT_PULLUP);
  pinMode(ENCODER_M3, INPUT_PULLUP);
  pinMode(ENCODER_M4, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER_M1), InterruptM1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_M2), InterruptM2, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_M3), InterruptM3, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_M4), InterruptM4, RISING);

  delay(100);

  //Wheels PID setup M1 ~ M4
  pPID.Set_PID(1, 0, -255, 255, 0.09, 0.5, 0.01);
  pPID.Set_PID(2, 0, -255, 255, 0.09, 0.5, 0.01);
  pPID.Set_PID(3, 0, -255, 255, 0.09, 0.5, 0.01);
  pPID.Set_PID(4, 0, -255, 255, 0.09, 0.5, 0.01);
}
//======================================================================
//Speed come from (0 ~ 255 * 16) ==> 0 ~ 4080
//馬達停止 LOW => 0 ( Stop )  HIGH => 4096 ( Brake )
//ie : Wheel_M1(1, 100);  ==> Forward
//     Wheel_M1(2, 120);  ==> Backward
//     Wheel_M1(0, 0);    ==> Stop
//     Wheel_M1(3, 0);    ==> Brake
//======================================================================
void DC_MOTOR::Wheel_M1(unsigned short turn, int sp)
{
  Wheel_Directory_1 = turn;

  if (turn == 0)
  {
    //Stop 停止 左前輪 M1
    analogWrite(M1IN1, 0);          
    analogWrite(M1IN2, 0);
  }
  else if (turn == 1)
  {
    //Forward  正轉 左前輪 M1
    analogWrite(M1IN1, sp);           
    analogWrite(M1IN2, 0);
  }
  else if (turn == 2)
  {
    //Backward 反轉 左前輪 M1
    analogWrite(M1IN1, 0);         
    analogWrite(M1IN2, sp);    
  }
  else if (turn == 3)
  {
    //Brake 停止 左前輪 M1
    analogWrite(M1IN1, 255);          
    analogWrite(M1IN2, 255);
  }
}
//======================================================================
void DC_MOTOR::Wheel_M2(unsigned short turn, int sp)
{
  Wheel_Directory_2 = turn;

  if (turn == 0)
  {
    //Stop 停止 左前輪 M2
    analogWrite(M2IN1, 0);          
    analogWrite(M2IN2, 0);
  }
  else if (turn == 1)
  {
    //Forward  正轉 左前輪 M2
    analogWrite(M2IN1, sp);           
    analogWrite(M2IN2, 0);
  }
  else if (turn == 2)
  {
    //Backward 反轉 左前輪 M2
    analogWrite(M2IN1, 0);         
    analogWrite(M2IN2, sp);    
  }
  else if (turn == 3)
  {
    //Brake 停止 左前輪 M2
    analogWrite(M2IN1, 255);          
    analogWrite(M2IN2, 255);
  }
}
//======================================================================
void DC_MOTOR::Wheel_M3(unsigned short turn, int sp)
{
  Wheel_Directory_3 = turn;

  if (turn == 0)
  {
    //Stop 停止 左前輪 M3
    analogWrite(M3IN1, 0);          
    analogWrite(M3IN2, 0);
  }
  else if (turn == 1)
  {
    //Forward  正轉 左前輪 M3
    analogWrite(M3IN1, sp);           
    analogWrite(M3IN2, 0);
  }
  else if (turn == 2)
  {
    //Backward 反轉 左前輪 M3
    analogWrite(M3IN1, 0);         
    analogWrite(M3IN2, sp);    
  }
  else if (turn == 3)
  {
    //Brake 停止 左前輪 M3
    analogWrite(M3IN1, 255);          
    analogWrite(M3IN2, 255);
  }
}
//======================================================================
void DC_MOTOR::Wheel_M4(unsigned short turn, int sp)
{
  Wheel_Directory_4 = turn;
  
  if (turn == 0)
  {
    //Stop 停止 左前輪 M4
    analogWrite(M4IN1, 0);          
    analogWrite(M4IN2, 0);
  }
  else if (turn == 1)
  {
    //Forward  正轉 左前輪 M4
    analogWrite(M4IN1, sp);           
    analogWrite(M4IN2, 0);
  }
  else if (turn == 2)
  {
    //Backward 反轉 左前輪 M4
    analogWrite(M4IN1, 0);         
    analogWrite(M4IN2, sp);    
  }
  else if (turn == 3)
  {
    //Brake 停止 左前輪 M4
    analogWrite(M4IN1, 255);          
    analogWrite(M4IN2, 255);
  }
}
//======================================================================
// Wheel Move Mode [Stop, Forward, Backward ....]
//======================================================================
void DC_MOTOR::Wheel_Move_Mode(unsigned short dir, int sp)
{
  switch (dir)
  {
    case 0:  //Stop
      Wheel_M1(0, 0);
      Wheel_M2(0, 0);
      Wheel_M3(0, 0);
      Wheel_M4(0, 0);
      break;
    case 1:  //Forward
      Wheel_M1(1, sp);
      Wheel_M2(1, sp);
      Wheel_M3(1, sp);
      Wheel_M4(1, sp);
      break;
    case 2:  //Backward
      Wheel_M1(2, sp);
      Wheel_M2(2, sp);
      Wheel_M3(2, sp);
      Wheel_M4(2, sp);
      break;
    case 11:  //LeftRevole //馬達左旋
      //M1  M2
      //M4  M3
      Wheel_M1(1, sp);
      Wheel_M2(2, sp);
      Wheel_M3(2, sp);
      Wheel_M4(1, sp);
      break;
    case 12:  //RightRevole //馬達右旋
      Wheel_M1(2, sp);
      Wheel_M2(1, sp);
      Wheel_M3(1, sp);
      Wheel_M4(2, sp);
      break;
  }
}
//======================================================================
// Wheel PID Control Move Mode [Stop, Forward, Backward ....]
//======================================================================
void DC_MOTOR::Wheel_PID_Move_Mode(unsigned short dir, int sp)
{
  switch (dir)
  {
    case 0:  //Stop
      Wheel_PID(1, 0, 0,false);
      Wheel_PID(2, 0, 0,false);
      Wheel_PID(3, 0, 0,false);
      Wheel_PID(4, 0, 0,false);
      break;
    case 1:  //Forward
      Wheel_PID(1, 1, sp,true);
      Wheel_PID(2, 1, sp,true);
      Wheel_PID(3, 1, sp,true);
      Wheel_PID(4, 1, sp,true);
      break;
    case 2:  //Backward
      Wheel_PID(1, 2, sp,true);
      Wheel_PID(2, 2, sp,true);
      Wheel_PID(3, 2, sp,true);
      Wheel_PID(4, 2, sp,true);
      break;
    case 11:  //LeftRevole //馬達左旋
      Wheel_PID(1, 1, sp,true);
      Wheel_PID(2, 2, sp,true);
      Wheel_PID(3, 1, sp,true);
      Wheel_PID(4, 2, sp,true);
      break;
    case 12:  //RightRevole //馬達右旋
      Wheel_PID(1, 2, sp,true);
      Wheel_PID(2, 1, sp,true);
      Wheel_PID(3, 2, sp,true);
      Wheel_PID(4, 1, sp,true);
      break;
  }
}
//======================================================================
//Wheel RPM PID Control
//======================================================================
void DC_MOTOR::Wheel_PID(unsigned short wheel, unsigned short dir, float input, bool do_pid)
{
  //To do PID control or not
  if(do_pid)
  {
    if(input > output[wheel])
    {
      Serial.print(output[wheel]);
      
      pPID.PID_Control(wheel, input); 

      output[wheel] = pPID.GetOutputValue(wheel);
    }
    else
    {
      output[wheel] = input;
    }
  }
  else
  {
    output[wheel] = input;
  }

  switch (wheel)
  {
    case 0:
      break;
    case 1:
      Wheel_M1(dir, (int) output[wheel]);
      break;
    case 2:
      Wheel_M2(dir, (int) output[wheel]);
      break;
    case 3:
      Wheel_M3(dir, (int) output[wheel]);
      break;
    case 4:
      Wheel_M4(dir, (int) output[wheel]);
      break;
  } 
}
//======================================================================
void DC_MOTOR::Clean_PID_LastInput()
{
  //pPID.CleanLastInput();
}
//======================================================================
// Velocity and Angular Velocity
// V1 = S/t
// S = 2 * PI * r ==> 2 * PI * 0.04 ==> 0.2513 m
// V1 = 0.2513 m/s 每一圈所走的距離
// V2 = V1 * (rpm / 60)
// Velocity = 每一圈所走的距離 * (每一分鐘所轉的圈數 / 60)
//======================================================================
float DC_MOTOR::RPMtoVelocity(unsigned short no)
{
  float V1 = 0.2513;
  int rpm = 0;

  switch (no)
  {
    case 1:
      rpm = M1_RPM;
      break;
    case 2:
      rpm = M2_RPM;
      break;
    case 3:
      rpm = M3_RPM;
      break;
    case 4:
      rpm = M4_RPM;
      break;
  }

  float V2 = (V1 * rpm) / 60.0;

  return V2;
}
//======================================================================
//speed Range from 0 to 255
//Motor maximum is 60 rpm [speed range is 255]
//ie : speed_range = (255 * 30) / 60 ==> 127.5
//======================================================================
int DC_MOTOR::RPMtoSpeed(int rpm)
{
  int speed_range = (255 * rpm) / 60.0;

  return speed_range;
}
//======================================================================
// V2 = V1 * rpm / 60
// rpm = (V2 * 60) / V1
//======================================================================
int DC_MOTOR::VelocityToRPM(float V2)
{
  float V1 = 0.2513;
  int rmp = (int)((V2 * 60) / V1);

  return rmp;
}
//======================================================================
unsigned short DC_MOTOR::Wheel_Directory(unsigned short wheel)
{
  switch (wheel)
  {
    case 0:
      break;
    case 1:
      return Wheel_Directory_1;
      break;
    case 2:
      return Wheel_Directory_2;
      break;
    case 3:
      return Wheel_Directory_3;
      break;
    case 4:
      return Wheel_Directory_4;
      break;
  }
}
//======================================================================
signed long int DC_MOTOR::GetEncoder(unsigned short wheel)
{
  switch (wheel)
  {
    case 0:
      break;
    case 1:
      return Encoder_1;
      break;
    case 2:
      return Encoder_2;
      break;
    case 3:
      return Encoder_3;
      break;
    case 4:
      return Encoder_4;
      break;
  }
}
//======================================================================
void DC_MOTOR::PrintRPM()
{
  Serial.print("RPM M1:");
  Serial.print(M1_RPM);
  Serial.print(" M2:");
  Serial.print(M2_RPM);
  Serial.print(" M3:");
  Serial.print(M3_RPM);
  Serial.print(" M4:");
  Serial.println(M4_RPM);
}
//=====================================================================
void DC_MOTOR::PrintVelocity()
{
  Serial.print("Velocity M1:");
  Serial.print(RPMtoVelocity(1));
  Serial.print(" m/s M2:");
  Serial.print(RPMtoVelocity(2));
  Serial.print(" m/s M3:");
  Serial.print(RPMtoVelocity(3));
  Serial.print(" m/s M4:");
  Serial.print(RPMtoVelocity(4));
  Serial.println(" m/s");
}
//======================================================================
void DC_MOTOR::PrintVelocityToRPM()
{
  float V2 = 0.0;
  int rpm = 0;

  Serial.print("(M1)V:");
  V2 = RPMtoVelocity(1);
  Serial.print(V2);
  Serial.print(" m/s rpm: ");
  rpm = VelocityToRPM(V2);
  Serial.print(rpm);

  Serial.print(" (M2)V:");
  V2 = RPMtoVelocity(2);
  Serial.print(V2);
  Serial.print(" m/s rpm: ");
  rpm = VelocityToRPM(V2);
  Serial.print(rpm);

  Serial.print(" (M3)V:");
  V2 = RPMtoVelocity(3);
  Serial.print(V2);
  Serial.print(" m/s rpm :");
  rpm = VelocityToRPM(V2);
  Serial.print(rpm);

  Serial.print(" (M3)V:");
  V2 = RPMtoVelocity(4);
  Serial.print(V2);
  Serial.print(" m/s rpm :");
  rpm = VelocityToRPM(V2);
  Serial.println(rpm);
}
//=====================================================================
void DC_MOTOR::PrintVelocityRPMSpeed()
{
  float V2 = 0.0;
  int rpm = 0;
  float sp = 0;

  Serial.print("(M1)V:");
  V2 = RPMtoVelocity(1);
  Serial.print(V2);
  Serial.print(" m/s rpm: ");
  rpm = VelocityToRPM(V2);
  Serial.print(rpm);
  sp = RPMtoSpeed(rpm);
  Serial.print(" sp: ");
  Serial.print(sp);

  Serial.print(" (M2)V:");
  V2 = RPMtoVelocity(2);
  Serial.print(V2);
  Serial.print(" m/s rpm: ");
  rpm = VelocityToRPM(V2);
  Serial.print(rpm);
  sp = RPMtoSpeed(rpm);
  Serial.print(" sp: ");
  Serial.print(sp);

  Serial.print(" (M3)V:");
  V2 = RPMtoVelocity(3);
  Serial.print(V2);
  Serial.print(" m/s rpm: ");
  rpm = VelocityToRPM(V2);
  Serial.print(rpm);
  sp = RPMtoSpeed(rpm);
  Serial.print(" sp: ");
  Serial.print(sp);

  Serial.print(" (M4)V:");
  V2 = RPMtoVelocity(4);
  Serial.print(V2);
  Serial.print(" m/s rpm: ");
  rpm = VelocityToRPM(V2);
  Serial.print(rpm);
  sp = RPMtoSpeed(rpm);
  Serial.print(" sp: ");
  Serial.println(sp);
}
//=====================================================================
void DC_MOTOR::PrintSpeed()
{
  Serial.print("Speed M1:");
  Serial.print((int)output[1]);
  Serial.print(" M2:");
  Serial.print((int)output[2]);
  Serial.print(" M3:");
  Serial.print((int)output[3]);
  Serial.print(" M4:");
  Serial.println((int)output[4]);
}
//======================================================================
int ti = 0;
int ii = 20;
int sspeed = 0;


void DC_MOTOR::Test()
{
  Test1(1,6);
}

//
//Low Velocity
//Velocity =0.12 , Speed = 28
//
//

void DC_MOTOR::Test1(int dir,int stopS)
{
  //Forward
  if (ti < (ii * 3))
  {
    sspeed = 255;
    
    Wheel_PID(1, dir, sspeed, true);
    Wheel_PID(2, dir, sspeed, true);
    Wheel_PID(3, dir, sspeed, true);
    Wheel_PID(4, dir, sspeed, true);
  }
  
  if((ti >= (ii * 3)) && (ti < (ii * 6)))
  {
    sspeed = 200;
    
    Wheel_PID(1, dir, sspeed, true);
    Wheel_PID(2, dir, sspeed, true);
    Wheel_PID(3, dir, sspeed, true);
    Wheel_PID(4, dir, sspeed, true);
  }  

  //Stop
  if((ti >= (ii * 6)) && (ti < (ii * 9)))
  {
    sspeed = 0;
    Wheel_PID_Move_Mode(0,0);
    Clean_PID_LastInput();
  }

  if((ti >= (ii * 9)) && (ti < (ii * 12)))
  {
    sspeed = 70;
    
    Wheel_PID(1, dir, sspeed, true);
    Wheel_PID(2, dir, sspeed, true);
    Wheel_PID(3, dir, sspeed, true);
    Wheel_PID(4, dir, sspeed, true);
  }
  
  //Stop
  if((ti >= (ii * 12)) && (ti < (ii * 15)))
  {
    sspeed = 0;
    Wheel_PID_Move_Mode(0,0);
    Clean_PID_LastInput();
  }

  if (ti >= (ii * 15))
  {
    ti = 0;
  }

  Serial.print("ti:");
  Serial.print(ti);
  Serial.print(" sspeed:");
  Serial.print(sspeed);  
  
  Serial.print(" M1:");
  Serial.print(M1_RPM);
  Serial.print(" V1:");
  Serial.print(RPMtoVelocity(1));

  Serial.print(" M2:");
  Serial.print(M2_RPM);
  Serial.print(" V2:");
  Serial.print(RPMtoVelocity(2));
  
  Serial.print(" M3:");
  Serial.print(M3_RPM);
  Serial.print(" V3:");
  Serial.print(RPMtoVelocity(3));
  
  Serial.print(" M4:");
  Serial.print(M4_RPM);  
  Serial.print(" V4:");
  Serial.println(RPMtoVelocity(4));

//  uint16_t M4_RPM = GetRPM_Interrupt(4);

//  Serial.print(sspeed);
//  Serial.print(": rpm:");
//  Serial.print(M4_RPM);
//  Serial.print(" Speed:");
//  Serial.print(RPMtoSpeed(M4_RPM));
//  Serial.print(" Velocity:");
//  Serial.print(RPMtoVelocity(4));
//  Serial.print(" Encoder:");
//  Serial.println(Encoder_4);
  
  ti++;
}
/*
  //random(0, 60)
  //表最小值是 0，最大值是 60。但這個實際上只會產生 0~59 的整數。
  M1_RPM = random(0, 60);
  Wheel_PID_M1(1,M1_RPM);

  M2_RPM = random(0, 60);
  Wheel_PID_M2(1,M2_RPM);

  M3_RPM = random(0, 60);
  Wheel_PID_M3(1,M3_RPM);

  M4_RPM = random(0, 60);
  Wheel_PID_M4(1,M4_RPM);
*/
//======================================================================
//======================================================================
