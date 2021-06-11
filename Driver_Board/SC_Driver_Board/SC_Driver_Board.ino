//=========================================================================
//Driver Board Control For Smart Car by Steven Tseng (C) Copyrights Reserved.
//  1. 2021/03/20  Version 0.0.5  Define Protocol for Driver Board Control
//  2. 2012/05/18  JGA25-370 Motor's Test
//=========================================================================
//From UART [Serial 0] to Recevied Commands and Publish Data
//
//    Forward  Left 1   Right 2
//
//    Backward Left 3   Right 4
//
//Recevied : 16 Bytes
//   [@ Start] [Motor Number] [Direction]     [Speed]        [PID]        [CRC8]   [! End]
//     0x40     0x01 ~ 0x04    0x00 ~ 0x03     0x01 ~ 0xFF   0x00 ~ 0x01   CRC8      0x21
// ex: 
//    0x40 0x01 0x01 0x78 0x02 0x01 0x78 0x03 0x01 0x78 0x04 0x01 0x78 0x01 CRC8 0x21  ==> froward  120
//    0x40 0x01 0x02 0x5A 0x02 0x02 0x5A 0x03 0x02 0x5A 0x04 0x02 0x5A 0x00 CRC8 0x21  ==> backward  90
//    0x40 0x01 0x01 0x64 0x02 0x01 0x6E 0x03 0x01 0x64 0x04 0x01 0x6E 0x00 CRC8 0x21  ==> turn left  100, 110
//    0x40 0x01 0x00 0x01 0x02 0x00 0x01 0x03 0x00 0x01 0x04 0x00 0x01 0x00 CRC8 0x21  ==> stop
//    0x40 0x01 0x01 0x78 0x02 0x02 0x78 0x03 0x01 0x78 0x04 0x02 0x78 0x00 CRC8 0x21  ==> RightRevole
//    0x40 0x01 0x02 0x78 0x02 0x01 0x78 0x03 0x02 0x78 0x04 0x01 0x78 0x00 CRC8 0x21  ==> LeftRevole
//
//Publish: 31 Bytes
//   [@ Start] [Motor Number] [Direction]    [Speed]        [Encoder]               [CRC8]  [! End]
//     0x40     0x01 ~ 0x04    0x00 ~ 0x03     0x00 ~ 0xFF    0x00 0x00 0x00 0x00    CRC8     0x21
// ex: [Encoder 2000 (0x00 0x00 0x07 0xD0)]
//   0x40 0x01 0x01 0x78 0x00 0x00 0x07 0xD0 0x02 0x01 0x78 0x00 0x00 0x07 0xD0 0x03 0x01 0x78 0x00 0x00 0x07 0xD0 0x04 0x01 0x78 0x00 0x00 0x07 0xD0 0x00 0x21
//
//=========================================================================
#include "DC_Motor.h"

DC_MOTOR  dcMotor;
//-----------------------------------------------------------------------
#include "SC_Timer.h"

SC_TIMER sc_timer;
//-----------------------------------------------------------------------

char inChar;
unsigned char inCommands[50]={0};
unsigned char outCommands[50]={0};
unsigned short n = 0;
bool thread_lock = false;

//0: standby, 1: do command, 2: start receiving command
unsigned short bRead=0;


struct 
{
  unsigned short dir1,dir2,dir3,dir4;
  int RPM1;
  int RPM2;
  int RPM3;
  int RPM4;
  signed long int Encoder1;
  signed long int Encoder2;
  signed long int Encoder3;
  signed long int Encoder4;
}pub;
//=========================================================================
//=========================================================================
void setup()    
{   
  //------------------------------
  dcMotor.Initinal();

  //------------------------------
  //Set Timer Interval
  //------------------------------
  //Interval at 50 (milliseconds) ==> Command_Analyze();
  sc_timer.TimerSetup(1, 50, true);
  
  //Interval at 300 (milliseconds) ==> Publish_Wheel_Speed_Report();
  sc_timer.TimerSetup(2, 300, true);


  //Set directory for front at startup
  Serial.begin(115200);   //啟動串行通信
  while (!Serial) 
  {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}
//=========================================================================
//When thread_lock as true, we need calcalate the wheel's rpm and encoder data, 
//While period it's not publish anydata to Serial(1)
//=========================================================================
void loop() 
{
  //Running the Timer
  switch (sc_timer.RunTimer())
  {
    case 1:
      thread_lock = true;
      Command_Analyze();
      thread_lock = false;
      break; 
    case 2:
      if(!thread_lock)
      {
        Publish_Wheel_Speed_Report();
      }
      break;           
  }
}
//=========================================================================
//Analyze Coming unsigned char from Serial(1)
//
//ie:
//@123123123000)!
//checked:41
//64 49 50 51 49 50 51 49 50 51 48 48 48 41 
//@1222211210127!
//checked:55
//64 49 50 50 50 50 49 49 50 49 48 49 50 55 
//=========================================================================
void Command_Analyze()
{
  if(Serial.available() > 0)
  {
    inChar = Serial.read();
    
    //Check Start Char "@"
    if(inChar == 0x40)
    {
      //2: start receiving command
      bRead = 2;
    }

    //Check End Char "!"
    if(inChar == 0x21)
    {
      //1: do command
      bRead = 1;
    }

    switch (bRead)
    {
      case 0:
          break;
      case 1:
          //Do Command
          Move_Wheels(inCommands);
          
          memset(inCommands,0x00,sizeof(inCommands));
          n = 0; 

          //0: standby
          bRead = 0;     
          break;
      case 2:
          inCommands[n] = inChar;
          n++;
          break;
    }
  }
}
//=========================================================================
//Following the Protocol To do Moving Wheels
//Byte 13 ==> PID Control
//Byte 14 ==> CRC8 Checked
//=========================================================================
void Move_Wheels(unsigned char commands[])
{
  unsigned char checked;
  //unsigned short len = strlen(commands);
  //Because strlen truncate when char is 0x00, it need constant len => 15
  unsigned short len = 15;
  checked = CRC8(commands,len-2);

  //Serial.print("checked:");
  //Serial.println(checked);
  
  if(checked == commands[len-1])
  {
    //0x40 0x01 0x01 0x78 0x02 0x01 0x78 0x03 0x01 0x78 0x04 0x01 0x78 0x01 CRC8 0x21  ==> froward  120
    int sp = 0;
    bool do_pid = false;

    //0x00 No PIC Control
    //0x01 For PIC Control
    if(commands[13]== 0x01)
    {
      do_pid = true;
    }
    
    if(commands[1] == 0x01)
    {
      pub.dir1 = commands[2];
      sp = commands[3];
      dcMotor.Wheel_PID(1, pub.dir1, sp,do_pid);
    }
    
    if(commands[4] == 0x02)
    {
      pub.dir2 = commands[5];
      sp = commands[6];
      dcMotor.Wheel_PID(2, pub.dir2, sp,do_pid);
    }   

    if(commands[7] == 0x03)
    {
      pub.dir3 = commands[8];
      sp = commands[9];
      dcMotor.Wheel_PID(3, pub.dir3, sp,do_pid);
    }
    
    if(commands[10] == 0x04)
    {
      pub.dir4 = commands[11];
      sp = commands[12];
      dcMotor.Wheel_PID(4, pub.dir4, sp,do_pid);
    }      
   
    //For Test Check The Received
//    outCommands[0]= 0x40;
//    for(int i=1;i < 30;i++)
//    {
//      outCommands[i]= 0xFF;
//    }

//    outCommands[2]=checked;
//    outCommands[30]= 0x21;

    //Oupt Byte Array
//    Serial.write(outCommands, sizeof(outCommands));
//    memset(outCommands,0x00,sizeof(outCommands));

  }
}
//=========================================================================
//Publish Wheels Speed and Encoder
//ie:
//0x40 
//0x01 0x01 0x78 
//0x00 0x00 0x07 0xD0 
//0x02 0x01 0x78 
//0x00 0x00 0x07 0xD0 
//0x03 0x01 0x78 
//0x00 0x00 0x07 0xD0 
//0x04 0x01 0x78 
//0x00 0x00 0x07 0xD0 
//CRC 0x21
//=========================================================================
void Publish_Wheel_Speed_Report()
{
  //To get RPM Values
  dcMotor.Display_RPM();
  
  pub.RPM1 = dcMotor.GetRPM(1);
  pub.RPM2 = dcMotor.GetRPM(2);
  pub.RPM3 = dcMotor.GetRPM(3);
  pub.RPM4 = dcMotor.GetRPM(4);

  //pub.RPM1 = 45;
  //pub.RPM2 = 22;
  //pub.RPM3 = 37;
  //pub.RPM4 = 53;
  
  //To get Encoder
  //-2,147,483,648 ~ 2,147,483,647
  //   signed long int Encoder1 = 2147483600;
  //   1.Set it to 4 bytes
  //      outCommands[4] = Encoder1 >> 24;
  //      outCommands[5] = Encoder1 >> 16;
  //      outCommands[6] = Encoder1 >> 8;
  //      outCommands[7] = Encoder1 & 0xFF;
  //
  //   2.Get 4 bytes
  //      unsigned char tmp[10] = { 0 };
  //      tmp[0] = outCommands[7];
  //      tmp[1] = outCommands[6];
  //      tmp[2] = outCommands[5];
  //      tmp[3] = outCommands[4];
  //
  //   signed long int Encoder1 = *(reinterpret_cast<signed long int *>(tmp));
  //

  pub.Encoder1 = dcMotor.GetEncoder(1);
  pub.Encoder2 = dcMotor.GetEncoder(2);
  pub.Encoder3 = dcMotor.GetEncoder(3);
  pub.Encoder4 = dcMotor.GetEncoder(4);

  //pub.Encoder1 =  214755555;
  //pub.Encoder2 = -14700000;
  //pub.Encoder3 =  2147000000;
  //pub.Encoder4 = -28888888;


  unsigned char checked;

  outCommands[0]= 0x40;
  
  outCommands[1]= 0x31;
  outCommands[2]= pub.dir1;
  outCommands[3]= pub.RPM1;

  outCommands[4] = pub.Encoder1 >> 24;
  outCommands[5] = pub.Encoder1 >> 16;
  outCommands[6] = pub.Encoder1 >> 8;
  outCommands[7] = pub.Encoder1 & 0xFF;
  
  outCommands[8]= 0x32;
  outCommands[9]= pub.dir2;
  outCommands[10]= pub.RPM2;
  
  outCommands[11] = pub.Encoder2 >> 24;
  outCommands[12] = pub.Encoder2 >> 16;
  outCommands[13] = pub.Encoder2 >> 8;
  outCommands[14] = pub.Encoder2 & 0xFF;
  
  outCommands[15]= 0x33;
  outCommands[16]= pub.dir3;
  outCommands[17]= pub.RPM3;

  outCommands[18] = pub.Encoder3 >> 24;
  outCommands[19] = pub.Encoder3 >> 16;
  outCommands[20] = pub.Encoder3 >> 8;
  outCommands[21] = pub.Encoder3 & 0xFF;
  
  outCommands[22]= 0x34;
  outCommands[23]= pub.dir4;
  outCommands[24]= pub.RPM4;

  outCommands[25] = pub.Encoder4 >> 24;
  outCommands[26] = pub.Encoder4 >> 16;
  outCommands[27] = pub.Encoder4 >> 8;
  outCommands[28] = pub.Encoder4 & 0xFF;
  
  //CRC
  //Because strlen truncate when char is 0x00, it need constant len => 28
  unsigned short len = 30;
  checked = CRC8(outCommands,len-2);
  
  outCommands[29]= checked;
  outCommands[30]= 0x21;
  
  //Oupt Byte Array
  Serial.write(outCommands, sizeof(outCommands));
  memset(outCommands,0x00,sizeof(outCommands));


//For Test Output
//  for(int i = 0; i < 31;i++)
//  {
//    //Serial.print(outCommands[i],HEX);
//    Serial.print(outCommands[i]);
//    Serial.print(" ");
//  }
//  Serial.println("");

}
//=========================================================================
//CRC8    check sum
//=========================================================================
unsigned char CRC8(unsigned char * data, int data_length)
{
  unsigned int i, j;
  unsigned char CRC8;

  CRC8 = 0; j = 0;
  while (j < data_length)
  {
    CRC8 ^= (data[j]);
    for (i = 0; i < 8; i++)
    {
      if ((CRC8 & 0x01) == 1)
      {
        CRC8 = (unsigned char)((CRC8 >> 1) ^ 0x8C);
      }
      else 
      {
        CRC8 >>= 1;
      }
    }

    j++;
  }
  
  return CRC8;  
}
//=========================================================================
//For Test as Below
//=========================================================================
/*
void loop_00() 
{

  for(int i=0;i < 100; i++)
  {  
    //dcMotor.Wheel_Move_Mode(1, 255);
    
    dcMotor.Wheel_PID_Move_Mode(1, 150);  
  
    //To get RPM Values
    dcMotor.Display_RPM();
    
    pub.RPM1 = dcMotor.GetRPM(1);
    pub.RPM2 = dcMotor.GetRPM(2);
    pub.RPM3 = dcMotor.GetRPM(3);
    pub.RPM4 = dcMotor.GetRPM(4);
  
    Serial.print("RPM1:");
    Serial.print(pub.RPM1);
      Serial.print(" RPM2:");
    Serial.print(pub.RPM2);
      Serial.print(" RPM3:");
    Serial.print(pub.RPM3);
      Serial.print(" RPM4:");
    Serial.println(pub.RPM4);

    delay(50);
  }


  dcMotor.Wheel_Move_Mode(0, 100); 
  //dcMotor.Wheel_PID_Move_Mode(0, 100);       
  delay(2000);
}
*/
/*
int count = 0;
void loop_Test() 
{
  
  if(count <= 500)
  { 
    //Forward
    dcMotor.Wheel_M1(1, 100);
    dcMotor.Wheel_M2(1, 100);
    dcMotor.Wheel_M3(1, 100);
    dcMotor.Wheel_M4(1, 100);
  }
  else
  {
    //Stop
    dcMotor.Wheel_M1(0, 0);
    dcMotor.Wheel_M2(0, 0);
    dcMotor.Wheel_M3(0, 0);
    dcMotor.Wheel_M4(0, 0);
  }
  count++;
  delay(50);
}
*/
/*
void Move_Wheels_00(unsigned char commands[])
{
  //0x40 0x01 0x01 0x78 0x02 0x01 0x78 0x03 0x01 0x78 0x04 0x01 0x78 0x01 CRC8 0x21  ==> froward  120
  commands[0]= 0x40;
  
  commands[1]=0x01; //Motor
  commands[2]=0x01; //Direction
  commands[3]=0x78; //Wheel Speed
  
  commands[4]=0x02; //Motor
  commands[5]=0x01; //Direction
  commands[6]=0x78; //Wheel Speed
  
  commands[7]=0x03; //Motor
  commands[8]=0x01; //Direction
  commands[9]=0x78; //Wheel Speed  
  
  commands[10]=0x04; //Motor
  commands[11]=0x01; //Direction
  commands[12]=0x78; //Wheel Speed
  
  commands[13]=0x00; //PID
  
  //CRC8
  commands[14] = CRC8(commands,13);
  commands[15] = 0x21; //End

  //Move_Wheels_01(commands);
}
*/



