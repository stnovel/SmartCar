//=========================================================================
#ifndef DC_MOTOR_H
#define DC_MOTOR_H

#include "Arduino.h"

//======================================================================
//=========================================================================
class DC_MOTOR
{
  public:

    DC_MOTOR();
    ~DC_MOTOR();

    //After PID to output speed to Motor
    float output[5];

    void Initinal();
    void PrintSpeed();
    void Wheel_Move_Mode(unsigned short dir,int sp);
    void Wheel_PID_Move_Mode(unsigned short dir,int sp);
    void Clean_PID_LastInput();

    void Wheel_M1(unsigned short turn,int sp);
    void Wheel_M2(unsigned short turn,int sp);
    void Wheel_M3(unsigned short turn,int sp);
    void Wheel_M4(unsigned short turn,int sp);

    void Wheel_PID(unsigned short wheel, unsigned short dir, float input, bool do_pid);

    void Display_RPM();
    int GetRPM(unsigned short wheel);
    float RPMtoVelocity(unsigned short no);
    int RPMtoSpeed(int rpm);
    int VelocityToRPM(float V2);
    signed long int GetEncoder(unsigned short wheel);
    //Wheel Running Directory
    unsigned short Wheel_Directory(unsigned short wheel);

    void PrintRPM();
    void PrintVelocity();
    void PrintVelocityToRPM();
    void PrintVelocityRPMSpeed();

    void Test();
    void Test1(int dir,int stopS);
    
  private:
    //set static for Interrupt
    static InterruptM1();
    static InterruptM2();
    static InterruptM3();
    static InterruptM4();
    uint16_t GetRPM_Interrupt(unsigned short wheel);
};
//=========================================================================
#endif
