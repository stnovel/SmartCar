//=========================================================================
#ifndef PID_H
#define PID_H

#include "Arduino.h"
//=========================================================================
class PID
{
  public:
    void Set_PID(unsigned short wheel,float Setpoint,float min_val, float max_val, float kp, float ki, float kd);

    void PID_Control(unsigned short wheel, float Input);
    void Update_Min_Max_Val(unsigned short wheel,float Setpoint,float min_val, float max_val);
    void Update_PID(float kp, float ki, float kd);
    void CleanLastInput();
    float GetOutputValue(unsigned short wheel);
    
  private:
    struct _para_
    {
      float min_val[5];
      float max_val[5];
      float kp_;
      float ki_;
      float kd_;

      double integral_;
      double derivative_;
      double prev_error_;
      
      //設定目標值
      float Setpoint[5];
      //輸入值
      float Input[5];
      //積分項
      float ITerm[5]; 
      //前次輸入               
      float lastInput[5];
      // PID輸出值          
      float Output[5];  
      //上一次讀數的時間         
      unsigned long nLastTime[5];  
    }para;
   
};
//=========================================================================
#endif
