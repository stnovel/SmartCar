//=========================================================================
#include "Arduino.h"
#include "PID.h"


//PID(float min_val, float max_val, float kp, float ki, float kd)
//min_val = min output PID value  // 輸出上限、輸出下限
//max_val = max output PID value
//kp = PID - P constant           // 比例係數、積分係數、微分係數
//ki = PID - I constant
//di = PID - D constant
//PID pid( -255, 255, 0.05, 0.9, 0.1);
//=========================================================================
void PID::Set_PID(unsigned short wheel,float Setpoint,float min_val, float max_val, float kp, float ki, float kd)
{
  para.Setpoint[wheel]= Setpoint;
  para.min_val[wheel]= min_val;
  para.max_val[wheel]= max_val;
  
  para.kp_ = kp;
  para.ki_ = ki;
  para.kd_ = kd;
}
//=========================================================================
void PID::Update_Min_Max_Val(unsigned short wheel,float Setpoint,float min_val, float max_val)
{
  para.Setpoint[wheel]= Setpoint;
  para.min_val[wheel]= min_val;
  para.max_val[wheel]= max_val;  
}
//=========================================================================
void PID::Update_PID(float kp, float ki, float kd)
{
  para.kp_ = kp;
  para.ki_ = ki;
  para.kd_ = kd;
}
//=========================================================================
//PID Control
//=========================================================================
void PID::PID_Control(unsigned short wheel, float Input)
{
  //當前時間(ms)
  unsigned long nCurTime = millis(); 
  //採樣時間(s)
  float TimeCh = (nCurTime - para.nLastTime[wheel]) / 1000.0;
  //採樣時間(s) 
  float SampleTime = 0.1;                               

  //到達預定採樣時間時              
  if (TimeCh >= SampleTime)                             
  {                
    //偏差值           
    float error = Input - para.Setpoint[wheel];                     

    if(para.lastInput[wheel] != Input)
    {
      para.ITerm[wheel] = 0;
    }
    else
    {
      //計算積分項
      para.ITerm[wheel] += (para.ki_ * error * TimeCh);   
    }
                  
    //限定值域
    para.ITerm[wheel] = constrain(para.ITerm[wheel], para.min_val[wheel], para.max_val[wheel]);  
         
    //計算微分項
    float DTerm = para.kd_ * (Input - para.lastInput[wheel]) / TimeCh;   
    //計算輸出值
    para.Output[wheel] = para.kp_ * error + para.ITerm[wheel] - DTerm;       
    
    //限定值域        
    para.Output[wheel] = constrain(para.Output[wheel], para.min_val[wheel], para.max_val[wheel]); 

    //記錄輸入值
    para.lastInput[wheel] = Input;         
    //記錄本次時間                         
    para.nLastTime[wheel] = nCurTime;                               
  }
}
//=========================================================================
//回傳速度(計算輸出值)
//=========================================================================
float PID::GetOutputValue(unsigned short wheel)
{
  return para.Output[wheel];
}
//=========================================================================
void PID::CleanLastInput()
{
  for(int i=0; i < 5; i++)
  {
    para.lastInput[i] = 0;
  }
}
//=========================================================================
//=========================================================================
