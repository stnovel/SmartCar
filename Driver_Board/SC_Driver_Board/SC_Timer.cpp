//=========================================================================
//=========================================================================
#include "Arduino.h"
#include "SC_Timer.h"

//=========================================================================
SC_TIMER::SC_TIMER() 
{
  for(int i=0; i < 20; i++)
  {
    TimerSetup(i,0,false);
  }
}
//=========================================================================
SC_TIMER::~SC_TIMER() {}
//=========================================================================
void SC_TIMER::TimerSetup(unsigned short no, int Interval,bool Active)
{
  sc_timer[no].previousMillis = 0;
  sc_timer[no].interval = Interval;
  sc_timer[no].Active = Active;
}
//=========================================================================
unsigned short SC_TIMER::RunTimer()
{
  unsigned long currentMillis = millis();
  
  for(int i=1; i < 21; i++)
  {
    if(sc_timer[i].Active)
    {
      if(currentMillis - sc_timer[i].previousMillis >= sc_timer[i].interval)
      {
        sc_timer[i].previousMillis = currentMillis;

        return i;
      }
    }
  } 

  return 0;
}
//=========================================================================
//=========================================================================
