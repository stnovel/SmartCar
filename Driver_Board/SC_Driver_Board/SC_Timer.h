//=========================================================================
#ifndef SC_TIMER_H
#define SC_TIMER_H

#include "Arduino.h"
//=========================================================================
class SC_TIMER
{
  public:
    SC_TIMER();
    ~SC_TIMER();

    void TimerSetup(unsigned short no, int Interval,bool Active);
    unsigned short RunTimer();

  private:
    //Interval at 1000 (milliseconds)
    struct _SC_Timer_Data
    {
      bool Active;
      unsigned long previousMillis;
      int interval;
    }sc_timer[21];
};
//=========================================================================
#endif
