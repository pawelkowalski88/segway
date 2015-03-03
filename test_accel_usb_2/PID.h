#ifndef __PID_H__
#define __PID_H__
#define OUTRF1 1
#define OUTRB1 2
#define OUTRF2 4
#define OUTRB2 8


#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"



extern volatile int runForward;
extern volatile int runBackward;
extern volatile int PIDout;
extern volatile float kp;
int processPID(float SP, float PV, float TD_in);
void Output_Init(void);


#endif
