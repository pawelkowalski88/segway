#include "PID.h"
volatile int runForward;
volatile int runBackward;
volatile int PIDout;
volatile float kp;

void Output_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct;
	
	/* Enable GPIO peripheral clock for button */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/**
	 * Pin set to PA0
	 * Output type is push-pull
	 * Mode is Input
	 * No pull resistor
	 * Speed 100MHz
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	
	/* Initialize pin */
	GPIO_Init(GPIOA, &GPIO_InitStruct);
}



int processPID(float SP, float PV, float Td_in){
	float error;
	
	float Td = 0;
	float Tp=50;
	static float lastError;

	
	error=SP-PV;
	
	runForward=error>0;
	runBackward=error<0;
	if(kp==0)kp=4.0;
	
	
	
	PIDout=kp*error+Td_in*(error-lastError)/Tp;
	if(PIDout<0)PIDout=-PIDout;
	lastError=error;
	return PIDout;
}
