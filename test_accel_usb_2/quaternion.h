#ifndef __QUATERNION_H__
#define __QUATERNION_H__
#include "math.h"
#include "stm32f4xx.h"
//const float Pi=3.14159;
//K¹ty Eulera podane w radianach

extern volatile float heading;
extern volatile float bank;
extern volatile float attitude; 

void quaternionToEuler(float q_0,float q_1, float q_2, float q_3);
//void floatToBytes(unsigned char &b0,unsigned char &b1,unsigned char &b2,unsigned char &b3, float input_value);

#endif