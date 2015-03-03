#include "quaternion.h"

//oblicza k¹ty eulera na podstawie kwaternionów

volatile float attitude = 0, heading = 0, bank = 0;

	float par_x;
	float par_y;

void quaternionToEuler(float q_0,float q_1, float q_2, float q_3) {
	float pi=3.14159;	
	float sqw = q_0*q_0;
		float sqx = q_1*q_1;
		float sqy = q_2*q_2;
		float sqz = q_3*q_3;
	float unit = sqx + sqy + sqz + sqw; // if normalised is one, otherwise is correction factor
	float test = q_1*q_2 + q_3*q_0;
	/*
	if (test > 0.499*unit) { // singularity at north pole
		heading = 2 * atan2(q_1,q_0);
		attitude = pi/2;
		bank = 0;
		
	heading*=180.0/pi;
	attitude*=180.0/pi;
	bank*=180.0/pi;
		return;
	}

	if (test < -0.499*unit) { // singularity at south pole
		heading = -2 * atan2(q_1,q_0);
		attitude = -pi/2;
		bank = 0;
		
	heading*=180.0/pi;
	attitude*=180.0/pi;
	bank*=180.0/pi;
		return;
	}*/
		//heading = atan2(2*q_2*q_0-2*q_1*q_3 , sqx - sqy - sqz + sqw);
	
	//bank = atan2(2*q_1*q_0-2*q_2*q_3 , -sqx + sqy - sqz + sqw);
//attitude = asin(2*test/unit);
	
	//heading = asin(2*test/unit);
	//bank = atan2(2*q_1*q_0-2*q_2*q_3 , -sqx + sqy - sqz + sqw);
	//attitude = atan2(2*q_2*q_0-2*q_1*q_3 , sqx - sqy - sqz + sqw);
	par_y=2*(q_0*q_3+q_1*q_2);
	par_x=1-2*(q_2*q_2+q_3*q_3);
	
	heading = atan2(2*(q_0*q_1+q_2*q_3),1-2*(q_1*q_1+q_2*q_2));
	attitude = asin(2*(q_0*q_2-q_3*q_1));
	bank = atan2(par_y,par_x);
	/*
	if((par_x<0.2)&& (par_x>-0.2) && (par_y<0)){
		bank=-pi;
	}*/
	heading*=180.0/pi;
	attitude*=180.0/pi;
	bank*=180.0/pi;
	
	
	
	
	
}



