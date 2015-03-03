 /*    PWM example for STM32F4 Discovery
*    It should work on STM32F429 Discovery too and all other STM32F4xx devices
*
*    @author     Tilen Majerle
*    @email        tilen@majerle.eu
*    @website    http://stm32f4-discovery.com
*    @ide        Keil uVision 5
*/

//#include "stm32f4xx.h"
#include "tm_stm32f4_i2c.h"
//#include "tm_stm32f4_delay.h"
#include "math.h"
#include "MadgwickAHRS.h"
#include "quaternion.h"
//#include "tm_stm32f4_usb_vcp.h"
#include "tm_stm32f4_pwm.h"
#include "stdio.h"
#include "PID.h"

 
 
 

//#define ADDRESS        0xD0 // 1101 000 0
 
 
					float liczba = 1.25;
					float liczba_back;
			uint32_t liczba_int = 0;
			
			char do_wyslania[4];
			
			uint8_t liczba_q0[4];
			uint8_t liczba_q1[4];
			uint8_t liczba_q2[4];
			uint8_t liczba_q3[4];
			
			
		uint16_t counter = 0;
		uint16_t counter_temp = 0;
		uint16_t counter_i2c = 0;
		uint16_t counter_i2c_last = 0;
		uint16_t counter_i2c_diff = 0;


		
		uint8_t data_x_l;
		uint8_t data_x_h;
		int16_t data_x;
		
		uint8_t data_y_l;
		uint8_t data_y_h;
		int16_t data_y;

		uint8_t data_z_l;
		uint8_t data_z_h;
		int16_t data_z;
		
		
		uint8_t data_x_l_a;
		uint8_t data_x_h_a;
		int16_t data_x_a;
		
		uint8_t data_y_l_a;
		uint8_t data_y_h_a;
		int16_t data_y_a;

		uint8_t data_z_l_a;
		uint8_t data_z_h_a;
		int16_t data_z_a;
		
		
		
		uint8_t data_x_l_m;
		uint8_t data_x_h_m;
		int16_t data_x_m;
		
		uint8_t data_y_l_m;
		uint8_t data_y_h_m;
		int16_t data_y_m;

		uint8_t data_z_l_m;
		uint8_t data_z_h_m;
		int16_t data_z_m;
		
		float data_x_f;
 		float data_y_f;
		float data_z_f;
		float data_x_n;
 		float data_y_n;
		float data_z_n;
		double sqrt_sum;
		double norm_sum;
		
		float data_x_f_a;
 		float data_y_f_a;
		float data_z_f_a;
		float data_x_n_a;
 		float data_y_n_a;
		float data_z_n_a;
		double sqrt_sum_a;
		double norm_sum_a;
			
		float data_x_f_m;
 		float data_y_f_m;
		float data_z_f_m;
		float data_x_n_m;
 		float data_y_n_m;
		float data_z_n_m;
		
		float angle_heading;
		float angle_attitude;
		float angle_bank;
		
		float gyroGain = 0.0024;
		
		float filterTime = 5;
		
		uint8_t REG26;
		
		float test_x=0;
		float test_y=0;
		float test_z=0;
		
		float offset_x_a = -16.48;
		float offset_y_a = -60.58;
		float offset_z_a = -28.45;
		float sum_x_a=0;
		float sum_y_a=0;
		float sum_z_a=0;
		uint32_t probes_a=0;
		uint8_t set_offset = 0;
		uint8_t set_offset_prev=0;
		float angle_attitude_filtered = 0;
		float filter_coeff = 2;
		
		TM_PWM_TIM_t TIM4_Data;
		
		uint32_t PWM4 = 0;
		
		float SP_angle = 0.0;
		
		float Td = 0;
	void refreshAcceleration(){
		
			data_x_l=TM_I2C_Read(I2C1, 0x3b, 0x28);
			data_x_h=TM_I2C_Read(I2C1, 0x3b, 0x29);
			data_x = data_x_l+(data_x_h<<8);
			data_x_f=data_x;
			
			data_y_l=TM_I2C_Read(I2C1, 0x3b, 0x2a);
			data_y_h=TM_I2C_Read(I2C1, 0x3b, 0x2b);
			data_y = data_y_l+(data_y_h<<8);	
			data_y_f=data_y;

			data_z_l=TM_I2C_Read(I2C1, 0x3b, 0x2c);
			data_z_h=TM_I2C_Read(I2C1, 0x3b, 0x2d);
			data_z = data_z_l+(data_z_h<<8);
			data_z_f=data_z;

			sqrt_sum=(data_x_f*data_x_f+data_y_f*data_y_f+data_z_f*data_z_f);
		
			data_x_n=data_x_f/sqrt_sum;
			data_y_n=data_y_f/sqrt_sum;
			data_z_n=data_z_f/sqrt_sum;
}

	void refreshAngularAcceleration(){
			
			data_x_l_a=TM_I2C_Read(I2C1, 0xD7, 0x28);
			data_x_h_a=TM_I2C_Read(I2C1, 0xD7, 0x29);
			data_x_a = data_x_l_a+(data_x_h_a<<8);
			data_x_f_a=data_x_a-offset_x_a;
			
			data_y_l_a=TM_I2C_Read(I2C1, 0xD7, 0x2a);
			data_y_h_a=TM_I2C_Read(I2C1, 0xD7, 0x2b);
			data_y_a = data_y_l_a+(data_y_h_a<<8);	
			data_y_f_a=data_y_a-offset_y_a;

			data_z_l_a=TM_I2C_Read(I2C1, 0xD7, 0x2c);
			data_z_h_a=TM_I2C_Read(I2C1, 0xD7, 0x2d);
			data_z_a = data_z_l_a+(data_z_h_a<<8);
			data_z_f_a=data_z_a-offset_z_a;
	}
	
	void refreshMagneticField(){
		
			data_x_l_m=TM_I2C_Read(I2C1, 0x3b, 0x08);
			data_x_h_m=TM_I2C_Read(I2C1, 0x3b, 0x09);
			data_x_m = data_x_l+(data_x_h<<8);
			data_x_f_m=data_x_m;
			
			data_y_l_m=TM_I2C_Read(I2C1, 0x3b, 0x0a);
			data_y_h_m=TM_I2C_Read(I2C1, 0x3b, 0x0b);
			data_y_m = data_y_l+(data_y_h<<8);	
			data_y_f_m=data_y_m;

			data_z_l_m=TM_I2C_Read(I2C1, 0x3b, 0x0c);
			data_z_h_m=TM_I2C_Read(I2C1, 0x3b, 0x0d);
			data_z_m = data_z_l+(data_z_h<<8);
			data_z_f_m=data_z_m;

}
	


				


void SysTick_Handler(void){
			
			counter_i2c++;
			counter_temp++;
			if(counter_temp>100){
				counter++;
				counter_temp = 0;
			}
			

					
			MadgwickAHRSupdate(data_x_f_a*gyroGain, data_y_f_a*gyroGain, data_z_f_a*gyroGain, data_x_f, data_y_f, data_z_f,data_x_f_m,data_y_f_m,data_z_f_m);
					
			quaternionToEuler(q0,q1,q2,q3);
					
			angle_heading = heading;
			angle_attitude = attitude;
			angle_bank = bank;
			
			angle_attitude_filtered = (angle_attitude_filtered*(filter_coeff-1)+angle_attitude)/filter_coeff;

			processPID(SP_angle, angle_attitude_filtered, Td);
}

/*
void floatToBytes(float input_value, uint8_t* tab){
				uint32_t liczba_int;
				uint8_t abc;
				liczba_int=*((uint32_t *)&input_value);
				abc = liczba_int&0xff;
				tab[0] = 
				abc = (liczba_int&0xff00)>>8;
				tab[1] = abc;
				abc = (liczba_int&0xff0000)>>16;
				tab[2] = abc;
				abc = (liczba_int&0xff000000)>>24;
				tab[3] = abc;
				liczba_back=*((float *)&liczba_int);
}

	*/	

void PWM_Initialize(){

	
	TM_PWM_InitTimer(TIM4, &TIM4_Data, 200);
	
	/* Initialize PWM on TIM4, Channel 1 and PinsPack 2 = PD12 */
	TM_PWM_InitChannel(&TIM4_Data, TM_PWM_Channel_1, TM_PWM_PinsPack_2);
	/* Initialize PWM on TIM4, Channel 2 and PinsPack 2 = PD13 */
	TM_PWM_InitChannel(&TIM4_Data, TM_PWM_Channel_2, TM_PWM_PinsPack_2);
	/* Initialize PWM on TIM4, Channel 3 and PinsPack 2 = PD14 */
	TM_PWM_InitChannel(&TIM4_Data, TM_PWM_Channel_3, TM_PWM_PinsPack_2);
	/* Initialize PWM on TIM4, Channel 4 and PinsPack 2 = PD15 */
	TM_PWM_InitChannel(&TIM4_Data, TM_PWM_Channel_4, TM_PWM_PinsPack_2);
	
	/* Set channel 1 value, 50% duty cycle */
	//TM_PWM_SetChannel(&TIM4_Data, TM_PWM_Channel_1, TIM4_Data.Period / 2);
	/* Set channel 2 value, 33% duty cycle */
	//TM_PWM_SetChannel(&TIM4_Data, TM_PWM_Channel_2, TIM4_Data.Period / 3);
	/* Set channel 3 value, 25% duty cycle */
	//TM_PWM_SetChannel(&TIM4_Data, TM_PWM_Channel_3, TIM4_Data.Period / 4);
	/* Set channel 4 value, 5% duty cycle*/
	//TM_PWM_SetChannelPercent(&TIM4_Data, TM_PWM_Channel_4, 5);
}
int main(void) {
	
		 uint8_t c;
			int i;

    //Initialize system
    SystemInit();
		PWM_Initialize();
		Output_Init();
	
    //TM_DISCO_LedInit();
	
	  //TM_USB_VCP_Init();
		
		SysTick_Config(SystemCoreClock/50);
		
    //Initialize I2C, SCL: PB6 and SDA: PB7 with 100kHt serial clock
	
		//for(PWM4=0;PWM4<10000000;PWM4++){
			
		//}
		
		
    TM_I2C_Init(I2C1, TM_I2C_PinsPack_1, 200000);

		TM_I2C_Write(I2C1, 0x3a, 0x20, 0x97);
		TM_I2C_Write(I2C1, 0xD6, 0x20, 0xDF);
		TM_I2C_Write(I2C1, 0xD6, 0x23, 0x20);
		TM_I2C_Write(I2C1, 0x3A, 0x26, 0xE0);
    

    while (1) {



				runForward;
				runBackward;
			
			

				TM_PWM_SetChannelPercent(&TIM4_Data, TM_PWM_Channel_4, PIDout+10);
				TM_PWM_SetChannelPercent(&TIM4_Data, TM_PWM_Channel_2, PIDout+10);
			
				if(runForward){
					GPIOA->BSRRL=OUTRF1 | OUTRF2;
				GPIOA->BSRRH=OUTRB1 | OUTRB2;
				
				}
				
				if(runBackward){
				GPIOA->BSRRL=OUTRB1 | OUTRB2;
					GPIOA->BSRRH=OUTRF1 | OUTRF2;
				}
				
				
				//if(PIDout<0)


			/*
			if (angle_heading>0){
				TM_PWM_SetChannelPercent(&TIM4_Data, TM_PWM_Channel_1, angle_heading);
				TM_PWM_SetChannelPercent(&TIM4_Data, TM_PWM_Channel_3, 0);
			}
			else{
				TM_PWM_SetChannelPercent(&TIM4_Data, TM_PWM_Channel_3, -angle_heading);
				TM_PWM_SetChannelPercent(&TIM4_Data, TM_PWM_Channel_1, 0);
			}*/
			
			refreshAcceleration();
			refreshAngularAcceleration();
			refreshMagneticField();
			
			if(set_offset){
				if(!set_offset_prev){
					probes_a=0;
				sum_x_a=0;
					sum_y_a=0;
					sum_z_a=0;
				}
				sum_x_a+=data_x_a;
				sum_y_a+=data_y_a;
				sum_z_a+=data_z_a;
				probes_a++;
				
			}
			
			if(!set_offset && set_offset_prev)
			{
				offset_x_a=sum_x_a/probes_a;
				offset_y_a=sum_y_a/probes_a;
				offset_z_a=sum_z_a/probes_a;	
			}
			set_offset_prev=set_offset;
			counter_i2c_diff=counter_i2c-counter_i2c_last;
			counter_i2c_last=counter_i2c;



		}
}