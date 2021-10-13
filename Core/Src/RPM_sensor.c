/*
 * RPM_sensor.c
 *
 *  Created on: Oct 7, 2021
 *      Author: linco
 */
#include "Control_Motors.h"
#include "RPM_sensor.h"
#include "main.h"
#include "cmsis_os.h"
#include "dma.h"
#include "tim.h"

pulsos D;
pulsos E;


double movingAvg_Dir(double *ptrArrNumbers, uint8_t len, double nextNum, uint8_t init){
	static uint8_t pos=0;
	static float Sum=0;
	if(init==1){
		pos = 0;
		Sum = 0;
	}
	/*Subtract the oldest number from the prev sum, add the new number*/
	Sum = Sum - ptrArrNumbers[pos] + nextNum;
	/*Assign the nextNum to the position in the array*/
	ptrArrNumbers[pos] = nextNum;
	pos++;
	if(pos >= len){
		pos = 0;
	}
	/*return the average*/
	return Sum / len;
}

double duty_2_rpm(double duty){
	return 24.75*duty - 149.25;
}

double rpm_2_duty(double rpm){
	return (rpm + 149.25)/24.75;
}

double movingAvg_Esq(double *ptrArrNumbers, uint8_t len, double nextNum, uint8_t init){
	static uint8_t pos=0;
	static float Sum=0;
	if(init==1){
		pos = 0;
		Sum = 0;
	}
	/*Subtract the oldest number from the prev sum, add the new number*/
	Sum = Sum - ptrArrNumbers[pos] + nextNum;
	/*Assign the nextNum to the position in the array*/
	ptrArrNumbers[pos] = nextNum;
	pos++;
	if(pos >= len){
		pos = 0;
	}
	/*return the average*/
	return Sum / len;
}

void init_pulso(void){
	D.ANG=0;
	D.VEL=0;
	E.ANG=0;
	E.VEL=0;
}
void reset_pulso_vel(uint8_t MOT){
	if(MOT == M_ESQ){
		E.VEL=0;
	}else{
		D.VEL=0;
	}
}
void reset_pulso_ang(void){
	E.ANG=0;
	D.ANG=0;
}

void inc_pulso(uint8_t MOT){
	if(MOT == M_ESQ){
		E.VEL++;
		E.ANG++;
	}else{
		D.VEL++;
		D.ANG++;
	}
}

uint32_t get_pulso_vel(uint8_t MOT){
	if(MOT == M_ESQ){
		return E.VEL;
	}else{
		return D.VEL;
	}
}

uint32_t get_pulso_ang(uint8_t MOT){
	if(MOT == M_ESQ){
		return E.ANG;
	}else{
		return D.ANG;
	}
}

