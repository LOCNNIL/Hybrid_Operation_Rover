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

pulsos count_pulsos;

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
	count_pulsos.dir=0;
	count_pulsos.esq=0;
}
void reset_pulso(uint8_t MOT){
	if(MOT == M_ESQ){
		count_pulsos.esq=0;
	}else{
		count_pulsos.dir=0;
	}
}

void inc_pulso(uint8_t MOT){
	if(MOT == M_ESQ){
		count_pulsos.esq++;
	}else{
		count_pulsos.dir++;
	}
}

uint32_t get_pulso(uint8_t MOT){
	if(MOT == M_ESQ){
		return count_pulsos.esq;
	}else{
		return count_pulsos.dir;
	}
}


