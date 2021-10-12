/*
 * Control_Motors.c
 *
 *  Created on: 30 de set de 2021
 *      Author: linco
 */


#include "Control_Motors.h"
#include "gpio.h"
#include "tim.h"
#include "main.h"
#include "stm32f401xc.h"
#include "math.h"
#include "RPM_sensor.h"


static Velocidades VEL;
extern TIM_HandleTypeDef htim1;

void stop(void);
void re(double percentage);
void frente(double percentage);
void direita(double percentage);
void esquerda(double percentage);
void rot_dir(double percentage);
void rot_esq(double percentage);
void speed(double DIR, double ESQ);
void set_speed(double DC, uint8_t motor);

void speed(double DIR, double ESQ){
	VEL.DIR = duty_2_rpm(DIR);
	VEL.ESQ = duty_2_rpm(ESQ);
}

double get_speed(uint8_t motor){
	if(motor == M_ESQ){
		return VEL.ESQ;
	}else{
		return VEL.DIR;
	}
}

uint32_t time_wait_ms(void){
	/*float tempo = (7500/((tensao*30*sqrt(atual.VEL_DIR)) - 40)) + 100;*/
	float tempo = -2*VEL.DIR + 240;
	return (uint32_t)tempo;
}


void set_speed(double DC, uint8_t motor){
	if(motor == M_ESQ){
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_ESQ, (uint16_t)(655.35*DC));
	}else{
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_DIR, (uint16_t)(655.35*DC));
	}
}



void init_motors(double percentage){
	speed(percentage, percentage);
}


void stop(void){
	speed(0,0);
	IN1_ON;
	IN2_ON;
	IN3_ON;
	IN4_ON;
}

void frente(double percentage) {
	speed(percentage, percentage);
/*	set_speed(percentage,M_ESQ);
	set_speed(percentage,M_DIR);*/
	IN1_ON;
	IN2_OFF;
	IN3_ON;
	IN4_OFF;
}

void re(double percentage) {
	speed(percentage, percentage);
/*	set_speed(percentage,M_ESQ);
	set_speed(percentage,M_DIR);*/
	IN1_OFF;
	IN2_ON;
	IN3_OFF;
	IN4_ON;
}

void direita(double percentage){
	/*reduce duty cicle on right road and frent*/
	speed(percentage-3, percentage);

/*	set_speed(percentage,M_ESQ);
	set_speed(percentage - 3,M_DIR);*/
	IN1_ON;
	IN2_OFF;
	IN3_ON;
	IN4_OFF;
}

void esquerda(double percentage){
	/*reduce duty cicle on right road and frent*/
	speed(percentage, percentage-3);

/*	set_speed(percentage,M_ESQ);
	set_speed(percentage,M_DIR);*/
	IN1_ON;
	IN2_OFF;
	IN3_ON;
	IN4_OFF;
}

void rot_dir(double percentage){
	speed(percentage, percentage);
/*	set_speed(percentage,M_ESQ);
	set_speed(percentage,M_DIR);*/
	IN1_OFF;
	IN2_ON;
	IN3_ON;
	IN4_OFF;
}

void rot_esq(double percentage){
	speed(percentage, percentage);
/*	set_speed(percentage,M_ESQ);
	set_speed(percentage,M_DIR);*/
	IN1_ON;
	IN2_OFF;
	IN3_OFF;
	IN4_ON;
}

