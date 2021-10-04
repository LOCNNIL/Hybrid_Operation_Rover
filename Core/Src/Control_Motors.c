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


static Velocidades atual;
extern TIM_HandleTypeDef htim1;

void stop(void);
void re(uint8_t percentage);
void frente(uint8_t percentage);
void direita(void);
void esquerda(void);
void rot_dir(uint8_t percentage);
void rot_esq(uint8_t percentage);
void speed(uint8_t percentage);
void set_speed(uint8_t DC, uint8_t motor);

void speed(uint8_t percentage){
	atual.VEL_DIR = (uint8_t)percentage+1;
	atual.VEL_ESQ = percentage;
}


uint32_t time_wait_ms(void){
	/*float tempo = (7500/((tensao*30*sqrt(atual.VEL_DIR)) - 40)) + 100;*/
	float tempo = -2*atual.VEL_DIR + 240;
	atual.VEL_DIR = atual.VEL_ESQ+1;
	return (uint32_t)tempo;
}


void set_speed(uint8_t DC, uint8_t motor){
	if(motor == M_ESQ){
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_ESQ, DC);
		atual.VEL_ESQ = DC;
	}else{
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_DIR, DC);
		atual.VEL_DIR = DC;
	}
}

float get_speed(uint8_t motor){
	if(motor == M_ESQ){
		return atual.VEL_ESQ;
	}else{
		return atual.VEL_DIR;
	}
}

void init_motors(float percentage){
	speed(percentage);
	set_speed(percentage, M_ESQ);
	set_speed(percentage, M_DIR);
}


void stop(void){
	IN1_ON;
	IN2_ON;
	IN3_ON;
	IN4_ON;
}

void frente(uint8_t percentage) {
	speed(percentage);
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_DIR, atual.VEL_DIR);
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_ESQ, atual.VEL_ESQ);
	IN1_ON;
	IN2_OFF;
	IN3_ON;
	IN4_OFF;
}

void re(uint8_t percentage) {
	speed(percentage);
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_DIR, atual.VEL_DIR);
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_ESQ, atual.VEL_ESQ);
	IN1_OFF;
	IN2_ON;
	IN3_OFF;
	IN4_ON;
}

void direita(void){
	/*reduce duty cicle on right road and frent*/
	atual.VEL_DIR = (uint8_t)(atual.VEL_DIR-3);
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_DIR, (uint8_t)atual.VEL_DIR);
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_ESQ, (uint8_t)atual.VEL_ESQ);
	IN1_ON;
	IN2_OFF;
	IN3_ON;
	IN4_OFF;
}

void esquerda(void){
	/*reduce duty cicle on right road and frent*/
	atual.VEL_ESQ = (uint8_t)(atual.VEL_ESQ-3);
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_DIR, (uint8_t)atual.VEL_DIR);
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_ESQ, (uint8_t)atual.VEL_ESQ);
	IN1_ON;
	IN2_OFF;
	IN3_ON;
	IN4_OFF;
}

void rot_dir(uint8_t percentage){
	if(percentage<= 90){
		speed(percentage + 10);
	}
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_DIR, atual.VEL_DIR);
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_ESQ, atual.VEL_ESQ);
	IN1_OFF;
	IN2_ON;
	IN3_ON;
	IN4_OFF;
}

void rot_esq(uint8_t percentage){
	if(percentage<= 90){
		speed(percentage + 10);
	}
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_DIR, atual.VEL_DIR);
	__HAL_TIM_SET_COMPARE(&htim1, MOTOR_ESQ, atual.VEL_ESQ);
	IN1_ON;
	IN2_OFF;
	IN3_OFF;
	IN4_ON;
}

