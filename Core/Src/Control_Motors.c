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

extern TIM_HandleTypeDef htim1;
Velocidades VEL;

void stop(void);
void re(double rpm);
void re_dist(double rpm, double distance);
void frente(double rpm);
void direita(double rpm);
void esquerda(double rpm);
uint8_t dist_2_pulso(double dist);
void speed(double DIR, double ESQ);
void adjust_PWM(double DC, uint8_t motor);

void speed(double DIR, double ESQ){
	VEL.DIREITA = DIR;
	VEL.ESQUERDA = ESQ;
}

double get_speed(uint8_t motor){
	if(motor == M_ESQ){
		return VEL.ESQUERDA;
	}else{
		return VEL.DIREITA;
	}
}

uint32_t time_wait_ms(uint8_t distancia, double vel){
	/*Tempo[ms] = (distancia[cm]*1000)/(vel[rpm]*0.356)*/
	return (uint32_t)((distancia*1000)/(vel*0.356));
}

void adjust_PWM(double DC, uint8_t motor){
	if(motor == M_ESQ){
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_ESQ, (uint16_t)(655.35*DC));
	}else{
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_DIR, (uint16_t)(655.35*DC));
	}
}

uint8_t angle_2_pulso(uint8_t angle){
	return ((uint8_t)((4.0*angle)/9.0))%360;
}
uint8_t pulso_2_angle(uint8_t pulse){
	 return (uint8_t)(pulse%360)*(9.0/4.0);
}

uint8_t dist_2_pulso(double dist){
	return (uint8_t)(dist+1);
}

void init_motors(double rpm){
	speed(rpm, rpm);
}

void stop(void){
	speed(0,0);
	reset_pulso_ang();
	IN1_ON;
	IN2_ON;
	IN3_ON;
	IN4_ON;
}

void frente(double rpm) {
	speed(rpm, rpm);
	IN1_ON;
	IN2_OFF;
	IN3_ON;
	IN4_OFF;
}

void re(double rpm) {
	speed(rpm, rpm);
	IN1_OFF;
	IN2_ON;
	IN3_OFF;
	IN4_ON;
}

void direita(double rpm){
	/*reduce duty cicle on right road and frent*/
	speed(rpm-3, rpm);
	IN1_ON;
	IN2_OFF;
	IN3_ON;
	IN4_OFF;
}

void esquerda(double rpm){
	/*reduce duty cicle on right road and frent*/
	speed(rpm, rpm-3);
	IN1_ON;
	IN2_OFF;
	IN3_ON;
	IN4_OFF;
}

void re_dist(double rpm, double distance){
	speed(rpm, rpm);
	__disable_irq();
	uint32_t pulse_D_old = get_pulso_ang(M_DIR);
	uint32_t pulse_E_old = get_pulso_ang(M_ESQ);
	__enable_irq();
	uint32_t pulse_D_new;
	uint32_t pulse_E_new;
	do {
		IN1_OFF;
		IN2_ON;
		IN3_OFF;
		IN4_ON;
		__disable_irq();
		pulse_D_new = get_pulso_ang(M_DIR);
		pulse_E_new = get_pulso_ang(M_ESQ);
		__enable_irq();
	} while (((pulse_D_new - pulse_D_old)< dist_2_pulso(distance)) &&
			((pulse_E_new - pulse_E_old)< dist_2_pulso(distance)));
}

void rot_dir(double rpm, uint8_t angle){
	speed(rpm, rpm);
	__disable_irq();
	uint32_t pulse_D_old = get_pulso_ang(M_DIR);
	uint32_t pulse_E_old = get_pulso_ang(M_ESQ);
	__enable_irq();
	uint32_t pulse_D_new;
	uint32_t pulse_E_new;
	do {
		IN1_OFF;
		IN2_ON;
		IN3_ON;
		IN4_OFF;
		__disable_irq();
		pulse_D_new = get_pulso_ang(M_DIR);
		pulse_E_new = get_pulso_ang(M_ESQ);
		__enable_irq();
	} while (((pulse_D_new - pulse_D_old)< angle_2_pulso(angle)) &&
			((pulse_E_new - pulse_E_old)< angle_2_pulso(angle)));
}



void rot_esq(double rpm, uint8_t angle){
	speed(rpm, rpm);
	__disable_irq();
	uint32_t pulse_D_old = get_pulso_ang(M_DIR);
	uint32_t pulse_E_old = get_pulso_ang(M_ESQ);
	__enable_irq();
	uint32_t pulse_D_new;
	uint32_t pulse_E_new;
	do {
		IN1_ON;
		IN2_OFF;
		IN3_OFF;
		IN4_ON;
		__disable_irq();
		pulse_D_new = get_pulso_ang(M_DIR);
		pulse_E_new = get_pulso_ang(M_ESQ);
		__enable_irq();
	} while (((pulse_D_new - pulse_D_old)< angle_2_pulso(angle)) &&
			((pulse_E_new - pulse_E_old)< angle_2_pulso(angle)));
}

/*Rotations Functions for manual mode*/
void rotacao_D(void){
	IN1_OFF;
	IN2_ON;
	IN3_ON;
	IN4_OFF;
}
/*Rotations Functions for manual mode*/
void rotacao_E(void){
	IN1_ON;
	IN2_OFF;
	IN3_OFF;
	IN4_ON;
}


