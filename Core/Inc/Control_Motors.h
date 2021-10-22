/*
 * Control_Motors.h
 *
 *  Created on: 30 de set de 2021
 *      Author: linco
 */
#ifndef INC_CONTROL_MOTORS_H_
#define INC_CONTROL_MOTORS_H_

#include "stdint.h"

#define IN1_ON		HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET)
#define IN1_OFF 	HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET)
#define IN2_ON		HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET)
#define IN2_OFF 	HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET)
#define IN3_ON		HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET)
#define IN3_OFF 	HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET)
#define IN4_ON		HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET)
#define IN4_OFF 	HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET)
#define M_ESQ		0x0
#define M_DIR		0x1

#define MOTOR_ESQ	TIM_CHANNEL_2
#define MOTOR_DIR	TIM_CHANNEL_1

typedef struct{
	double ESQUERDA;
	double DIREITA;
}Velocidades;

double get_speed(uint8_t motor);
void adjust_PWM(double DC, uint8_t motor);
uint32_t time_wait_ms(uint8_t distancia, double vel);
void init_motors(double rpm);
void stop(void);
void re(double rpm);
void re_dist(double rpm, double distance);
void frente(double rpm);
void direita(double rpm);
void esquerda(double rpm);
void rot_dir(double rpm, uint8_t angle);
void rot_esq(double rpm, uint8_t angle);
void rotacao_D(void);
void rotacao_E(void);




#endif /* INC_CONTROL_MOTORS_H_ */
