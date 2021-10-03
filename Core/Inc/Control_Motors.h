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
	uint8_t VEL_ESQ;
	uint8_t VEL_DIR;
}Velocidades;

uint32_t time_wait_ms(void);
void init_motors(float percentage);
void stop(void);
void re(uint8_t percentage);
void frente(uint8_t percentage);
void direita(void);
void esquerda(void);
void rot_dir(uint8_t percentage);
void rot_esq(uint8_t percentage);

float get_speed(uint8_t motor);

#endif /* INC_CONTROL_MOTORS_H_ */
