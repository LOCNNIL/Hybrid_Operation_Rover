/*
 * RPM_sensor.h
 *
 *  Created on: Oct 7, 2021
 *      Author: linco
 */
#ifndef INC_RPM_SENSOR_H_
#define INC_RPM_SENSOR_H_

#include "stdint.h"

typedef struct {
	uint32_t ANG;
	uint32_t VEL;
}pulsos;

double duty_2_rpm(double duty);
double rpm_2_duty(double duty);
double movingAvg_Dir(double *ptrArrNumbers, uint8_t len, double nextNum, uint8_t init);
double movingAvg_Esq(double *ptrArrNumbers, uint8_t len, double nextNum, uint8_t init);
void init_pulso(void);
void inc_pulso(uint8_t MOT);
void reset_pulso(uint8_t MOT);
uint32_t get_pulso(uint8_t MOT);

#endif /* INC_RPM_SENSOR_H_ */
