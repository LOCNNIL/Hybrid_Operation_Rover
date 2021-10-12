/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "usart.h"
#include "stm32f401xc.h"
#include "dma.h"
#include "stdio.h"
#include "string.h"
#include "RPM_sensor.h"
#include "arm_math.h"
#include "Control_Motors.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
	double dir;
	double esq;
} RPM_mensures_t;

typedef struct{
	double duty;
	double SETPOINT;
	uint32_t old_time;
	double vet_mov_avr[4];
	uint8_t len_mov_avr;
	double pid_error;
	double pid_out;
	uint8_t saturation;
	int8_t sign;
	double RPM;
	uint8_t MOT;
	uint8_t reload;
	arm_pid_instance_f32 PID;
	float32_t Ki_old;
	float32_t Kd_old;
	float32_t Kp_old;
}params_PID;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TURN_ON_AUTONOMUS 	(1<<0)	// 0b00000000000000000000000000000001	#01
#define TURN_OFF_AUTONOMUS	(1<<1)	// 0b00000000000000000000000000000010	#02
#define TURN_ON_MANUAL 		(1<<2)	// 0b00000000000000000000000000000100	#04
#define TURN_OFF_MANUAL		(1<<3)	// 0b00000000000000000000000000001000	#08

/* Choose PID parameters */
#define PID_PARAM_KP	0.1			/* Proporcional */
#define PID_PARAM_KI	10			/* Integral */
#define PID_PARAM_KD	1			/* Derivative */
#define FS				10			/*Sampling Frequency*/
#define TIMEHOLD		100		/*Sampling Period in ms*/
#define true			1
#define false			0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LED_ON				HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
#define LED_OFF				HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
#define LED_TOGGLE			HAL_GPIO_TogglePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

/*Ultrassonic Sensor Variables*/
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?

/*Button Variables*/
uint32_t BUT_COUNT = 0;
uint32_t ERROR_COUNT = 0;

/*Motor Velocity Variables*/
pulsos count_pulsos;
static double veloc = 15;
RPM_mensures_t RPM;
RPM_mensures_t RPM_filtered;

/*Infrared Sensor Variables*/
static uint32_t count_re = 0;
static uint32_t count_dir = 0;
static uint32_t count_esq = 0;
static uint32_t Distancia = 0xff;
static GPIO_PinState pin_dir = GPIO_PIN_SET;
static GPIO_PinState pin_esq = GPIO_PIN_SET;
static GPIO_PinState pin_re = GPIO_PIN_SET;

/*PID instancies*/
static uint8_t reload_PID=0;
params_PID DIR;
params_PID ESQ;
uint8_t ESQ_TEST;
uint8_t DIR_TEST;

uint8_t velocidade=15;
uint8_t comando[5];
uint8_t UART1_rxBuffer[5];
uint8_t to_send[5];

HAL_StatusTypeDef feedback_uart;
uint8_t count_ble=0;

/* USER CODE END Variables */
/* Definitions for Autonomus_Mode */
osThreadId_t Autonomus_ModeHandle;
const osThreadAttr_t Autonomus_Mode_attributes = {
  .name = "Autonomus_Mode",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Manual_Mode */
osThreadId_t Manual_ModeHandle;
const osThreadAttr_t Manual_Mode_attributes = {
  .name = "Manual_Mode",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for SensorsReading */
osThreadId_t SensorsReadingHandle;
const osThreadAttr_t SensorsReading_attributes = {
  .name = "SensorsReading",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Ultrasonic_Read */
osThreadId_t Ultrasonic_ReadHandle;
const osThreadAttr_t Ultrasonic_Read_attributes = {
  .name = "Ultrasonic_Read",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for IDLE_Task */
osThreadId_t IDLE_TaskHandle;
const osThreadAttr_t IDLE_Task_attributes = {
  .name = "IDLE_Task",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for PID_DIR_Task */
osThreadId_t PID_DIR_TaskHandle;
const osThreadAttr_t PID_DIR_Task_attributes = {
  .name = "PID_DIR_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for PID_ESQ_Task */
osThreadId_t PID_ESQ_TaskHandle;
const osThreadAttr_t PID_ESQ_Task_attributes = {
  .name = "PID_ESQ_Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for Sensors_State */
osMessageQueueId_t Sensors_StateHandle;
const osMessageQueueAttr_t Sensors_State_attributes = {
  .name = "Sensors_State"
};
/* Definitions for Bluetooth_comands */
osMessageQueueId_t Bluetooth_comandsHandle;
const osMessageQueueAttr_t Bluetooth_comands_attributes = {
  .name = "Bluetooth_comands"
};
/* Definitions for Operation_Modes */
osEventFlagsId_t Operation_ModesHandle;
const osEventFlagsAttr_t Operation_Modes_attributes = {
  .name = "Operation_Modes"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void HCSR04_Read(void);
/* USER CODE END FunctionPrototypes */

void Autonomus(void *argument);
void Manual(void *argument);
void Sensors(void *argument);
void Ultrasonic(void *argument);
void IDLE(void *argument);
void PID_M_DIR(void *argument);
void PID_M_ESQ(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Sensors_State */
  Sensors_StateHandle = osMessageQueueNew (3, sizeof(uint8_t), &Sensors_State_attributes);

  /* creation of Bluetooth_comands */
  Bluetooth_comandsHandle = osMessageQueueNew (8, sizeof(uint8_t), &Bluetooth_comands_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	if (Sensors_StateHandle == NULL) {
		Error_Handler();
	}

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Autonomus_Mode */
  Autonomus_ModeHandle = osThreadNew(Autonomus, NULL, &Autonomus_Mode_attributes);

  /* creation of Manual_Mode */
  Manual_ModeHandle = osThreadNew(Manual, NULL, &Manual_Mode_attributes);

  /* creation of SensorsReading */
  SensorsReadingHandle = osThreadNew(Sensors, NULL, &SensorsReading_attributes);

  /* creation of Ultrasonic_Read */
  Ultrasonic_ReadHandle = osThreadNew(Ultrasonic, NULL, &Ultrasonic_Read_attributes);

  /* creation of IDLE_Task */
  IDLE_TaskHandle = osThreadNew(IDLE, NULL, &IDLE_Task_attributes);

  /* creation of PID_DIR_Task */
  PID_DIR_TaskHandle = osThreadNew(PID_M_DIR, NULL, &PID_DIR_Task_attributes);

  /* creation of PID_ESQ_Task */
  PID_ESQ_TaskHandle = osThreadNew(PID_M_ESQ, NULL, &PID_ESQ_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */

	osThreadSuspend(Autonomus_ModeHandle);
	osThreadSuspend(Manual_ModeHandle);
	osThreadSuspend(SensorsReadingHandle);
	osThreadSuspend(Ultrasonic_ReadHandle);
	osThreadSuspend(PID_DIR_TaskHandle);
	osThreadSuspend(PID_ESQ_TaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of Operation_Modes */
  Operation_ModesHandle = osEventFlagsNew(&Operation_Modes_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Autonomus */
/**
 * @brief  Function implementing the Autonomus_Mode thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_Autonomus */
void Autonomus(void *argument)
{
  /* USER CODE BEGIN Autonomus */
	/*static double veloc = 8;*/
	/*	static GPIO_PinState pin_dir;
	 static GPIO_PinState pin_esq;
	 static GPIO_PinState pin_re;*/
	static uint8_t min_dist = 10;
	static uint32_t decisao;

	const uint8_t REFdebounce = 5;
	static uint8_t in1_0_dir = 0;
	static uint8_t in1_1_dir = 0;
	static uint8_t in1_0_esq = 0;
	static uint8_t in1_1_esq = 0;
	static uint8_t in1_0_re = 0;
	static uint8_t in1_1_re = 0;

	static const uint8_t distancia = 10;
	static const uint8_t angulo_90 = 45;
	static const uint8_t angulo_60 = 30;

	HAL_TIM_PWM_Start(&htim1, MOTOR_ESQ);
	osDelay(10);
	HAL_TIM_PWM_Start(&htim1, MOTOR_DIR);
	osDelay(10);
	init_motors(veloc);
	frente(veloc);
/*	set_speed(12,M_ESQ);
	set_speed(12,M_DIR);*/

	/* Infinite loop */
	for (;;) {
		osMessageQueueGet(Sensors_StateHandle, &Distancia, osPriorityNormal,
				(uint32_t)0U);
		if (Distancia <= min_dist) {
			stop();
			RPM.dir=0;
			RPM.esq=0;
			decisao = HAL_GetTick(); /*Consulta o tick atual: "Olha a hora"*/
			if ((decisao % 2) == 0) {
				re(veloc);
				osDelay(time_wait_ms(distancia, duty_2_rpm(veloc)));
				rot_esq(veloc,angulo_90);
				frente(veloc);
			} else {
				re(veloc);
				osDelay(time_wait_ms(distancia, duty_2_rpm(veloc)));
				rot_dir(veloc,angulo_90);
				frente(veloc);
			}

		}
		if (pin_re == GPIO_PIN_RESET) {

			in1_0_re++;
			in1_1_re = 0;
			if (in1_0_re >= REFdebounce) {
				in1_1_re = REFdebounce + 1;
				stop();
				RPM.dir=0;
				RPM.esq=0;
				frente(veloc);
				osDelay(time_wait_ms(distancia, duty_2_rpm(veloc)));
				count_re++;
			}

		} else {
			in1_0_re = 0;
			in1_1_re++;
			if (in1_1_re >= REFdebounce) {
				in1_1_re = REFdebounce + 1;
				frente(veloc);
			}
		}

		if (pin_dir == GPIO_PIN_RESET) {
			in1_0_dir++;
			in1_1_dir = 0;
			if (in1_0_dir >= REFdebounce) {
				in1_1_dir = REFdebounce + 1;

				/*Confirmado acionamento*/
				stop();
				RPM.dir=0;
				RPM.esq=0;
				re(veloc);
				osDelay(time_wait_ms(distancia, duty_2_rpm(veloc)));
				rot_esq(veloc,angulo_60);
				frente(veloc);
				count_dir++;
			}
		} else {
			in1_0_dir = 0;
			in1_1_dir++;
			if (in1_1_dir >= REFdebounce) {
				in1_1_dir = REFdebounce + 1;
			}
		}

		if (pin_esq == GPIO_PIN_RESET) {
			in1_0_esq++;
			in1_1_esq = 0;
			if (in1_0_esq >= REFdebounce) {
				in1_1_esq = REFdebounce + 1;
				stop();
				RPM.dir=0;
				RPM.esq=0;
				re(veloc);
				osDelay(time_wait_ms(distancia, duty_2_rpm(veloc)));
				rot_dir(veloc,angulo_60);
				frente(veloc);
				count_esq++;
			}
		} else {
			in1_0_esq = 0;
			in1_1_esq++;
			if (in1_1_esq >= REFdebounce) {
				in1_1_esq = REFdebounce + 1;
			}
		}

		if ((pin_dir && pin_esq) == GPIO_PIN_SET) {
			frente(veloc);
		}
		osDelay(10);
	}
  /* USER CODE END Autonomus */
}

/* USER CODE BEGIN Header_Manual */
/**
 * @brief Function implementing the Manual_Mode thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Manual */
void Manual(void *argument)
{
  /* USER CODE BEGIN Manual */
	/* Infinite loop */
	//uint8_t comando;
	int veloc_buff[3];
	/*uint8_t velocidade=15;*/
	for (;;) {
		osMessageQueueGet(Bluetooth_comandsHandle, &comando, osPriorityRealtime,
				0U);
		switch (comando[0]) {
		case 'S':
			stop();
			RPM.dir=0;
			RPM.esq=0;
			break;
		case 'F':
			frente(velocidade);
			break;
		case 'L':
			rotacao_E();
			break;
		case 'R':
			rotacao_D();
			break;
		case 'G':
			re(velocidade);
			break;
		case 'E':
			direita(velocidade);
			break;
		case 'Q':
			esquerda(velocidade);
			break;
		default:
			/*stop();*/
			break;
		}
		memset(comando, 0x0, sizeof(comando));
		osDelay(2);
	}
  /* USER CODE END Manual */
}

/* USER CODE BEGIN Header_Sensors */
/**
 * @brief Function implementing the SensorsReading thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Sensors */
void Sensors(void *argument)
{
  /* USER CODE BEGIN Sensors */
	/* Infinite loop */
	for (;;) {
		/*Lendo o Estado de sensores infravermelho*/
		pin_re = HAL_GPIO_ReadPin(INF_RE_GPIO_Port, INF_RE_Pin);
		pin_esq = HAL_GPIO_ReadPin(INF_ESQ_GPIO_Port, INF_ESQ_Pin);
		pin_dir = HAL_GPIO_ReadPin(INF_DIR_GPIO_Port, INF_DIR_Pin);
		osDelay(2);
	}
  /* USER CODE END Sensors */
}

/* USER CODE BEGIN Header_Ultrasonic */
/**
 * @brief Function implementing the Ultrasonic_Read thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_Ultrasonic */
void Ultrasonic(void *argument)
{
  /* USER CODE BEGIN Ultrasonic */
	/* Infinite loop */
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	for (;;) {
		HCSR04_Read();
		osDelay(100);
	}
  /* USER CODE END Ultrasonic */
}

/* USER CODE BEGIN Header_IDLE */
/**
 * @brief Function implementing the IDLE_Task thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_IDLE */
void IDLE(void *argument)
{
  /* USER CODE BEGIN IDLE */
	HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 10);
	uint32_t flagsX;
	/* Infinite loop */
	for (;;) {
		flagsX =  osEventFlagsWait(Operation_ModesHandle,TURN_ON_AUTONOMUS|
				TURN_ON_MANUAL|TURN_OFF_AUTONOMUS|TURN_OFF_MANUAL, osFlagsWaitAny, osWaitForever);
		if(flagsX&TURN_ON_AUTONOMUS){
			osDelay(2000); /*Aguarda 2seg*/
			osThreadSuspend(Manual_ModeHandle);
			osThreadResume(Autonomus_ModeHandle);
			osThreadResume(SensorsReadingHandle);
			osThreadResume(Ultrasonic_ReadHandle);
			osThreadResume(PID_DIR_TaskHandle);
			osThreadResume(PID_ESQ_TaskHandle);
		}else if(flagsX &TURN_ON_MANUAL){
			stop();
			osThreadSuspend(Autonomus_ModeHandle);
			osThreadSuspend(SensorsReadingHandle);
			osThreadSuspend(Ultrasonic_ReadHandle);
			osThreadResume(Manual_ModeHandle);
			osThreadResume(PID_DIR_TaskHandle);
			osThreadResume(PID_ESQ_TaskHandle);
		}else if(flagsX &TURN_OFF_MANUAL){
			osThreadSuspend(Autonomus_ModeHandle);
			osThreadSuspend(SensorsReadingHandle);
			osThreadSuspend(Ultrasonic_ReadHandle);
		}else if(flagsX &TURN_OFF_MANUAL){
			osThreadSuspend(Manual_ModeHandle);
		}
		osDelay(1);
	}
  /* USER CODE END IDLE */
}

/* USER CODE BEGIN Header_PID_M_DIR */
/**
* @brief Function implementing the PID_DIR_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PID_M_DIR */
void PID_M_DIR(void *argument) {
	/* USER CODE BEGIN PID_M_DIR */
	/* Infinite loop */
	init_pulso();
	/*System Params*/
	static const uint32_t timehold = TIMEHOLD;
	/*PID instances*/
	/*Motor Dir*/
	DIR.PID.Kp = 2; /* Proporcional */
	DIR.PID.Ki = 0.005; /* Integral */
	DIR.PID.Kd = 0.004; /* Derivative */
	DIR.Kp_old = DIR.PID.Kp;
	DIR.Ki_old = DIR.PID.Ki;
	DIR.Kd_old = DIR.PID.Kd;
	/* PID error */
	DIR.pid_error = 0;
	/*BUffer Len to mov. avrg*/
	DIR.len_mov_avr = (uint8_t) sizeof(DIR.vet_mov_avr) / sizeof(double);
	/*Velocity Setpoint*/
	DIR.SETPOINT = 100;
	DIR.MOT = M_DIR;
	uint32_t iteration_time;
	/* Initialize PID system, float32_t format */
	arm_pid_init_f32(&DIR.PID, 1);
	for (;;) {
		if (DIR.reload == true) {
			arm_pid_init_f32(&DIR.PID, 0);
			DIR.Kp_old = DIR.PID.Kp;
			DIR.Ki_old = DIR.PID.Ki;
			DIR.Kd_old = DIR.PID.Kd;
			DIR.reload = false;
		}
		iteration_time = HAL_GetTick() - DIR.old_time;
		if (iteration_time >= TIMEHOLD) {
			/*getting setpoint value for motor dir*/
			__disable_irq();
			/*RPM calculus*/
			DIR.RPM = get_pulso(DIR.MOT) * 3000
					/ (HAL_GetTick() - DIR.old_time);
			DIR.old_time = HAL_GetTick();
			reset_pulso(DIR.MOT);
			__enable_irq();
		}
		DIR_TEST = (uint8_t)DIR.RPM;
		/*PID error calculus*/
		DIR.pid_error = DIR.SETPOINT - DIR.RPM;/* movingAvg_Dir(array_dir, len, RPM.dir, 0);*/
		DIR.pid_out = arm_pid_f32(&DIR.PID, DIR.pid_error);
		if (((DIR.pid_out > 0) && (DIR.pid_error>0))
				|| ((DIR.pid_out<0) && (DIR.pid_error <0 ))) {
			DIR.sign = 1;
		} else {
			DIR.sign = 0;
		}
		DIR.duty = rpm_2_duty(DIR.pid_out);
		/*		pid_curr_error_D = SET_POINT.DIR - movingAvg_Dir(array_dir, len, RPM.dir, 0);
		 integration_sum_D += (pid_curr_error_D * iteration_time);
		 duty_dir = PID_dir.Kp * pid_curr_error_D + PID_dir.Ki * integration_sum_D + PID_dir.Kd * 1000 * (pid_curr_error_D - pid_error_dir)/iteration_time;
		 pid_error_dir = pid_curr_error_D;*/
		/*Anti Wind-up implementation for motor dir*/
		if (DIR.duty > 100.0) {
			DIR.duty = 99.7;
			DIR.saturation = 1;
		} else if (DIR.duty < 0) {
			DIR.duty = 0;
			DIR.saturation = 1;
		} else {
			DIR.saturation = 0;
		}
		adjust_PWM(DIR.duty, DIR.MOT);
		if (DIR.saturation && DIR.sign) {
			DIR.PID.Ki = 0;
			DIR.reload = true;
		} else {
			DIR.PID.Ki = DIR.Ki_old;
		}
		osDelay(1);
	}
	/* USER CODE END PID_M_DIR */
}

/* USER CODE BEGIN Header_PID_M_ESQ */
/**
* @brief Function implementing the PID_ESQ_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_PID_M_ESQ */
void PID_M_ESQ(void *argument) {
	/* USER CODE BEGIN PID_M_ESQ */
	/* Infinite loop */
	init_pulso();
	/*System Params*/
	static const uint32_t timehold = TIMEHOLD;

	/*PID instances*/
	/*Motor Esq*/
	ESQ.PID.Kp = 2; /* Proporcional */
	ESQ.PID.Ki = 0.005; /* Integral */
	ESQ.PID.Kd = 0.006; /* Derivative */
	ESQ.Kp_old = ESQ.PID.Kp;
	ESQ.Ki_old = ESQ.PID.Ki;
	ESQ.Kd_old = ESQ.PID.Kd;
	/* PID error */
	ESQ.pid_error = 0;

	/*BUffer Len to mov. avrg*/
	ESQ.len_mov_avr = (uint8_t) sizeof(ESQ.vet_mov_avr) / sizeof(double);

	/*Velocity Setpoint*/
	ESQ.SETPOINT = 100;

	ESQ.MOT = M_ESQ;

	uint32_t iteration_time;

	/* Initialize PID system, float32_t format */
	arm_pid_init_f32(&ESQ.PID, 1);

	for (;;) {
		if (ESQ.reload == true) {
			arm_pid_init_f32(&ESQ.PID, 0);
			ESQ.Kp_old = ESQ.PID.Kp;
			ESQ.Ki_old = ESQ.PID.Ki;
			ESQ.Kd_old = ESQ.PID.Kd;
			ESQ.reload = false;
		}
		iteration_time = HAL_GetTick() - ESQ.old_time;
		if (iteration_time >= TIMEHOLD) {
			/*getting setpoint value for motor esq*/
			__disable_irq();
			/*RPM calculus*/
			ESQ.RPM = get_pulso(ESQ.MOT) * 3000
					/ (HAL_GetTick() - ESQ.old_time);
			ESQ.old_time = HAL_GetTick();
			reset_pulso(ESQ.MOT);
			__enable_irq();
		}

		/*PID error calculus*/
		ESQ_TEST = (uint8_t)ESQ.RPM;
		ESQ.pid_error = ESQ.SETPOINT - ESQ.RPM;/* movingAvg_Dir(array_dir, len, RPM.dir, 0);*/
		ESQ.pid_out = arm_pid_f32(&ESQ.PID, ESQ.pid_error);
		if (((ESQ.pid_out > 0) && (ESQ.pid_error>0))
				|| ((ESQ.pid_out<0) && (ESQ.pid_error <0 ))) {
			ESQ.sign = 1;
		} else {
			ESQ.sign = 0;
		}
		ESQ.duty = rpm_2_duty(ESQ.pid_out);
		/*		pid_curr_error_D = SET_POINT.DIR - movingAvg_Dir(array_dir, len, RPM.dir, 0);
		 integration_sum_D += (pid_curr_error_D * iteration_time);
		 duty_dir = PID_dir.Kp * pid_curr_error_D + PID_dir.Ki * integration_sum_D + PID_dir.Kd * 1000 * (pid_curr_error_D - pid_error_dir)/iteration_time;
		 pid_error_dir = pid_curr_error_D;*/
		/*Anti Wind-up implementation for motor dir*/
		if (ESQ.duty > 100.0) {
			ESQ.duty = 99.7;
			ESQ.saturation = 1;
		} else if (ESQ.duty < 0) {
			ESQ.duty = 0;
			ESQ.saturation = 1;
		} else {
			ESQ.saturation = 0;
		}
		adjust_PWM(ESQ.duty, ESQ.MOT);
		if (ESQ.saturation && ESQ.sign) {
			ESQ.PID.Ki = 0;
			ESQ.reload = true;
		} else {
			ESQ.PID.Ki = ESQ.Ki_old;
		}
		osDelay(1);
	}
	/* USER CODE END PID_M_ESQ */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	switch (GPIO_Pin) {
	case BUT_Pin:
		stop();
		BUT_COUNT++;
		LED_OFF;
		veloc = veloc+1;
		osEventFlagsSet(Operation_ModesHandle, TURN_ON_AUTONOMUS);
		break;

	case VEL_DIR_Pin:
		inc_pulso(M_DIR);
		break;

	case VEL_ESQ_Pin:
		inc_pulso(M_ESQ);
		break;

	default:
		ERROR_COUNT++;
		break;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	static uint8_t Distance = 0xff;
	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) // if the interrupt source is channel1
			{
		if (Is_First_Captured == 0) // if the first value is not captured
				{
			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
			Is_First_Captured = 1;  // set the first captured as true
			// Now change the polarity to falling edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_FALLING);
		}

		else if (Is_First_Captured == 1)   // if the first is already captured
				{
			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read second value
			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1) {
				Difference = IC_Val2 - IC_Val1;
			}

			else if (IC_Val1 > IC_Val2) {
				Difference = (0xffff - IC_Val1) + IC_Val2;
			}

			Distance = Difference * .034 / 2;
			Is_First_Captured = 0; // set it back to false

			osMessageQueuePut(Sensors_StateHandle, &Distance, osPriorityNormal,
					0U);

			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1,
					TIM_INPUTCHANNELPOLARITY_RISING);
			__HAL_TIM_DISABLE_IT(&htim1, TIM_IT_CC1);
		}
	}
}

void HCSR04_Read(void) {
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET); // pull the TRIG pin HIGH
	osDelay(10);  // wait for 10 us
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET); // pull the TRIG pin low

	__HAL_TIM_ENABLE_IT(&htim2, TIM_IT_CC1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	static uint8_t i;
	static uint8_t j;
	HAL_UART_Receive_IT(&huart1, UART1_rxBuffer, 3);

	switch(UART1_rxBuffer[0]){
	case 'X':
		osEventFlagsSet(Operation_ModesHandle, TURN_ON_MANUAL);
		break;
	case 'Y':
		osEventFlagsSet(Operation_ModesHandle, TURN_ON_AUTONOMUS);
		break;
	case 'J':
		velocidade = (UART1_rxBuffer[1]-48)*10 + (UART1_rxBuffer[2]-48);
		break;
	case 'P':
		DIR.PID.Kp++; /*= PID_esq.Kp + 0.1;*/       /* Proporcional */
		DIR.Kp_old = DIR.PID.Kp;
		DIR.reload = true;
		ESQ.PID.Kp++; /*= PID_dir.Kp + 0.1;*/
		ESQ.Kp_old = ESQ.PID.Kp;
		ESQ.reload = true;
		break;
	case 'p':
		DIR.PID.Kp--; /*= PID_esq.Kp - 0.1;  */     /* Proporcional */
		DIR.Kp_old = DIR.PID.Kp;
		DIR.reload = true;
		ESQ.PID.Kp--; /*= PID_dir.Kp - 0.1;*/
		ESQ.Kp_old = ESQ.PID.Kp;
		ESQ.reload = true;
		break;
	case 'I':
		DIR.PID.Ki++; /* = PID_esq.Ki + 0.1;*/        /* Integral */
		DIR.Ki_old = DIR.PID.Ki;
		DIR.reload = true;
		ESQ.PID.Ki++; /*= PID_dir.Ki + 0.1;*/
		ESQ.Ki_old = ESQ.PID.Ki;
		ESQ.reload = true;
		break;
	case 'i':
		DIR.PID.Ki--; /*= PID_esq.Ki - 0.1;*/         /* Integral */
		DIR.Ki_old = DIR.PID.Ki;
		DIR.reload = true;
		ESQ.PID.Ki--; /*= PID_dir.Ki - 0.1;*/
		ESQ.Ki_old = ESQ.PID.Ki;
		ESQ.reload = true;
		break;
	case 'D':
		DIR.PID.Kd++; /*= PID_esq.Kd + 0.1; */        /* Derivative */
		DIR.Kd_old = DIR.PID.Kd;
		DIR.reload = true;
		ESQ.PID.Kd++; /*= PID_dir.Kd + 0.1;*/
		ESQ.Kd_old = ESQ.PID.Kd;
		ESQ.reload = true;
		break;
	case 'd':
		DIR.PID.Kd--; /* = PID_esq.Kd - 0.1;*/        /* Derivative */
		DIR.Kd_old = DIR.PID.Kd;
		DIR.reload = true;
		ESQ.PID.Kd--; /*= PID_dir.Kd - 0.1;*/
		ESQ.Kd_old = ESQ.PID.Kd;
		ESQ.reload = true;
		break;
	}
/*	j=0;
	for(i=0;i<5;i++){
		if(UART1_rxBuffer[i] != 10){
			to_send[j] = UART1_rxBuffer[i];
			j++;
		}
	}
	osMessageQueuePut(Bluetooth_comandsHandle, &to_send, osPriorityRealtime, 0U);
	memset(UART1_rxBuffer, 0x0, 5);
	memset(to_send, 0x0, 5);*/
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
