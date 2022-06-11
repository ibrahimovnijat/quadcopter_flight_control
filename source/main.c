/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "mpu9250.h"
#include "ms5611.h"
#include "serialCommProtocols.h"
#include "filters.h"
#include "dwt_stm32_delay.h"
#include "auxiliaryFuncs.h"
#include "pid.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  volatile float accelerometerData[3] = {0.0f}, gyroscopeData[3] = {0.0f}, gyroscopeData_filt[3] = {0.0f};
  float accelerometerBias[3] = {0.0f}, gyroscopeBias[3] = {0.0f};
  float prev_accelData[3] = {0.0f}, prev_gryoData[3] = {0.0f};

  uint16_t C[7] = {0};
  int32_t* press_temp_vals;
  float start_altitute = 0.0f, start_altitude_filt = 0.0f;
  float current_altitude=0.0f, current_altitude_filt = 0.0f;
  int32_t pressure = 0, pressure_filt = 0, prev_pressure = 0, prev_pressure2 = 0;
  float prev_alt = 0.0f, prev_alt2 = 0.0f, prev_alt3 = 0.0f;

  volatile float rollAngle = 0.0f, pitchAngle = 0.0f;
  float rollAngle_prev = 0.0f, pitAngle_prev = 0.0f;
  volatile float rollAngle_ref = 0.0f, pitchAngle_ref = 0.0f, yawAngle_error=0.0f;
  volatile float throttle=0.0f;

  volatile float rollRate_ref = 0.0f, pitchRate_ref = 0.0f, yawRate_ref = 0.0f;
  float rollRate_ref_bf = 0.0f, pitchRate_ref_bf = 0.0f, yawRate_ref_bf = 0.0f;
  float rollRate_ref_prev = 0.0f, pitchRate_ref_prev = 0.0f, yawRate_ref_prev = 0.0f;

  volatile uint16_t captured_value[4] = {0};
  volatile uint8_t pointer = 0;

  /*Main PID initialization -------------------*/
  float rollAngle_ref_prev = 0.0f, pitchAngle_ref_prev = 0.0f, yawAngle_error_prev = 0.0f;
  float I_val_roll = 0.0f, I_val_pitch= 0.0f, I_val_yaw = 0.0f;
  float D_val_roll = 0.0f, D_val_pitch = 0.0f, D_val_yaw = 0.0f, D_val_roll_prev = 0.0f, D_val_pitch_prev = 0.0f, D_val_yaw_prev = 0.0f;
  volatile float PID_val_roll = 0.0f, PID_val_pitch = 0.0f;// PID_val_yaw = 0.0f;

  /*Angular rate PID initialization -----------*/
  float I_val_roll_angularRate = 0.0f, I_val_pitch_angularRate = 0.0f, I_val_yaw_angularRate = 0.0f;
  float rollPid_prev = 0.0f, pitchPid_prev = 0.0f, yawPid_prev = 0.0f;
  volatile float PID_val_roll_angularRate = 0.0f, PID_val_pitch_angularRate = 0.0f, PID_val_yaw_angularRate = 0.0f;
  float gyroscopeData_roll_prev = 0.0f, gyroscopeData_pitch_prev = 0.0f, gyroscopeData_yaw_prev = 0.0f;
  float D_val_roll_angularRate_prev = 0.0f, D_val_pitch_angularRate_prev = 0.0f, D_val_yaw_angularRate_prev = 0.0f;


  /* PID gains ---------------------------------*/
  float _rollRate_error_prev = 0.0f, _pitchRate_error_prev = 0.0f, _yawRate_error_prev = 0.0f;
  float  rollAngle_error_prev = 0.0f, pitchAngle_error_prev = 0.0f;

  const float  P_gain_roll_angularRate  = 0.50f, I_gain_roll_angularRate  = 0.008f,  D_gain_roll_angularRate  = 0.005f;
  const float	 P_gain_pitch_angularRate = 0.65f, I_gain_pitch_angularRate = 0.008f,  D_gain_pitch_angularRate = 0.005f;
  const float  P_gain_yaw_angularRate   = 2.80f, I_gain_yaw_angularRate   = 0.001f,  D_gain_yaw_angularRate =   0.01f;

  const float P_gain_roll  = 3.5f, I_gain_roll  = 0.01f,  D_gain_roll  = 0.0f;  // D_gain_roll = 0.03f;
  const float P_gain_pitch = 3.5f, I_gain_pitch = 0.01f,  D_gain_pitch = 0.0f;  // D_gain_pitch = 0.03f;

  /*Motor Initialization ----------------------- */
  uint32_t leftFrontMotor = 0, rightFrontMotor = 0, leftBackMotor = 0, rightBackMotor = 0;

  volatile uint32_t start_throttle_cnt = 0, stop_throttle_cnt = 0;
  volatile bool start_throttle = 0, stop_throttle = 1;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

volatile uint32_t angle_pid_counter = 0, rate_pid_counter = 0, fusion_counter = 0, input_counter = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	if (htim->Instance == TIM11){
		if (throttle < 1070.0f){
			resetPID_ar();
		}
		else if (throttle >= 1070.0f){
			ef_to_bf_conversion(&rollRate_ref_bf, &pitchRate_ref_bf, &yawRate_ref_bf, PID_val_roll, PID_val_pitch, -yawRate_ref); //why yawRate_ref negative??
			pid_inner(PID_val_roll, PID_val_pitch, yawRate_ref, (float*)&gyroscopeData_filt);
		}
		motorMixer(throttle, PID_val_roll_angularRate, PID_val_pitch_angularRate, PID_val_yaw_angularRate);
		if (rollAngle > 70.0f || rollAngle < -70.0f){
			resetPID_ar();
			resetPID();
			turn_off_motors();
		}
		else{
			motors_output(leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor);
		}
	}
	if (htim->Instance == TIM13){
		if (throttle < 1070.0f){
			resetPID();
		}
		else if (throttle >= 1070.0f){
			pid_outer();
		}
	}

	if (htim->Instance == TIM7){
		readAccelerometerData_SPI((float*)&accelerometerData);
		readGyroscopeData_SPI((float*)&gyroscopeData);
		lowPassGyro_MoveAve((float*)&gyroscopeData, (float*)&gyroscopeData_filt);
		MadgwickAHRSupdateIMU(-gyroscopeData_filt[0], gyroscopeData_filt[1], -gyroscopeData_filt[2], -accelerometerData[0], accelerometerData[1], -accelerometerData[2]);
		calculateAnglesFromQuaternion();
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef* htim)
{
	uint16_t temp;
	temp = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

	if (temp > 10000){
		pointer = 0;
	}
	else{
		captured_value[pointer] = temp;
		if (captured_value[pointer] < 1050){
			captured_value[pointer] = 1000;
		}
		else if (captured_value[pointer] > 2000){
			captured_value[pointer] = 2000;
		}
		pointer++;
	    rollAngle_ref  = (float)roundRoll(trimAngle(PWMReadingToRollReferenceAngle(captured_value[2]), 3.0, -3.0));
	    pitchAngle_ref = (float)roundPitch(trimAngle(PWMReadingToPitchReferenceAngle(captured_value[1]), 3.0, -3.0));
	    yawRate_ref    = (float)roundYawError(trimAngle(PWMReadingToYawRateEror(captured_value[3]), 10.0, -10.0));
	    throttle 	   = (float)captured_value[0];

	    if (throttle <= 1050.0f){
	    	if (yawRate_ref <= -190.0f){
				if ((start_throttle == 0) && (stop_throttle == 1)){
					start_throttle_cnt += 1;
					if ((start_throttle_cnt >= 200) && (stop_throttle_cnt <= 50)){
						start_throttle = 1;
						stop_throttle  = 0;
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
						DWT_Delay_us(1000000); // 1 sec delay
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
					}
				}
				else if ((start_throttle == 1) && (stop_throttle == 0)){
					stop_throttle_cnt += 1;
					//printf("stop th: %lu\n", stop_throttle_cnt);
					if ((stop_throttle_cnt >= 200) && (start_throttle_cnt <= 50)){
						stop_throttle = 1;
						start_throttle = 0;
					}
				}
	    	}
	    }
	}
	if (pointer == 4){
		pointer = 0;
	}

    if (stop_throttle == 1){
    	throttle = 0.0f;
    }
}

uint32_t micros()
{
	return DWT->CYCCNT/180;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */

int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */



  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  MX_TIM13_Init();
  MX_TIM11_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_I2C1_Init();
  MX_UART5_Init();


  /* USER CODE BEGIN 2 */


  HAL_Delay(500);
  printf("Initializing...\n");

  DWT_Delay_Init();

  // initialize barometer and MPU
  initMPU9250_SPI();
  whoAmI_MPU9250_SPI();
  init_MS5611();

  mpuCalibration_SPI((float*)&accelerometerBias, (float*)&gyroscopeBias);
  HAL_Delay(500);

  // start timers in interrupt mode
  HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Base_Start_IT(&htim11);
  HAL_TIM_Base_Start_IT(&htim13);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
