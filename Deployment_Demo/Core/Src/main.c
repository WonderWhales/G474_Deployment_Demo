/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "usart.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "BNO055.h"
#include "motor.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void deployDirtbrake(void);
void bayOrientation(void);
void armDeployment(void);
void armRetraction(void);
void retractDirtbrake(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
BNO055_Axis_Vec_t eulerVec;
BNO055_AXIS_CONFIG_t axisConfig;
DCMotor_Config_t dcConfig;
DCMotor_Instance_t bayMotor;
DCMotor_Instance_t armMotor;
Actuator_Config_t actConfig;
Actuator_Instance_t actInstance;

typedef struct{

  void (*taskFunc)();

} state_task_t;

typedef enum{
  ARM_DEPLOYING,
  ARM_DONE_DEPLOYING,
  ARM_RETRACTING,
  ARM_DONE_RETRACTING
} Arm_State_t;

Arm_State_t armState;

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
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //Actuator Config
  actConfig.Min_Pulse           = 900;
  actConfig.Max_Pulse           = 2100;
  actConfig.Min_Length          = 0;
  actConfig.Max_Length          = 27;
  actConfig.Desired_Min_Length  = 0;
  actConfig.Desired_Max_Length  = 16;

  actInstance.Act_Timer = &htim3;
  actInstance.Channel   = TIM_CHANNEL_1;
  actInstance.config    = &actConfig;

  if(Actuator_Init(&actInstance) != ACTUATOR_OK){
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    while(1);
  }

  //DC Motor Initialization
  dcConfig.Min_Speed = 0;
  dcConfig.Max_Speed = 100;

  bayMotor.DC_Timer    = &htim2;
  bayMotor.IN1_Channel = TIM_CHANNEL_1;
  bayMotor.IN2_Channel = TIM_CHANNEL_2;
  bayMotor.config      = &dcConfig;

  if(DCMotor_Init(&bayMotor) != DC_MOTOR_OK){
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    while(1);
  }

  // Drive_DCMotor(&bayMotor, 70, CLOCKWISE);

  armMotor.DC_Timer    = &htim2;
  armMotor.IN1_Channel = TIM_CHANNEL_3;
  armMotor.IN2_Channel = TIM_CHANNEL_4;
  armMotor.config      = &dcConfig;

  if(DCMotor_Init(&armMotor) != DC_MOTOR_OK){
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    while(1);
  }

  // Drive_DCMotor(&armMotor, 70, CLOCKWISE);

  //BNO055 Initialization
  HAL_Delay(700);
  BNO055_I2C_Mount(&hi2c1);
  if(BNO055_Init() != BNO055_SUCCESS){
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    while(1);
  }
  BNO055_Set_OP_Mode(NDOF);

  state_task_t task[5];

  task[0].taskFunc = deployDirtbrake;
  task[1].taskFunc = bayOrientation;
  task[2].taskFunc = armDeployment;
  task[3].taskFunc = armRetraction;
  task[4].taskFunc = retractDirtbrake;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    task[0].taskFunc();
    task[1].taskFunc();
    task[2].taskFunc();
    task[3].taskFunc();
    task[4].taskFunc();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
  if(armState == ARM_DEPLOYING)
    armState = ARM_DONE_DEPLOYING;
  else if(armState == ARM_RETRACTING)
    armState = ARM_DONE_RETRACTING;
}

void deployDirtbrake(void){
  HAL_GPIO_WritePin(DIRTBRAKE_LED_GPIO_Port, DIRTBRAKE_LED_Pin, GPIO_PIN_SET);
  Drive_Actuator(&actInstance, 16);
  HAL_Delay(5000);
}

void bayOrientation(void){
  HAL_GPIO_WritePin(BAY_LED_GPIO_Port, BAY_LED_Pin, GPIO_PIN_SET);
  Drive_DCMotor(&bayMotor, 70, CLOCKWISE);
  
  for(uint8_t i = 0; i < 100; i++){
    BNO055_Get_Euler_Vec(&eulerVec);
    HAL_Delay(10);
  }

  if(eulerVec.y < 0){
    while(eulerVec.y <= 0){
      BNO055_Get_Euler_Vec(&eulerVec);
      HAL_Delay(10);
    }
  }
  else if(eulerVec.y > 0){
    while(eulerVec.y >= 0){
      BNO055_Get_Euler_Vec(&eulerVec);
      HAL_Delay(10);
    }
  }

  Stop_DCMotor(&bayMotor);
}

void armDeployment(void){
  armState = ARM_DEPLOYING;
  Drive_DCMotor(&armMotor, 70, COUNTER_CLOCKWISE);
  while(armState == ARM_DEPLOYING);
  Stop_DCMotor(&armMotor);
}

void armRetraction(void){
  armState = ARM_RETRACTING;
  Drive_DCMotor(&armMotor, 70, CLOCKWISE);
  while(armState == ARM_RETRACTING);
  Stop_DCMotor(&armMotor);
}

void retractDirtbrake(void){
  Drive_Actuator(&actInstance, 0);
  HAL_Delay(5000);
}

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
