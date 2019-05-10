/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
const uint16_t delay_time=10;
uint8_t linear_direction;
uint8_t rotary_direction;
uint8_t r_data;
uint8_t data[30];

uint8_t stepper_motion=0;
volatile uint8_t  steps_counter=0;
volatile uint16_t stepper_counter=0;
const uint16_t step_high_a[]={STEPPER_A_Pin, STEPPER_B_Pin, STEPPER_B_Pin, STEPPER_A_Pin};
const uint16_t step_high_b[]={STEPPER_C_Pin, STEPPER_C_Pin, STEPPER_D_Pin, STEPPER_D_Pin};
const uint16_t step_low_a[]={STEPPER_B_Pin, STEPPER_A_Pin, STEPPER_A_Pin, STEPPER_B_Pin};
const uint16_t step_low_b[]={STEPPER_D_Pin, STEPPER_D_Pin, STEPPER_C_Pin, STEPPER_C_Pin};
volatile char command[6];
volatile uint8_t command_counter=0;
volatile uint8_t stepper_direction=0;
volatile int8_t stepper_output=0;
volatile uint16_t stepper_steps=0;
volatile uint16_t encoder_one_pulses, encoder_two_pulses;
volatile uint16_t systick_counter=0;
volatile uint16_t delay_counter;

//speed measure variables
volatile uint32_t position_A_tmp_1, position_A_tmp_2, position_B_tmp_1, position_B_tmp_2, counts_A, counts_B;
uint32_t position_dif_A[3];
uint32_t position_dif_B[3];
volatile uint8_t  measurement_counter=0;
volatile uint8_t sampling;
const uint8_t sampling_time=3;
const uint8_t sampling_window=3;
volatile uint8_t actual_speed_A, actual_speed_B;
volatile uint8_t motor_A_direction=0;
volatile uint8_t motor_B_direction=0;
uint8_t enable_A_speed_regulation=0, enable_B_speed_regulation=0;
volatile uint8_t target_speed_A, target_speed_B;
const uint8_t speed_margin = 3;
const uint8_t pwm_direction_offset = 125;
const uint8_t proportional_gain = 2;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_SYSTICK_Callback(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void parse_command(volatile char* command);
void execute_command(char function, char parameter, uint8_t value);
void stop_motor_AB();
//instrukcje sterujace
void set_speed_linear(uint8_t linear_speed);
void set_speed_rotary(uint8_t roatry_speed);
void set_speed_A(uint8_t A_speed);
void set_speed_B(uint8_t B_speed);
//rozkazy
void move_forward(uint8_t value);
void move_backward(uint8_t value);
void turn_right(uint8_t value);
void turn_left(uint8_t value);
void turret_left(uint8_t value);
void turret_right(uint8_t value);
void turret_up(uint8_t value);
void turret_down(uint8_t value);
void turret_position(uint8_t value);
void servo_A_position(uint8_t angle);
void servo_B_position(uint8_t angle);
void stepper_left(uint8_t value);
void stepper_right(uint8_t value);
void stepper_position(uint8_t value);
void set_stepper_output(uint8_t sequence_index);
void stop_stepper();
void delay_ms(uint16_t time);
uint32_t min_array(uint32_t array[], uint8_t size);
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //DC motors
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); //DC motors
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); //DC motors
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4); //DC motors
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //servo A
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //servo B
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL); //encoder A
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL); //encoder B
  HAL_UART_Receive_IT(&huart3,&r_data,1); //begin receiving

  //stop_motor_AB();
  HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, 0);
  TIM1->CCR1=0;
  TIM1->CCR2=0;
  TIM1->CCR3=0;
  TIM1->CCR4=0;
  stop_stepper();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

//	  HAL_Delay(40);
//	  		cnt++;
//	  if(cnt<90){
//	  	TIM1->CCR1++;
//	  	TIM1->CCR2++;
//	  	TIM1->CCR3++;
//	  	TIM1->CCR4++;
//	  }
//	  else if(cnt>90){
//	  	TIM1->CCR1--;
//	  		TIM1->CCR2--;
//	  		TIM1->CCR3--;
//	  		TIM1->CCR4--;
//	  }
//	  if(cnt==180){
//	  	cnt=0;
//	  }

  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 12;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 249;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 639;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 7683;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 7683;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STEPPER_A_Pin|STEPPER_B_Pin|STEPPER_C_Pin|STEPPER_D_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TEST_LED_Pin */
  GPIO_InitStruct.Pin = TEST_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TEST_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STEPPER_A_Pin STEPPER_B_Pin STEPPER_C_Pin STEPPER_D_Pin */
  GPIO_InitStruct.Pin = STEPPER_A_Pin|STEPPER_B_Pin|STEPPER_C_Pin|STEPPER_D_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void){
systick_counter++;

//if(systick_counter>1000){
//		systick_counter=0;
//
//		uint8_t size = sprintf(data, "Aa:%d B act:%d\nAt:%d Bt:%d\n", actual_speed_A, actual_speed_B, target_speed_A, target_speed_B);
//		HAL_UART_Transmit_IT(&huart3, data, size);
//		HAL_GPIO_TogglePin(TEST_LED_GPIO_Port, TEST_LED_Pin);
//}

/**********POMIAR PREDKOSCI SILNIKOW DC**********/

	if(sampling == 0){
		position_A_tmp_1=TIM3->CNT;
		position_B_tmp_1=TIM4->CNT;
	}
	sampling++;
	if(sampling == sampling_time){
		position_A_tmp_2=TIM3->CNT;
		position_B_tmp_2=TIM4->CNT;
		position_dif_A[measurement_counter]=position_A_tmp_2-position_A_tmp_1;
		position_dif_B[measurement_counter]=position_B_tmp_2-position_B_tmp_1;
		measurement_counter++;
		if(measurement_counter==sampling_window){
			measurement_counter=0;
			actual_speed_B = min_array(position_dif_B, sampling_window);
		}
	}

/*************************************************/

/**********REGULACJA PREDKOSCI SILNIKOW DC*********/


////regulacja predkosci silnikow
//	if(enable_A_speed_regulation==2){
////			if(actual_speed_A-speed_margin<target_speed_A){
////				TIM1->CCR1+(target_speed_A - actual_speed_A)/2;
////				TIM1->CCR2+(target_speed_A - actual_speed_A)/2;
////			}else if(actual_speed_A+speed_margin>target_speed_A){
////				TIM1->CCR1-(target_speed_A - actual_speed_A)/2;
////				TIM1->CCR2-(target_speed_A - actual_speed_A)/2;
////			}
//	}
//
//	switch(motor_B_direction){
//		case 0:
//			if(enable_B_speed_regulation==1  && target_speed_B-actual_speed_B>0){
//					if(actual_speed_B < target_speed_B){
//						TIM1->CCR3--;
//						TIM1->CCR4--;
//					}else if(actual_speed_B > target_speed_B){
//						TIM1->CCR3++;
//						TIM1->CCR4++;
//					}
//			}
//			break;
//		case 1:
//			if(enable_B_speed_regulation==1 && target_speed_B-actual_speed_B>0){
//						if(actual_speed_B < target_speed_B){
//							TIM1->CCR3++;
//							TIM1->CCR4++;
//						}else if(actual_speed_B > target_speed_B){
//							TIM1->CCR3--;
//							TIM1->CCR4--;
//						}
//				}
//
//		}
//		sampling=0;
//	}
//}


/*************************************************/

/**********STEROWANIE SILNIKIEM KROKOWYM**********/
	//obroty krokowki w lewo
if(stepper_motion==1){
	HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, 1);
	stepper_counter++;
	if(stepper_counter==5){
		stepper_counter=0;
		switch(stepper_direction){
		case 0:
			set_stepper_output(steps_counter);
			steps_counter++;
			if(steps_counter==4){
				steps_counter=0;
				stepper_steps--;
//				HAL_GPIO_TogglePin(TEST_LED_GPIO_Port,TEST_LED_Pin);
			}
			break;

		case 1:
			set_stepper_output(steps_counter);
			steps_counter--;
			if(steps_counter==0){
				steps_counter=4;
				stepper_steps--;
//				HAL_GPIO_TogglePin(TEST_LED_GPIO_Port,TEST_LED_Pin);
			}
			break;
		default : break;
		}

	}
	if(stepper_steps==0){
		stepper_motion=0;
		stop_stepper();
		HAL_GPIO_WritePin(TEST_LED_GPIO_Port, TEST_LED_Pin, 0);
	}
}
/*************************************************/

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//HAL_GPIO_TogglePin(TEST_LED_GPIO_Port,TEST_LED_Pin);
	char c_received = (char)r_data;
	command[command_counter]=r_data;
	if(c_received == '\n'){
		command_counter=0;
		parse_command(command);
	}
	else{
		command_counter++;
	}
	HAL_UART_Receive_IT(&huart3,&r_data,1);
}

uint32_t min_array(uint32_t array[], uint8_t size){
	uint32_t tmp_min=array[0];
	for(uint8_t elem=1;elem<size;elem++){
		if(array[elem]<tmp_min){
			tmp_min=array[elem];
		}
	}
	return tmp_min;
}

void set_stepper_output(uint8_t sequence_index){
	HAL_GPIO_WritePin(GPIOB, step_high_a[sequence_index], 1);
	HAL_GPIO_WritePin(GPIOB, step_high_b[sequence_index],1);
	HAL_GPIO_WritePin(GPIOB, step_low_a[sequence_index] ,0);
	HAL_GPIO_WritePin(GPIOB, step_low_b[sequence_index] ,0);
}

void stop_motor_AB(){
	TIM1->CCR1=pwm_direction_offset;
	TIM1->CCR2=pwm_direction_offset;
	TIM1->CCR3=pwm_direction_offset;
	TIM1->CCR4=pwm_direction_offset;
}

void stop_stepper(){
	HAL_GPIO_WritePin(GPIOB, STEPPER_A_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, STEPPER_B_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, STEPPER_C_Pin, 1);
	HAL_GPIO_WritePin(GPIOB, STEPPER_D_Pin, 1);
}
void parse_command(volatile char* command){
	char function, parameter;
	char value[4];
	uint8_t t_data[6];
	uint8_t num_value;
	function=command[0];
	parameter=command[1];
	for (uint8_t c=0;c<4;c++){
		value[c]=command[c+2];
	}
	num_value=atoi(value);
	uint8_t size = sprintf(t_data,"C:%c, P:%c, V:%d\n",function, parameter, num_value);
	HAL_UART_Transmit_IT(&huart3,t_data,size);
	execute_command(function, parameter, num_value);
}

void set_speed_A(uint8_t A_speed){

	TIM1->CCR1=A_speed;
	TIM1->CCR2=A_speed;
//	if(A_speed==0){
//			TIM1->CCR1=125;
//			TIM1->CCR2=125;
//			enable_A_speed_regulation=0;
//	}
//	else{
//		if(A_speed>100){
//			target_speed_A = A_speed - 100;
//			motor_A_direction = 1;
//		}
//		else{
//			target_speed_A = A_speed;
//			motor_A_direction=0;
//		}
//		target_speed_A = A_speed;
//		enable_A_speed_regulation = 1;
//	}
}

void set_speed_B(uint8_t B_speed){
	TIM1->CCR3=B_speed;
	TIM1->CCR4=B_speed;
//	target_speed_B = B_speed;
//	if(B_speed==0){
//			TIM1->CCR3=125;
//			TIM1->CCR4=125;
//			enable_B_speed_regulation=0;
//	}
//	else{
//		if(B_speed>100){
//			target_speed_B = B_speed - 100;
//			motor_B_direction = 1;
//		}
//		else{
//			target_speed_B = B_speed;
//			motor_B_direction = 0;
//		}
//		enable_B_speed_regulation = 1;
//	}
}

void set_speed_linear(uint8_t linear_speed){
	set_speed_A(linear_speed);
	set_speed_B(linear_speed);
}

void set_speed_rotary(uint8_t rotary_speed){

}

void move_forward(uint8_t value){

}

void move_backward(uint8_t value){

}

void turn_left(uint8_t value){

}

void turn_right(uint8_t value){

}

void turret_left(uint8_t value){
	stepper_left(value);
}

void turret_right(uint8_t value){
	stepper_right(value);
}

void turret_up(uint8_t value){

}

void turret_down(uint8_t value){

}

void servo_A_position(uint8_t angle){
	uint16_t pwm_value = 100 + (angle*125)/135;
	TIM2->CCR1=pwm_value;
}

void servo_B_position(uint8_t angle){
	uint16_t pwm_value = 100 + (angle*125)/135;
	TIM2->CCR2=pwm_value;
}

void stepper_left(uint8_t value){
	stepper_motion=1;
	stepper_direction=0;
	stepper_steps=value;
	steps_counter=0;
}


void stepper_right(uint8_t value){
	stepper_motion=1;
	stepper_direction=1;
	stepper_steps=value;
	steps_counter=4;
}

//void stepper_position(uint8_t value){
//
//
//}

void delay_ms(uint16_t time){
	delay_counter=0;
	while(delay_counter!=time);
	delay_counter=0;
}

void execute_command(char function, char parameter, uint8_t value){
	switch(function){
	//ruch liniowy
	case 'M':
		switch(parameter){
		//ruch naprzod
		case 'F': move_forward(value);
		break;

		//ruch do tylu
		case 'R': move_backward(value);
		break;

		//ruch napedem A
		case 'A': set_speed_A(value);
		break;

		//ruch napedem B
		case 'B': set_speed_B(value);
		break;

		default: break;
		}
		break;
	//ruch obrotowy
	case 'R':
		switch(parameter){
		//obrot w lewo
		case 'L': turn_left(value);
		break;

		//obrot w prawo
		case 'R':turn_right(value);
		break;

		default: break;
		}
		break;
	//sterowanie wiezyczka
	case 'T':
		switch(parameter){
		//obrot w lewo
		case 'L': stepper_left(value);
		break;


		//obrot w prawo
		case 'R': stepper_right(value);
		break;

		//pozycja poziomo
		case 'P':
		break;

		//ruch w gore
		case 'U': turret_up(value);
		break;

		//ruch w dol
		case 'D': turret_down(value);
		break;

		default: break;
		}
		break;
	//sterowanie serwomechanizmami
	case 'S':
		switch(parameter){
		//pozycja serwomechanizmu A
		case 'A': servo_A_position(value);
		break;

		//pozycja serwomechanizmu B
		case 'B': servo_B_position(value);
		break;

		default: break;
		}
		break;
	//sterowanie silnikiem krokowym
	case 's':
		switch(parameter){
		//ruch w kierunku pozytywnym
		case 'P': stepper_left(value);
		break;

		//ruch w kierunku negatywnyn
		case 'N': stepper_right(value);
		break;

		//polozenie katowe
		case 'A': //stepper_position(value);
		break;

		default: break;
		}
		break;
	default:	break;
	}
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
