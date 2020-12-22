/* USER CODE BEGIN Header */

/* VERSION PROBADA 2020 12 20
 * VERSION SOLO PARA TARJETA DC
 * SE ELIMINAN FUNCIONES DE AC; SE ELIMINAN MEMORIAS M1 Y M2
 */

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

#define STATE_SBY		0		//STAND BY
#define STATE_SS		1		//SUBE SILLA
#define STATE_BS		2		//BAJA SILLA
#define STATE_SE		3		//SUBE ESPALDAR
#define STATE_BE		4		//BAJA EPALDAR
#define STATE_ESC		5		//ESCUPIDERA
#define STATE_LA		6		//LAMPARA
#define STATE_M0		7		//MEMORIA M0
#define STATE_M3		8		//MEMORIA M3

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
int STATE;
int TIME_OFF = 100;

int32_t ADC_NIV_34V;

int	FLAG_NIV_34 = 0;
int NIV_34_INIT = 0;
int NIV_34_INIT_MIN = 0;
int NIV_34_INIT_MAX = 0;
int NIV_34_INIT_PROM = 0;

int FLAG_STOP_MOTORS = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void BEEP();
void BEEPx(int x);
void FUN_INI();
void All_Off();

void FUN_ME();			//Funcion Emergencia
void FUN_SBY();			//Funcion Stand By
void FUN_SS();			//Funcion Sube Silla
void FUN_BS();			//Funcion Baja Silla
void FUN_SE();			//Funcion Sube Espaldar
void FUN_BE();			//Funcion Baja Espaldar
void FUN_LA();			//Funcion Lampara
void FUN_ESC();			//Funcion Escupidera
void FUN_M0();			//Funcion Memoria M0
void FUN_M3();			//Funcion Memoria M3

void FUN_CHECK();		//Chequeo de entradas

int stop_teclas();
int stop_tiempos();	//Base de tiempo 1ms
int stop_microsuiches();

uint32_t analogRead();
void read_niv34();

void Send_UART_TX1();

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
  MX_ADC_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  	FUN_INI();						//Inicializar variables
  	All_Off();						//Apagar todas las salidas

  	HAL_TIM_Base_Start_IT(&htim2);	//Base de tiempo 1ms
  	//HAL_UART_Transmit_IT(&huart1, "X", 100);

  	//CHEQUEO DE ENTRADAS
  	if ((HAL_GPIO_ReadPin(IN_ESC_GPIO_Port, IN_ESC_Pin)==GPIO_PIN_RESET)
  	  &&(HAL_GPIO_ReadPin(IN_LA_GPIO_Port, IN_LA_Pin)==GPIO_PIN_RESET)){
  		FUN_CHECK();
  	}

  	//IR A POSICION DE MEMORIA M0: SILLA ABAJO; ESPALDAR ARRIBA
  	else{			//Tres Beep de encendido
  		BEEPx(100);
  		BEEPx(100);
  		BEEPx(100);

  		//FUN_M0();
  	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  	read_niv34();

		switch(STATE){
		  case STATE_SBY:
				FUN_SBY();
				break;
		  case STATE_SS:
				FUN_SS();
				break;
		  case STATE_BS:
				FUN_BS();
				break;
		  case STATE_SE:
				FUN_SE();
				break;
		  case STATE_BE:
				FUN_BE();
				break;
		  case STATE_LA:
				FUN_LA();
				break;
		  case STATE_ESC:
				FUN_ESC();
				break;
		  case STATE_M0:
				FUN_M0();
				break;
		  case STATE_M3:
				FUN_M3();
				break;
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 79;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUZZ_Pin|OUT_MB_DT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT_ESC_Pin|OUT_LA_Pin|OUT_BE_Pin|OUT_SE_Pin
                          |OUT_BS_Pin|OUT_SS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN_SS_Pin IN_BS_Pin IN_SE_Pin IN_BE_Pin
                           IN_M0_Pin IN_M1_Pin IN_M2_Pin AC_DC_Pin */
  GPIO_InitStruct.Pin = IN_SS_Pin|IN_BS_Pin|IN_SE_Pin|IN_BE_Pin
                          |IN_M0_Pin|IN_M1_Pin|IN_M2_Pin|AC_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IN_M3_Pin IN_ESC_Pin IN_LA_Pin IN_ME_Pin
                           IN_MSS_Pin IN_MBS_Pin IN_MBE_Pin IN_MSE_Pin
                           IN_CALIBRA_Pin */
  GPIO_InitStruct.Pin = IN_M3_Pin|IN_ESC_Pin|IN_LA_Pin|IN_ME_Pin
                          |IN_MSS_Pin|IN_MBS_Pin|IN_MBE_Pin|IN_MSE_Pin
                          |IN_CALIBRA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZ_Pin OUT_MB_DT_Pin */
  GPIO_InitStruct.Pin = BUZZ_Pin|OUT_MB_DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_ESC_Pin OUT_LA_Pin OUT_BE_Pin OUT_SE_Pin
                           OUT_BS_Pin OUT_SS_Pin */
  GPIO_InitStruct.Pin = OUT_ESC_Pin|OUT_LA_Pin|OUT_BE_Pin|OUT_SE_Pin
                          |OUT_BS_Pin|OUT_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/*** INTERRUPCIONES ***/
/*
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	STATE = STATE_ME;
}
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

}



/*** DIAGRMA DE ESTADOS ***/

void  FUN_SBY(){
	//Apagar motores
	All_Off();
	int time_delay = 50;

	//Sube Silla
	if (HAL_GPIO_ReadPin(IN_SS_GPIO_Port, IN_SS_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_SS_GPIO_Port, IN_SS_Pin)==GPIO_PIN_RESET){
			  STATE = STATE_SS;
		}
	}

	//Baja Silla
	else if (HAL_GPIO_ReadPin(IN_BS_GPIO_Port, IN_BS_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_BS_GPIO_Port, IN_BS_Pin)==GPIO_PIN_RESET){
			STATE = STATE_BS;
		}
	}

	//Sube Espaldar
	else if (HAL_GPIO_ReadPin(IN_SE_GPIO_Port, IN_SE_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_SE_GPIO_Port, IN_SE_Pin)==GPIO_PIN_RESET){
			STATE = STATE_SE;
		}
	}

	//Baja Espaldar
	else if (HAL_GPIO_ReadPin(IN_BE_GPIO_Port, IN_BE_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_BE_GPIO_Port, IN_BE_Pin)==GPIO_PIN_RESET){
			STATE = STATE_BE;
		}
	}

	//Lampara
	else if (HAL_GPIO_ReadPin(IN_LA_GPIO_Port, IN_LA_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_LA_GPIO_Port, IN_LA_Pin)==GPIO_PIN_RESET){
			STATE = STATE_LA;
		}
	}

	//Escupidera
	else if (HAL_GPIO_ReadPin(IN_ESC_GPIO_Port, IN_ESC_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_ESC_GPIO_Port, IN_ESC_Pin)==GPIO_PIN_RESET){
			STATE = STATE_ESC;
		}
	}

	//Memoria M0
	else if (HAL_GPIO_ReadPin(IN_M0_GPIO_Port, IN_M0_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_M0_GPIO_Port, IN_M0_Pin)==GPIO_PIN_RESET){
			STATE = STATE_M0;
		}
	}

	//Memoria M3
	else if (HAL_GPIO_ReadPin(IN_M3_GPIO_Port, IN_M3_Pin)==GPIO_PIN_RESET){
		HAL_Delay(time_delay);
		if (HAL_GPIO_ReadPin(IN_M3_GPIO_Port, IN_M3_Pin)==GPIO_PIN_RESET){
			STATE = STATE_M3;
		}
	}
}


void FUN_SS(){
	BEEP();

	//SUBE SILLA
	read_niv34();
	while((HAL_GPIO_ReadPin(IN_SS_GPIO_Port, IN_SS_Pin)==GPIO_PIN_RESET)
	  && (stop_microsuiches() == HAL_OK)){
		read_niv34();
		HAL_GPIO_WritePin(OUT_SS_GPIO_Port, OUT_SS_Pin, GPIO_PIN_SET);
		HAL_Delay(TIME_OFF);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	STATE = STATE_SBY;
	BEEP();
}

void FUN_BS(){
	BEEP();

	//BAJA SILLA
	read_niv34();
	while((HAL_GPIO_ReadPin(IN_BS_GPIO_Port, IN_BS_Pin)==GPIO_PIN_RESET)
	  && (stop_microsuiches() == HAL_OK)){
		read_niv34();
		HAL_GPIO_WritePin(OUT_BS_GPIO_Port, OUT_BS_Pin, GPIO_PIN_SET);
		HAL_Delay(TIME_OFF);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	STATE = STATE_SBY;
	BEEP();
}

void FUN_SE(){
	BEEP();

	//SUBE ESPALDAR
	read_niv34();
	while((HAL_GPIO_ReadPin(IN_SE_GPIO_Port, IN_SE_Pin)==GPIO_PIN_RESET)
	  && (stop_microsuiches() == HAL_OK)){
		read_niv34();
		HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_SET);
		HAL_Delay(TIME_OFF);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	STATE = STATE_SBY;
	BEEP();
}

void FUN_BE(){
	BEEP();

	//BAJA ESPALDAR
	read_niv34();
	while((HAL_GPIO_ReadPin(IN_BE_GPIO_Port, IN_BE_Pin)==GPIO_PIN_RESET)
	  && (stop_microsuiches() == HAL_OK)){
		read_niv34();
		HAL_GPIO_WritePin(OUT_BE_GPIO_Port, OUT_BE_Pin, GPIO_PIN_SET);
		HAL_Delay(TIME_OFF);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	STATE = STATE_SBY;
	BEEP();
}

void FUN_LA(){
	BEEP();

	//LAMPARA
	HAL_GPIO_TogglePin(OUT_LA_GPIO_Port, OUT_LA_Pin);
	while(HAL_GPIO_ReadPin(IN_LA_GPIO_Port, IN_LA_Pin)==GPIO_PIN_RESET);

	STATE = STATE_SBY;
	BEEP();
}

void FUN_ESC(){
	BEEP();

	//Apagar lampara
	HAL_GPIO_WritePin(OUT_LA_GPIO_Port, OUT_LA_Pin, GPIO_PIN_RESET);

	//Subir Espaldar hasta llegar al microsuiche
	STATE = STATE_SE;
	read_niv34();
	while(stop_microsuiches() == HAL_OK
	  && stop_teclas() == HAL_OK){
		read_niv34();
		HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_SET);
		HAL_Delay(TIME_OFF);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_RESET);
	HAL_Delay(TIME_OFF);
	HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_RESET);

	STATE = STATE_SBY;
	FLAG_STOP_MOTORS = 0;
	BEEP();
}


void FUN_M0(){
	BEEP();

	//Apagar Lampara
	HAL_GPIO_WritePin(OUT_LA_GPIO_Port, OUT_LA_Pin, GPIO_PIN_RESET);


	//Subir Espaldar hasta el microcuiche
	STATE = STATE_SE;
	read_niv34();
	while(stop_microsuiches() == HAL_OK
	  && stop_teclas() == HAL_OK){
		read_niv34();
		HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_SET);
		HAL_Delay(TIME_OFF);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_RESET);
	BEEP();


	//Bajar Silla hasta el microsuiche
	read_niv34();
	STATE = STATE_BS;
	while(stop_microsuiches() == HAL_OK
	  && stop_teclas() == HAL_OK){
		read_niv34();
		HAL_GPIO_WritePin(OUT_BS_GPIO_Port, OUT_BS_Pin, GPIO_PIN_SET);
		HAL_Delay(TIME_OFF);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT_BS_GPIO_Port, OUT_BS_Pin, GPIO_PIN_RESET);


	FLAG_STOP_MOTORS = 0;
	STATE = STATE_SBY;
}


void FUN_M3(){
	BEEP();

	//Apagar Lampara
	HAL_GPIO_WritePin(OUT_LA_GPIO_Port, OUT_LA_Pin, GPIO_PIN_RESET);

	//TREN DEL EMBUR: LA SILLA SE QUEDA IGUAL, EL ESPALDAR HASTA ABAJO
	STATE = STATE_BE;
	read_niv34();
	while(stop_microsuiches() == HAL_OK
			&& stop_teclas() == HAL_OK){
		read_niv34();
		HAL_GPIO_WritePin(OUT_BE_GPIO_Port, OUT_BE_Pin, GPIO_PIN_SET);
		HAL_Delay(TIME_OFF);
		HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	}

	FLAG_STOP_MOTORS = 0;
	STATE = STATE_SBY;
	BEEP();
}

void FUN_CHECK(){
	BEEPx(100);
	BEEPx(100);

	//Suena un beep cuando se presiona algun boton
	while(1){
		while (HAL_GPIO_ReadPin(IN_SS_GPIO_Port, IN_SS_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_BS_GPIO_Port, IN_BS_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_SE_GPIO_Port, IN_SE_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_BE_GPIO_Port, IN_BE_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_M0_GPIO_Port, IN_M0_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_M1_GPIO_Port, IN_M1_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_M2_GPIO_Port, IN_M2_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_M3_GPIO_Port, IN_M3_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_ESC_GPIO_Port, IN_ESC_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}
		while (HAL_GPIO_ReadPin(IN_LA_GPIO_Port, IN_LA_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}

		while (HAL_GPIO_ReadPin(IN_CALIBRA_GPIO_Port, IN_CALIBRA_Pin)==GPIO_PIN_RESET){
			BEEPx(100);
		}

	}
}


/*** FUNCIONES ***/

void BEEP(){
/*
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
*/
}

void BEEPx(int x){
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_SET);
	HAL_Delay(x);
	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
	HAL_Delay(x);
}

void FUN_INI(){

	//Inicializamos variables
	STATE = STATE_SBY;

	//Medir voltaje con Mosfet Activado
	HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_SET);
	HAL_Delay(500);

	NIV_34_INIT = analogRead();
	NIV_34_INIT_MIN = NIV_34_INIT;
	NIV_34_INIT_MAX = NIV_34_INIT;
	NIV_34_INIT_PROM = NIV_34_INIT;

	for(int x = 0; x<99; x++){
		NIV_34_INIT = analogRead();
		NIV_34_INIT_PROM += NIV_34_INIT;
		if(NIV_34_INIT < NIV_34_INIT_MIN)
			NIV_34_INIT_MIN = NIV_34_INIT;
		else if (NIV_34_INIT > NIV_34_INIT_MAX)
			NIV_34_INIT_MAX = NIV_34_INIT;
		HAL_Delay(1);
	}

	NIV_34_INIT_PROM /= 100 ;
	NIV_34_INIT = NIV_34_INIT_MIN;

	HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_RESET);

}

void All_Off(){
	FLAG_STOP_MOTORS = 0;
	HAL_GPIO_WritePin(OUT_MB_DT_GPIO_Port, OUT_MB_DT_Pin, GPIO_PIN_RESET);
	HAL_Delay(TIME_OFF);

	HAL_GPIO_WritePin(BUZZ_GPIO_Port, BUZZ_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT_SS_GPIO_Port, OUT_SS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT_BS_GPIO_Port, OUT_BS_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT_SE_GPIO_Port, OUT_SE_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(OUT_BE_GPIO_Port, OUT_BE_Pin, GPIO_PIN_RESET);

	//HAL_GPIO_WritePin(OUT_LA_GPIO_Port, OUT_LA_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(OUT_ESC_GPIO_Port, OUT_ESC_Pin, GPIO_PIN_RESET);
}



int stop_teclas(){

	if(FLAG_STOP_MOTORS == 0){	//M0 = Activacion de tecla... M1 = Loop de funcion
		HAL_Delay(1000);
		FLAG_STOP_MOTORS = 1;
	}else if(FLAG_STOP_MOTORS == 2){
		return HAL_ERROR;
	}


	if(HAL_GPIO_ReadPin(IN_SS_GPIO_Port, IN_SS_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_BS_GPIO_Port, IN_BS_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_SE_GPIO_Port, IN_SE_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_BE_GPIO_Port, IN_BE_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_M0_GPIO_Port, IN_M0_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_M1_GPIO_Port, IN_M1_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_M2_GPIO_Port, IN_M2_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_M3_GPIO_Port, IN_M3_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_ESC_GPIO_Port, IN_ESC_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else if(HAL_GPIO_ReadPin(IN_LA_GPIO_Port, IN_LA_Pin)==GPIO_PIN_RESET){
		FLAG_STOP_MOTORS = 2;
		return HAL_ERROR;
	}
	else {
		return HAL_OK;
	}
}

int stop_microsuiches(){
	if(ADC_NIV_34V >= NIV_34_INIT){
		if(STATE == STATE_SS){
			return HAL_ERROR;
		}
		if(STATE == STATE_BS){
			return HAL_ERROR;
		}
		if(STATE == STATE_SE){
			return HAL_ERROR;
		}
		if(STATE == STATE_BE){
			return HAL_ERROR;
		}
	}

	return HAL_OK;
}

void read_niv34(){
	ADC_NIV_34V = 0;
	for(int x = 0; x<10; x++){
		ADC_NIV_34V += analogRead();
	}	ADC_NIV_34V /= 10;
/*

	char buffer34[150] = "";
	sprintf(buffer34, "NIV_34_INIT_MIN = %i    NIV_34_INIT_MAX = %i    NIV_34_INIT_PROM = %i    ADC_NIV_34 = %i\n", NIV_34_INIT_MIN, NIV_34_INIT_MAX, NIV_34_INIT_PROM, ADC_NIV_34V);
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)buffer34, sizeof(buffer34));
	HAL_Delay(500);
*/
}

uint32_t analogRead(){
  uint32_t ADCValue = 0;
  HAL_ADC_Start(&hadc);
  if (HAL_ADC_PollForConversion(&hadc, 1000000) == HAL_OK) {
      ADCValue = HAL_ADC_GetValue(&hadc);
  }
  HAL_ADC_Stop(&hadc);

  return ADCValue;
}

void Send_UART_TX1(){
/*
	char buffer[50] = "";
	sprintf(buffer, "\nTIME_S = %u\nTIME_E = %u\n\n", TIME_S, TIME_E);
	HAL_UART_Transmit_IT(&huart1, (uint8_t *)buffer, sizeof(buffer));
	HAL_Delay(250);
*/
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
