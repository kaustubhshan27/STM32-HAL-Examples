/* USER CODE BEGIN Header */
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
#include <string.h>
#include <stdio.h>
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
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void can1_tx(void);
void can1_rx(void);
void can1_filter_config(void);
void LED_Manage_Output(uint8_t val);
void Send_Response(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t req_counter = 0;
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
  MX_CAN1_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  can1_filter_config();

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF) != HAL_OK) //set interrupt enable bits
  {
	Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK) //moves operating mode from initialization to normal
  {
    Error_Handler();
  }

  char arr[] = "Hello";
  HAL_UART_Transmit(&huart4, (uint8_t*) arr, (uint16_t) strlen(arr), HAL_MAX_DELAY);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

}

/* USER CODE BEGIN 4 */
void LED_Manage_Output(uint8_t val)
{
	switch(val)
	{
		case 1:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

			break;
		case 2:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_13, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

			break;

		case 3:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
			break;
		case 4:
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		    break;

		default:
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
		    break;
	}
}

void Send_Response(void)
{
	CAN_TxHeaderTypeDef TxHeader;   //header of the CAN Tx message
	uint32_t txMailbox;
	uint8_t msg[2] = {0xAB, 0xBA};

	TxHeader.DLC = 2;				//Length of data to be sent in bytes
	TxHeader.StdId = 0x651; 		//Standard CAN message 11-bit identifier, not extended identifier 29-bit identifier
	TxHeader.IDE = CAN_ID_STD;		//Type of identifier (standard or extended)
	TxHeader.RTR = CAN_RTR_DATA; 	//N2 sending data frame in response to remote frame from N1

	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, msg, &txMailbox) != HAL_OK) {
			Error_Handler();
		}
}

void can1_filter_config(void) {
	CAN_FilterTypeDef can1_filter_init;

	can1_filter_init.FilterActivation = CAN_FILTER_ENABLE; //enable the CAN filter
	can1_filter_init.FilterBank = 0; //out of 28 filter banks, use the 0th filter bank
	can1_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0; //selecting the FIFO to be used (FIFO 0 or FIFO 1)
	can1_filter_init.FilterIdHigh = 0x0000; //setting the identifier reigster of the filter bank (accept all messages)
	can1_filter_init.FilterIdLow = 0x0000;
	can1_filter_init.FilterMaskIdHigh = 0x0000; //setting the mask reigster of the filter bank (accept all messages)
	can1_filter_init.FilterMaskIdLow = 0x0000;
	can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK; //to select the filter mode (mask mode or identifier list mode)
	can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT; //to select filter scale (filter registers to be used as 16 bit or 32 bit registers). Mostly used are 32 bit configurations.

	if (HAL_CAN_ConfigFilter(&hcan1, &can1_filter_init) != HAL_OK) { //configure the filter for CAN1
		Error_Handler();
	}
}

uint8_t led_num = 0;

void can1_tx(void) {
	CAN_TxHeaderTypeDef TxHeader; //header of the CAN Tx message

	/*HAL_CAN_AddTxMessage() updates the value of this variable depending
	 the Tx mailbox (out of the 3) in which the message is added
	 */
	uint32_t txMailbox;

	//message to be sent
	uint8_t msg;

	TxHeader.DLC = 1;				//Length of data to be sent in bytes
	TxHeader.StdId = 0x65D; 		//Standard CAN message 11-bit identifier, not extended identifier 29-bit identifier
	TxHeader.IDE = CAN_ID_STD;		//Type of identifier (standard or extended)
	TxHeader.RTR = CAN_RTR_DATA; 	//Sending data frame

	msg = ++led_num;
	if (led_num == 3)
	{
		led_num = 1;
		msg = 1;
	}

	//to add the Tx message to the mailbox and triggers transmission
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &msg, &txMailbox) != HAL_OK) {
		Error_Handler();
	}

	/*
	 //polling for message to be sent
	 while(HAL_CAN_IsTxMessagePending(&hcan1, txMailbox));

	 //sending a message over UART after CAN message is transmitted
	 char arr[50];
	 sprintf((char *)arr, "Message Transmitted\n");
	 HAL_UART_Transmit(&huart4, (uint8_t *)arr, (uint16_t) strlen(arr), HAL_MAX_DELAY); //blocking UART Tx
	 */
}

//not used
void can1_rx(void) {
	CAN_RxHeaderTypeDef RxHeader;
	/*
	 * Why do we not need to initialize variable RxHeader here? We initialized TxHeader in CAN1_Tx().
	 *	-> It is not mandatory. Since all the members of this structure will be filled by the Rx API.
	 */

	uint8_t rcvd_msg[5];			//to store received data

	while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0))
		; //check if any message present in Rx FIFO. Returns the no. of mailboxes filled in the FIFO specified

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rcvd_msg)
			!= HAL_OK) //polling based reception
			{
		Error_Handler();
	}

	char arr[50];
	sprintf(arr, "Message Recived - %s\n", rcvd_msg);
	HAL_UART_Transmit(&huart4, (uint8_t*) arr, (uint16_t) strlen(arr),
	HAL_MAX_DELAY); //blocking UART Tx
}

//Tx complete callback is called when the CAN Tx is completed
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) {
	char arr[50];
	sprintf((char*) arr, "Message Transmitted - Mailbox 0\n");
	HAL_UART_Transmit(&huart4, (uint8_t*) arr, (uint16_t) strlen(arr),
	HAL_MAX_DELAY); //blocking UART Tx
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) {
	char arr[50];
	sprintf((char*) arr, "Message Transmitted - Mailbox 1\n");
	HAL_UART_Transmit(&huart4, (uint8_t*) arr, (uint16_t) strlen(arr),
	HAL_MAX_DELAY); //blocking UART Tx
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) {
	char arr[50];
	sprintf((char*) arr, "Message Transmitted - Mailbox 2\n");
	HAL_UART_Transmit(&huart4, (uint8_t*) arr, (uint16_t) strlen(arr), HAL_MAX_DELAY); //blocking UART Tx
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef RxHeader;
	/*
	 * Why do we not need to initialize variable RxHeader here? We initialized TxHeader in CAN1_Tx().
	 *	-> It is not mandatory. Since all the members of this structure will be filled by the Rx API.
	 */

	uint8_t rcvd_msg[8]; //to store received data, max DLC = 8bytes
	char msg[50];

	//while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)); //check if any message present in Rx FIFO. Returns the no. of mailboxes filled in the FIFO specified

	if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rcvd_msg) != HAL_OK) //polling based reception
	{
		Error_Handler();
	}

	if(RxHeader.StdId == 0x65D && RxHeader.RTR == CAN_RTR_DATA)//For N2
	{
		//data frame sent from N1 to N2 specified by CAN_RTR_DATA
		/*We know it is a message meant for N2 from N1 because we are sending
		 *a CAN message with ID=0x65D from N1 to N2
		*/
		LED_Manage_Output(rcvd_msg[0]);
		sprintf(msg, "Reply from N2 received: %#x", rcvd_msg[0]);
	}
	else if(RxHeader.StdId == 0x651 && RxHeader.RTR == CAN_RTR_REMOTE)//For N2
	{
		//remote frame sent from N1 to N2. Response has to be sent back to N1
		/*
		 * The response message which will be sent from N2 to N1 will have the
		 * same identifier value (0x651) but it will be a data frame
		 * */
		Send_Response();
		return;
	}
	else if(RxHeader.StdId == 0x651 && RxHeader.RTR == CAN_RTR_DATA)//For N1
	{
		//reply from N2 to N1
		sprintf(msg, "Reply from N2 received: %#x", ((rcvd_msg[0] << 8) | (rcvd_msg[1])));
	}

	HAL_UART_Transmit(&huart4, (uint8_t*) msg, (uint16_t) strlen(msg), HAL_MAX_DELAY); //blocking UART Tx
	/*
	char arr[50];
	sprintf(arr, "Message Recived - %s\n", rcvd_msg);
	HAL_UART_Transmit(&huart4, (uint8_t*) arr, (uint16_t) strlen(arr), HAL_MAX_DELAY); //blocking UART Tx
	*/
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {

	char arr[50];
	sprintf(arr, "Error Occurred\n");
	HAL_UART_Transmit(&huart4, (uint8_t*) arr, (uint16_t) strlen(arr),
	HAL_MAX_DELAY); //blocking UART Tx
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	//CAN_TxHeaderTypeDef TxHeader; //header of the CAN Tx message
	//uint32_t txMailbox;

	/*
	if(req_counter == 4)
	{

		uint8_t msg = 0; //msg is meaningless when sending remote frame

		TxHeader.DLC = 2;				//Length of data to be sent in bytes, N1 demanding 2 bytes of reply
		TxHeader.StdId = 0x651; 		//Standard CAN message 11-bit identifier, not extended identifier 29-bit identifier
		TxHeader.IDE = CAN_ID_STD;		//Type of identifier (standard or extended)
		TxHeader.RTR = CAN_RTR_REMOTE; 	//Sending data frame

		if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, &msg, &txMailbox) != HAL_OK) //msg is meaningless when sending remote frame
		{
			Error_Handler();
		}

		can1_tx();
		req_counter = 0;
	}
	else
	{
		can1_tx();
		req_counter++;
	}
	*/
	can1_tx();
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
