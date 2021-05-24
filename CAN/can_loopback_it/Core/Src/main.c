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
#include <stdio.h>
#include <string.h>
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

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */
void can1_tx(void);
void can1_rx(void);
void can1_filter_config(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	/* USER CODE BEGIN 2 */
	can1_filter_config();

	if (HAL_CAN_ActivateNotification(&hcan1,
	CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF)
			!= HAL_OK) //set interrupt enable bits
			{
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan1) != HAL_OK) //moves operating mode from initialization to normal
			{
		Error_Handler();
	}

	can1_tx();
	//can1_rx(); - directly receive in the CAN Rx interrupt
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief CAN1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN1_Init(void) {

	/* USER CODE BEGIN CAN1_Init 0 */

	/* USER CODE END CAN1_Init 0 */

	/* USER CODE BEGIN CAN1_Init 1 */

	/* USER CODE END CAN1_Init 1 */
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 16;
	hcan1.Init.Mode = CAN_MODE_LOOPBACK;
	hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
	hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.AutoBusOff = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CAN1_Init 2 */

	/* USER CODE END CAN1_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

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
	if (HAL_UART_Init(&huart4) != HAL_OK) {
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
static void MX_GPIO_Init(void) {

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
void can1_filter_config(void) {
	CAN_FilterTypeDef can1_filter_init;

	can1_filter_init.FilterActivation = CAN_FILTER_ENABLE;
	can1_filter_init.FilterBank = 0;
	can1_filter_init.FilterFIFOAssignment = CAN_RX_FIFO0;
	can1_filter_init.FilterIdHigh = 0x0000;
	can1_filter_init.FilterIdLow = 0x0000;
	can1_filter_init.FilterMaskIdHigh = 0x0000;
	can1_filter_init.FilterMaskIdLow = 0x0000;
	can1_filter_init.FilterMode = CAN_FILTERMODE_IDMASK;
	can1_filter_init.FilterScale = CAN_FILTERSCALE_32BIT;

	if (HAL_CAN_ConfigFilter(&hcan1, &can1_filter_init) != HAL_OK) {
		Error_Handler();
	}
}

void can1_tx(void) {
	CAN_TxHeaderTypeDef TxHeader; //header of the CAN Tx message

	/*HAL_CAN_AddTxMessage() updates the value of this variable depending
	 the Tx mailbox (out of the 3) in which the message is added
	 */
	uint32_t txMailbox;

	//message to be sent
	uint8_t msg[5] = { 'H', 'E', 'L', 'L', 'O' };

	TxHeader.DLC = 5;				//Length of data to be sent in bytes
	TxHeader.StdId = 0x65D; //Standard CAN message 11-bit identifier, not extended identifier 29-bit identifier
	TxHeader.IDE = CAN_ID_STD;		//Type of identifier (standard or extended)
	TxHeader.RTR = CAN_RTR_DATA; 	//Sending data frame

	//to add the Tx message to the mailbox and triggers transmission
	if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, msg, &txMailbox) != HAL_OK) {
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
	HAL_UART_Transmit(&huart4, (uint8_t*) arr, (uint16_t) strlen(arr),
	HAL_MAX_DELAY); //blocking UART Tx
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	CAN_RxHeaderTypeDef RxHeader;
	/*
	 * Why do we not need to initialize variable RxHeader here? We initialized TxHeader in CAN1_Tx().
	 *	-> It is not mandatory. Since all the members of this structure will be filled by the Rx API.
	 */

	uint8_t rcvd_msg[5];			//to store received data

	//while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0)); //check if any message present in Rx FIFO. Returns the no. of mailboxes filled in the FIFO specified

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

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {

	char arr[50];
	sprintf(arr, "Error Occurred\n");
	HAL_UART_Transmit(&huart4, (uint8_t*) arr, (uint16_t) strlen(arr),
			HAL_MAX_DELAY); //blocking UART Tx
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
