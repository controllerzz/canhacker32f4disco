/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "strings.h"
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

uint32_t tim5cnt;
uint8_t bufferTextCan[256];
uint32_t ticStartLog;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;

uint8_t TxData[8];
uint8_t RxData[8];

uint32_t TxMailbox;
uint32_t mailBoxNum = 0;

uint8_t bufferCmd[128];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t sendLen;

	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
	{
	    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);

		if (RxHeader.IDE == CAN_ID_EXT){
	        sendLen = sprintf((char*)bufferTextCan, "T%8.8X%1.1X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%4.4X\r", \
				(unsigned int)RxHeader.ExtId, \
				(unsigned int)RxHeader.DLC, \

				RxData[0], \
				RxData[1], \
				RxData[2], \
				RxData[3], \
				RxData[4], \
				RxData[5], \
				RxData[6], \
				RxData[7], \
				(unsigned int)htim5.Instance->CNT);
		}
		else{
			sendLen = sprintf((char*)bufferTextCan, "t%3.3X%1.1X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%2.2X%4.4X\r", \
				(unsigned int)RxHeader.ExtId, \
				(unsigned int)RxHeader.DLC, \

				RxData[0], \
				RxData[1], \
				RxData[2], \
				RxData[3], \
				RxData[4], \
				RxData[5], \
				RxData[6], \
				RxData[7], \
				(unsigned int)htim5.Instance->CNT);
		}

	    tim5cnt = htim5.Instance->CNT;
	    CDC_Transmit_FS(bufferTextCan, sendLen);
	}
}

typedef enum
{
  CONVERT_ERROR = 0,
  CONVERT_OK
}ConvertReturnType;


uint8_t HexToHalfByte(uint8_t hex)
{
  if(hex >= '0' && hex <= '9')
    return hex - '0';
  else if(hex >= 'A' && hex <= 'F')
    return 10 + (hex - 'A');
  else if(hex >= 'a' && hex <= 'f')
    return 10 + (hex - 'a');

  return 0;
}

ConvertReturnType IsHex(uint8_t hex)
{
  if((hex >= '0' && hex <= '9') || (hex >= 'A' && hex <= 'F') || (hex >= 'a' && hex <= 'b'))
    return CONVERT_OK;

  return CONVERT_ERROR;
}


ConvertReturnType HexToByte(uint8_t * hex, uint8_t * data){
  *data = 0;

  for(uint8_t idx = 0; idx < 2; idx++)
  {
    if(IsHex(hex[idx]) == CONVERT_ERROR)
      return CONVERT_ERROR;

    *data |= (HexToHalfByte(hex[idx]) & 0xF) << (4 * (1-idx));
  }

  return CONVERT_OK;
}

uint16_t FindSymbol(uint8_t * string, uint8_t symbol){
  uint16_t index = 0;

  while(*string != symbol)
  {
    if(++index == 0xFFFF)
      return 0xFFFF;
    string++;
  }

  return index;
}

void CDC_Receive(uint8_t* Buf, uint32_t *Len){
	uint8_t sendLen = 0, byte;
	uint32_t id;

	switch(Buf[0])
	{
	  case 'L':
	    hcan1.Init.Mode = CAN_MODE_SILENT;
	  case 'O':
	    ticStartLog = HAL_GetTick();

	    htim5.Instance->CNT = 0;
	    HAL_TIM_Base_Start(&htim5);

	    while (HAL_CAN_Init(&hcan1) != HAL_OK)
	    {
	      HAL_CAN_DeInit(&hcan1);
	      HAL_Delay(100);
	    }

	    HAL_CAN_Start(&hcan1);
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	    break;

	  case 'V':
	    sendLen = sprintf((char*)bufferCmd,"V0101\r");
	    break;

	  case 'v':
	    sendLen = sprintf((char*)bufferCmd,"CanSee\r");
	    break;

	  case 'S':
	    switch(Buf[1])
	    {
	    // 10k = 110k
	    case '0':
	      hcan1.Init.Prescaler = 13;
	      hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	      hcan1.Init.TimeSeg1 = CAN_BS1_16TQ;
	      hcan1.Init.TimeSeg2 = CAN_BS2_8TQ;
	      break;
	    // 20k
	    case '1':
	      hcan1.Init.Prescaler = 150;
	      hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	      hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
	      hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
	      break;
	    // 50k
	    case '2':
	      hcan1.Init.Prescaler = 72;
	      hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	      hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
	      hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
	      break;
	    // 100k
	    case '3':
	      hcan1.Init.Prescaler = 36;
	      hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	      hcan1.Init.TimeSeg1 = CAN_BS1_5TQ;
	      hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
	      break;
	    // 125k
	    case '4':
	      hcan1.Init.Prescaler = 32;
	      hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	      hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
	      hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
	      break;
	    // 250k
	    case '5':
	      hcan1.Init.Prescaler = 16;
	      hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	      hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
	      hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
	      break;
	    // 500k
	    case '6':
	      hcan1.Init.Prescaler = 8;
	      hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	      hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
	      hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
	      break;
	    // 800k
	    case '7':
	      hcan1.Init.Prescaler = 5;
	      hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	      hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
	      hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
	      break;
	    // 1000k
	    case '8':
	      hcan1.Init.Prescaler = 4;
	      hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	      hcan1.Init.TimeSeg1 = CAN_BS1_4TQ;
	      hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
	      break;
	    }
	    break;

	  case 'T':
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);

	    if(FindSymbol(Buf, '\r') <= 26)
	    {
	      TxHeader.IDE = CAN_ID_EXT;

	      HexToByte(&Buf[1], &byte);
	      id = byte << 24;
	      HexToByte(&Buf[3], &byte);
	      id |= byte << 16;
	      HexToByte(&Buf[5], &byte);
	      id |= byte << 8;
	      HexToByte(&Buf[7], &byte);
	      id |= byte;
	      TxHeader.ExtId = id;

	      HexToByte(&Buf[8], &byte);
	      TxHeader.DLC = byte & 0x0F;

	      for(uint8_t idx = 0; idx < TxHeader.DLC; idx ++)
	      {
	        HexToByte(&Buf[10 + (idx * 2)], &byte);
	        TxData[idx] = byte;
	      }

	      if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
	        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &mailBoxNum);
	    }
	    break;

	  case 't':
		HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_SET);

	    if(FindSymbol(Buf, '\r') <= 26)
	    {
	      TxHeader.IDE = CAN_ID_STD;

	      HexToByte(&Buf[1], &byte);
	      id = byte << 4;
	      HexToByte(&Buf[3], &byte);
	      id |= byte >> 4;
	      TxHeader.StdId = id;

	      TxHeader.DLC = byte & 0x0F;

	      for(uint8_t idx = 0; idx < TxHeader.DLC; idx ++)
	      {
	        HexToByte(&Buf[5 + (idx * 2)], &byte);
	        TxData[idx] = byte;
	      }

	      if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
	        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &mailBoxNum);
	    }
	    break;

	  case 's':
	    break;

	  case 'N':
	    sendLen = sprintf((char*)bufferCmd,"N9876\r");
	    break;

	  case 'r':
	    break;

	  case 'R':
	    break;

	  case 'C':
	    if(Buf[1] == '\r')
	    {
	      hcan1.Init.Mode = CAN_MODE_NORMAL;
	      HAL_CAN_DeInit(&hcan1);
	      HAL_TIM_Base_Stop(&htim5);

	      break;
	    }

	  default:
	    break;
	  }

	  if(sendLen > 0)
	  {
	    CDC_Transmit_FS(bufferCmd, sendLen);
	    sendLen = 0;
	  }
	  else
	  {
	    CDC_Transmit_FS(Buf, *Len);
	  }

}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_CAN1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef  sFilterConfig;

  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterIdHigh = 0x00;
  sFilterConfig.FilterIdLow = 0x00;
  sFilterConfig.FilterMaskIdHigh = 0x00;
  sFilterConfig.FilterMaskIdLow = 0x00;

  if(HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); // | CAN_IT_ERROR | CAN_IT_BUSOFF | CAN_IT_LAST_ERROR_CODE);

  TxHeader.ExtId = 0x123;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.TransmitGlobalTime = DISABLE;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	HAL_Delay(500);
    HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
