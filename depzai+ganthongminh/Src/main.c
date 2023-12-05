#include "main.h"
#include "stm32f1xx_hal_flash_ex.h"
#include "stm32f1xx_hal_def.h"

CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);

/* USER CODE BEGIN PFP */

CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader;
CAN_FilterTypeDef  		sFilterConfig0;
CAN_FilterTypeDef  		sFilterConfig1;
CAN_FilterTypeDef  		sFilterConfig2;
CAN_FilterTypeDef  		sFilterConfig3;
CAN_FilterTypeDef  		sFilterConfig4;
CAN_FilterTypeDef  		sFilterConfig5;
CAN_FilterTypeDef  		sFilterConfig6;
CAN_FilterTypeDef  		sFilterConfig7;
CAN_FilterTypeDef  		sFilterConfig8;
CAN_FilterTypeDef  		sFilterConfig9;
CAN_FilterTypeDef  		sFilterConfig10;

uint8_t               TxData[8];
uint8_t               RxData[8];
uint32_t              TxMailbox;
uint8_t 							sent[13]={0x2E,0x03,0x08,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x05};
uint8_t 							rev;
uint16_t 							num=0,dem=0;
uint8_t 							ngat_truyen=0;


void cau_hinh_can_1()
{
	//================can tx===================//
	TxHeader.StdId = 0x069;//dia chi cua can
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.DLC = 8;	//so byte truyen di
  TxHeader.TransmitGlobalTime = DISABLE;
	
	//=================can filter==============//
	/* -- id 0X2B0 ----------------------------------*/
	sFilterConfig0.FilterBank = 0;
  sFilterConfig0.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig0.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig0.FilterIdHigh = 0x2B0<<5;	
  sFilterConfig0.FilterIdLow = 0;
  sFilterConfig0.FilterMaskIdHigh = 0x7FF<<5;	
  sFilterConfig0.FilterMaskIdLow = 0;
  sFilterConfig0.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig0.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig0);
//  sFilterConfig0.SlaveStartFilterBank = 14;
	
	/* -- id 0X4F1 ----------------------------------*/
	sFilterConfig1.FilterBank = 1;
  sFilterConfig1.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig1.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig1.FilterIdHigh = 0x4F1<<5;	
  sFilterConfig1.FilterIdLow = 0;
  sFilterConfig1.FilterMaskIdHigh = 0x7FF<<5;	
  sFilterConfig1.FilterMaskIdLow = 0;
  sFilterConfig1.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig1.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig1);
	
	/* -- id 0X440----------------------------------*/
	sFilterConfig2.FilterBank = 2;
  sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig2.FilterIdHigh = 0x440<<5;	
  sFilterConfig2.FilterIdLow = 0;
  sFilterConfig2.FilterMaskIdHigh = 0x7FF<<5;	
  sFilterConfig2.FilterMaskIdLow = 0;
  sFilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig2.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig2);
	
	/* -- id 0X510 ----------------------------------*/
	sFilterConfig3.FilterBank = 3;
  sFilterConfig3.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig3.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig3.FilterIdHigh = 0x510<<5;	
  sFilterConfig3.FilterIdLow = 0;
  sFilterConfig3.FilterMaskIdHigh = 0x7FF<<5;	
  sFilterConfig3.FilterMaskIdLow = 0;
  sFilterConfig3.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig3.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig3);
	
	/* -- id 0X436 ----------------------------------*/
	sFilterConfig4.FilterBank = 4;
  sFilterConfig4.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig4.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig4.FilterIdHigh = 0x436<<5;	
  sFilterConfig4.FilterIdLow = 0;
  sFilterConfig4.FilterMaskIdHigh = 0x7FF<<5;	
  sFilterConfig4.FilterMaskIdLow = 0;
  sFilterConfig4.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig4.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig4);
	
	/* -- id 0X50E ----------------------------------*/
	sFilterConfig5.FilterBank = 5;
  sFilterConfig5.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig5.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig5.FilterIdHigh = 0x50E<<5;	
  sFilterConfig5.FilterIdLow = 0;
  sFilterConfig5.FilterMaskIdHigh = 0x7FF<<5;	
  sFilterConfig5.FilterMaskIdLow = 0;
  sFilterConfig5.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig5.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig5);
	
	/* -- id 0X316 ----------------------------------*/
	sFilterConfig6.FilterBank = 6;
  sFilterConfig6.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig6.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig6.FilterIdHigh = 0x316<<5;	
  sFilterConfig6.FilterIdLow = 0;
  sFilterConfig6.FilterMaskIdHigh = 0x7FF<<5;	
  sFilterConfig6.FilterMaskIdLow = 0;
  sFilterConfig6.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig6.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig6);
	
	/* -- id 0X43F ----------------------------------*/
	sFilterConfig7.FilterBank = 7;
  sFilterConfig7.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig7.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig7.FilterIdHigh = 0x43F<<5;	
  sFilterConfig7.FilterIdLow = 0;
  sFilterConfig7.FilterMaskIdHigh = 0x7FF<<5;	
  sFilterConfig7.FilterMaskIdLow = 0;
  sFilterConfig7.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig7.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig7);
	
	/* -- id 0X018 ----------------------------------*/
	sFilterConfig8.FilterBank = 8;
  sFilterConfig8.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig8.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig8.FilterIdHigh = 0x018<<5;	
  sFilterConfig8.FilterIdLow = 0;
  sFilterConfig8.FilterMaskIdHigh = 0x7FF<<5;	
  sFilterConfig8.FilterMaskIdLow = 0;
  sFilterConfig8.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig8.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig8);
	
	/* -- id 0X111 ----------------------------------*/
	sFilterConfig9.FilterBank = 9;
  sFilterConfig9.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig9.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig9.FilterIdHigh = 0x111<<5;	
  sFilterConfig9.FilterIdLow = 0;
  sFilterConfig9.FilterMaskIdHigh = 0x7FF<<5;	
  sFilterConfig9.FilterMaskIdLow = 0;
  sFilterConfig9.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig9.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig9);
	
		/* -- id 0X112 ----------------------------------*/
	sFilterConfig10.FilterBank = 10;
  sFilterConfig10.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig10.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig10.FilterIdHigh = 0x112<<5;	
  sFilterConfig10.FilterIdLow = 0;
  sFilterConfig10.FilterMaskIdHigh = 0x7FF<<5;	
  sFilterConfig10.FilterMaskIdLow = 0;
  sFilterConfig10.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig10.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig10);
	

	//==================cho phep can hoat dong=================//
	HAL_CAN_Start(&hcan1);
	
	//==================kich hoat ngat can=====================//
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	
	//======== kich chan can vat ly =================
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);		//EN
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_11,GPIO_PIN_SET);		//VCC
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_12,GPIO_PIN_SET);		//STB
}



/* USER CODE END PFP */

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */
//	HAL_UART_Receive_DMA(&huart4, &rev, 1);
	HAL_TIM_Base_Start_IT(&htim4);
	cau_hinh_can_1();
//	sent[12]=0x01&0x03;
	sent[12]=((sent[0]&sent[1]&sent[2]&sent[3]&sent[4]&sent[5]&sent[6]&sent[7]&sent[8]&sent[9]&sent[10]&sent[11])^0xFF)&0xFF;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		while(ngat_truyen==1&num<=5)
//		{
//			num++;
//			HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
//			HAL_Delay(100);
//		}
		HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
		HAL_Delay(1000);
//		num=0;
//		ngat_truyen=0;

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.Prediv1Source = RCC_PREDIV1_SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL2.PLL2State = RCC_PLL_NONE;
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
  /** Configure the Systick interrupt time 
  */
  __HAL_RCC_PLLI2S_ENABLE();
}


static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}


static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 36000;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 199;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
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


static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel3_IRQn);

}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB11 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PC6 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{ 
 
}
#endif /* USE_FULL_ASSERT */
