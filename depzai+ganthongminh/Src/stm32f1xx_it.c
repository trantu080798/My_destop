#include "main.h"
#include "stm32f1xx_it.h"

extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_uart4_rx;
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart4;
extern CAN_RxHeaderTypeDef   RxHeader;
extern uint8_t               RxData[8];
extern CAN_TxHeaderTypeDef   TxHeader;
extern uint8_t               TxData[8];
extern uint32_t              TxMailbox;
extern uint8_t							 sent[13];
extern uint16_t							 dem;
extern uint8_t							 ngat_truyen;

uint8_t bitread(uint8_t byte,uint8_t bit)
{
	switch(bit)
	{
		case 0:
			return byte & 0x01;
		break;
		
		case 1:
			return byte & 0x02;
		break;
		
		case 2:
			return byte & 0x04;
		break;
		
		case 3:
			return byte & 0x08;
		break;
		
		case 4:
			return byte & 0x10;
		break;
		
		case 5:
			return byte & 0x20;
		break;
		
		case 6:
			return byte & 0x40;
		break;
		
		case 7:
			return byte & 0x80;
		break;
		default: break;
	}
}

/* USER CODE END EV */

//void NMI_Handler(void)
//{
// 
//}

//void HardFault_Handler(void)
//{
//  while (1)
//  {

//  }
//}

//void MemManage_Handler(void)
//{
//  while (1)
//  {
// 
//  }
//}


//void BusFault_Handler(void)
//{
//  while (1)
//  {

//  }
//}

//void UsageFault_Handler(void)
//{
//  while (1)
//  {

//  }
//}

//void SVC_Handler(void)
//{
// 
//}

//void DebugMon_Handler(void)
//{

//}

//void PendSV_Handler(void)
//{
// 
//}

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}
uint16_t 	 angle;
uint8_t 	 speed_km_h;
uint16_t 	 engine_speed;
uint8_t		 gear;
uint8_t		 rear_radar[4];
uint8_t		 front_radar[4];
uint8_t		 ill_state,turn_left,turn_right,hazard,high_beam;
uint8_t		 door_F,door_FL,door_FR,door_RL,door_RR,door_R;
uint8_t		 curent_view_state;
uint8_t		 gear,gear_P,gear_R,gear_N,gear_D,gear_DM,gear_DP;


void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);

	switch(RxHeader.StdId)
		{
			case 0x2B0:
				angle = (RxData[1] << 8) | (RxData[0]);
//				angle_state = map(angle, -60, 60, 255, 0);
			break;
				
			case 0x4F1:
				if(RxHeader.DLC==0x04)
				{
					speed_km_h = RxData[1] / 2;
				}
			break;
			
			case 0x440:
				if(RxHeader.DLC == 0x08 && RxData[0] == 0xFF)
				{
					speed_km_h = RxData[2] * 1.6;
				}
			break;
			
			case 0x436:
				//rear
				rear_radar[0] = RxData[2] & 0x03;
				rear_radar[1] = (RxData[1] >> 3) & 0x03;
				rear_radar[2] = rear_radar[1];
				rear_radar[3] = (RxData[2] >> 3) & 0x03;
				// font
				front_radar[0] = RxData[0] & 0x03;
				front_radar[1] = RxData[1] & 0x03;
				front_radar[2] = front_radar[1];
				front_radar[3] = (RxData[0]>>3) & 0x03;
			break;
			
			case 0x50E:
				curent_view_state = RxData[6];
			break;
			
			case 0x316:
				speed_km_h = RxData[6];
				engine_speed = ((RxData[3] << 8) | RxData[2]) / 4;
				if(RxData[5] == 0x2D || RxData[5] == 0x2F || RxData[5] == 0x2E)
				{
					gear_P = 0x2D;
					gear_R = 0x2F;
					gear_N = 0x2D;
					gear_D = 0x2F;
					gear_DM = 0x2F;
					gear_DP = 0x2F;
					gear= RxData[5];
				}
			break;
			
			case 0x510:
				//rear
				rear_radar[0] = RxData[2] & 0x03;
				rear_radar[1] = (RxData[5] >> 1) & 0x03;
				rear_radar[2] = rear_radar[1];
				rear_radar[3] = (RxData[2] >> 3) & 0x03;
				// font
				front_radar[0] = RxData[0] & 0x03;
				front_radar[1] = (RxData[4] >> 6) & 0x03;
				front_radar[2] = front_radar[1];
				front_radar[3] = (RxData[0] >> 3) & 0x03;
			break;
			
			case 0x112:
				speed_km_h = RxData[2];
			break;
			
			case 0x018:
				//den xe, xi nhan
				ill_state = bitread(RxData[2], 0);
				turn_left = bitread(RxData[5], 6);
				turn_right = bitread(RxData[5], 5);
				hazard = turn_left & turn_right;
				high_beam = bitread(RxData[2], 0);
				
				// trang thai cua
				door_F = 0;												// lap carpo
				door_FL = bitread(RxData[0], 4);	// cua truoc trai
				door_FR = bitread(RxData[0], 7);	// cua truoc phai
				door_RL = bitread(RxData[4], 3);	// cua sau trai
				door_RL = bitread(RxData[4], 1);	// cua sau phai
				door_R = bitread(RxData[5], 7);		// cop
			break;
			
			case 0x111:
			case 0x43F:
				if(RxData[1]==0x40||RxData[1]==0x47||RxData[1]==0x46||RxData[1]==0x45||RxData[1]==0x48)
				{
					gear_P = 0x40;
					gear_R = 0x47;
					gear_N = 0x46;
					gear_D = 0x45;
					gear_DM = 0x48;
					gear_DP = 0x48;
					gear =	RxData[1];
				}
			break;
			
			default : 
				break;
		}
//		ngat_truyen=1;
		sent[12]=((sent[0]&sent[1]&sent[2]&sent[3]&sent[4]&sent[5]&sent[6]&sent[7]&sent[8]&sent[9]&sent[10]&sent[11])^0xFF)&0xFF;
  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

	HAL_UART_Transmit(&huart4,sent,13,100);

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

void DMA2_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Channel3_IRQn 0 */

  /* USER CODE END DMA2_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA2_Channel3_IRQn 1 */

  /* USER CODE END DMA2_Channel3_IRQn 1 */
}


