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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART4 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1,0xFFFF);

  return ch;
}
void ResetUserTimer(uint32_t *Timer)
{
	*Timer = HAL_GetTick();
}

uint32_t ReadUserTimer(uint32_t *Timer)
{
	return ( HAL_GetTick() - *Timer);
}

void SetUserTimer(uint32_t *Timer,uint32_t T)
{
	*Timer = HAL_GetTick() + T;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
uint8_t uart2recv[21] = {0x00};
UART_HandleTypeDef* BTHuart = &huart1;
BTDataInfo gBTInfo = {0};
void BTUartIrqHandle(void)
{
	uint32_t tmp;
	tmp = BTHuart->Instance->SR;  //清除状态寄存器SR,读取SR寄存器可以实现清除SR寄存器的功能
	tmp = BTHuart->Instance->DR;
	HAL_UART_DMAStop(BTHuart);

	tmp = hdma_usart1_rx.Instance->CNDTR;       // 获取DMA中未传输的数据个数
	gBTInfo.ihigh += (gBTInfo.iFreeSpaceDataNum - tmp);   //总计数减去未传输的数据个数，得到已经接收的数据个数
	printf("\r\nBT irq rxLen: %d | %d \r\n", gBTInfo.ihigh, gBTInfo.ilow);

//	if(gBTInfo.ilow > gBTInfo.ihigh)
//	{
//		gBTInfo.iFreeSpaceDataNum = gBTInfo.ilow - gBTInfo.ihigh;
//		gBTInfo.iFreeSpaceDataNum = BT_ONE_FRAME_MAX_LEN;
//}
//	else
	{
		if(gBTInfo.iRxDataMaxLen - gBTInfo.ihigh < BT_ONE_FRAME_MAX_LEN)
		{
			//剩余空间不足，从头开始
			gBTInfo.iLastDataNum = gBTInfo.iRxDataMaxLen - gBTInfo.ihigh;
			gBTInfo.ihigh = 0;
			gBTInfo.iFreeSpaceDataNum = BT_ONE_FRAME_MAX_LEN;
		}
		else
		{
			gBTInfo.iFreeSpaceDataNum = BT_ONE_FRAME_MAX_LEN;
		}
	}

	HAL_UART_Receive_DMA(BTHuart, &gBTInfo.RxData[gBTInfo.ihigh], gBTInfo.iFreeSpaceDataNum); 
}
void Command_UART_Init(void)
{
	if(gBTInfo.bBTUARTWork)
		return;

	printf("\r\nBT Command_UART_Init !!!");
	HAL_UART_Init(BTHuart);
	gBTInfo.bBTUARTWork = true;
}
void BTCommInit()
{
	memset(&gBTInfo, 0, sizeof(gBTInfo));
	gBTInfo.ihigh = 0;
	gBTInfo.ilow = 0;
	gBTInfo.iLastDataNum = 0;
	gBTInfo.iRxDataMaxLen = BT_RECV_MAX_LEN;
	gBTInfo.iFreeSpaceDataNum = gBTInfo.iRxDataMaxLen - gBTInfo.ihigh;

	__HAL_UART_ENABLE_IT(BTHuart, UART_IT_IDLE);
	HAL_UART_Receive_DMA(BTHuart, &gBTInfo.RxData[gBTInfo.ihigh], gBTInfo.iFreeSpaceDataNum);

	printf("\r\nBT Command_UART_Init !!!");
	gBTInfo.bBTUARTWork = true; //前面cubeMx已经生成了uart初始化
	Command_UART_Init();
}


#define HEAD_FF 0xFF
#define HEAD_55 0x55
#define CMD_01	0x01
#define CMD_02	0X02
#define CMD_03  0x03
#define CMD_04  0x04
#define CMD_05  0x05
#define CMD_06  0x06
#define CMD_07  0x07
#define CMD_08  0x08
uint8_t check_sum(uint8_t* buf, uint8_t len)
{
	unsigned char CheckSum = 0;
	for(int i=2;i<(len-1);i++)
	{
		CheckSum += buf[i];
	}
	buf[len-1] = CheckSum;
	return CheckSum;
}
void TransferUartData2BT()
{
	static uint32_t uarttimer = 0;
	static uint8_t sendlen = 0;
	static uint8_t uart3recv[21] = {0x00};
	if (ReadUserTimer(&uarttimer) > 1000*3)  // 没有新数据收到，则定时发送前一帧数据
	{
		ResetUserTimer(&uarttimer);
		
	for(int n = 0; n < gBTInfo.ihigh; n++)
	{
		printf("  0x%02x", gBTInfo.RxData[n]);
	}
	printf(".\r\n");
		
		HAL_UART_Transmit(&huart1, (uint8_t *)uart3recv, sendlen, 0xFFFF);
		printf("\r\nBTHuart sned: ");
		for(int i = 0; i < sendlen; i++)
		{
			printf("0x%02x ", uart3recv[i]);
		}
		printf(".\r\n");		
	}
	
	static uint8_t lastindex = 3;
	uint8_t len = 0;
	static uint8_t check3Cycles = 10;
	while (true)
	{
		if (gBTInfo.ilow != gBTInfo.ihigh)
		{
			if (gBTInfo.RxData[gBTInfo.ilow] == HEAD_FF && gBTInfo.RxData[gBTInfo.ilow+1] == HEAD_55)
			{
				len = gBTInfo.RxData[gBTInfo.ilow+2];
				static uint8_t tempbuf[21] = {0x00};
				memcpy(tempbuf, &gBTInfo.RxData[gBTInfo.ilow], len+2);
				uint8_t checksum = check_sum(tempbuf, len+2);
				if (lastindex != gBTInfo.ilow)
				{
					printf("\r\nstart %d from tempbuf: cksum=%02x-%02x-%02x\r\n", gBTInfo.ilow, checksum, tempbuf[len+1], gBTInfo.RxData[gBTInfo.ilow + len + 1]);
					for(int n = 0; n < len+2; n++)
					{
						printf(" 0x%02x", tempbuf[n]);
					}
					printf(".\r\n");
				}
				lastindex = gBTInfo.ilow;
				if (gBTInfo.RxData[gBTInfo.ilow + len + 1] != tempbuf[len+1])
				{
					printf("\r\nCheckSum Error, receive uncomplete...");
					check3Cycles--;
					if(check3Cycles == 0)
					{
						gBTInfo.ilow++;
						check3Cycles = 10;
					}
					break;
				}
				else
				{
					sendlen = len+2;
//					memcmp(uart3recv, &gBTInfo.RxData[gBTInfo.ilow], len+2);
					for (int m = 0; m < len+2; m++)
					{
						uart3recv[m] = gBTInfo.RxData[gBTInfo.ilow];
//						if (gBTInfo.ilow >= (gBTInfo.iRxDataMaxLen - gBTInfo.iLastDataNum - 1))
//						{
//							gBTInfo.ilow = 0;
//							gBTInfo.iLastDataNum = 0;
//						}
//						else
						{
							gBTInfo.ilow++;
						}
					}
					if (gBTInfo.ihigh == 0)  // 重新头开始读
					{
						gBTInfo.ilow = 0;
					}
					HAL_UART_Transmit(&huart1, (uint8_t *)uart3recv, sendlen, 0xFFFF);
					printf("\r\nBTHuart sned: ");
					for(int i = 0; i < len+2; i++)
					{
						printf("0x%02x ", uart3recv[i]);
					}
					printf(".\r\nlenght=%d", len+2);
					break;
				}
			}
			else
			{
				gBTInfo.ilow++;
				break;
			}
		}
		else
		{
			break;
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// 发送测试字符串
	uint8_t str[] = "\r\n-------------USART_DMA_Sending------------------\r\n";
	// 接收缓存区大小为20
	uint8_t recvStr1[200] = {0x00};
	uint8_t recvStr2[20] = {0x00};
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
//	HAL_UART_Transmit_DMA(&huart1, str, sizeof(str) - 1); 
//  HAL_Delay(1000);
//	HAL_UART_Receive_DMA(&huart1, (uint8_t *)recvStr1, 20);
//	HAL_Delay(1000);
//	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
//	HAL_UART_Transmit_DMA(&huart2, str, sizeof(str) - 1); 
//  HAL_Delay(1000);
//	HAL_UART_Receive_DMA(&huart2, (uint8_t *)recvStr2, 20);

	BTCommInit();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//循环等待hdma_usart3_rx.State状态的变换，当接收到20个字符的大小后，DMA传输结束
	  //注：因我们采用的是正常模式，而非循环模式，所以才会这么使用，循环模式下，该标志位貌似不起作用
		//串口1接收
//		if(hdma_usart1_rx.State == HAL_DMA_STATE_READY)
//		{
//			// 将接收到的字符打印出来进行观察
//			printf("\r\n:%s:",recvStr1);
//			HAL_UART_Transmit_DMA(&huart1, recvStr1, sizeof(recvStr1) - 1);
//			// 发到串口2
////			HAL_UART_Transmit_DMA(&huart2, recvStr1, sizeof(recvStr1) - 1);
//			HAL_Delay(1000);
//			// 清除缓存区内容，方便进行下次接收
//			memset(recvStr1,0,sizeof(recvStr1));
//			// 软件将标志位清零
//			hdma_usart1_rx.State = HAL_DMA_STATE_BUSY;
//			// 继续继续下一回合的DMA接收，因为采用的非循环模式，再次调用会再次使能DMA
//			HAL_UART_Receive_DMA(&huart1, (uint8_t *)recvStr1, 20);
//			
//			
//		}
//		HAL_Delay(5000);
//		static uint8_t sendCount[21] = {0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41,0x41};
//		sendCount[18] += 0x01;
//		if (sendCount[18] - sendCount[1] > 20) {sendCount[18] = 0x41;}
//		uart2recv[18] = sendCount[18];
//		if (uart2recv[6] == 0x41) {
//			HAL_UART_Transmit_DMA(&huart2, uart2recv, sizeof(uart2recv)-1);
//		} else {
//			HAL_UART_Transmit_DMA(&huart2, sendCount, sizeof(sendCount)-1);
//		}
		
//		//串口2接收
//		if(hdma_usart2_rx.State == HAL_DMA_STATE_READY)
//		{
//			// 将接收到的字符打印出来进行观察
//			// 发到串口1
//			HAL_UART_Transmit_DMA(&huart1, recvStr2, sizeof(recvStr2) - 1);
//			HAL_Delay(1000);
//			// 清除缓存区内容，方便进行下次接收
//			memset(recvStr2,0,sizeof(recvStr2));
//			// 软件将标志位清零
//			hdma_usart2_rx.State = HAL_DMA_STATE_BUSY;
//			// 继续继续下一回合的DMA接收，因为采用的非循环模式，再次调用会再次使能DMA
//			HAL_UART_Receive_DMA(&huart2, (uint8_t *)recvStr2, 20);
//		}

	TransferUartData2BT();
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
