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
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include<stdio.h>
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
//#ifdef __GNUC__
//	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
//#else
//	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
//#endif
//PUTCHAR_PROTOTYPE
//{
//		HAL_UART_Transmit(&huart2 , (uint8_t *)&ch, 1, 0xFFFF);
//		return ch;
//}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define rx_size 2
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
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
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// 发送测试字符串
	uint8_t str[] = "\r\n-------------Running Started------------------\r\n";
	// 接收缓存区大小为20
	uint8_t recvStr2[20] = {0x00};
	uint8_t recvStr3[rx_size] = {0x99, 0xaa};
	uint8_t sendStr3[rx_size] = {0x88, 0x00};
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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
//	HAL_UART_Transmit_DMA(&huart3, str, sizeof(str) - 1); 
//  HAL_Delay(1000);
//	HAL_UART_Receive_DMA(&huart3, (uint8_t *)recvStr3, rx_size);
//	HAL_Delay(1000);
	HAL_UART_Transmit_DMA(&huart2, str, sizeof(str) - 1); 
  HAL_Delay(1000);
	HAL_UART_Receive_DMA(&huart2, recvStr2, 20);
//	printf("%s\r\n",str);
//	for(int i=0; i<sizeof(recvStr3); i++)
//	{
//		printf("0x%02x ", recvStr3[i]);
//	}
//	printf("\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		static uint32_t uart3timer = 0;
		if (ReadUserTimer(&uart3timer) > 1000)
		{
			ResetUserTimer(&uart3timer);
//			//串口3接收
////			printf("\r\n uart3_rxdma_state: %d", hdma_usart3_rx.State);
//			if(hdma_usart3_rx.State == HAL_DMA_STATE_READY)
//			{
//				// 将接收到的字符打印出来进行观察
//				// 发到串口3
//				printf("\r\n receive data: ");
//				for(int i=0; i<sizeof(recvStr3); i++)
//				{
//					printf("0x%02x ", recvStr3[i]);
//				}
//				printf(".\r\n");
//				HAL_UART_Transmit_DMA(&huart3, sendStr3, sizeof(sendStr3) - 1);
//				// 清除缓存区内容，方便进行下次接收
//				memset(recvStr3,0,sizeof(recvStr3));
//				// 软件将标志位清零
//				hdma_usart3_rx.State = HAL_DMA_STATE_BUSY;
//				// 继续继续下一回合的DMA接收，因为采用的非循环模式，再次调用会再次使能DMA
//				HAL_UART_Receive_DMA(&huart3, (uint8_t *)recvStr3, rx_size);
//				sendStr3[1]+=0x01;
//			}
			
			//串口2接收
//			printf("\r\n uart2_rxdma_state: %d", hdma_usart2_rx.State);
			if(hdma_usart2_rx.State == HAL_DMA_STATE_READY)
			{
				// 将接收到的字符打印出来进行观察
				// 发到串口2
				HAL_UART_Transmit_DMA(&huart2, recvStr2, sizeof(recvStr2) - 1);
				HAL_Delay(1000);
				// 清除缓存区内容，方便进行下次接收
				memset(recvStr2,0,sizeof(recvStr2));
				// 软件将标志位清零
				hdma_usart2_rx.State = HAL_DMA_STATE_BUSY;
				// 继续继续下一回合的DMA接收，因为采用的非循环模式，再次调用会再次使能DMA
				HAL_UART_Receive_DMA(&huart2, recvStr2, 20);
			}
		}
		
		static uint32_t lifetimer, lifeCount = 0;
		if (ReadUserTimer(&lifetimer) > 1000*10)
		{
			ResetUserTimer(&lifetimer);
//			printf("\r\nrunning %d(0) s", lifeCount++);
			uint8_t runn[10] = "running";
			HAL_UART_Transmit_DMA(&huart2, runn, sizeof(runn) - 1);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
