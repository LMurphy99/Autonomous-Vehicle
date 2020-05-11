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
//#include <string.h>
//#include "stm32l4xx.h"
//#include "stm32l4xx_nucleo_32.h"
#include <stdlib.h>
#include <stdio.h>
//#include <arm_math.h>
//#include <arm_const_structs.h>
#include <arm_common_tables.h>
//#include <core_cm4.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct sample{		// RP LIDAR scan sample.
	uint16_t ang;
	uint16_t dist;
} ;

struct edge{
	uint16_t idx;	// Index of sample where edge occurs.
	int8_t sign;	// Sign of edge.
};

struct edgePair{
	uint16_t idx_0;
	uint16_t idx_1;
	float32_t width;
};

struct skittle{
	float32_t ang;
	float32_t dist;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// CUSTOMIZABLE CONSTANTS ----------------------------------------------------
#define LIDAR_SAMPLES 	8000
#define PING_PONG_BUFFER_SIZE	256*5
#define HALF_PING_PONG_BUFFER_SIZE	PING_PONG_BUFFER_SIZE/2
#define EDGE_THRESHOLD	512
#define NUM_EDGES	60		// Max number of edges (256) determined by type of "idx" in edge struct.
#define NUM_EDGE_PAIRS 30
#define SKITTLE_WIDTH	65
#define POSITIVE_TOLERANCE	7
#define NEGATIVE_TOLERANCE	20

// RPLIDAR COMMANDS ----------------------------------------------------------
// Commands without payload and response
#define RPLIDAR_CMD_STOP 0x25
#define RPLIDAR_CMD_SCAN 0x20
#define RPLIDAR_CMD_FORCE_SCAN 0x21
#define RPLIDAR_CMD_RESET 0x40

// Commands without payload but have response
#define RPLIDAR_CMD_GET_DEVICE_INFO 0x50
#define RPLIDAR_GET_DEVICE_HEALTH 0x52

// Flags
#define RPLIDAR_START_FLAG1 0xA5
#define RPLIDAR_START_FLAG2 0x5A

// USER FLAGS ----------------------------------------------------------------
uint8_t flags = 0;
#define DATA_BUF_HALF_FULL		1
#define DATA_BUF_FULL			2
#define RESERVE2				4
#define RESERVE3				8
#define RESERVE4				16
#define RESERVE5				32
#define RESERVE6				64
#define RESERVE7				128

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
// Data Acquisition -------------------------------------
uint16_t tx_buffer;		// UART Transmit buffer.
uint8_t response_buffer[7];	// LIDAR Response buffer.
uint8_t pingpong_buffer[PING_PONG_BUFFER_SIZE];	// LIDAR Data Buffer

// Data Processing --------------------------------------
uint16_t n = 0;	// used as counter throughout program.
uint8_t m = 0; // secondary counter.
struct sample obj[LIDAR_SAMPLES];

// Edge Detection ---------------------------------------
int16_t convolution[LIDAR_SAMPLES];	// 360 convolution buffer
struct edge edges[NUM_EDGES];
struct edgePair edgePairs[NUM_EDGE_PAIRS];
struct skittle skittles[NUM_EDGE_PAIRS];

// Width deduction --------------------------------------
float32_t dist0;	// Buffer for distance 1 as floating point.
float32_t dist1;	// Buffer for distance 2 as floating point.
float32_t ang0;		// Buffer for angle 1 as floating point.
float32_t ang1;		// Buffer for angle 2 as floating point.

// Testing ----------------------------------------------
uint16_t test = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
int cmp(const void* a, const void* b);	// compare function used in sorting algorithm.
float32_t getWidth(float32_t d0, float32_t d1, float32_t a0, float32_t a1);
void edgeDetect( struct	sample *pObj, uint16_t objLen, int16_t* pDst);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);	//Enable 100% PWM for RPLIDAR motor.

  tx_buffer = RPLIDAR_CMD_STOP << 8 | RPLIDAR_START_FLAG1;	// RPLIDAR "stop" command.
  HAL_UART_Transmit_DMA(&huart2, &tx_buffer, 2);
  HAL_Delay(100);

  // RECEIVING SAMPLES VIA DMA ---------------------------------------------------------------
  tx_buffer = RPLIDAR_CMD_SCAN << 8 | RPLIDAR_START_FLAG1;	// Start scanning.
  HAL_UART_Transmit_DMA(&huart2, &tx_buffer, 2);
  HAL_UART_Receive(&huart2, &response_buffer, 7, 10);	// Store response conventionally.

  HAL_UART_Receive_DMA(&huart2, &pingpong_buffer, PING_PONG_BUFFER_SIZE);	// Start storing samples via DMA into ping pong buffer.


  // PING PONG BUFFER -------------------------------------------------------------------------
  while(n < LIDAR_SAMPLES)	// While number of valid samples is less than LIDAR_SAMPLES
  {
	  if(CHECK_BIT(flags,0))	// If half filled interrupt triggered...
	  {
		  flags &= ~DATA_BUF_HALF_FULL;	// Set flag low.
		  for (int i=0; i<HALF_PING_PONG_BUFFER_SIZE; i+=5)	// Process first half...
		  {
			  if ((pingpong_buffer[i+3] && pingpong_buffer[i+4] != 0) && (n < LIDAR_SAMPLES))	// if sample is valid
			  {
				  obj[n].ang  = (pingpong_buffer[i+2] << 7 | pingpong_buffer[i+1] >> 1); // /64;	// Store sample
				  obj[n].dist = (pingpong_buffer[i+4] << 7 | pingpong_buffer[i+3] >> 1); // /2;		// Lose 1 bit of precision for convolution.
				  n += 1;
			  }
		  }
	  }

  	  if(CHECK_BIT(flags,1))	// If complete filled interrupt triggered...
  	  {
  		  flags &= ~DATA_BUF_FULL;	// Set flag low.
		  for (int i=HALF_PING_PONG_BUFFER_SIZE; i<PING_PONG_BUFFER_SIZE; i+=5) // Process second half...
		  {
			  if ((pingpong_buffer[i+3] && pingpong_buffer[i+4] != 0) && (n < LIDAR_SAMPLES))	// if sample is valid
			  {
				  obj[n].ang  = (pingpong_buffer[i+2] << 7 | pingpong_buffer[i+1] >> 1); // /64;	// Store sample
				  obj[n].dist = (pingpong_buffer[i+4] << 7 | pingpong_buffer[i+3] >> 1); // /2; 	// Lose 1 bit of precision for convolution.
				  n += 1;
			  }
		  }
  	  }
  }

   tx_buffer = RPLIDAR_CMD_STOP << 8 | RPLIDAR_START_FLAG1;	// RPLIDAR "stop" command.
   HAL_UART_Transmit_DMA(&huart2, &tx_buffer, 2);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);	// Disable RPLIDAR motor.

   // DATA PROCESSING -------------------------------------------------------------------
   //	SORTING -------------------------------------------------------------------------
   qsort(obj, LIDAR_SAMPLES, sizeof(obj[0]), cmp);	// Sort sample structures according to angle.


   //	EDGE DETECTION ------------------------------------------------------------------
   edgeDetect(obj, LIDAR_SAMPLES, convolution);

   //	EDGE CLASSIFICATION -------------------------------------------------------------
   n = 0;	// number of edges.
   for (int i=0; i<LIDAR_SAMPLES; i++)
   {
	   if (abs(convolution[i]) > EDGE_THRESHOLD)
	   {
		   if (convolution[i] < 0){
			   edges[n].idx = i;
			   edges[n].sign = -1;
		   }
		   else{
			   edges[n].idx = i-1;
			   edges[n].sign =  1;
		   }
		   n++;
	   }
   }

   //	EDGE PAIRING -------------------------------------------------------------------
   m = 0;	// number of edge pairs
   for (int i=0; i<n-1; i++)
   {
	   if (edges[i].sign - edges[i+1].sign < 0)	// if first edge is negative and second is positive...
	   {
		   edgePairs[m].idx_0 = edges[i].idx;	// ... pair edges.
		   edgePairs[m].idx_1 = edges[i+1].idx;

		   dist0 = (obj[edgePairs[m].idx_0].dist / 2.0);	// Scale angles and distances for cosine rule.
		   dist1 = (obj[edgePairs[m].idx_1].dist / 2.0);
		   ang0 = (obj[edgePairs[m].idx_0].ang / 64.0) * PI/180;
		   ang1 = (obj[edgePairs[m].idx_1].ang / 64.0) * PI/180;

		   edgePairs[m].width = getWidth(dist0, dist1, ang0, ang1);
		   m++;
	   }
   }

   // Check last and first edge...
   if (edges[n-1].sign - edges[0].sign < 0)
   {
	   edgePairs[m].idx_0 = edges[n-1].idx;	// ... pair edges.
	   edgePairs[m].idx_1 = edges[0].idx;

	   dist0 = (obj[edgePairs[m].idx_0].dist / 2.0);	// Scale angles and distances for cosine rule.
	   dist1 = (obj[edgePairs[m].idx_1].dist / 2.0);
	   ang0 = (obj[edgePairs[m].idx_0].ang / 64.0) * PI/180;
	   ang1 = (obj[edgePairs[m].idx_1].ang / 64.0) * PI/180;

	   edgePairs[m].width = getWidth(dist0, dist1, ang0, ang1);
	   m++;
   }


   //	SKITTLE COORDINATES -------------------------------------------------------------
   n = 0;
   for (int i=0; i<m; i++)
   {
	   if ((SKITTLE_WIDTH - NEGATIVE_TOLERANCE < edgePairs[i].width) && (SKITTLE_WIDTH + POSITIVE_TOLERANCE > edgePairs[i].width))
	   {
		   ang0 = obj[edgePairs[i].idx_0].ang * PI/(64.0*180.0);
		   ang1 = obj[edgePairs[i].idx_1].ang * PI/(64.0*180.0);

		   skittles[n].ang = atan2(sin(ang0)+sin(ang1), cos(ang0)+cos(ang1))*180/PI;	// average cartesian coords and convert back to polar.
		   skittles[n].dist= (obj[edgePairs[i].idx_0].dist + obj[edgePairs[i].idx_1].dist)/(2.0*2.0);
		   n+=1;
	   }
   }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1){}
    /* USER CODE END WHILE */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
// BEGIN CUBEIDE AUTO-GENERATED FUNCTIONS
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
// END CUBEIDE AUTO-GENERATED FUNCTIONS

/* USER CODE BEGIN 4 */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
	test++;						   // If response flag is high...
	flags |= DATA_BUF_HALF_FULL;	// ..then referring to data.
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	test++;					// If response flag is high...
	flags |= DATA_BUF_FULL;	// ..then referring to data.
}


int cmp(const void* a, const void* b)	// compare function
{
	struct sample *a1 = (struct sample*)a;
	struct sample *a2 = (struct sample*)b;
	if ((*a1).ang < (*a2).ang)
		return -1;
	else if ((*a1).ang > (*a2).ang)
		return 1;
	else
		return 0;
}

void edgeDetect( struct	sample *pObj, uint16_t objLen, int16_t* pDst)
{
	// Ciruclar convolution with kernel {1 -1}.
	*pDst = - pObj->dist + (pObj+objLen-1)->dist;	// convolve first and last element.

	for (int i=1; i<objLen; i++)
	{
		*(pDst+i) = (pObj+i)->dist - (pObj+i-1)->dist;
	}
}

float32_t getWidth(float32_t d0, float32_t d1, float32_t a0, float32_t a1)
{
	return sqrt( (d0*d0)+(d1*d1) - 2*d0*d1*cos(a1-a0 + (2*PI)) );	// cosine rule
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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
