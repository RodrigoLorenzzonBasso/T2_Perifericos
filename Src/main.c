/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma2d.h"
#include "i2c.h"
#include "ltdc.h"
#include "rtc.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_sdram.h"
#include "stm32f429i_discovery_ts.h"
#include "string.h"
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint8_t dadoRX[10];

uint8_t hora;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  hora = (dadoRX[0] - 0x30)*10 + (dadoRX[1] - 0x30);
	
	HAL_UART_Receive_IT(&huart1,dadoRX,2);
}

//endereco do sensor de pressao 0xBA e 0xBC

// endereco do sensor ::: 0xBE (Write) 0xBF (Read)
float le_umidade(void)
{
	uint8_t dado[2];
	dado[0] = 0x82;
	dado[1] = 0;
	
	uint16_t H0_rH_x2, H1_rH_x2;
	
	int16_t H0_T0_OUT, H1_T0_OUT;
	
	int16_t H_OUT;
	
	//escrever na memoria do sensor pra dar WAKE UP
	HAL_I2C_Mem_Write(&hi2c3,0xBE,0x20,I2C_MEMADD_SIZE_8BIT,&dado[0],1,200);
	
	//agora seguir roteiro
	
	//1 leitura dos registradores das posicoes 0x30 e 0x31
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x30,I2C_MEMADD_SIZE_8BIT,&dado[0],1,200);
	
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x31,I2C_MEMADD_SIZE_8BIT,&dado[1],1,200);
	
	H0_rH_x2 = dado[0]/2;
	H1_rH_x2 = dado[1]/2;
	
	float Humidity = 0;
	
	//3 leitura dos 0x36, 0x37
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x36,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x37,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	H0_T0_OUT = (dado[1] << 8) + dado[0];
	
	//4 leitura 0x3a, 0x3b
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x3a,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x3b,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	H1_T0_OUT = (dado[1] << 8) + dado[0];
	
	// 5 leitura do 0x28 e 0x29
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x28,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x29,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	H_OUT = (dado[1] << 8) + dado[0];
	
	// 6 calcular
	
	Humidity = (((H1_rH_x2 - H0_rH_x2) * (H_OUT - H0_T0_OUT))/(H1_T0_OUT - H0_T0_OUT))+H0_rH_x2;
		
	return Humidity;
}

//
//	9 SDA - PC9
//	10 SCL - PA8
//	2 -> 3V
//	6 -> GND
//
//

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	TS_StateTypeDef TsState;
	
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	
	float umidade = 0;

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
  MX_DMA2D_Init();
  MX_I2C3_Init();
  MX_LTDC_Init();
  MX_SPI5_Init();
  MX_FMC_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	
	BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER,LCD_FRAME_BUFFER);
	BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER,LCD_FRAME_BUFFER);
	BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
	BSP_LCD_DisplayOn();
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_SetFont(&Font16);
	//BSP_LCD_DisplayStringAtLine(1,(uint8_t*)"TESTE LINHA 1");
	BSP_TS_Init(240, 320);
	
	
	sTime.Hours = 18;
	sTime.Minutes = 30;
	sTime.Seconds = 0;
	HAL_RTC_SetDate(&hrtc, &sDate, FORMAT_BIN);
  HAL_RTC_SetTime(&hrtc, &sTime, FORMAT_BIN);


	// armando primeira interrupcao
	HAL_UART_Receive_IT(&huart1,dadoRX,2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		

		unsigned char tt[30];
		
		umidade = le_umidade();
		
		sprintf((char*)tt,"%f",umidade);
		
		//BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAtLine(12,tt);
		
		//HAL_UART_Receive(&huart1,tt,30,1000);
		//BSP_LCD_DisplayStringAtLine(4,tt);
		
		
		HAL_RTC_GetTime(&hrtc, &sTime, FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, FORMAT_BIN);
		
		unsigned char print_vector[30];
		
		sprintf((char*)print_vector,"%02d:%02d:%02d",sTime.Hours,sTime.Minutes,sTime.Seconds);
    BSP_LCD_DisplayStringAtLine(8,print_vector);
		
		int Pot = 0;
		int Current = 0;
		
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1,100);
		Pot = HAL_ADC_GetValue(&hadc1); //LEITURA DO CANAL 5
		HAL_ADC_PollForConversion(&hadc1,100);
		Current = HAL_ADC_GetValue(&hadc1); //LEITURA DO CANAL 13 (na ordem RANK) //Pino PC3
		HAL_ADC_Stop(&hadc1);
		
		sprintf((char*)print_vector,"%04d",Pot);
		BSP_LCD_DisplayStringAtLine(6,print_vector);
		
		if(Pot > 2000 & Pot < 2095)
		{
			BSP_LCD_SetFont(&Font12);
			BSP_LCD_DisplayStringAtLine(2,(uint8_t*)"motor disligado pora");
			BSP_LCD_SetFont(&Font16);
		}
		else if(Pot >= 2095)
		{
			sprintf((char*)print_vector,"Motor Direita : %04d",((Pot-2095)*100)/2000);
			BSP_LCD_SetFont(&Font12);
			BSP_LCD_DisplayStringAtLine(2,print_vector);
			BSP_LCD_SetFont(&Font16);
		}
		else if(Pot <= 2000)
		{
			sprintf((char*)print_vector,"Motor Esquerda : %04d",((2000-Pot)*100)/2000);
			BSP_LCD_SetFont(&Font12);
			BSP_LCD_DisplayStringAtLine(2,print_vector);
			BSP_LCD_SetFont(&Font16);
		}
		
		HAL_Delay(200);
		
		
		
		/*BSP_TS_GetState(&TsState);
		if(TsState.TouchDetected)
		{
			//sprintf((char*)vetor,"X=%03d, Y=%03d",TsState.X,TsState.Y);
			//BSP_LCD_DisplayStringAtLine(4,(uint8_t*)vetor);
			TsState.X=0;
			TsState.Y=0;
		}*/
		
		//uint8_t vectorNha[4] = "nha\n";
		//HAL_UART_Transmit(&huart1,vectorNha, 4, 60);
		
		//HAL_Delay(30);
		
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 216;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
