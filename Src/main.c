/* USER CODE BEGIN Header */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma2d.h"
#include "fatfs.h"
#include "i2c.h"
#include "ltdc.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_host.h"
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
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

struct Control{

  RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

  uint8_t dadoRX[30];

  float temperatura, umidade, pressao, corrente;
  int potenciometro, X;
	
	int indice;

  uint8_t vetor_print[30];
	
	int8_t estado_atual;
	int8_t estado_anterior;

}c;

struct PenDriveControl{
	FIL fp; //file handle
	FATFS fatfs; //structure with file system information
	char* text[100]; //text which will be written into file
	char filename[100];
	char buffer[200]; //buffer for data read from file
	uint32_t ret; //return variable
}p;

extern ApplicationTypeDef Appli_state;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
float le_umidade(void);
float le_temperatura(void);
float le_pressao(void);
float le_corrente(void);
void inicializa_vetor_uint8(uint8_t vetor[], int tam);
void leitura_AD(int tempo);
void aciona_PWM(void);
void renderiza_RTC(void);
void renderiza_sensores(void);
void configura_hora(void);
void inicializa_display(void);
void pendrive(void);
void monta_pendrive(void);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	c.umidade = 0;
  c.corrente = 0;
  c.pressao = 0;
  c.temperatura = 0;
  c.potenciometro = 0;
	c.indice = 0;

  inicializa_vetor_uint8(c.dadoRX, 30);
  inicializa_vetor_uint8(c.vetor_print, 30);

  c.sTime.Hours = 18;
  c.sTime.Minutes = 30;
  c.sTime.Seconds = 0;
	
	c.sDate.Year = 70;
	c.sDate.Month = 01;
	c.sDate.Date = 01;
	
	c.estado_anterior = -1;
	c.estado_atual = -1;
	
	strcpy(p.filename,"log.csv");

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
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USB_HOST_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
	
  inicializa_display();
	
	// seta primeira hora e data RTC
	HAL_RTC_SetDate(&hrtc, &c.sDate, FORMAT_BIN);
  HAL_RTC_SetTime(&hrtc, &c.sTime, FORMAT_BIN);


	// armando primeira interrupcao
	HAL_UART_Receive_IT(&huart1,c.dadoRX,17);
	
	// inicializando PWM
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	
	monta_pendrive();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
		
	  aciona_PWM();
		renderiza_RTC();
		leitura_AD(200);
		renderiza_sensores();

		HAL_Delay(800);
		MX_USB_HOST_Process();
		pendrive();
		
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV8;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

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

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	configura_hora();
	HAL_UART_Receive_IT(&huart1,c.dadoRX,17);
}
float le_umidade(void)
{
	uint8_t dado[2];
	dado[0] = 0x82;
	dado[1] = 0;
	
	uint16_t H0_rH_x2, H1_rH_x2;
	
	int16_t H0_T0_OUT, H1_T0_OUT;
	
	int16_t H_OUT;
	
	//escrever na memoria do sensor pra dar WAKE UP
	HAL_I2C_Mem_Write(&hi2c3,0xBE,0x20,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	
	//agora seguir roteiro
	
	//1 leitura dos registradores das posicoes 0x30 e 0x31
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x30,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x31,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	H0_rH_x2 = dado[0]/2;
	H1_rH_x2 = dado[1]/2;
	
	float h = 0;
	
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
	
	h = (((H1_rH_x2 - H0_rH_x2) * (H_OUT - H0_T0_OUT))/(H1_T0_OUT - H0_T0_OUT))+H0_rH_x2;
		
	return h;
}

float le_temperatura(void)
{
	uint8_t dado[2];
	uint16_t T0_degC = 0, T1_degC = 0;
	int16_t  T1_out = 0, T0_out = 0,T_out= 0;
	
	dado[0] = 0x82;
	HAL_I2C_Mem_Write(&hi2c3,0xBE,0x20,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x32,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x33,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	T0_degC = dado[0]; 
	T1_degC = dado[1];
	
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x35,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	
	T1_degC = ((dado[0] & 0xC) << 6) + T1_degC; 
	T0_degC = ((dado[0]  & 3) << 8) + T0_degC;
	T0_degC = T0_degC /8; 
	T1_degC = T1_degC / 8;
	
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x3C,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x3D,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	T0_out = (dado[1] << 8) + dado[0];
	
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x3E,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x3F,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	T1_out = (dado[1] << 8) + dado[0];
	
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x2A,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	HAL_I2C_Mem_Read(&hi2c3,0xBF,0x2B,I2C_MEMADD_SIZE_8BIT,&dado[1],1,50);
	
	T_out = (dado[1] << 8) + dado[0];
	
	return (((T1_degC - T0_degC) * (T_out - T0_out))/(T1_out - T0_out) + T0_degC);
}
float le_pressao(void)
{
	uint8_t dado[2];
	
	int pressao = 0;
	float p = 0;
	uint8_t buffer1 = 0;
	uint8_t buffer2 = 0;
	uint8_t buffer3 = 0;
	
	dado[0] = 0x3A;	
	HAL_I2C_Mem_Write(&hi2c3,0xBA,0x10,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	dado[0] = 0x10;
	HAL_I2C_Mem_Write(&hi2c3,0xBA,0x11,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	
	HAL_I2C_Mem_Read(&hi2c3,0xBB,0x28,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	buffer1 = dado[0];
	
	HAL_I2C_Mem_Read(&hi2c3,0xBB,0x29,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	buffer2 = dado[0];
	
	HAL_I2C_Mem_Read(&hi2c3,0xBB,0x2A,I2C_MEMADD_SIZE_8BIT,&dado[0],1,50);
	buffer3 = dado[0];
	
	pressao = buffer1 + (buffer2 << 8) + (buffer3 << 16);
	
	pressao = pressao/4096;
	
	p = pressao;
	
	return p;	
}
float le_corrente(void)
{
	float corrente = 0;
	
	double V = (double)c.X * 0.00073378;
	double vzinho = V - 1.5;
	corrente = (vzinho / 0.185) * 100;
	
	return corrente;
}
void monta_pendrive(void)
{
	if ( f_mount( &p.fatfs,"" ,0) != FR_OK )
	{
		BSP_LCD_SetFont(&Font16);
		sprintf((char*)c.vetor_print,"erro ao montar pendrive");
		BSP_LCD_DisplayStringAtLine(9,c.vetor_print);
		while(1);
	}
}
void inicializa_vetor_uint8(uint8_t vetor[], int tam)
{
  for(int i = 0; i < tam; i++)
  {
    vetor[i] = 0;
  }
}

void leitura_AD(int tempo)
{
  HAL_ADC_Start(&hadc1);
  HAL_ADC_PollForConversion(&hadc1,tempo/2);
  c.potenciometro = HAL_ADC_GetValue(&hadc1); //LEITURA DO CANAL 5
  HAL_ADC_PollForConversion(&hadc1,tempo/2);
  c.X = HAL_ADC_GetValue(&hadc1); //LEITURA DO CANAL 13 (na ordem RANK) //Pino PC3
  HAL_ADC_Stop(&hadc1);
}

void aciona_PWM(void)
{
  if(c.potenciometro > 2000 & c.potenciometro < 2095)
  {
		c.estado_atual = 0;
		if(c.estado_atual != c.estado_anterior)
		{
			BSP_LCD_Clear(LCD_COLOR_WHITE);
		}
		
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAtLine(3,(uint8_t*)"Motor Parado!");

		
		TIM3->CCR3 = 0;
		TIM3->CCR1 = 0;
  }
  else if(c.potenciometro >= 2095)
  {
		c.estado_atual = 1;
		if(c.estado_atual != c.estado_anterior)
		{
			BSP_LCD_Clear(LCD_COLOR_WHITE);
		}
			
    int pwm_percent = ((c.potenciometro-2095)*100)/2000;
    sprintf((char*)c.vetor_print,"Motor direita: %03d%%",pwm_percent);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAtLine(3,c.vetor_print);

		TIM3->CCR3 = pwm_percent;
		TIM3->CCR1 = 0;
  }
  else if(c.potenciometro <= 2000)
  {
		c.estado_atual = 2;
		if(c.estado_atual != c.estado_anterior)
		{
			BSP_LCD_Clear(LCD_COLOR_WHITE);
		}
		
    int pwm_percent = ((2000-c.potenciometro)*100)/2000;
    sprintf((char*)c.vetor_print,"Motor esquerda: %03d%%", pwm_percent);
    BSP_LCD_SetFont(&Font16);
    BSP_LCD_DisplayStringAtLine(3,c.vetor_print);
		
		TIM3->CCR3 = 0;
		TIM3->CCR1 = pwm_percent;
  }
	
	c.estado_anterior = c.estado_atual;
}

void renderiza_RTC(void)
{
  HAL_RTC_GetTime(&hrtc, &c.sTime, FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &c.sDate, FORMAT_BIN);
  
  BSP_LCD_SetFont(&Font16);
  sprintf((char*)c.vetor_print,"%02d:%02d:%02d",c.sTime.Hours,c.sTime.Minutes,c.sTime.Seconds);
  BSP_LCD_DisplayStringAtLine(1,c.vetor_print);
	
	sprintf((char*)c.vetor_print,"%02d/%02d/%02d",c.sDate.Date,c.sDate.Month,c.sDate.Year);
  BSP_LCD_DisplayStringAtLine(2,c.vetor_print);
}

void renderiza_sensores(void)
{
  c.umidade = le_umidade();
  c.temperatura = le_temperatura();
	c.pressao = le_pressao();
	c.corrente = le_corrente();

  BSP_LCD_SetFont(&Font16);
  sprintf((char*)c.vetor_print,"Umidade: %2.0f [%%]",c.umidade);
  BSP_LCD_DisplayStringAtLine(6,c.vetor_print);

  BSP_LCD_SetFont(&Font16);
  sprintf((char*)c.vetor_print,"Temp: %2.0f [C]",c.temperatura);
  BSP_LCD_DisplayStringAtLine(7,c.vetor_print);
	
	BSP_LCD_SetFont(&Font16);
  sprintf((char*)c.vetor_print,"Pressao: %4.0f [hPA]",c.pressao);
  BSP_LCD_DisplayStringAtLine(8,c.vetor_print);
	
	BSP_LCD_SetFont(&Font16);
  sprintf((char*)c.vetor_print,"Corrente: %4.0f [mA]",c.corrente);
  BSP_LCD_DisplayStringAtLine(9,c.vetor_print);
	
}

void configura_hora(void)
{	
	c.sTime.Hours = (c.dadoRX[0] - 0x30)*10 + (c.dadoRX[1] - 0x30);
	c.sTime.Minutes = (c.dadoRX[3] - 0x30)*10 + (c.dadoRX[4] - 0x30);
	c.sTime.Seconds = (c.dadoRX[6] - 0x30)*10 + (c.dadoRX[7] - 0x30);
	
	c.sDate.Date = (c.dadoRX[9] - 0x30)*10 + (c.dadoRX[10] - 0x30);
	c.sDate.Month = (c.dadoRX[12] - 0x30)*10 + (c.dadoRX[13] - 0x30);
	c.sDate.Year = (c.dadoRX[15] - 0x30)*10 + (c.dadoRX[16] - 0x30);
	
	
	HAL_RTC_SetDate(&hrtc, &c.sDate, FORMAT_BIN);
  HAL_RTC_SetTime(&hrtc, &c.sTime, FORMAT_BIN);
}

void inicializa_display(void)
{
  BSP_LCD_Init();
	BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER,LCD_FRAME_BUFFER);
	BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER,LCD_FRAME_BUFFER);
	BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
	BSP_LCD_DisplayOn();
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_RED);
	BSP_LCD_SetFont(&Font16);
	BSP_TS_Init(240, 320);
}

void pendrive(void)
{
	static int flag = 0;
	HAL_GPIO_TogglePin(GPIOG,1<<14);
	
	while(flag == 0) // abre o arquivo para escrita na primeira vez e deixa aberto
	{
		MX_USB_HOST_Process();
		if(Appli_state==APPLICATION_READY)
		{		
			if(f_open(&p.fp,p.filename,FA_CREATE_ALWAYS | FA_WRITE)!=FR_OK)
			{
				BSP_LCD_SetFont(&Font16);
				sprintf((char*)c.vetor_print,"erro ao abrir arquivo");
				BSP_LCD_DisplayStringAtLine(9,c.vetor_print);
				while(1);
			}
			else
			{
				flag=1;
				//sprintf((char*)p.buffer,"i,temp,umid,press,amps,hora,min,seg,dia,mes,ano\n");
				//f_write(&p.fp,p.buffer,strlen((char*)p.buffer),&p.ret);
			}
		}
	}
	
	// configura buffer
	HAL_RTC_GetTime(&hrtc, &c.sTime, FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &c.sDate, FORMAT_BIN);
	sprintf((char*)p.buffer,"%2d,%2.0f,%2.0f,%3.0f,%4.0f,%2d,%2d,%2d,%2d,%2d,%2d\n",c.indice,c.temperatura,c.umidade,c.pressao,c.corrente,c.sTime.Hours,
																																		c.sTime.Minutes,c.sTime.Seconds,c.sDate.Date,c.sDate.Month,c.sDate.Year);
	c.indice++;
	//
	
	if(f_write(&p.fp,p.buffer,strlen((char*)p.buffer),&p.ret)!=FR_OK) // vai escrevendo até pressionar o botao azul
	{
				BSP_LCD_SetFont(&Font16);
				sprintf((char*)c.vetor_print,"erro ao escrever no arquivo");
				BSP_LCD_DisplayStringAtLine(9,c.vetor_print);
				while(1);
	}
	
	if(HAL_GPIO_ReadPin(GPIOA,1)==1) // quando pressiona botao azul para de gravar e fecha o
	{
		f_close(&p.fp);
		GPIOG->BSRR=1<<13;
	}
	
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
