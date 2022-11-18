/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "DS18B20.h"
#include <stdio.h>
#include <string.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

//page x= 0x08000000+1024*x
#define FLASH_ADDR_PAGE_126 ((uint32_t) 0x0801F800) //Page 126
#define FLASH_ADDR_PAGE_118 ((uint32_t) 0x0801D800) //Page 118
#define FLASH_ADDR_PAGE_127 ((uint32_t) 0x0801FC00) //Page 127
#define FLASH_USER_START_ADDR2    FLASH_ADDR_PAGE_118
#define FLASH_USER_END_ADDR2      FLASH_ADDR_PAGE_127 + FLASH_PAGE_SIZE

uint32_t startpage2 = FLASH_USER_START_ADDR2;


#define FLASH_ADDR_PAGE_108 ((uint32_t) 0x0801B000) //Page 108
#define FLASH_ADDR_PAGE_117 ((uint32_t) 0x0801D400) //Page 117
#define FLASH_USER_START_ADDR1    FLASH_ADDR_PAGE_108
#define FLASH_USER_END_ADDR1    FLASH_ADDR_PAGE_117 + FLASH_PAGE_SIZE

uint32_t startpage1 = FLASH_USER_START_ADDR1;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint32_t var=0;
float a=0;
float tb=0;
float tds=0;
float data[100];
int adc[50];
float ndo[10];
int count=0;
int page=0;
int b=0;
int d1=0,d2=0,d3=0;
int dieukhien=0;
DS18B20_Name DS1;
float Temp=0;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

char sdt[]="0349597770";
char ATcommand[100];
uint8_t buffer[100]={0};
uint8_t check=0;

 void FLASH_ErasePage (uint32_t startPage, uint32_t endPage)
 {		HAL_FLASH_Unlock ();
			FLASH_EraseInitTypeDef EraseInit;
			EraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
			EraseInit. PageAddress = startPage;
			EraseInit.NbPages = (endPage - startPage) /FLASH_PAGE_SIZE;
			uint32_t PageError = 0;
			HAL_FLASHEx_Erase(&EraseInit, &PageError);
			
			HAL_FLASH_Lock ();
 }
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
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
	
	FLASH_ErasePage (FLASH_ADDR_PAGE_108, FLASH_USER_END_ADDR2);
	DS18B20_Init(&DS1, &htim4, DS18B20_GPIO_Port, DS18B20_Pin);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	
		//kiem tra sim
    while(!check){
		memset (buffer,0,sizeof(buffer));
		memset (ATcommand,0,sizeof(ATcommand));
		sprintf(ATcommand,"AT\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);	
	
		if(HAL_UART_Receive(&huart1, buffer ,1,1000)==HAL_OK){HAL_Delay(1000);	
			memset (buffer,0,sizeof(buffer));		
			memset (ATcommand,0,sizeof(ATcommand));	
			sprintf(ATcommand,"AT+CFUN:1\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
		if(HAL_UART_Receive (&huart1, buffer, 1, 1000)==HAL_OK){HAL_Delay(1000);	
			memset (buffer,0,sizeof(buffer));	
			memset (ATcommand,0,sizeof(ATcommand));				
			sprintf(ATcommand,"AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
			if(HAL_UART_Receive(&huart1, buffer ,1,1000)==HAL_OK){HAL_Delay(1000);	
				
				
				
//				memset (buffer,0,sizeof(buffer));
//				memset (ATcommand,0,sizeof(ATcommand));	
//				sprintf(ATcommand,"AT+SAPBR=3,1,\"APN\",\"v-internet\"\r\n");
//				HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//				if(HAL_UART_Receive(&huart1, buffer ,1,1000)==HAL_OK){
					
					memset (buffer,0,sizeof(buffer));
					memset (ATcommand,0,sizeof(ATcommand));	
					
					sprintf(ATcommand,"AT+CSTT=\"3gprs\",\"3gprs\",\"3gprs\"\r\n");
					HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
					if(HAL_UART_Receive(&huart1, buffer ,1,1000)==HAL_OK){
					HAL_Delay(1000);	
					memset (buffer,0,sizeof(buffer));
					memset (ATcommand,0,sizeof(ATcommand));	
				
					sprintf(ATcommand,"AT+CGATT=1\r\n");
					HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
					if(HAL_UART_Receive(&huart1, buffer ,1,1000)==HAL_OK){
					HAL_Delay(1000);	
					memset (buffer,0,sizeof(buffer));
					memset (ATcommand,0,sizeof(ATcommand));	
					sprintf(ATcommand,"AT+SAPBR=1,1\r\n");
					HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
//					if(HAL_UART_Receive(&huart1, buffer ,1,1000)==HAL_OK){	
//						
//						memset (buffer,0,sizeof(buffer));
//						memset (ATcommand,0,sizeof(ATcommand));	
//						sprintf(ATcommand,"AT+SAPBR=2,1\r\n");
//						HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
						if(HAL_UART_Receive(&huart1, buffer ,1,1000)==HAL_OK){check=1;}}
//				}
//			}
		}
	}
	}
}
}
		HAL_Delay(1000);
					
					
	//do nhiet do
Temp=0;
for (int nd=0;nd<10;nd++){
	ndo[nd]= DS18B20_ReadTemp(&DS1);
	Temp+=ndo[nd];
	if (ndo[nd]>100||ndo[nd]<0) {nd--;}
	HAL_Delay(100);
}
Temp=Temp/10;
	HAL_Delay(2000);
		   


		memset (buffer,0,sizeof(buffer));
		memset (ATcommand,0,sizeof(ATcommand));
		sprintf(ATcommand,"AT+HTTPINIT\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);

		if(HAL_UART_Receive (&huart1, buffer, 1, 1000)==HAL_OK){
			HAL_Delay(1000); 
					
			memset (buffer,0,sizeof(buffer));
			memset (ATcommand,0,sizeof(ATcommand));
			sprintf(ATcommand,"AT+HTTPPARA=\"CID\",1\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
			if(HAL_UART_Receive (&huart1, buffer, 1,1000)==HAL_OK){
				HAL_Delay(1000); 
		

			memset (buffer,0,sizeof(buffer));
			memset (ATcommand,0,sizeof(ATcommand));
			sprintf(ATcommand,"AT+HTTPPARA=\"URL\",\"http://sensors-sanslab.herokuapp.com/sensors/nhiet-do?data=%0.2f\"\r\n",Temp);
			HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
			if(HAL_UART_Receive (&huart1, buffer, 1,2000)==HAL_OK){
					HAL_Delay(2000); 
				

				memset (buffer,0,sizeof(buffer));
				memset (ATcommand,0,sizeof(ATcommand));
				sprintf(ATcommand,"AT+HTTPACTION=1\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
				if(HAL_UART_Receive (&huart1, buffer, 1,2000)==HAL_OK){
				HAL_Delay(3000);
				memset (buffer,0,sizeof(buffer));
				memset (ATcommand,0,sizeof(ATcommand));
					
				sprintf(ATcommand,"AT+HTTPTERM\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
					HAL_Delay(1000);	
				}	
		
		}
	}
} 
		
 HAL_Delay(5000);
					
	//do tds
var=0;
for (int i=0;i<50;i++){
	HAL_ADC_Start(&hadc1);
		HAL_Delay(60);
		adc[i]=(HAL_ADC_GetValue(&hadc1));	
		if (adc[i]==0) {i--;}
		HAL_ADC_Stop(&hadc1);
		var= var+ adc[i];
}
	tb=var/50;

	a=3.3*tb/(1+0.02*(Temp-25));
		
		tds= (133.42*a*a*a/(4096*4096*4096)-255.86*a*a/(4096*4096)+857.39*a/4096)/2;
HAL_Delay(2000);
	//gui data
		memset (buffer,0,sizeof(buffer));
		memset (ATcommand,0,sizeof(ATcommand));
		sprintf(ATcommand,"AT+HTTPINIT\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);

		if(HAL_UART_Receive (&huart1, buffer, 1, 2000)==HAL_OK){
				HAL_Delay(2000); 
					
			memset (buffer,0,sizeof(buffer));
			memset (ATcommand,0,sizeof(ATcommand));
			sprintf(ATcommand,"AT+HTTPPARA=\"CID\",1\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
			if(HAL_UART_Receive (&huart1, buffer, 1, 2000)==HAL_OK){
				HAL_Delay(2000); 
		

			memset (buffer,0,sizeof(buffer));
			memset (ATcommand,0,sizeof(ATcommand));
			sprintf(ATcommand,"AT+HTTPPARA=\"URL\",\"http://sensors-sanslab.herokuapp.com/sensors/tds?data=%0.2f\"\r\n",tds);
			HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
			if(HAL_UART_Receive (&huart1, buffer, 1, 2000)==HAL_OK){
					HAL_Delay(2000); 
				

				memset (buffer,0,sizeof(buffer));
				memset (ATcommand,0,sizeof(ATcommand));
				sprintf(ATcommand,"AT+HTTPACTION=1\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
				if(HAL_UART_Receive (&huart1, buffer, 1, 2000)==HAL_OK){
				HAL_Delay(3000);
				memset (buffer,0,sizeof(buffer));
				memset (ATcommand,0,sizeof(ATcommand));
				sprintf(ATcommand,"AT+HTTPTERM\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
				}	
		
		}
	}
}
			HAL_Delay(5000);

		//luu vao mang
		
		if (b<=99){data[b]=tds;}
		else{
			for(int i=0;i<=98;i++){data[i]=data[i+1];}
			data[99]=tds;
		}
		b++;
		
		HAL_Delay(2000);
		//luu data vao flash	
		if(count>=2560){
			if(page==0){FLASH_ErasePage (FLASH_ADDR_PAGE_118, FLASH_USER_END_ADDR2);}
			else{FLASH_ErasePage (FLASH_ADDR_PAGE_108, FLASH_USER_END_ADDR1);}
			page=!page;
			count=0;
			
		}
		
		if (page==0){
			HAL_FLASH_Unlock ();
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_ADDR_PAGE_108+count*4,tds);
			HAL_FLASH_Lock ();
			}
		if (page==1){
			HAL_FLASH_Unlock ();
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,FLASH_ADDR_PAGE_118+count*4,tds);
			HAL_FLASH_Lock ();
			}
			count++;
		HAL_Delay(500);
		
//check dieu khien dong co
for(dieukhien=0;dieukhien<25;dieukhien++){
	//dong co 

			memset (buffer,0,sizeof(buffer));
		memset (ATcommand,0,sizeof(ATcommand));
		sprintf(ATcommand,"AT+HTTPINIT\r\n");
		HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);

		if(HAL_UART_Receive (&huart1, buffer, 1, 1000)==HAL_OK){
			
					HAL_Delay(1000);
			memset (buffer,0,sizeof(buffer));
			memset (ATcommand,0,sizeof(ATcommand));
			sprintf(ATcommand,"AT+HTTPPARA=\"CID\",1\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
			if(HAL_UART_Receive (&huart1, buffer, 1, 1000)==HAL_OK){
				
		HAL_Delay(1000);

			memset (buffer,0,sizeof(buffer));
			memset (ATcommand,0,sizeof(ATcommand));
			sprintf(ATcommand,"AT+HTTPPARA=\"URL\",\"http://sensors-sanslab.herokuapp.com/dieukhien/readall\"\r\n");
			HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
			if(HAL_UART_Receive (&huart1, buffer, 1, 1000)==HAL_OK){
					HAL_Delay(1000);
		

				memset (buffer,0,sizeof(buffer));
				memset (ATcommand,0,sizeof(ATcommand));
				sprintf(ATcommand,"AT+HTTPACTION=0\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
				if(HAL_UART_Receive (&huart1, buffer, 1, 1000)==HAL_OK){
				HAL_Delay(3000);
				memset (buffer,0,sizeof(buffer));
				memset (ATcommand,0,sizeof(ATcommand));
				sprintf(ATcommand,"AT+HTTPREAD\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
				HAL_UART_Receive (&huart1, buffer, 100, 5000);
					for (int z=0;z<81;z++){
						if (buffer[z]==91&&buffer[z+1]==102&&buffer[z+2]==97&&buffer[z+3]==108&&buffer[z+4]==115&&buffer[z+5]==101){
								d1=0;
								if (buffer[z+7]==102&&buffer[z+8]==97&&buffer[z+9]==108&&buffer[z+10]==115&&buffer[z+11]==101){
										d2=0;
										if (buffer[z+13]==102&&buffer[z+14]==97&&buffer[z+15]==108&&buffer[z+16]==115&&buffer[z+17]==101){
											d3=0;
										}
										if((buffer[z+13]==116&&buffer[z+14]==114&&buffer[z+15]==117&&buffer[z+16]==101)){d3=1;}
								}
								if((buffer[z+7]==116&&buffer[z+8]==114&&buffer[z+9]==117&&buffer[z+10]==101)){
										d2=1;
										if (buffer[z+12]==102&&buffer[z+13]==97&&buffer[z+14]==108&&buffer[z+15]==115&&buffer[z+16]==101){
												d3=0;
										}
										if((buffer[z+12]==116&&buffer[z+13]==114&&buffer[z+14]==117&&buffer[z+15]==101)){d3=1;}
								}
						}
						if (buffer[z]==91&&buffer[z+1]==116&&buffer[z+2]==114&&buffer[z+3]==117&&buffer[z+4]==101){
								d1=1;
								if (buffer[z+6]==102&&buffer[z+7]==97&&buffer[z+8]==108&&buffer[z+9]==115&&buffer[z+10]==101){
										d2=0;
										if (buffer[z+12]==102&&buffer[z+13]==97&&buffer[z+14]==108&&buffer[z+15]==115&&buffer[z+16]==101){
											d3=0;
										}
										if((buffer[z+12]==116&&buffer[z+13]==114&&buffer[z+14]==117&&buffer[z+15]==101)){d3=1;}
								}
								if((buffer[z+6]==116&&buffer[z+7]==114&&buffer[z+8]==117&&buffer[z+9]==101)){
										d2=1;
										if (buffer[z+11]==102&&buffer[z+12]==97&&buffer[z+13]==108&&buffer[z+14]==115&&buffer[z+15]==101){
												d3=0;
										}
										if((buffer[z+11]==116&&buffer[z+12]==114&&buffer[z+13]==117&&buffer[z+14]==101)){d3=1;}
								}
						}
				}
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, d1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, d2);
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, d3);
					memset (buffer,0,sizeof(buffer));
				memset (ATcommand,0,sizeof(ATcommand));
				sprintf(ATcommand,"AT+HTTPTERM\r\n");
				HAL_UART_Transmit(&huart1,(uint8_t *)ATcommand,strlen(ATcommand),1000);
		}
		}
	}
}
		HAL_Delay(500);
}

	


		    
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
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	 HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DS18B20_GPIO_Port, DS18B20_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
  /*Configure GPIO pin : DS18B20_Pin */
  GPIO_InitStruct.Pin = DS18B20_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DS18B20_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
if (hadc->Instance == hadc1.Instance){
	
	var=HAL_ADC_GetValue(&hadc1);
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
