/**
  ******************************************************************************
  * File Name          : main.c
  * Date               : 06/03/2015 17:35:50
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* USER CODE BEGIN Includes */
#include "arm_sin_f32.c"
#include "arm_math.h"
#include "oled.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

HRTIM_HandleTypeDef hhrtim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_HRTIM1_Init(void);

/* USER CODE BEGIN PFP */
#define BUCK_PWM_PERIOD ((uint16_t) 65503) /* 70kHz */
#define SAT_LIMIT ((int32_t)(BUCK_PWM_PERIOD * 50)/100) // PI Integral term saturation value
#define MIN_DUTY_A ((int32_t)(BUCK_PWM_PERIOD * 5)/100) // % MinDuty for Buck
#define MAX_DUTY_A ((int32_t)(BUCK_PWM_PERIOD * 95)/100) // % MaxDuty for Buck

 void Reset_PI(void);
	
static int32_t Kp;
static int32_t Ki;
static int32_t Kp1;
static int32_t Ki1;
//static int32_t Int_term_Buck,Int_term_BuckB;
static int32_t Int_term_I,Int_term_V,Iref;
extern int32_t VoutT;
extern int32_t VoutReal,VinReal,IReal,Vrms,Irms,IoutA,IoutB;
extern uint32_t CTMin;
extern uint32_t CTMax;
extern int temp1;
unsigned int cont=0;
unsigned int f=50;
float D=0;
extern  uint8_t Run;
extern unsigned int CurrentDutyA;
extern unsigned char ptr0[],ptr[],ptr1[],ptrs[],fptr[],dptr[],modep[];
// long int temp1;
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
//	GPIO_InitTypeDef GPIO_InitStruct;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_HRTIM1_Init();

  /* USER CODE BEGIN 2 */
	for(unsigned int i=0;i<2000;i++)
	{
		for(unsigned int j=0;j<800;j++);
	}
	
	
	OLED_Init(); //OLED³õÊ¼»¯
	OLED_Fill(0x00);
	HAL_HRTIM_WaveformOutputStart(&hhrtim1,HRTIM_OUTPUT_TA1 |HRTIM_OUTPUT_TA2);

	HAL_HRTIM_WaveformCounterStart_IT(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_MASTER);

	/*HAL_HRTIM_WaveformOutputStart(&hhrtim1, HRTIM_OUTPUT_TB1 |HRTIM_OUTPUT_TB2 |HRTIM_OUTPUT_TA1 |HRTIM_OUTPUT_TA2);

	HAL_HRTIM_WaveformCounterStart_IT(&hhrtim1, HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_MASTER);
    //HAL_HRTIM_WaveformCounterStart(&hhrtim1, HRTIM_TIMERID_TIMER_B);*/
 HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_RESET);
	
	
	//OLED_Fill(0xff);
		
		OLED_P8x16Str(0,2,"D:");
		OLED_P8x16Str(0,4,"Vi");
		OLED_P8x16Str(0,6,"Vo");
	//OLED_P6x8Str(0,3,"POWER BUCK");
	Reset_PI();
	
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	// HAL_ADC_Start_IT(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc2);
  /* USER CODE END 2 */

  /* USER CODE BEGIN 3 */
  /* Infinite loop */
  while (1)
  {
		//		{
		
			OLED_P8x16Str(0,0,modep);
		
		
			ptr0[0]=IReal/1000;
			temp1=IReal-(ptr0[0]*1000);
			ptr0[1]=temp1/100;
			temp1=temp1-ptr0[1]*100;
			ptr0[2]=temp1/10;
			temp1=temp1-ptr0[2]*10;
			ptr0[3]=temp1;
			ptr0[0]=ptr0[0]+48;
			ptr0[1]=ptr0[1]+48;
			ptr0[2]=ptr0[2]+48;
			ptr0[3]=ptr0[3]+48;
			OLED_P8x16Str(60,6,ptr0);
		
		
			ptr1[0]=VoutReal/1000;
			temp1=VoutReal-(ptr1[0]*1000);
			ptr1[1]=temp1/100;
			temp1=temp1-ptr1[1]*100;
			ptr1[2]=temp1/10;
			temp1=temp1-ptr1[2]*10;
			ptr1[3]=temp1;
			ptr1[0]=ptr1[0]+48;
			ptr1[1]=ptr1[1]+48;
			ptr1[2]=ptr1[2]+48;
			ptr1[3]=ptr1[3]+48;
			OLED_P8x16Str(25,4,ptr1);
		
			
			
			ptr[0]=VoutReal/1000;
			temp1=VoutReal-(ptr[0]*1000);
			ptr[1]=temp1/100;
			temp1=temp1-ptr[1]*100;
			ptr[2]=temp1/10;
			temp1=temp1-ptr[2]*10;
			ptr[3]=temp1;
			ptr[0]=ptr[0]+48;
			ptr[1]=ptr[1]+48;
			ptr[2]=ptr[2]+48;
			ptr[3]=ptr[3]+48;
		OLED_P8x16Str(25,6,ptr);
		
		
			fptr[0]=f/100;
	temp1=f-(fptr[0]*100);
	fptr[1]=temp1/10;
	temp1=temp1-fptr[1]*10;
	fptr[2]=temp1;
			
	fptr[0]=fptr[0]+48;
	fptr[1]=fptr[1]+48;
	fptr[2]=fptr[2]+48;
			
	OLED_P8x16Str(86,4,fptr);
	
	
	D= (CurrentDutyA)*100/BUCK_PWM_PERIOD;
	dptr[0]=D/10;
	dptr[1]=D-(dptr[0]*10);
	dptr[0]=dptr[0]+48;
	dptr[1]=dptr[1]+48;
	OLED_P8x16Str(25,2,dptr);
	
	
	OLED_P8x16Str(86,2,ptrs);
//	if(Run==0){ptrs[0]='S';OLED_P8x16Str(86,2,ptrs);}
//	else {ptrs[0]='R';OLED_P8x16Str(86,2,ptrs);}
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_HRTIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Hrtim1ClockSelection = RCC_HRTIM1CLK_PLLCLK;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  __SYSCFG_CLK_ENABLE();

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  ADC_InjectionConfTypeDef sConfigInjected;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc1.Init.Resolution = ADC_RESOLUTION12b;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc1);

    /**Configure Inj ected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_HRTIM_TRG2;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected);

}

/* ADC2 init function */
void MX_ADC2_Init(void)
{

  ADC_InjectionConfTypeDef sConfigInjected;

    /**Common config 
    */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC;
  hadc2.Init.Resolution = ADC_RESOLUTION12b;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = EOC_SEQ_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = OVR_DATA_OVERWRITTEN;
  HAL_ADC_Init(&hadc2);

    /**Configure Inj ected Channel 
    */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = 1;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedNbrOfConversion = 2;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJECCONV_HRTIM_TRG4;//TRG4 TIMERA PA4
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

    /**Configure Inj ected Channe2*/
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = 2;
  HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected);

}

/* HRTIM1 init function */
void MX_HRTIM1_Init(void)
{

  HRTIM_ADCTriggerCfgTypeDef pADCTriggerCfg;
  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg;
  HRTIM_TimerCfgTypeDef pTimerCfg;
  HRTIM_CompareCfgTypeDef pCompareCfg;
  HRTIM_OutputCfgTypeDef pOutputCfg;
  HRTIM_DeadTimeCfgTypeDef pDeadTimeCfg;

  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  HAL_HRTIM_Init(&hhrtim1);

  HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_14);

  HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 100);

  pADCTriggerCfg.UpdateSource = HRTIM_ADCTRIGGERUPDATE_TIMER_A;
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT24_TIMERA_CMP2;
  HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_2, &pADCTriggerCfg);

//	pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT24_TIMERA_CMP4;
//  HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_3, &pADCTriggerCfg);
	
  pADCTriggerCfg.Trigger = HRTIM_ADCTRIGGEREVENT24_TIMERA_CMP4;
  HAL_HRTIM_ADCTriggerConfig(&hhrtim1, HRTIM_ADCTRIGGER_4, &pADCTriggerCfg);

  pTimeBaseCfg.Period = BUCK_PWM_PERIOD;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimeBaseCfg);

  pTimerCfg.InterruptRequests = HRTIM_MASTER_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_MASTER_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0;
  pTimerCfg.DMADstAddress = 0x0;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_MASTER, &pTimerCfg);

  pCompareCfg.CompareValue = 40000;
  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &pCompareCfg);

  pCompareCfg.CompareValue = 30000;
  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &pCompareCfg);

  pTimeBaseCfg.Period = BUCK_PWM_PERIOD;
  pTimeBaseCfg.RepetitionCounter = 4;
  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg);

  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_REP;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0;
  pTimerCfg.DMADstAddress = 0x0;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_ENABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMDELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_MASTER_PER;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg);

  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMASrcAddress = 0x0;
  pTimerCfg.DMADstAddress = 0x0;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimerCfg);

  pCompareCfg.CompareValue = 20000;
  pCompareCfg.AutoDelayedMode = HRTIM_AUTODELAYEDMODE_REGULAR;
  pCompareCfg.AutoDelayedTimeout = 0x0000;
	
	
  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, &pCompareCfg);


	HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_4, &pCompareCfg);
	
  pCompareCfg.CompareValue = 15000;
  pCompareCfg.AutoDelayedTimeout = 0x0000;

  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_2, &pCompareCfg);

  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_TIMPER;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_TIMCMP1;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg);

  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB1, &pOutputCfg);

	pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_LOW;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA2, &pOutputCfg);

  HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_OUTPUT_TB2, &pOutputCfg);

  pDeadTimeCfg.Prescaler = HRTIM_TIMDEADTIME_PRESCALERRATIO_MUL8;
  pDeadTimeCfg.RisingValue = 220;
  pDeadTimeCfg.RisingSign = HRTIM_TIMDEADTIME_RISINGSIGN_POSITIVE;
  pDeadTimeCfg.RisingLock = HRTIM_TIMDEADTIME_RISINGLOCK_WRITE;
  pDeadTimeCfg.RisingSignLock = HRTIM_TIMDEADTIME_RISINGSIGNLOCK_WRITE;
  pDeadTimeCfg.FallingValue = 110;
  pDeadTimeCfg.FallingSign = HRTIM_TIMDEADTIME_FALLINGSIGN_POSITIVE;
  pDeadTimeCfg.FallingLock = HRTIM_TIMDEADTIME_FALLINGLOCK_WRITE;
  pDeadTimeCfg.FallingSignLock = HRTIM_TIMDEADTIME_FALLINGSIGNLOCK_WRITE;
  HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pDeadTimeCfg);

  HAL_HRTIM_DeadTimeConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pDeadTimeCfg);

  pTimeBaseCfg.Period = BUCK_PWM_PERIOD;
  pTimeBaseCfg.RepetitionCounter = 0;
  HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, &pTimeBaseCfg);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_15|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


	
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
//int32_t PI_Buck(void)
//{
//  /* Compute PI for Buck Mode */
//  /* Every time the PI order sets extreme values then CTMax or CTMin are managed */
//  int32_t seterr, pid_out;
//  int32_t error;

//  error = (int32_t ) VoutConversion - (int32_t) 1830;
//  seterr = (-Kp * error) / 200;

//  Int_term_Buck = Int_term_Buck + ((-Ki * error) / 200);

//  if (Int_term_Buck > SAT_LIMIT)
//  {
//    Int_term_Buck = SAT_LIMIT;
//  }
//  if (Int_term_Buck < -(SAT_LIMIT))
//  {
//    Int_term_Buck = -(SAT_LIMIT);
//  }
//  pid_out = seterr + Int_term_Buck;
//  pid_out += BUCK_PWM_PERIOD / 2;

//  if (pid_out >= MAX_DUTY_A)
//  {
//    pid_out = MAX_DUTY_A;
//    //CTMax++;
//  }
//  else
//  {
//    if (CTMax != 0)
//    {
//     // CTMax--;
//    }
//  }
//  if (pid_out <= MIN_DUTY_A)
//  {
//    pid_out = MIN_DUTY_A;
//    //CTMin++;
//  }
//  else
//  {
//    if (CTMin != 0)
//    {
//    //  CTMin--;
//    }
//  }
//  return  pid_out;
//}

 void Reset_PI(void)
{
  /* Reset integral terms for PI */
  //Int_term_Buck = 0;
	Int_term_I=0;
	Int_term_V=0;
//  Int_term_Boost = 0;
//  Int_term_Mixed = 0;
  /* Reset Counters Min and Max */
  CTMax = 0;
  CTMin = 0;
  /* Set Proportional and Integral constant terms*/
  Ki = 5;
  Kp = 20;
	Ki1=200;
	Kp1=200;
}

int32_t PI_I(void)
{
  /* Compute PI for Buck Mode */
  /* Every time the PI order sets extreme values then CTMax or CTMin are managed */
  int32_t seterr, pid_out;
  int32_t error;

  error = (int32_t ) IReal- (int32_t) Iref*VinReal/200;
  seterr = (-Kp1 * error)/200;


  Int_term_I = Int_term_I + ((-Ki1 * error) / 200);

  if (Int_term_I > SAT_LIMIT)
  {
    Int_term_I = SAT_LIMIT;
  }
  if (Int_term_I < -(SAT_LIMIT))
  {
    Int_term_I = -(SAT_LIMIT);
  }
  pid_out = seterr + Int_term_I;
  pid_out += 65503-(VinReal)/VoutReal;

  if (pid_out >= MAX_DUTY_A)
  {
    pid_out = MAX_DUTY_A;
    //CTMax++;
  }
  else
  {
    if (CTMax != 0)
    {
     // CTMax--;
    }
  }
  if (pid_out <= MIN_DUTY_A)
  {
    pid_out = MIN_DUTY_A;
    //CTMin++;
  }
  else
  {
    if (CTMin != 0)
    {
    //  CTMin--;
    }
  }
  return  pid_out;
}
int32_t PI_V(void)
{
  /* Compute PI for Buck Mode */
  /* Every time the PI order sets extreme values then CTMax or CTMin are managed */
  int32_t seterr, pid_out;
  int32_t error;

  error = (int32_t ) VoutReal- (int32_t) 550;
  seterr = (-Kp * error)/200;


  Int_term_V = Int_term_V + ((-Ki * error) / 200);

  if (Int_term_V > 500)
  {
    Int_term_V = 500;
  }
  if (Int_term_V < -(500))
  {
    Int_term_V = -(500);
  }
  pid_out = seterr + Int_term_V;
  pid_out += 500;

  if (pid_out >= 1000)
  {
    pid_out = 1000;
    //CTMax++;
  }
  else
  {
    if (CTMax != 0)
    {
     // CTMax--;
    }
  }
  if (pid_out <= 0)
  {
    pid_out = 0;
    //CTMin++;
  }
  else
  {
    if (CTMin != 0)
    {
    //  CTMin--;
    }
  }
  return  pid_out;
}
//int32_t PI_I(void)
//{
//  /* Compute PI for Buck Mode */
//  /* Every time the PI order sets extreme values then CTMax or CTMin are managed */
//  int32_t seterr, pid_out;
//  int32_t error;

//  error = (int32_t ) IoutA- (int32_t) 1060;
//  seterr = (-Kp1 * error)/100;

//  Int_term_SPWM = Int_term_SPWM + (-Ki1 * error)/100;
//	if (Int_term_SPWM > SAT_LIMIT)
//  {
//    Int_term_SPWM = SAT_LIMIT;
//  }
//  if (Int_term_SPWM < -(SAT_LIMIT))
//  {
//    Int_term_SPWM = -(SAT_LIMIT);
//  }
//	
//  if (pid_out >= MAX_DUTY_A)
//  {
//    pid_out = MAX_DUTY_A;
//    //CTMax++;
//  }
//  else
//  {
//    if (CTMax != 0)
//    {
//     // CTMax--;
//    }
//  }
//  if (pid_out <= MIN_DUTY_A)
//  {
//    pid_out = MIN_DUTY_A;
//    //CTMin++;
//  }
//  else
//  {
//    if (CTMin != 0)
//    {
//    //  CTMin--;
//    }
//  }
//  pid_out = seterr + Int_term_SPWM;
//  //pid_out+=500;
////	if(pid_out>2300)pid_out=2300;
//  
//  return  pid_out;
//}


/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
