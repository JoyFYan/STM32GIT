/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @date    06/03/2015 17:35:46
  * @brief   Interrupt Service Routines.
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
#include "stm32f3xx.h"
#include "stm32f3xx_it.h"
/* USER CODE BEGIN 0 */
#define DT_RISING       ((uint16_t)75) // Dead time rising edge
#define DT_FALLING      ((uint16_t)75) // Dead time falling edge
#define BUCK_PWM_PERIOD ((uint16_t) 0xFFDF) /* 70kHz */
#define MAX_ERROR     ((uint16_t)500)
#define Uesrconst 0.0010048273320293597

#include "arm_math.h"

extern int32_t PI_Buck(void);
extern int32_t PI_SPWM(void);
extern HRTIM_HandleTypeDef hhrtim1;
extern unsigned int cont;
extern unsigned int f;
extern void OLED_P8x16Str(unsigned char x,unsigned char y,unsigned char ch[]);
uint32_t VoutT=2100,VoutConversion=1,VoutConversion1=0,IoutConversion=0,Vrms=0,Irms=0,ticks=0;
static uint8_t Run=1,overload=0;
uint32_t *Vout;
uint32_t CurrentDutyA;
uint32_t CTMin;
uint32_t CTMax;
char topflag=0;
unsigned char ptr[6]={0,0,0,0,' ','\0'};
unsigned char ptr0[6]={0,0,0,0,' ','\0'};
unsigned char ptr1[6]={0,0,0,0,' ','\0'};
unsigned char addmin=1;
unsigned int T,temp1;
float32_t temp;
/* USER CODE END 0 */
/* External variables --------------------------------------------------------*/

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern HRTIM_HandleTypeDef hhrtim1;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13))
	{
			Run=1;
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_SET);
	}
	
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_14))
	{
			f++;
	}
	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_15))
	{
			f--;
	}
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  /* USER CODE BEGIN EXTI15_10_IRQn 1 */

  /* USER CODE END EXTI15_10_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	Vout=&VoutConversion;
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
	
			if(Irms>=4000&&Run==1)
		{
			if(overload++>100)
			{
				overload=0;
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15, GPIO_PIN_RESET);
				Run=0;
			}
		}
			else
			{
				overload=0;
			}
			
		
			
			
		
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/**
* @brief This function handles HRTIM timer A global interrupt.
*/
void HRTIM1_TIMA_IRQHandler(void)
{
  /* USER CODE BEGIN HRTIM1_TIMA_IRQn 0 */

  /* USER CODE END HRTIM1_TIMA_IRQn 0 */
  HAL_HRTIM_IRQHandler(&hhrtim1,HRTIM_TIMERINDEX_TIMER_A);
  /* USER CODE BEGIN HRTIM1_TIMA_IRQn 1 */
	
	
	//if(Run==1)
	//{
		CurrentDutyA = PI_Buck();
		__HAL_HRTIM_SetCompare(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, CurrentDutyA);
		if (CurrentDutyA > BUCK_PWM_PERIOD / 2)
      {
        /* Set ADC trigger position according CurrentDutyA */
        __HAL_HRTIM_SetCompare(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, DT_RISING + CurrentDutyA / 2);
      }
    else
      {
        __HAL_HRTIM_SetCompare(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_2, ((BUCK_PWM_PERIOD - CurrentDutyA) / 2) + CurrentDutyA);
     }
			
			
//		if (CTMax >= MAX_ERROR)
//      {
//        Run = 0;
//      }
//		if (CTMin >= MAX_ERROR)
//      {
//      /* Stop converter */
//        Run = 0;
//      }
			

//	
//	T = 32752+(30000*arm_sin_f32(Uesrconst *cont*f));
//	if(cont++>6253)cont=0;
//	//HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &pCompareCfg);
////	if(T==32752)
//		__HAL_HRTIM_SetCompare(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1,T);
//	
//   
//      /* Min % duty reached for a given time */
//    if(T>62730&&topflag==0)topflag=1;
//	 if(T<=62730&&topflag==1)
//	 {
//		 topflag=0;
//		 Vrms=VoutConversion1;
//		 Irms=IoutConversion;
//		 VoutT = PI_SPWM();
//	 }
//	 
	
	
  /* USER CODE END HRTIM1_TIMA_IRQn 1 */
}

/**
* @brief This function handles ADC1 and ADC2 interrupts.
*/
void ADC1_2_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_2_IRQn 0 */
	
	if( __HAL_ADC_GET_FLAG(&hadc1, ADC_ISR_JEOS))
	{

		VoutConversion=HAL_ADCEx_InjectedGetValue(&hadc1, ADC_INJECTED_RANK_1);
		

	}
  /* USER CODE END ADC1_2_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  HAL_ADC_IRQHandler(&hadc2);
  /* USER CODE BEGIN ADC1_2_IRQn 1 */
	
  /* USER CODE END ADC1_2_IRQn 1 */
}

/* USER CODE BEGIN 1 */
	//if(cont<0||cont>=3.1415)t=-t;
//	if(x>=1.570796)t=-t;
	//cont=cont+t;
	//pCompareCfg.CompareValue = 32752+(uint32_t)32000*arm_sin_f32(Uesrconst*cont*f);
	//cont++;	
	//if(cont>5000)cont=0;
		//	if(x<0||x>=3.1415)t=-t;

	//x=x+t;
		//pCompareCfg.CompareValue=1000+(unsigned int)(60000*arm_sin_f32(x));
	//pCompareCfg.CompareValue = 32752+(uint32_t)32000*arm_sin_f32(Uesrconst*cont*f);
//			pCompareCfg.CompareValue =cont;
//			cont=cont+1;
//			if(cont>50000)cont=0;
//  HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &pCompareCfg);*/

		//cont=cont+1;
		//if(cont>80)cont=0;
		//temp=arm_sin_f32(Uesrconst*cont*f);
//		if(Uesrconst*cont*f<0||cont<=1)
//		{
//			temp=0;
//			addmin=-addmin;
//		}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
