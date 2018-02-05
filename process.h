///**
//  ********************************  STM32F0xx  *********************************
//  * @�ļ���     �� process.c
//  * @����       �� HarryZeng
//  * @��汾     �� V1.5.0
//  * @�ļ��汾   �� V1.0.0
//  * @����       �� 2017��04��21��
//  * @ժҪ       �� ���ݴ���
//  ******************************************************************************/
///*----------------------------------------------------------------------------
//  ������־:
//  2017-04-21 V1.0.0:��ʼ�汾
//  ----------------------------------------------------------------------------*/
///* ������ͷ�ļ� --------------------------------------------------------------*/
/* Includes ------------------------------------------------------------------*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __process_H
#define __process_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "stm32f0xx.h"

#define ADC_IN_Pin 				GPIO_Pin_0
#define ADC_IN_GPIO_Port 	GPIOA
#define PWM1_Pin 					GPIO_Pin_2
#define PWM1_GPIO_Port 		GPIOA
#define GOODBAD_Pin 			GPIO_Pin_3
#define GOODBAD_GPIO_Port GPIOA
#define SET_Pin 					GPIO_Pin_4
#define SET_GPIO_Port 		GPIOA
#define PWMX_Pin 					GPIO_Pin_8
#define PWMX_GPIO_Port		GPIOA
#define PWMY_Pin 					GPIO_Pin_9
#define PWMY_GPIO_Port 		GPIOA
#define PWMZ_Pin 					GPIO_Pin_10
#define PWMZ_GPIO_Port 		GPIOA
#define FB_Pin 						GPIO_Pin_11
#define FB_GPIO_Port 			GPIOA
#define OUT_Pin 					GPIO_Pin_12
#define OUT_GPIO_Port 		GPIOA	 
	 
#define Algorithm_P				4096	 
	 
#define 	FLASH_START_ADDR 	 						0x08007000

/*Ӧ��ֵ*/	

	#define PWM1_HIGH  	100
	#define PWM1_LOW  	20
	
	#define PWM_C_Start  	1
	
	#define PWMx_HIGH  	1
	#define PWMx_LOW  	0
	#define PWMy_HIGH  	2
	#define PWMy_LOW  	0
	#define PWMz_HIGH  	3
	#define PWMz_LOW  	0


/*PWMͨ��״̬���� �궨��*/

#define PWM1_ON  	GPIO_WriteBit(PWM1_GPIO_Port, PWM1_Pin, Bit_SET)
#define PWM1_OFF  GPIO_WriteBit(PWM1_GPIO_Port, PWM1_Pin, Bit_RESET)

#define PWMX_ON  	GPIO_WriteBit(PWMX_GPIO_Port, PWMX_Pin, Bit_SET)
#define PWMX_OFF  GPIO_WriteBit(PWMX_GPIO_Port, PWMX_Pin, Bit_RESET)

#define PWMY_ON  	GPIO_WriteBit(PWMY_GPIO_Port, PWMY_Pin, Bit_SET)
#define PWMY_OFF  GPIO_WriteBit(PWMY_GPIO_Port, PWMY_Pin, Bit_RESET)

#define PWMZ_ON  	GPIO_WriteBit(PWMZ_GPIO_Port, PWMZ_Pin, Bit_SET)
#define PWMZ_OFF  GPIO_WriteBit(PWMZ_GPIO_Port, PWMZ_Pin, Bit_RESET)

#define MainTIMER  TIM3

//#define PWM1_ON				TIM_SetCompare3(TIM2,PWM1_HIGH);					
//#define PWM1_OFF			TIM_SetCompare3(TIM2,PWM1_LOW)

//#define PWMX_ON				TIM_SetCompare1(TIM1,PWMx_HIGH)
//#define PWMX_OFF			TIM_SetCompare1(TIM1,PWMx_LOW)

//#define PWMY_ON				TIM_SetCompare2(TIM1,PWMy_HIGH)
//#define PWMY_OFF			TIM_SetCompare2(TIM1,PWMy_LOW)

//#define PWMZ_ON				TIM_SetCompare3(TIM1,PWMz_HIGH)
//#define PWMZ_OFF			TIM_SetCompare3(TIM1,PWMz_LOW)

#define SETPin				GPIO_ReadInputDataBit(SET_GPIO_Port,SET_Pin)

#define shortKEY 	50 
#define middleKEY	100
#define longKEY		150	 
	 
extern uint32_t ADC_value;	 
	 
typedef enum
{
  PWMX = 0U,
  PWMY,
	PWMZ
}PWM_Number;
	 

typedef enum
{
  PWM_ON = 0U,
	PWM_OFF
}PWM_STATE;
	 


void DataProcess(void);
void ResetParameter(void);


#ifdef __cplusplus
}
#endif
#endif 
/**** Copyright (C)2017 HarryZeng. All Rights Reserved **** END OF FILE ****/
