#include "bool.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "MPU6050.h"
#include "usart.h"
#include <stdio.h>
#include <math.h>

extern uint16_t CCR3_Val ;
extern uint16_t CCR4_Val ;
uint16_t PrescalerValue = 0;
TIM_OCInitTypeDef  TIM_OCInitStructure;
extern int16_t buff[6];
extern float acc[3],gyro[3],num;
extern float tan_x;
extern float angle_x;
extern float w_z,w_y;
extern float err,setpoint;
extern float kp,kd,ka;
extern int i,f,g;
void time_delay(uint32_t count)
{
	while (i<count)
	{
      

	}

	i = 0 ;
}
void delay(uint32_t delay_count)
{
	while (delay_count) delay_count--;
}
void init_led()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	/* GPIOA Periph clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* Configure PA0 and PA1 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}
void gpio_toggle(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIOx->ODR ^= GPIO_Pin;
}
void init_tim2() //timer for time delay (per second)
{	
	RCC_PCLK1Config(RCC_HCLK_Div2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM2 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 7200-1;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(100-1);

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// /* Prescaler configuration */
	// TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);

	TIM_ITConfig( TIM2, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
}
void init_tim3() //timer for contral  100HZ
{	
	RCC_PCLK1Config(RCC_HCLK_Div2);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the TIM2 global Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	NVIC_Init(&NVIC_InitStructure);

	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 7200-1;
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)(100-1);

	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// /* Prescaler configuration */
	// TIM_PrescalerConfig(TIM2, PrescalerValue, TIM_PSCReloadMode_Immediate);

	TIM_ITConfig( TIM3, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIM3, ENABLE);
}

void init_tim4_pwm() //timer for PWM
{	
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	

	/* TIM3 clock enable */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

  	/* GPIOA and GPIOB clock enable */
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);	

  	/*GPIOB Configuration: TIM3 channel1, 2, 3 and 4 */
  	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 | GPIO_Pin_9;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  	GPIO_Init(GPIOB, &GPIO_InitStructure);

 
  	
	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 57600 - 1;
	TIM_TimeBaseStructure.TIM_Prescaler = 25 -1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* PWM1 Mode configuration: Channel1 */
	  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
	  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	  TIM_OC3Init(TIM4, &TIM_OCInitStructure);

	  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);

	  /* PWM1 Mode configuration: Channel2 */
	  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;

	  TIM_OC4Init(TIM4, &TIM_OCInitStructure);

	  TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);

	  TIM_ARRPreloadConfig(TIM4, ENABLE);

	  /* TIM3 enable counter */
	  TIM_Cmd(TIM4, ENABLE);
}

int main(void)
{
	
	
	//float prev_err;
	//float integral,ki = 0
	init_led();
	init_usart1();

	init_tim2();
	

     init_tim4_pwm();

	MPU6050_I2C_Init();
	MPU6050_Initialize();
	if( MPU6050_TestConnection() == TRUE)
	{
	   puts("connection success\r\n");
	}else {
	   puts("connection failed\r\n");
	}
	printf("test float%f\r\n",num);


	init_tim3();
	while (1) {
		
		printf("acc_x,%f,acc_y,%f,acc_z,%f,gyro_x,%f,gyro_y,%f,gyro_z,%f\r\n",
			acc[0], acc[1], acc[2],
			gyro[0], gyro[1], gyro[2]);
		

		//printf("abcc,%f\r\n",acc[0]/acc[2]);



		//turn right when x is positive
		//turn left when f is negtive
		//turn left when gyro[1] is positive

		gpio_toggle(GPIOA, GPIO_Pin_0);
		gpio_toggle(GPIOA, GPIO_Pin_1);
		time_delay(100); //5

		/*if ( (CCR3_Val>462) || (CCR4_Val > 642) ){

			CCR3_Val = 432;
			CCR4_Val = 612;

		} else if ( (CCR3_Val<402) || (CCR4_Val <592) ){

			CCR3_Val = 432;
			CCR4_Val = 612;

		}*/
		//printf("tan_x,%f\r\n",tan_x);	
		//printf("atan,%f\r\n",y);
		//printf("Angle_x,%f\r\n",angle_x);
		//printf("erro,%f\r\n",err);
		//printf("Angular velocity(rad),%f\r\n",gyro[2]);
		printf("Angular velocity(degree),%f\r\n",w_z);
		printf("Angular velocity(degree),%f\r\n",w_y);
		printf("G,%d\r\n\n",g);
		printf("F,%d\r\n\n",f);

		//printf("ccr3,%d\r\n",CCR3_Val);
		//printf("ccr4,%d\r\n\n",CCR4_Val);	

		//prev_err = err;

         //setpoint = -prev_err ;

		

	}
}
