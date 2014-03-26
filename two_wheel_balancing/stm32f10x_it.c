/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"

#include "bool.h"
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "MPU6050.h"
#include "usart.h"
#include <stdio.h>
#include <math.h>

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
int i=0,f,g;
int16_t buff[6];
float acc[3],gyro[3],num=3.444;
float tan_x;
float angle_x;
float pitch,yaw;
float w_z,w_y;
float err,integal,setpoint=0;
float kp=60,kd=0,ka=0,ki=0.5;
//float kp=0,kd=0,ka=0,ki= 0;
uint16_t CCR3_Val = 3564;
uint16_t CCR4_Val = 4896;


void TIM2_IRQHandler()
{
        if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET){
               
                TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
                i++;
        }
}
void TIM3_IRQHandler()
{


        if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET){
               
                    //puts("running now\r\n");
                    MPU6050_GetRawAccelGyro(buff);
                    for ( int i = 0; i<3; i++)
                      acc[i] = (buff[i]/16384.0);
                    for ( int i = 0; i<3; i++)
                      gyro[i] = (buff[i+2]/131.0);

                    tan_x = acc[0]/acc[2] ;
                    angle_x = atanf(tan_x)*57.29578;

                    err = setpoint - angle_x;
                    integal = integal + err;

                    /*if ( (gyro[2] < 4.4) && (gyro[2] > 3.7) )
                    {
                        gyro[2] = 0;
                    }

                    if ( (gyro[1] < -0.5) && (gyro[1] > -1.2)  )
                    {
                        gyro[1] = 0;
                    }*/


                    w_z = gyro[2]*57.29578;
                    w_y = gyro[1]*57.29578;


                    f = (kp*err + kd*w_z + ki*integal)/36;
                    g = ka*w_y-288; 
                    


                    
                    CCR3_Val = 3564 +36*f+g;  
                    CCR4_Val = 4896 -36*f+g;
                  
                    
                    TIM4->CCR3 = CCR3_Val;
                    TIM4->CCR4 = CCR4_Val;


                    if ( CCR3_Val>6336 ){

                      CCR3_Val = 6336;

                    } else if ( CCR3_Val<576 ){

                      CCR3_Val = 576;
        
                    }

                    if ( CCR4_Val >7776 ){

                      CCR4_Val = 7776;

                    } else if ( CCR4_Val <2016 ){

                      CCR4_Val = 2016;

                    }
                    

                    /*if ( (CCR3_Val>57600) || (CCR4_Val >57600) ){

                      CCR3_Val = 3564;
                      CCR4_Val = 4896;

                    } else if ( (CCR3_Val<0) || (CCR4_Val <0) ){

                      CCR3_Val = 3564;
                      CCR4_Val = 4896;

                    }*/





                TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
                
        }
}
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1) {
	}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1) {
	}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1) {
	}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1) {
	}
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
