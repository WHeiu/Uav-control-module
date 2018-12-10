/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */
#include "stm32l4xx_hal.h"
#include <stdint.h>
#include "stm32l4xx_ll_crs.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_exti.h"
#include "stm32l4xx_ll_cortex.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_pwr.h"
#include "stm32l4xx_ll_dma.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx.h"
#include "stm32l4xx_ll_gpio.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define EC20_STA_Pin GPIO_PIN_15
#define EC20_STA_GPIO_Port GPIOA
#define MP_SCK_Pin GPIO_PIN_3
#define MP_SCK_GPIO_Port GPIOB
#define MP_CS_Pin GPIO_PIN_6
#define MP_CS_GPIO_Port GPIOB
#define MP_SDO_Pin GPIO_PIN_4
#define MP_SDO_GPIO_Port GPIOB
#define MP_SDI_Pin GPIO_PIN_5
#define MP_SDI_GPIO_Port GPIOB
#define LED_GREEN_Pin GPIO_PIN_10
#define LED_GREEN_GPIO_Port GPIOA
#define EC20_RESET_Pin GPIO_PIN_9
#define EC20_RESET_GPIO_Port GPIOC
#define EC20_CTL_Pin GPIO_PIN_7
#define EC20_CTL_GPIO_Port GPIOC
#define EC20_DISABLE_Pin GPIO_PIN_8
#define EC20_DISABLE_GPIO_Port GPIOC
#define EC20_RI_Pin GPIO_PIN_6
#define EC20_RI_GPIO_Port GPIOC
#define BMP_SDO_Pin GPIO_PIN_2
#define BMP_SDO_GPIO_Port GPIOC
#define BMP_SDI_Pin GPIO_PIN_15
#define BMP_SDI_GPIO_Port GPIOB
#define EC20_NETMODE_Pin GPIO_PIN_14
#define EC20_NETMODE_GPIO_Port GPIOB
#define EC20_TX_Pin GPIO_PIN_1
#define EC20_TX_GPIO_Port GPIOA
#define BMP_CS_Pin GPIO_PIN_3
#define BMP_CS_GPIO_Port GPIOC
#define UAV_232_RDY_Pin GPIO_PIN_12
#define UAV_232_RDY_GPIO_Port GPIOB
#define BMP_SCK_Pin GPIO_PIN_13
#define BMP_SCK_GPIO_Port GPIOB
#define EC20_RX_Pin GPIO_PIN_0
#define EC20_RX_GPIO_Port GPIOA
#define LE_RED_Pin GPIO_PIN_2
#define LE_RED_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
//void EC20_Transmit(uint8_t* UART4_Tx);
// void USART3_CLR_RecvBuf(void);
// void EC20_Receive(void);
//  void UART4_CLR_RecvBuf(void);
	//void Delay(unsigned int xms);
//	void USART3_Ack_Interaction(void);
//	void convertUnCharToStr(char* str, unsigned char* UnChar, int ucLen);
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
