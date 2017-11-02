/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define CE_Pin GPIO_PIN_15
#define CE_GPIO_Port GPIOC
#define OSC_IN_Pin GPIO_PIN_0
#define OSC_IN_GPIO_Port GPIOH
#define OSC_OUT_Pin GPIO_PIN_1
#define OSC_OUT_GPIO_Port GPIOH
#define L_EN_Pin GPIO_PIN_0
#define L_EN_GPIO_Port GPIOC
#define R_RX_Pin GPIO_PIN_1
#define R_RX_GPIO_Port GPIOC
#define RF_RX_Pin GPIO_PIN_2
#define RF_RX_GPIO_Port GPIOC
#define LF_RX_Pin GPIO_PIN_3
#define LF_RX_GPIO_Port GPIOC
#define L_CHA_Pin GPIO_PIN_0
#define L_CHA_GPIO_Port GPIOA
#define L_CHB_Pin GPIO_PIN_1
#define L_CHB_GPIO_Port GPIOA
#define L_RX_Pin GPIO_PIN_2
#define L_RX_GPIO_Port GPIOA
#define R_CHA_Pin GPIO_PIN_5
#define R_CHA_GPIO_Port GPIOA
#define L_PWM_Pin GPIO_PIN_6
#define L_PWM_GPIO_Port GPIOA
#define L_PH_Pin GPIO_PIN_7
#define L_PH_GPIO_Port GPIOA
#define R_PH_Pin GPIO_PIN_5
#define R_PH_GPIO_Port GPIOC
#define R_PWM_Pin GPIO_PIN_0
#define R_PWM_GPIO_Port GPIOB
#define R_EN_Pin GPIO_PIN_12
#define R_EN_GPIO_Port GPIOB
#define SCK_Pin GPIO_PIN_13
#define SCK_GPIO_Port GPIOB
#define MISO_Pin GPIO_PIN_14
#define MISO_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_15
#define MOSI_GPIO_Port GPIOB
#define IMU_CS_Pin GPIO_PIN_6
#define IMU_CS_GPIO_Port GPIOC
#define IMU_INT2_Pin GPIO_PIN_8
#define IMU_INT2_GPIO_Port GPIOC
#define IMU_INT1_Pin GPIO_PIN_11
#define IMU_INT1_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define TDI_Pin GPIO_PIN_15
#define TDI_GPIO_Port GPIOA
#define L_TX_Pin GPIO_PIN_10
#define L_TX_GPIO_Port GPIOC
#define LF_TX_Pin GPIO_PIN_11
#define LF_TX_GPIO_Port GPIOC
#define RF_TX_Pin GPIO_PIN_12
#define RF_TX_GPIO_Port GPIOC
#define R_TX_Pin GPIO_PIN_2
#define R_TX_GPIO_Port GPIOD
#define R_CHB_TDO_Pin GPIO_PIN_3
#define R_CHB_TDO_GPIO_Port GPIOB
#define NTRST_Pin GPIO_PIN_4
#define NTRST_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_5
#define SW2_GPIO_Port GPIOB
#define DEBUG_Pin GPIO_PIN_8
#define DEBUG_GPIO_Port GPIOB
#define SW1_Pin GPIO_PIN_9
#define SW1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
