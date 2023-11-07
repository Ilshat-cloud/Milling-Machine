/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EXTI_A_Pin GPIO_PIN_4
#define EXTI_A_GPIO_Port GPIOE
#define Feed_TR_H2_m_Pin GPIO_PIN_5
#define Feed_TR_H2_m_GPIO_Port GPIOE
#define Feed_TR_L2_m_Pin GPIO_PIN_6
#define Feed_TR_L2_m_GPIO_Port GPIOE
#define DO_305_Feed_y_Pin GPIO_PIN_13
#define DO_305_Feed_y_GPIO_Port GPIOC
#define S2rocking_Pin GPIO_PIN_0
#define S2rocking_GPIO_Port GPIOF
#define S28Z_endswitch1_Pin GPIO_PIN_1
#define S28Z_endswitch1_GPIO_Port GPIOF
#define EXTI_B_Pin GPIO_PIN_2
#define EXTI_B_GPIO_Port GPIOF
#define EXTI_C_Pin GPIO_PIN_3
#define EXTI_C_GPIO_Port GPIOF
#define S25Z_endswitch2_Pin GPIO_PIN_4
#define S25Z_endswitch2_GPIO_Port GPIOF
#define S27Y_endswitch1_Pin GPIO_PIN_5
#define S27Y_endswitch1_GPIO_Port GPIOF
#define DI_82_S13_m_Pin GPIO_PIN_6
#define DI_82_S13_m_GPIO_Port GPIOF
#define triac_A_m_Pin GPIO_PIN_7
#define triac_A_m_GPIO_Port GPIOF
#define triac_B_m_Pin GPIO_PIN_8
#define triac_B_m_GPIO_Port GPIOF
#define triac_C_m_Pin GPIO_PIN_9
#define triac_C_m_GPIO_Port GPIOF
#define DI_34_S8_m_Pin GPIO_PIN_10
#define DI_34_S8_m_GPIO_Port GPIOF
#define Q7_Pin GPIO_PIN_0
#define Q7_GPIO_Port GPIOC
#define Q8_Pin GPIO_PIN_1
#define Q8_GPIO_Port GPIOC
#define SPI_MISO_Pin GPIO_PIN_2
#define SPI_MISO_GPIO_Port GPIOC
#define SPI_MOSI_Pin GPIO_PIN_3
#define SPI_MOSI_GPIO_Port GPIOC
#define AI_L14_374_m_Pin GPIO_PIN_0
#define AI_L14_374_m_GPIO_Port GPIOA
#define AI_manual_364_m_Pin GPIO_PIN_1
#define AI_manual_364_m_GPIO_Port GPIOA
#define Reserve_AI_m_Pin GPIO_PIN_2
#define Reserve_AI_m_GPIO_Port GPIOA
#define I1_mesure_m_Pin GPIO_PIN_3
#define I1_mesure_m_GPIO_Port GPIOA
#define DI_277_S31_m_Pin GPIO_PIN_4
#define DI_277_S31_m_GPIO_Port GPIOA
#define tacho_frequency_Pin GPIO_PIN_5
#define tacho_frequency_GPIO_Port GPIOA
#define DI_276_S30_m_Pin GPIO_PIN_6
#define DI_276_S30_m_GPIO_Port GPIOA
#define DI_275_S29_m_Pin GPIO_PIN_7
#define DI_275_S29_m_GPIO_Port GPIOA
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOC
#define INT_Pin GPIO_PIN_5
#define INT_GPIO_Port GPIOC
#define DI_34_S7_m_Pin GPIO_PIN_11
#define DI_34_S7_m_GPIO_Port GPIOF
#define S6_m1_on_m_Pin GPIO_PIN_12
#define S6_m1_on_m_GPIO_Port GPIOF
#define ReserveDI2_m_Pin GPIO_PIN_13
#define ReserveDI2_m_GPIO_Port GPIOF
#define DI_26_S4_m_Pin GPIO_PIN_14
#define DI_26_S4_m_GPIO_Port GPIOF
#define DI_87_S15_m_Pin GPIO_PIN_15
#define DI_87_S15_m_GPIO_Port GPIOF
#define DO_304_Feed_x_Pin GPIO_PIN_0
#define DO_304_Feed_x_GPIO_Port GPIOG
#define Feed_TR_H1_m_Pin GPIO_PIN_9
#define Feed_TR_H1_m_GPIO_Port GPIOE
#define Feed_TR_L1_m_Pin GPIO_PIN_11
#define Feed_TR_L1_m_GPIO_Port GPIOE
#define tacho_dir_right_Pin GPIO_PIN_13
#define tacho_dir_right_GPIO_Port GPIOE
#define tacho_dir_left_Pin GPIO_PIN_14
#define tacho_dir_left_GPIO_Port GPIOE
#define DI_63_S9_m_Pin GPIO_PIN_15
#define DI_63_S9_m_GPIO_Port GPIOE
#define SPI_CLK_Pin GPIO_PIN_10
#define SPI_CLK_GPIO_Port GPIOB
#define Q2_Pin GPIO_PIN_11
#define Q2_GPIO_Port GPIOB
#define Q3_Pin GPIO_PIN_12
#define Q3_GPIO_Port GPIOB
#define Q4_Pin GPIO_PIN_13
#define Q4_GPIO_Port GPIOB
#define Q5_Pin GPIO_PIN_14
#define Q5_GPIO_Port GPIOB
#define Q6_Pin GPIO_PIN_15
#define Q6_GPIO_Port GPIOB
#define DI_252_m_Pin GPIO_PIN_8
#define DI_252_m_GPIO_Port GPIOD
#define DI_246_m_Pin GPIO_PIN_9
#define DI_246_m_GPIO_Port GPIOD
#define DI_253_m_Pin GPIO_PIN_10
#define DI_253_m_GPIO_Port GPIOD
#define DI_29_S5_m_Pin GPIO_PIN_11
#define DI_29_S5_m_GPIO_Port GPIOD
#define DI_92_S20_m_Pin GPIO_PIN_12
#define DI_92_S20_m_GPIO_Port GPIOD
#define DI_91_S19_m_Pin GPIO_PIN_13
#define DI_91_S19_m_GPIO_Port GPIOD
#define DI_90_S18_m_Pin GPIO_PIN_14
#define DI_90_S18_m_GPIO_Port GPIOD
#define DI_88_S16_m_Pin GPIO_PIN_15
#define DI_88_S16_m_GPIO_Port GPIOD
#define RE_DE_Pin GPIO_PIN_8
#define RE_DE_GPIO_Port GPIOG
#define Feed_TR_H3_m_Pin GPIO_PIN_6
#define Feed_TR_H3_m_GPIO_Port GPIOC
#define Feed_TR_L3_m_Pin GPIO_PIN_7
#define Feed_TR_L3_m_GPIO_Port GPIOC
#define RST_Pin GPIO_PIN_8
#define RST_GPIO_Port GPIOC
#define DO_308_Clamp_x_Pin GPIO_PIN_9
#define DO_308_Clamp_x_GPIO_Port GPIOC
#define DI_260_S22_m_Pin GPIO_PIN_8
#define DI_260_S22_m_GPIO_Port GPIOA
#define DI_203_S35_m_Pin GPIO_PIN_9
#define DI_203_S35_m_GPIO_Port GPIOA
#define DI_280_S34_m_Pin GPIO_PIN_10
#define DI_280_S34_m_GPIO_Port GPIOA
#define DI_279_S33_m_Pin GPIO_PIN_11
#define DI_279_S33_m_GPIO_Port GPIOA
#define DI_278_S32_m_Pin GPIO_PIN_12
#define DI_278_S32_m_GPIO_Port GPIOA
#define TX_Pin GPIO_PIN_10
#define TX_GPIO_Port GPIOC
#define RX_Pin GPIO_PIN_11
#define RX_GPIO_Port GPIOC
#define DO_306_Feed_z_Pin GPIO_PIN_12
#define DO_306_Feed_z_GPIO_Port GPIOC
#define DI_89_S17_m_Pin GPIO_PIN_0
#define DI_89_S17_m_GPIO_Port GPIOD
#define ESD_m_Pin GPIO_PIN_1
#define ESD_m_GPIO_Port GPIOD
#define F1_DI_m_Pin GPIO_PIN_2
#define F1_DI_m_GPIO_Port GPIOD
#define F2_DI_m_Pin GPIO_PIN_3
#define F2_DI_m_GPIO_Port GPIOD
#define F3_DI_m_Pin GPIO_PIN_4
#define F3_DI_m_GPIO_Port GPIOD
#define S6Gearbox_Pin GPIO_PIN_5
#define S6Gearbox_GPIO_Port GPIOD
#define S11Gripe_Pin GPIO_PIN_6
#define S11Gripe_GPIO_Port GPIOD
#define S12ungrip_Pin GPIO_PIN_7
#define S12ungrip_GPIO_Port GPIOD
#define DO_310_Clamp_y_Pin GPIO_PIN_9
#define DO_310_Clamp_y_GPIO_Port GPIOG
#define DO_312_Clamp_z_Pin GPIO_PIN_10
#define DO_312_Clamp_z_GPIO_Port GPIOG
#define led_oil_Pin GPIO_PIN_11
#define led_oil_GPIO_Port GPIOG
#define led_Temper_Pin GPIO_PIN_12
#define led_Temper_GPIO_Port GPIOG
#define DO_63_K7_Pin GPIO_PIN_13
#define DO_63_K7_GPIO_Port GPIOG
#define DO_51_K6_Pin GPIO_PIN_14
#define DO_51_K6_GPIO_Port GPIOG
#define res_do24v1_Pin GPIO_PIN_15
#define res_do24v1_GPIO_Port GPIOG
#define res_do24v2_Pin GPIO_PIN_3
#define res_do24v2_GPIO_Port GPIOB
#define led_reserve1_m_Pin GPIO_PIN_4
#define led_reserve1_m_GPIO_Port GPIOB
#define led_X_lock_m_Pin GPIO_PIN_5
#define led_X_lock_m_GPIO_Port GPIOB
#define led_Z_lock_m_Pin GPIO_PIN_6
#define led_Z_lock_m_GPIO_Port GPIOB
#define led_Y_lock_m_Pin GPIO_PIN_7
#define led_Y_lock_m_GPIO_Port GPIOB
#define led_reserve3_m_Pin GPIO_PIN_8
#define led_reserve3_m_GPIO_Port GPIOB
#define led_2t_m_Pin GPIO_PIN_9
#define led_2t_m_GPIO_Port GPIOB
#define led_manual_Pin GPIO_PIN_0
#define led_manual_GPIO_Port GPIOE
#define led_error_m_Pin GPIO_PIN_1
#define led_error_m_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */
#define M1_start_PWM 300  //0-1000
#define Roking_time 300 //0.005s*time
#define Brake_time 400  //0.005s*time
#define Acceleration_time 400  //0.005s*time
#define P1      100 //proportional
#define I1      5  //integral     
#define D1      3  //defferential
#define dead_zone1      3 // deadband
#define direct_pid 0 // direct or reverce pid
#define Hi_Current_error 900 // limit for hight current 0-1000
#define Hi_Current_lim 800 // limit for hight current 0-1000
#define Rotation_lim 50 // limit for current for direction check 0-1000
#define Feed_Fast_SP 800
#define  Comissioning 1  //0 when done
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
