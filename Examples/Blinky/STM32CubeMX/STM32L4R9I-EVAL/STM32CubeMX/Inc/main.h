/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32l4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
extern int stdio_init   (void);
extern int app_main     (void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define FMC_NBL0_Pin GPIO_PIN_0
#define FMC_NBL0_GPIO_Port GPIOE
#define TRST_Pin GPIO_PIN_4
#define TRST_GPIO_Port GPIOB
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define TDI_Pin GPIO_PIN_15
#define TDI_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SPI2_NSS_Pin GPIO_PIN_0
#define SPI2_NSS_GPIO_Port GPIOI
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOH
#define FMC_NBL1_Pin GPIO_PIN_1
#define FMC_NBL1_GPIO_Port GPIOE
#define SAI1_SDB_Pin GPIO_PIN_5
#define SAI1_SDB_GPIO_Port GPIOB
#define MFX_WAKEUP_Pin GPIO_PIN_9
#define MFX_WAKEUP_GPIO_Port GPIOG
#define D2_Pin GPIO_PIN_0
#define D2_GPIO_Port GPIOD
#define SPI2_MISO_Pin GPIO_PIN_2
#define SPI2_MISO_GPIO_Port GPIOI
#define SPI2_SCK_Pin GPIO_PIN_1
#define SPI2_SCK_GPIO_Port GPIOI
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOH
#define OCTOSPI2_IO7_Pin GPIO_PIN_12
#define OCTOSPI2_IO7_GPIO_Port GPIOH
#define SAI1_MCLKA_Pin GPIO_PIN_8
#define SAI1_MCLKA_GPIO_Port GPIOB
#define SHIELD_Pin GPIO_PIN_6
#define SHIELD_GPIO_Port GPIOB
#define OCTOSPI2_DQS_Pin GPIO_PIN_15
#define OCTOSPI2_DQS_GPIO_Port GPIOG
#define FMC_NOE_Pin GPIO_PIN_4
#define FMC_NOE_GPIO_Port GPIOD
#define D3_Pin GPIO_PIN_1
#define D3_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOH
#define SPI2_MOSI_Pin GPIO_PIN_3
#define SPI2_MOSI_GPIO_Port GPIOI
#define OCTOSPI2_IO4_Pin GPIO_PIN_9
#define OCTOSPI2_IO4_GPIO_Port GPIOH
#define A20_Pin GPIO_PIN_4
#define A20_GPIO_Port GPIOE
#define A19_Pin GPIO_PIN_3
#define A19_GPIO_Port GPIOE
#define A23_Pin GPIO_PIN_2
#define A23_GPIO_Port GPIOE
#define SAI1_FSA_Pin GPIO_PIN_9
#define SAI1_FSA_GPIO_Port GPIOB
#define SHIELD_CS_Pin GPIO_PIN_7
#define SHIELD_CS_GPIO_Port GPIOB
#define FMC_NE3_Pin GPIO_PIN_10
#define FMC_NE3_GPIO_Port GPIOG
#define FMC_NWE_Pin GPIO_PIN_5
#define FMC_NWE_GPIO_Port GPIOD
#define SDIO1_CMD_Pin GPIO_PIN_2
#define SDIO1_CMD_GPIO_Port GPIOD
#define SDIO1_D2_Pin GPIO_PIN_10
#define SDIO1_D2_GPIO_Port GPIOC
#define Audio_INT_Pin GPIO_PIN_4
#define Audio_INT_GPIO_Port GPIOI
#define USBOTG_ID_Pin GPIO_PIN_10
#define USBOTG_ID_GPIO_Port GPIOA
#define USBOTG_DP_Pin GPIO_PIN_12
#define USBOTG_DP_GPIO_Port GPIOA
#define WAKEUP_Pin GPIO_PIN_13
#define WAKEUP_GPIO_Port GPIOC
#define A22_Pin GPIO_PIN_6
#define A22_GPIO_Port GPIOE
#define A21_Pin GPIO_PIN_5
#define A21_GPIO_Port GPIOE
#define FMC_NWAIT_Pin GPIO_PIN_6
#define FMC_NWAIT_GPIO_Port GPIOD
#define LCD_CLK_Pin GPIO_PIN_3
#define LCD_CLK_GPIO_Port GPIOD
#define SDIO1_D3_Pin GPIO_PIN_11
#define SDIO1_D3_GPIO_Port GPIOC
#define OCTOSPI2_NCS_Pin GPIO_PIN_5
#define OCTOSPI2_NCS_GPIO_Port GPIOI
#define SAI1_SCKA_Pin GPIO_PIN_8
#define SAI1_SCKA_GPIO_Port GPIOA
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define USBOTG_DM_Pin GPIO_PIN_11
#define USBOTG_DM_GPIO_Port GPIOA
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOF
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOF
#define A0_Pin GPIO_PIN_0
#define A0_GPIO_Port GPIOF
#define SPI_CS_Pin GPIO_PIN_12
#define SPI_CS_GPIO_Port GPIOG
#define FMC_NE1_Pin GPIO_PIN_7
#define FMC_NE1_GPIO_Port GPIOD
#define SDIO1_CLK_Pin GPIO_PIN_12
#define SDIO1_CLK_GPIO_Port GPIOC
#define SDIO1_D0_Pin GPIO_PIN_8
#define SDIO1_D0_GPIO_Port GPIOC
#define LPUART__RX_Pin GPIO_PIN_8
#define LPUART__RX_GPIO_Port GPIOG
#define TKEY_Pin GPIO_PIN_6
#define TKEY_GPIO_Port GPIOC
#define A3_Pin GPIO_PIN_3
#define A3_GPIO_Port GPIOF
#define A4_Pin GPIO_PIN_4
#define A4_GPIO_Port GPIOF
#define A5_Pin GPIO_PIN_5
#define A5_GPIO_Port GPIOF
#define PMOD_INT_Pin GPIO_PIN_13
#define PMOD_INT_GPIO_Port GPIOG
#define A14_Pin GPIO_PIN_4
#define A14_GPIO_Port GPIOG
#define A13_Pin GPIO_PIN_3
#define A13_GPIO_Port GPIOG
#define A15_Pin GPIO_PIN_5
#define A15_GPIO_Port GPIOG
#define LPUART1_TX_Pin GPIO_PIN_7
#define LPUART1_TX_GPIO_Port GPIOG
#define TKEY_CS_Pin GPIO_PIN_7
#define TKEY_CS_GPIO_Port GPIOC
#define SWIRE_Pin GPIO_PIN_6
#define SWIRE_GPIO_Port GPIOG
#define SDIO1_D1_Pin GPIO_PIN_9
#define SDIO1_D1_GPIO_Port GPIOC
#define DFSDM_CLK_Pin GPIO_PIN_10
#define DFSDM_CLK_GPIO_Port GPIOF
#define A11_Pin GPIO_PIN_1
#define A11_GPIO_Port GPIOG
#define D7_Pin GPIO_PIN_10
#define D7_GPIO_Port GPIOE
#define UART3_RX_Pin GPIO_PIN_11
#define UART3_RX_GPIO_Port GPIOB
#define A18_Pin GPIO_PIN_13
#define A18_GPIO_Port GPIOD
#define A12_Pin GPIO_PIN_2
#define A12_GPIO_Port GPIOG
#define D1_Pin GPIO_PIN_15
#define D1_GPIO_Port GPIOD
#define D0_Pin GPIO_PIN_14
#define D0_GPIO_Port GPIOD
#define DFSDM_DATA4_Pin GPIO_PIN_0
#define DFSDM_DATA4_GPIO_Port GPIOC
#define SAI1_SDA_Pin GPIO_PIN_1
#define SAI1_SDA_GPIO_Port GPIOC
#define LCD_INT_Pin GPIO_PIN_2
#define LCD_INT_GPIO_Port GPIOC
#define A10_Pin GPIO_PIN_0
#define A10_GPIO_Port GPIOG
#define D6_Pin GPIO_PIN_9
#define D6_GPIO_Port GPIOE
#define D12_Pin GPIO_PIN_15
#define D12_GPIO_Port GPIOE
#define A17_Pin GPIO_PIN_12
#define A17_GPIO_Port GPIOD
#define A16_Pin GPIO_PIN_11
#define A16_GPIO_Port GPIOD
#define D15_Pin GPIO_PIN_10
#define D15_GPIO_Port GPIOD
#define MFX_IRQ_OUT_Pin GPIO_PIN_0
#define MFX_IRQ_OUT_GPIO_Port GPIOA
#define A9_Pin GPIO_PIN_15
#define A9_GPIO_Port GPIOF
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOE
#define D11_Pin GPIO_PIN_14
#define D11_GPIO_Port GPIOE
#define I2C2_SCL_Pin GPIO_PIN_4
#define I2C2_SCL_GPIO_Port GPIOH
#define D14_Pin GPIO_PIN_9
#define D14_GPIO_Port GPIOD
#define D13_Pin GPIO_PIN_8
#define D13_GPIO_Port GPIOD
#define LCD_BL_CTRL_Pin GPIO_PIN_5
#define LCD_BL_CTRL_GPIO_Port GPIOA
#define OCTOSPI1_IO3_Pin GPIO_PIN_6
#define OCTOSPI1_IO3_GPIO_Port GPIOA
#define OCTOSPI1_IO0_Pin GPIO_PIN_1
#define OCTOSPI1_IO0_GPIO_Port GPIOB
#define A8_Pin GPIO_PIN_14
#define A8_GPIO_Port GPIOF
#define D4_Pin GPIO_PIN_7
#define D4_GPIO_Port GPIOE
#define D10_Pin GPIO_PIN_13
#define D10_GPIO_Port GPIOE
#define I2C2_SDA_Pin GPIO_PIN_5
#define I2C2_SDA_GPIO_Port GPIOH
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOB
#define OpAmp1_INM_Pin GPIO_PIN_1
#define OpAmp1_INM_GPIO_Port GPIOA
#define OCTOSPI1_CLK_Pin GPIO_PIN_3
#define OCTOSPI1_CLK_GPIO_Port GPIOA
#define OCTOSPI1_IO2_Pin GPIO_PIN_7
#define OCTOSPI1_IO2_GPIO_Port GPIOA
#define DSI_TE_Pin GPIO_PIN_11
#define DSI_TE_GPIO_Port GPIOF
#define A7_Pin GPIO_PIN_13
#define A7_GPIO_Port GPIOF
#define D9_Pin GPIO_PIN_12
#define D9_GPIO_Port GPIOE
#define OCTOSPI2_IO5_Pin GPIO_PIN_10
#define OCTOSPI2_IO5_GPIO_Port GPIOH
#define OCTOSPI2_IO6_Pin GPIO_PIN_11
#define OCTOSPI2_IO6_GPIO_Port GPIOH
#define PMOD_RST_Pin GPIO_PIN_14
#define PMOD_RST_GPIO_Port GPIOB
#define OCTOSPI1_NCS_Pin GPIO_PIN_2
#define OCTOSPI1_NCS_GPIO_Port GPIOA
#define ADC_DAC_Pin GPIO_PIN_4
#define ADC_DAC_GPIO_Port GPIOA
#define OCTOSPI1_IO1_Pin GPIO_PIN_0
#define OCTOSPI1_IO1_GPIO_Port GPIOB
#define OCTOSPI1_DQS_Pin GPIO_PIN_2
#define OCTOSPI1_DQS_GPIO_Port GPIOB
#define A6_Pin GPIO_PIN_12
#define A6_GPIO_Port GPIOF
#define D8_Pin GPIO_PIN_11
#define D8_GPIO_Port GPIOE
#define UART3_TX_Pin GPIO_PIN_10
#define UART3_TX_GPIO_Port GPIOB
#define LPUART1_RTS_Pin GPIO_PIN_12
#define LPUART1_RTS_GPIO_Port GPIOB
#define LPUART1_CTS_Pin GPIO_PIN_13
#define LPUART1_CTS_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
