/*****************************************************************************
 * Copyright (c) 2019, Nations Technologies Inc.
 *
 * All rights reserved.
 * ****************************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Nations' name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY NATIONS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL NATIONS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ****************************************************************************/

/**
 * @file main.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */
#ifndef __MAIN_H__
#define __MAIN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "n32wb03x.h"



#define SPI_MASTER              SPI1
#define SPI_MASTER_CLK          RCC_APB2_PERIPH_SPI1
#define SPI_MASTER_GPIO         GPIOA
#define SPI_MASTER_GPIO_AF        GPIO_AF1_SPI1
#define SPI_MASTER_GPIO_CLK     RCC_APB2_PERIPH_GPIOA
#define SPI_MASTER_PIN_SCK      GPIO_PIN_1
#define SPI_MASTER_PIN_MISO     GPIO_PIN_3
#define SPI_MASTER_PIN_MOSI     GPIO_PIN_2
#define SPI_MASTER_IRQn         SPI1_IRQn

#define SPI_SLAVE                  SPI2
#define SPI_SLAVE_CLK              RCC_APB2_PERIPH_SPI2
#define SPI_SLAVE_GPIO             GPIOB
#define SPI_SLAVE_GPIO_AF        GPIO_AF2_SPI2
#define SPI_SLAVE_GPIO_CLK         RCC_APB2_PERIPH_GPIOB
#define SPI_SLAVE_PIN_SCK          GPIO_PIN_1
#define SPI_SLAVE_PIN_MISO         GPIO_PIN_3
#define SPI_SLAVE_PIN_MOSI         GPIO_PIN_2
#define SPI_SLAVE_IRQn             SPI2_IRQn

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
