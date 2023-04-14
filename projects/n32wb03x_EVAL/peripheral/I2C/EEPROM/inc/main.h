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

typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} Status;

#define DMA_CHANNEL_USED DMA1_CH1
#define DMA_IRQ_HANDLER  DMA1_Channel1_IRQHandler
#define DMA_IT_FLAG_TC   DMA1_INT_TXC1
#define DMA_IT_FLAG_GL   DMA1_INT_GLB1
#define DMA_IRQN         DMA1_Channel1_IRQn

#define LOG_USARTx      USART2
#define LOG_PERIPH      RCC_APB1_PERIPH_USART2
#define LOG_GPIO        GPIOB
#define LOG_PERIPH_GPIO RCC_APB2_PERIPH_GPIOB
#define LOG_TX_PIN      GPIO_PIN_4
#define LOG_RX_PIN      GPIO_PIN_5
#define LOG_GPIO_AF     GPIO_AF3_USART2            //GPIO_AF4_USART1

#define log_info(...)     printf(__VA_ARGS__)
#define log_error(...)    printf(__VA_ARGS__)
void log_init(void);
void log_buff(uint8_t *data, int len);

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
