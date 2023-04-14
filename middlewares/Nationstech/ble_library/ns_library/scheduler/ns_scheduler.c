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
 * @file ns_scheduler.c
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */

/** @addtogroup 
 * @{
 */

 /* Includes ------------------------------------------------------------------*/
#include "ns_scheduler.h"



#define CRITICAL_REGION_ENTER() 
#define CRITICAL_REGION_EXIT()


typedef struct
{
    app_sched_event_handler_t handler;          /**< Pointer to event handler to receive the event. */
    uint16_t                  event_data_size;  /**< Size of event data. */
} event_header_t;

static event_header_t * m_queue_event_headers;  /**< Array for holding the queue event headers. */
static uint8_t        * m_queue_event_data;     /**< Array for holding the queue event data. */
static volatile uint8_t m_queue_start_index;    /**< Index of queue entry at the start of the queue. */
static volatile uint8_t m_queue_end_index;      /**< Index of queue entry at the end of the queue. */
static uint16_t         m_queue_event_size;     /**< Maximum event size in queue. */
static uint16_t         m_queue_size;           /**< Number of queue entries. */



static __inline bool is_word_aligned(void const* p)
{
    return (((uintptr_t)p & 0x03) == 0);
}


/**@brief Function for incrementing a queue index, and handle wrap-around.
 *
 * @param[in]   index   Old index.
 *
 * @return      New (incremented) index.
 */
static __inline uint8_t next_index(uint8_t index)
{
    return (index < m_queue_size) ? (index + 1) : 0;
}


static __inline uint8_t app_sched_queue_full()
{
  uint8_t tmp = m_queue_start_index;
  return next_index(m_queue_end_index) == tmp;
}

/**@brief Macro for checking if a queue is full. */
#define APP_SCHED_QUEUE_FULL() app_sched_queue_full()


static __inline uint8_t app_sched_queue_empty()
{
  uint8_t tmp = m_queue_start_index;
  return m_queue_end_index == tmp;
}

/**@brief Macro for checking if a queue is empty. */
#define APP_SCHED_QUEUE_EMPTY() app_sched_queue_empty()


uint32_t app_sched_init(uint16_t event_size, uint16_t queue_size, void * p_event_buffer)
{
    uint16_t data_start_index = (queue_size + 1) * sizeof(event_header_t);

    // Check that buffer is correctly aligned
    if (!is_word_aligned(p_event_buffer))
    {
        return 1;
    }

    // Initialize event scheduler
    m_queue_event_headers = p_event_buffer;
    m_queue_event_data    = &((uint8_t *)p_event_buffer)[data_start_index];
    m_queue_end_index     = 0;
    m_queue_start_index   = 0;
    m_queue_event_size    = event_size;
    m_queue_size          = queue_size;

    return 0;
}



uint16_t app_sched_queue_space_get()
{
    uint16_t start = m_queue_start_index;
    uint16_t end   = m_queue_end_index;
    uint16_t free_space = m_queue_size - ((end >= start) ?
                           (end - start) : (m_queue_size + 1 - start + end));
    return free_space;
}


uint32_t app_sched_event_put(void const              * p_event_data,
                             uint16_t                  event_data_size,
                             app_sched_event_handler_t handler)
{
    uint32_t err_code;

    if (event_data_size <= m_queue_event_size)
    {
        uint16_t event_index = 0xFFFF;

        CRITICAL_REGION_ENTER();

        if (!APP_SCHED_QUEUE_FULL())
        {
            event_index       = m_queue_end_index;
            m_queue_end_index = next_index(m_queue_end_index);
        }

        CRITICAL_REGION_EXIT();

        if (event_index != 0xFFFF)
        {
            // NOTE: This can be done outside the critical region since the event consumer will
            //       always be called from the main loop, and will thus never interrupt this code.
            m_queue_event_headers[event_index].handler = handler;
            if ((p_event_data != NULL) && (event_data_size > 0))
            {
                memcpy(&m_queue_event_data[event_index * m_queue_event_size],
                       p_event_data,
                       event_data_size);
                m_queue_event_headers[event_index].event_data_size = event_data_size;
            }
            else
            {
                m_queue_event_headers[event_index].event_data_size = 0;
            }

            err_code = 0;
        }
        else
        {
            err_code = 1;
        }
    }
    else
    {
        err_code = 1;
    }

    return err_code;
}





void app_sched_execute(void)
{
    while (!APP_SCHED_QUEUE_EMPTY())
    {
        // Since this function is only called from the main loop, there is no
        // need for a critical region here, however a special care must be taken
        // regarding update of the queue start index (see the end of the loop).
        uint16_t event_index = m_queue_start_index;

        void * p_event_data;
        uint16_t event_data_size;
        app_sched_event_handler_t event_handler;

        p_event_data = &m_queue_event_data[event_index * m_queue_event_size];
        event_data_size = m_queue_event_headers[event_index].event_data_size;
        event_handler   = m_queue_event_headers[event_index].handler;

        event_handler(p_event_data, event_data_size);

        // Event processed, now it is safe to move the queue start index,
        // so the queue entry occupied by this event can be used to store
        // a next one.
        m_queue_start_index = next_index(m_queue_start_index);
    }
}





