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
 * @file ancc_task.h
 * @author Nations Firmware Team
 * @version v1.0.0
 *
 * @copyright Copyright (c) 2019, Nations Technologies Inc. All rights reserved.
 */


#ifndef ANCC_TASK_H_
#define ANCC_TASK_H_

/**
 ****************************************************************************************
 * @addtogroup ANCCTASK Apple Notification Center Service Client Task
 * @ingroup ANCC
 * @brief Apple Notification Center Service Client Task
 *
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#if (BLE_ANC_CLIENT)

#include "ke_task.h"
#include "gattc_task.h"
#include "co_error.h"
#include "ancc.h"
#include "anc_common.h"
#include "prf_types.h"

#define ANCC_GET_ATT_CMD_MAX_LEN        (19)
#define ANCC_PERF_NTF_ACT_CMD_MAX_LEN   (6)

/// Possible states of the ANCC task
enum ancc_state
{
    /// Free state
    ANCC_FREE,
    /// IDLE state
    ANCC_IDLE,
    /// Discovering Apple Notification Service
    ANCC_DISCOVERING,
    /// Busy state
    ANCC_BUSY,

    /// Number of defined states
    ANCC_STATE_MAX
};

/// Message IDs
enum
{
    /// Enable the ANCS Client task - at connection
    ANCC_ENABLE_REQ = TASK_FIRST_MSG(TASK_ID_ANCC),
    /// Disable Indication
    ANCC_ENABLE_RSP,
    /// Write Notification Configuration Request
    ANCC_WR_CFG_NTF_REQ,
    /// Write Notification Configuration Response
    ANCC_WR_CFG_NTF_RSP,
    /// Read Notification Configuration Request
    ANCC_RD_CFG_NTF_REQ,
    /// Read Notification Configuration Response
    ANCC_RD_CFG_NTF_RSP,
    /// Indicate that a notification of the Notification Source characteristic has been received
    ANCC_NTF_SRC_IND,
    /// Write a "Get Notification Attributes" command in Control Point
    ANCC_GET_NTF_ATTS_CMD_REQ,
    /// Indicate that the value of a notification attribute is available
    ANCC_NTF_ATT_IND,
    /// Write a "Get App Attributes" command in Control Point
    ANCC_GET_APP_ATTS_CMD_REQ,
    /// Indicate that the value of an application attribute is available
    ANCC_APP_ATT_IND,
    /// Write a "Perform Notification Action" command in Control Point
    ANCC_PERF_NTF_ACT_CMD_REQ,
    /// Procedure complete information
    ANCC_CMP_EVT,
};

enum
{
    ANCC_INVALID_OP_CODE = 0x00,

    /// Write notification configuration for Notification Source characteristic
    ANCC_WR_NTF_NTF_SRC_OP_CODE,
    /// Write notification configuration for Data Source characteristic
    ANCC_WR_NTF_DATA_SRC_OP_CODE,
    /// Read notification configuration for Notification Source characteristic
    ANCC_RD_NTF_NTF_SRC_OP_CODE,
    /// Read notification configuration for Data Source characteristic
    ANCC_RD_NTF_DATA_SRC_OP_CODE,

    // Get notification attributes command
    ANCC_GET_NTF_ATTS_CMD_OP_CODE,
    // Get application attributes command
    ANCC_GET_APP_ATTS_CMD_OP_CODE,
    // Perform notification action command
    ANCC_PERF_NTF_ACT_CMD_OP_CODE,
};

/// Data Source notification reception states
enum
{
    /**
     * DATA SOURCE NOTIFICATION STATES
     */
    /// Waiting for next notification
    ANCC_DATA_SRC_NTF_IDLE,
    /// Waiting for next APP_ID
    ANCC_DATA_SRC_APP_ID_IDLE,
    /// Waiting for App Identifier
    ANCC_DATA_SRC_NTF_APP_ID,
    /// Waiting for next attribute
    ANCC_DATA_SRC_NTF_NEXT_ATT,
    /// Waiting for attribute msb
    ANCC_DATA_SRC_NTF_ATT_MSB,
    /// Waiting for attribute length
    ANCC_DATA_SRC_NTF_ATT_LENGTH,
    /// Waiting for attribute value
    ANCC_DATA_SRC_NTF_ATT_VALUE,
    /// Enter this state when in idle state and reveive an invalid cmd or uid
    ANCC_DATA_SRC_NTF_CMD_UID_ERROR,
    /// Enter this state when in idle state and reveive an invalid cmd
    ANCC_DATA_SRC_APP_CMD_ERROR,
};

/*
 * TYPE DEFINITIONS
 ****************************************************************************************
 */

/// Parameters of the @ref ANCC_ENABLE_REQ message
struct ancc_enable_req
{
    /// Connection type
    uint8_t con_type;
    /// ANC handle values and characteristic properties
    struct ancc_content anc;
};

/// Parameters of the @ref ANCC_ENABLE_RSP message
struct ancc_enable_rsp
{
    /// Status
    uint8_t status;
    /// ANC handle values and characteristic properties
    struct ancc_content anc;
};

/// Parameters of the @ref ANCC_WR_CFG_NTF_REQ message
struct ancc_wr_cfg_ntf_req
{
    /// Operation code
    uint8_t op_code;
    /// Notification value
    uint16_t cfg_val;
};

/// Parameters of the @ref ANCC_WR_CFG_NTF_RSP message
struct ancc_wr_cfg_ntf_rsp
{
    /// Operation code
    uint8_t op_code;
    /// Operation status
    uint8_t status;
};

/// Parameters of the @ref ANCC_RD_CFG_NTF_REQ message
struct ancc_rd_cfg_ntf_req
{
    /// Operation code
    uint8_t op_code;
};

/// Parameters of the @ref ANCC_RD_CFG_NTF_RSP message
struct ancc_rd_cfg_ntf_rsp
{
    /// Operation code
    uint8_t op_code;
    /// Operation status
    uint8_t status;
    /// Notification value
    uint16_t cfg_val;
};

/// Parameters of the @ref ANCC_NTF_SRC_IND message
struct ancc_ntf_src_ind
{
    /// Value
    struct anc_ntf_src value;
};

/// Parameters of the @ref ANCC_GET_NTF_ATTS_CMD_REQ message
struct ancc_get_ntf_atts_cmd_req
{
    /// Notification UID
    uint32_t ntf_uid;
    /// Attribute bit mask
    uint8_t atts;
    /// Title attribute maximum length
    uint16_t title_len;
    /// Subtitle attribute maximum length
    uint16_t subtitle_len;
    /// Message attribute maximum length
    uint16_t message_len;
};

/// Parameters of the @ref ANCC_NTF_ATT_IND message
struct ancc_ntf_att_ind
{
    /// Attribute Code
    uint32_t ntf_uid;
    /// Attribute ID
    uint8_t att_id;
    /// Attribute length
    uint16_t length;
    /// Value
    uint8_t value[__ARRAY_EMPTY];
};

/// Parameters of the @ref ANCC_GET_APP_ATTS_CMD_REQ message
struct ancc_get_app_atts_cmd_req
{
    /// Requested attributes bit mask
    uint8_t atts;
    /// App Identifier length (not including NULL termination)
    uint8_t app_id_length;
    /// App Identifier
    uint8_t app_id[__ARRAY_EMPTY];
};

/// Parameters of the @ref ANCC_APP_ATT_IND message
struct ancc_app_att_ind
{
    /// app id
    uint8_t app_id_length;
    /// app AttributeID
    uint8_t att_id;
    /// Attribute length
    uint16_t att_length;
    /// Value
    uint8_t values[__ARRAY_EMPTY];
};

/// Parameters of the @ref ANCC_PERF_NTF_ACT_CMD_REQ message
struct ancc_perf_ntf_act_cmd_req
{
    /// Notification UID
    uint32_t ntf_uid;
    /// Requested Action
    uint8_t action;
};

/// Parameters of the @ref ANCC_CMP_EVT message
struct ancc_cmp_evt
{
    /// Operation code
    uint8_t operation;
    /// Status
    uint8_t status;
};

/*
 * GLOBAL VARIABLES DECLARATIONS
 ****************************************************************************************
 */
extern const struct ke_state_handler ancc_default_handler;

#endif // (BLE_ANC_CLIENT)

/// @} ANCCTASK

#endif // ANCC_TASK_H_
