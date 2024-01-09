/*****************************************************************************
* File Name  : smbus_slave.h
*
* Description: This file contains definitions of constants, structures and 
*              functions for SMBus slave implementation.
*
*******************************************************************************
* Copyright 2023, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/
#ifndef SMBUS_SLAVE_H_
#define SMBUS_SLAVE_H_

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "cy_pdl.h"

/*******************************************************************************
*                              Enumerated Types
*******************************************************************************/
typedef enum
{
    /** Return success */
    SMBUS_SUCCESS = 0u,

    /** Return error */
    SMBUS_ERROR = 1u,

} en_smbus_status_t;

typedef enum
{
    SMBUS_CMD_TYPE_NULL                 = 0u,
    SMBUS_CMD_TYPE_SEND_BYTE            = 1u,
    SMBUS_CMD_TYPE_RW_BYTE              = 2u,
    SMBUS_CMD_TYPE_RW_WORD              = 3u,
    SMBUS_CMD_TYPE_RW_BLOCK             = 4u,
    SMBUS_CMD_TYPE_PROCESS_CALL         = 5u,
    SMBUS_CMD_TYPE_BLOCK_PROCESS_CALL   = 6u

} en_smbus_cmd_type_t;

/*******************************************************************************
*                                 API Constants
*******************************************************************************/
/* SMBus packet size */
#define SMBUS_PACKET_SIZE             (256UL)

/* End of command */
#define SMBUS_CMD_LIST_END            {NULL, 0, 0, 0}

/*******************************************************************************
*                              Type Definitions
*******************************************************************************/
/** SMBus command object structure */
typedef struct
{
    const char* name;
    uint8_t code;
    en_smbus_cmd_type_t type;
    uint8_t size;
} smbus_cmd_t;

/** SMBus callback function */
typedef bool (* smbus_slave_callback_t)(void *arg, const smbus_cmd_t *cmd, bool is_read, uint8_t *value);

/** SMBus slave structure object */
typedef struct
{
    CySCB_Type* i2c_base;
    cy_stc_scb_i2c_context_t i2c_context;
    const smbus_cmd_t *cmd_list;
    uint16_t cmd_count;
    const smbus_cmd_t *last_cmd;
    smbus_slave_callback_t callback;
    void *callback_arg;
    GPIO_PRT_Type *port;
    uint8_t pin;
    uint8_t write_buffer[SMBUS_PACKET_SIZE];
    uint8_t read_buffer[SMBUS_PACKET_SIZE]; 
} smbus_slave_t;

/*******************************************************************************
*                            Function Prototypes
*******************************************************************************/
en_smbus_status_t SMBus_Slave_Init(smbus_slave_t *smbus, CySCB_Type* i2c, uint8_t i2c_address);
en_smbus_status_t SMBus_Slave_AddAlertPin(smbus_slave_t *smbus, GPIO_PRT_Type *port, uint8_t pin);
en_smbus_status_t SMBus_Slave_AddCommands(smbus_slave_t *smbus, const smbus_cmd_t *list);
en_smbus_status_t SMBus_Slave_Alert(smbus_slave_t *smbus, uint8_t value);
en_smbus_status_t SMBus_Slave_RegisterCallback(smbus_slave_t *smbus, smbus_slave_callback_t callback, void *arg);
en_smbus_status_t SMBUs_Slave_CompleteTransaction(smbus_slave_t *smbus);
void SMBus_Slave_InterruptHandler(smbus_slave_t *smbus);
void SMBus_Slave_Timeout(smbus_slave_t *smbus);
void SMBus_Slave_Deinit(smbus_slave_t *smbus);


#endif /* AMUX_H_ */