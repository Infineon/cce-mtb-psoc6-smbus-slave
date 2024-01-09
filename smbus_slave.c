/*******************************************************************************
* File Name: amux.c
*
*  Description: This file contains the implementation of the Analog mux (amux)
*   designed to work with ADC or other analog components.
*
******************************************************************************
* (c) 2023, Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
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
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnify Cypress against all liability.
*****************************************************************************/

#include "smbus_slave.h"

/*******************************************************************************
* Constants
*******************************************************************************/
#define SMBUS_MAX_NUM_CMD           256

/*******************************************************************************
* Local Functions
*******************************************************************************/
static void SMBus_Slave_InternalCallback(uint32_t event);
static bool SMBus_Slave_CheckCommand(smbus_slave_t *smbus, uint8_t code, const smbus_cmd_t **cmd);

/*******************************************************************************
* Global Variables
*******************************************************************************/
static smbus_slave_t *smbus_current_irq;

/*******************************************************************************
* Function Name: SMBus_Slave_Init
********************************************************************************
* Summary:
*   Initialize the SMBus slave block.
*
* Parameters:
*   smbus: SMBus object
*   i2c: pointer to I2C base
*   i2c_address: I2C device address
*
* Return:
*   If initialized correctly, returns SUCCESS, otherwise ERROR.
*
*******************************************************************************/
en_smbus_status_t SMBus_Slave_Init(smbus_slave_t *smbus, CySCB_Type* i2c, 
                                                         uint8_t i2c_address)
{
    if (i2c == NULL || smbus == NULL)
    {
        return SMBUS_ERROR;
    }

    cy_en_scb_i2c_status_t i2c_status;
    const cy_stc_scb_i2c_config_t I2C_config = 
    {
        .i2cMode = CY_SCB_I2C_SLAVE,
        .useRxFifo = false,
        .useTxFifo = false,
        .slaveAddress = i2c_address,
        .slaveAddressMask = 254,
        .acceptAddrInFifo = true,
        .ackGeneralAddr = false,
        .enableWakeFromSleep = false,
        .enableDigitalFilter = false,
        .lowPhaseDutyCycle = 0,
        .highPhaseDutyCycle = 0,
    };

    smbus->i2c_base = i2c;
    smbus->port = NULL;
    smbus->cmd_count = 0;
    smbus->callback = NULL;
    smbus->read_buffer[0] = 0xAA;
    smbus->read_buffer[1] = 0xBB;
    smbus->read_buffer[2] = 0xCC;

    i2c_status = Cy_SCB_I2C_Init(smbus->i2c_base, &I2C_config, &smbus->i2c_context);
    if (i2c_status != CY_SCB_I2C_SUCCESS)
    {
        return SMBUS_ERROR;
    }

    /* Configure read buffer */
    Cy_SCB_I2C_SlaveConfigReadBuf(smbus->i2c_base, smbus->read_buffer,
                                  SMBUS_PACKET_SIZE, &smbus->i2c_context);

    /* Configure write buffer */
    Cy_SCB_I2C_SlaveConfigWriteBuf(smbus->i2c_base, smbus->write_buffer,
                                   SMBUS_PACKET_SIZE, &smbus->i2c_context);

    /* Register Callback function for interrupt */
    Cy_SCB_I2C_RegisterEventCallback(smbus->i2c_base,
                                    (cy_cb_scb_i2c_handle_events_t) SMBus_Slave_InternalCallback,
                                    &smbus->i2c_context);

    Cy_SCB_I2C_Enable(smbus->i2c_base);

    return SMBUS_SUCCESS;
}

/*******************************************************************************
* Function Name: SMBus_Slave_Deinit
********************************************************************************
* Summary:
*   De-initialize an SMBus object.
*
* Parameters:
*   smbus: SMBus object
*
*******************************************************************************/
void SMBus_Slave_Deinit(smbus_slave_t *smbus)
{
    if (smbus == NULL)
    {
        return;
    }

    Cy_SCB_I2C_DeInit(smbus->i2c_base);
}

/*******************************************************************************
* Function Name: SMBus_Slave_AddAlertPin
********************************************************************************
* Summary:

*
* Parameters:
*   smbus: SMBus object
*   port: port base to the alert pin
*   pin: pin number in the port
*
* Return:
*   If added correctly, returns SUCCESS, otherwise ERROR.
*
*******************************************************************************/
en_smbus_status_t SMBus_Slave_AddAlertPin(smbus_slave_t *smbus, 
                                          GPIO_PRT_Type *port, uint8_t pin)
{
    smbus->port = port;
    smbus->pin = pin;

    return SMBUS_SUCCESS;
}

/*******************************************************************************
* Function Name: SMBus_Slave_AddCommands
********************************************************************************
* Summary:
*   Add a list of commands and find out the number of commands.
*
* Parameters:
*   smbus: SMBus object
*   smbus_cmd_t: pointer to a list of commands
*
* Return:
*   If added correctly, returns SUCCESS, otherwise ERROR.
*
*******************************************************************************/
en_smbus_status_t SMBus_Slave_AddCommands(smbus_slave_t *smbus, 
                                          const smbus_cmd_t *list)
{
    uint16_t cmd_count = 0;

    if (list == NULL)
    {
        return SMBUS_ERROR;
    }
    smbus->cmd_list = list;

    while (cmd_count < SMBUS_MAX_NUM_CMD)
    {
        if (smbus->cmd_list[cmd_count].type == SMBUS_CMD_TYPE_NULL)
        {
            break;
        }
        cmd_count++;
    }
    smbus->cmd_count = cmd_count;

    return SMBUS_SUCCESS;
}

/*******************************************************************************
* Function Name: SMBus_Slave_Alert
********************************************************************************
* Summary:
*   Assert or de-assert the SMBus Alert pin.
*
* Paramters:
*   smbus: SMBus object
*   value: pin state.
*
* Return:
*   If set correctly, returns SUCCESS, otherwise ERROR.
*
*******************************************************************************/
en_smbus_status_t SMBus_Slave_Alert(smbus_slave_t *smbus, uint8_t value)
{
    if (smbus->port == NULL)
    {
        return SMBUS_ERROR;
    }

    Cy_GPIO_Write(smbus->port, smbus->pin, value);

    return SMBUS_SUCCESS;
}

/*******************************************************************************
* Function Name: SMBus_Slave_RegisterCallback
********************************************************************************
* Summary:
*   Register a callback to execute on every SMBus frame.
*
* Paramters:
*   smbus: SMBus object
*   callback: callback function
*   arg: argument to the callback
*
* Return:
*   If registered correctly, returns SUCCESS, otherwise ERROR.
*
*******************************************************************************/
en_smbus_status_t SMBus_Slave_RegisterCallback(smbus_slave_t *smbus, 
                                               smbus_slave_callback_t callback, void *arg)
{
    smbus->callback = callback;
    smbus->callback_arg = arg;
    return SMBUS_SUCCESS;
}

/*******************************************************************************
* Function Name: SMBUs_Slave_CompleteTransaction
********************************************************************************
* Summary:
*   Complete a pending transaction.
*
* Paramters:
*   smbus: SMBus object
*
* Return:
*   If completed correctly, returns SUCCESS, otherwise ERROR.
*
*******************************************************************************/
en_smbus_status_t SMBUs_Slave_CompleteTransaction(smbus_slave_t *smbus)
{
    return SMBUS_SUCCESS;
}

/*******************************************************************************
* Function Name: SMBus_Slave_InterruptHandler
********************************************************************************
* Summary:
*   SMBus interrupt handler.
*
* Paramters:
*   smbus: SMBus object
*
*******************************************************************************/
void SMBus_Slave_InterruptHandler(smbus_slave_t *smbus)
{
    smbus_current_irq = smbus;
    Cy_SCB_I2C_SlaveInterrupt(smbus->i2c_base, &smbus->i2c_context);
    return;
}

/*******************************************************************************
* Function Name: SMBus_Slave_Timeout
********************************************************************************
* Summary:
*   SMBus timeout interrupt handler.
*
* Paramters:
*   smbus: SMBus object
*
*******************************************************************************/
void SMBus_Slave_Timeout(smbus_slave_t *smbus)
{
    Cy_SCB_I2C_Disable(smbus->i2c_base, &smbus->i2c_context);

    Cy_SCB_I2C_Enable(smbus->i2c_base);
}

/*******************************************************************************
* Function Name: SMBus_Slave_InternalCallback
********************************************************************************
* Summary:
*   SMBus interrupt handler.
*
* Paramters:
*   smbus: SMBus object
*
*******************************************************************************/
void SMBus_Slave_InternalCallback(uint32_t event)
{
    /* Check write complete event */
    if (0UL == (CY_SCB_I2C_SLAVE_ERR_EVENT & event))
    {
        if (0UL != (CY_SCB_I2C_SLAVE_WR_CMPLT_EVENT & event))
        {
            const smbus_cmd_t *cmd;
            uint16_t num_bytes = smbus_current_irq->i2c_context.slaveRxBufferIdx;
            bool check_result = SMBus_Slave_CheckCommand(smbus_current_irq, 
                                                         smbus_current_irq->write_buffer[1],
                                                         &cmd);

            if ((smbus_current_irq->callback != NULL) && check_result)
            {
                /* Only execute the write callback if some data is provided */
                if (num_bytes > 2)
                {
                    smbus_current_irq->callback(smbus_current_irq->callback_arg, cmd, false, 
                                               &smbus_current_irq->write_buffer[2]);
                }
                smbus_current_irq->last_cmd = cmd;
            }
            else
            {
                smbus_current_irq->last_cmd = NULL;
            }

            /* Configure write buffer for the next write */
            Cy_SCB_I2C_SlaveConfigWriteBuf(smbus_current_irq->i2c_base, 
                                           smbus_current_irq->write_buffer, SMBUS_PACKET_SIZE, 
                                           &smbus_current_irq->i2c_context);
        }
    }

    /* Check read complete event */
    if (0UL != (CY_SCB_I2C_SLAVE_READ_EVENT & event))
    {
        if (smbus_current_irq->last_cmd != NULL)
        {
            if (smbus_current_irq->callback != NULL)
            {
                smbus_current_irq->callback(smbus_current_irq->callback_arg, smbus_current_irq->last_cmd, true, &smbus_current_irq->read_buffer[0]);
            }

            /* Configure read buffer for the next read */
            Cy_SCB_I2C_SlaveConfigReadBuf(smbus_current_irq->i2c_base, smbus_current_irq->read_buffer,
                                          smbus_current_irq->last_cmd->size, &smbus_current_irq->i2c_context);

        }
        else
        {
            /* Configure read buffer for the next read */
            Cy_SCB_I2C_SlaveConfigReadBuf(smbus_current_irq->i2c_base, smbus_current_irq->read_buffer,
                                        0, &smbus_current_irq->i2c_context);
        }


    }

    /* Check read complete event */
    if (0UL != (CY_SCB_I2C_SLAVE_RD_CMPLT_EVENT & event))
    {
        /* Configure read buffer for the next read */
        Cy_SCB_I2C_SlaveConfigReadBuf(smbus_current_irq->i2c_base, smbus_current_irq->read_buffer,
                                      SMBUS_PACKET_SIZE, &smbus_current_irq->i2c_context);

        /* Configure write buffer for the next write */
        Cy_SCB_I2C_SlaveConfigWriteBuf(smbus_current_irq->i2c_base, 
                                        smbus_current_irq->write_buffer, SMBUS_PACKET_SIZE, 
                                        &smbus_current_irq->i2c_context);
    }
}

/*******************************************************************************
* Function Name: SMBus_Slave_CheckCommand
********************************************************************************
* Summary:
*   Check if the command received is valid. If yes, returned the given command
*   info.
*
* Paramters:
*   smbus: SMBus object
*
* Return:
*   If found, returns SUCCESS, otherwise ERROR.
*
*******************************************************************************/
bool SMBus_Slave_CheckCommand(smbus_slave_t *smbus, uint8_t code, const smbus_cmd_t **cmd)
{
    uint16_t count;
    bool ret = false;
    const smbus_cmd_t *curr_cmd = smbus->cmd_list;

    for (count = 0; count < smbus->cmd_count; count++)
    {
        if (curr_cmd->code == code)
        {
            *cmd = curr_cmd;
            ret = true;
            break;
        }
        curr_cmd++;
    }

    return ret;
}
/* [] END OF FILE */