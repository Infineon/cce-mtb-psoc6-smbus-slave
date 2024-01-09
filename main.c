/*******************************************************************************
* File Name:   main.c
*
* Description: This example project demonstrates the basic operation of the
* SMBus slave.
*
* Related Document: See README.md
*
*
********************************************************************************
* Copyright 2019-2023, Cypress Semiconductor Corporation (an Infineon company) or
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include "smbus_slave.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* I2C slave address to communicate with */
#define I2C_SLAVE_ADDR          (0x08UL)

/* I2C slave interrupt priority */
#define I2C_SLAVE_IRQ_PRIORITY  (7u)

/* Timer interrupt priority */
#define TIMER_IRQ_PRIORITY  (7u)

#define CMD_ONE_BYTE    0
#define CMD_TWO_BYTES   2
#define CMD_PROC_CALL   3
#define CMD_BLK_CMD     4
#define CMD_BLK_PROC    5

/*******************************************************************************
* Structure definition
*******************************************************************************/
/* SMBus registers structure */
typedef struct
{
    uint8_t  one_byte;
    uint16_t two_bytes;
    uint16_t process_call;
    uint8_t  block_array[8];
    uint8_t  block_proc[5];
} smbus_regs_t;

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* I2C IRQ Config structure */
const cy_stc_sysint_t I2C_SCB_IRQ_config = {
        .intrSrc = (IRQn_Type) I2C_IRQ,
        .intrPriority = I2C_SLAVE_IRQ_PRIORITY
};

/* Timer IRQ Config structure */
const cy_stc_sysint_t Timer_TCPWM_IRQ_config = {
        .intrSrc = (IRQn_Type) Timer_IRQ,
        .intrPriority = TIMER_IRQ_PRIORITY
};


smbus_slave_t smbus;
smbus_regs_t  smbus_regs = {0};

/* SMBus commands */
smbus_cmd_t smbus_cmd_list[] = {
    {"BYTE CMD", CMD_ONE_BYTE, SMBUS_CMD_TYPE_RW_BYTE, 1},
    {"WORD CMD", CMD_TWO_BYTES, SMBUS_CMD_TYPE_RW_WORD, 2},
    {"PROC CALL", CMD_PROC_CALL, SMBUS_CMD_TYPE_PROCESS_CALL, 2},
    {"BLOCK CMD", CMD_BLK_CMD, SMBUS_CMD_TYPE_RW_BLOCK, 9},
    {"BLOCK PROC", CMD_BLK_PROC, SMBUS_CMD_TYPE_BLOCK_PROCESS_CALL, 5},
    SMBUS_CMD_LIST_END
};

bool print_flag = false;
bool timer_flag = false;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void I2C_InterruptHandler(void);
void Timer_InterruptHandler(void);
bool SMBus_Callback(void *arg, const smbus_cmd_t *cmd, bool is_read, uint8_t *value);

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  uint32_t status - status indicates success or failure
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(uint32_t status)
{
    if (status != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }
}


/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function.
*   1. Initializes the board, retarget-io and led
*   2. Configures the I2C slave to receive packet from the master
*
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    en_smbus_status_t status;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    /* Board init failed. Stop program execution */
    handle_error(result);

    /* Initialize the retarget-io */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                  CY_RETARGET_IO_BAUDRATE);
    /* Retarget-io init failed. Stop program execution */
    handle_error(result);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("************** "
           "SMBus Slave test"
           "************** \r\n\n");

    /* Initialize user LED */
    printf(">> Configuring user LED..... \n\r");
    result = cyhal_gpio_init(P10_0, CYHAL_GPIO_DIR_OUTPUT,
                              CYHAL_GPIO_DRIVE_STRONG, CYBSP_LED_STATE_OFF);
    /* LED init failed. Stop program execution */
    handle_error(result);
    printf("Done\r\n");

    /* I2C Slave configuration settings */
    printf(">> Configuring SMBus Slave..... \n\r");

    Cy_TCPWM_Counter_Init(Timer_HW, Timer_NUM, &Timer_config);
    Cy_TCPWM_Counter_Enable(Timer_HW, Timer_NUM);

    /* Configure SMBus slave */
    status = SMBus_Slave_Init(&smbus, I2C_HW, I2C_SLAVE_ADDR);
    handle_error(status);
    status = SMBus_Slave_AddCommands(&smbus, smbus_cmd_list);
    handle_error(status);
    printf("Added %d commands\n\r", smbus.cmd_count);
    status = SMBus_Slave_RegisterCallback(&smbus, SMBus_Callback, NULL);
    handle_error(status);

    /* Init interrupts */
    Cy_SysInt_Init(&I2C_SCB_IRQ_config, &I2C_InterruptHandler);
    NVIC_EnableIRQ((IRQn_Type) I2C_SCB_IRQ_config.intrSrc);
    Cy_SysInt_Init(&Timer_TCPWM_IRQ_config, &Timer_InterruptHandler);
    NVIC_EnableIRQ((IRQn_Type) Timer_TCPWM_IRQ_config.intrSrc);

    /* Enable interrupts */
    __enable_irq();

    for (;;)
    {
        if (print_flag)
        {
            print_flag = false;

            printf("\n\rSMBus registers: \n\r");
            printf("One byte: %x\n\r", smbus_regs.one_byte);
            printf("Two bytes: %x\n\r", smbus_regs.two_bytes);
            printf("Proc call: %x\n\r", smbus_regs.process_call);
            printf("Block cmd: %x %x%x%x%x%x%x%x\n\r", smbus_regs.block_array[0], smbus_regs.block_array[1], smbus_regs.block_array[2], smbus_regs.block_array[3],
                                                       smbus_regs.block_array[4], smbus_regs.block_array[5], smbus_regs.block_array[6], smbus_regs.block_array[7]);
            printf("Block proc: %x %x%x%x%x\n\r", smbus_regs.block_proc[0], smbus_regs.block_proc[1], smbus_regs.block_proc[2], 
                                                  smbus_regs.block_proc[3], smbus_regs.block_proc[4]);
        }
        if (timer_flag)
        {
            timer_flag = false;

            printf("Timeout\n\r");
        }
    }
}

void I2C_InterruptHandler(void)
{
    /* ISR implementation for SMBus */
    SMBus_Slave_InterruptHandler(&smbus); 
}

void Timer_InterruptHandler(void)
{
    timer_flag = true;

    SMBus_Slave_Timeout(&smbus);

    Cy_TCPWM_ClearInterrupt(Timer_HW, Timer_NUM, CY_TCPWM_INT_ON_TC);
}


bool SMBus_Callback(void *arg, const smbus_cmd_t *cmd, bool is_read, uint8_t *value)
{
    if (is_read == 0)
    {
        switch (cmd->code)
        {
            case CMD_ONE_BYTE: 
                smbus_regs.one_byte = *value;
                break;
            case CMD_TWO_BYTES: 
                smbus_regs.two_bytes = value[0] + (((uint16_t) value[1]) << 8);
                break;
            case CMD_PROC_CALL:
                smbus_regs.process_call = value[0] + (((uint16_t) value[1]) << 8);
                break;
            case CMD_BLK_CMD:
                memcpy(&smbus_regs.block_array, value, cmd->size);
                break;
            case CMD_BLK_PROC:
                memcpy(&smbus_regs.block_proc, value, cmd->size);
                break;
            default:
                break;
        }
    }
    else
    {
        switch (cmd->code)
        {
            case CMD_ONE_BYTE: 
                *value = smbus_regs.one_byte;
                break;
            case CMD_TWO_BYTES: 
                value[0] = smbus_regs.two_bytes & 0xFF;
                value[1] = smbus_regs.two_bytes >> 8;
                break;
            case CMD_PROC_CALL:
                value[0] = ~(smbus_regs.process_call & 0xFF);
                value[1] = ~(smbus_regs.process_call >> 8);
                break;
            case CMD_BLK_CMD:
                value[0] = 8;
                memcpy(&value[1], &smbus_regs.block_array[1], 8);
                break;
            case CMD_BLK_PROC:
                value[0] = 4;
                value[1] = smbus_regs.block_proc[1] + 1;
                value[2] = smbus_regs.block_proc[2] + 1;
                value[3] = smbus_regs.block_proc[3] + 1;
                value[4] = smbus_regs.block_proc[4] + 1;
                break;
            default:
                break;
        }
    }

    print_flag = true;
    return true;
}