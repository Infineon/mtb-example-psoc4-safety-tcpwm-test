/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Class-B Safety test code
*              example for TCPWM block, for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
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

#include "cy_pdl.h"
#include "cybsp.h"
#include "SelfTest.h"


/*******************************************************************************
* Macros
********************************************************************************/
/* Available commands */
#define SELFTEST_CMD_TIMER ('1')
#define SELFTEST_CMD_PWM ('2')
#define SELFTEST_CMD_PWM_GATE_KILL ('3')

/*******************************************************************************
* Global Variables
********************************************************************************/
cy_en_tcpwm_status_t api_status;

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void timer_test(void);
void pwm_test(void);
void pwm_gate_kill(void);

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function. It performs Class-B safety test for TCPMW block.
* SelfTest is performed for Timer/Counter, PWM and PWM gate Kill based on the
* user command.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_stc_scb_uart_context_t CYBSP_UART_context;

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\x1b[2J\x1b[;H");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\nClass-B Safety Test: TCPWM\r\n");

    /* Display available commands */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "Available commands \r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "1 : Run SelfTest for Timer/ Counter\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "2 : Run SelfTest for PWM\r\n");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "3 : Run SelfTest for PWM Gate Kill\r\n\n");

    Cy_SCB_UART_ClearRxFifo(CYBSP_UART_HW);

    for (;;)
    {
        result = Cy_SCB_UART_Get(CYBSP_UART_HW);
        if (result != CY_SCB_UART_RX_NO_DATA)
        {
            if (SELFTEST_CMD_TIMER == result)
            {
                Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n[Command] : Run SelfTest for Timer/ Counter\r\n");
                timer_test();

            }
            else if (SELFTEST_CMD_PWM == result)
            {
                Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n[Command] : Run SelfTest for PWM\r\n");
                pwm_test();
            }
            else if (SELFTEST_CMD_PWM_GATE_KILL == result)
            {
                Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\n[Command] : Run SelfTest for PWM Gate Kill\r\n");
                pwm_gate_kill();

            }
            else
            {
                Cy_SCB_UART_PutString(CYBSP_UART_HW, "\r\nEnter a valid command\r\n");
            }
        }

    }
}

/*******************************************************************************
* Function Name: timer_test
********************************************************************************
* Summary:
* This function configures the block to timer/counter personality and also
* the input clock to the CPU clock. The test verifies, if the counter is
* incrementing.
*
* Parameters:
*  none
*
* Return :
*  void
*
*******************************************************************************/
void timer_test(void)
{
    SelfTest_Timer_Counter_init(CYBSP_TIMER_COUNTER_TEST_HW, CYBSP_TIMER_COUNTER_TEST_NUM,
            &CYBSP_TIMER_COUNTER_TEST_config,CYBSP_TIMER_COUNTER_TEST_IRQ);

    /* Run Timer/Counter Self Test... */
    if (OK_STATUS != SelfTest_Counter_Timer())
    {
        /* Process error */
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Error: Timer Counter\r\n");
    }
    else
    {
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Success: Timer Counter\r\n");
    }
}

/*******************************************************************************
* Function Name: pwm_test
********************************************************************************
* Summary:
* The function configures a 32-bit PWM to run at 1/3 duty ON, 2/3 OFF duty cycle
* with a 1 millisecond period, and then start the PWM. The CPU is then run in a
* loop for 5 milliseconds and the output is polled continuously in the loop.
* The instances of `0` (low) and `1` (high) are counted. The on/off ratio is
* then calculated and checked if it falls within the expected thresholds.
*
* Parameters:
*  none
*
* Return:
*  void
*******************************************************************************/
void pwm_test(void)
{
    if (SelfTest_PWM_init(CYBSP_PWM_HW, CYBSP_PWM_NUM, &CYBSP_PWM_config,
            (IRQn_Type)CYBSP_PWM_IRQ) != OK_STATUS)
    {
        __disable_irq();
    }

    /* Run PWM Self Test... */
    if (OK_STATUS != SelfTest_PWM(PWM_IN_PIN_PORT, PWM_IN_PIN_NUM))
    {
        /* Process error */
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Error: PWM FAIL\r\n");
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Ensure PWM_IN_PIN is connected "
                "to PWM line - Refer Hardware setup section in Readme\r\n");
    }
    else
    {
        /* Process success */
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Success: PWM\r\n");
    }
}

/*******************************************************************************
* Function Name: pwm_gate_kill
********************************************************************************
* Summary:
* The function configures Kill mode of the TCPWM block as `Stop on Kill`. For
* PSOC™4 MCUs, the low-power comparator output is routed to the Kill signal of
* TCPWM indicating overvoltage/overcurrent condition if the voltage on the
* positive terminal is greater than the voltage on the negative terminal.
* The TCPWM base and CntNum are passed to check whether the counter is stopped
* or not. If the counter is not incrementing/decrementing, the PWM output is
* inactive that the PWM is killed.
*
* Parameters:
*  none
*
* Return:
*  void
*******************************************************************************/
void pwm_gate_kill(void)
{
    cy_stc_lpcomp_context_t lpcomp_context;

    /* Configure the TCPWM for PWM operation. */
     api_status = Cy_TCPWM_PWM_Init(CYBSP_PWM_GATEKILL_HW,
             CYBSP_PWM_GATEKILL_NUM, &CYBSP_PWM_GATEKILL_config);
    if( api_status != CY_TCPWM_SUCCESS)
    {
        __disable_irq();
    }

    Cy_TCPWM_PWM_Enable(CYBSP_PWM_GATEKILL_HW, CYBSP_PWM_GATEKILL_NUM);

    /*******************************/
    /* Run PWM GATEKill program... */
    /*******************************/
    Cy_TCPWM_TriggerReloadOrIndex(CYBSP_PWM_GATEKILL_HW, (1UL << CYBSP_PWM_GATEKILL_NUM));

    /*Initialize the LPCOMP with device configurator generated structure*/
    Cy_LPComp_Init(LPCOMP_HW, LPCOMP_CHANNEL, &LPCOMP_config, &lpcomp_context);


    /* Apply higher voltage to the LPCOMP +ve pin, will kill the PWM */
    Cy_LPComp_Enable(LPCOMP_HW, LPCOMP_CHANNEL, &lpcomp_context);
    while (0 == Cy_LPComp_GetCompare(LPCOMP_HW, LPCOMP_CHANNEL))
    {}

    if (OK_STATUS != SelfTest_PWM_GateKill(CYBSP_PWM_GATEKILL_HW, CYBSP_PWM_GATEKILL_NUM))
    {

          /* Process error */
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Error: PWM GateKill FAIL\r\n");
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Ensure LPCOMP +ve pin is connected to VCC -"
         " Refer Hardware setup section in Readme\r\n");

    }
    else
    {
          /* Process success */
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Success: PWM GateKill\r\n");
    }

}
/* [] END OF FILE */
