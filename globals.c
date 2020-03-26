#include "msp.h"
#include "driverlib.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "globals.h"

extern float dt = 0.002;                                        // TIMERA3 sends messages at a rate of 500 Hz
extern float q[3] = {0.0, 0.0, 0.0};          // Joint Angular Positions with initial values
//extern char q_s[1][8] = {};                                     // Buffer to hold strings of angular positions
//extern float qd[1] = {0.0};            // Joint Angular Velocities
//extern char qd_s[1][8] = {};                                    // Buffer to hold strings of angular velocities
extern float qp[3] = {0.0, 0.0, 0.0};            // Previous time step angular positions

extern float q_precision = 0.01;                                // Decimal places of conversion from float to string for joint angles/velocities
extern float V_precision = 0.0001;                              // Decimal places of conversion from float to string for currents and forces


// Configure UART for 115200 baud at SMCLK = 3 MHz
extern const eUSCI_UART_Config uartConfig =
{
    EUSCI_A_UART_CLOCKSOURCE_SMCLK,                     // selectClockSource
    1,                                                  // clockPrescalar
    10,                                                 // firstModReg
    0,                                                  // secondModReg
    EUSCI_A_UART_NO_PARITY,                             // no parity
    EUSCI_A_UART_LSB_FIRST,                             // LSB first
    EUSCI_A_UART_ONE_STOP_BIT,                          // One stop bit
    EUSCI_A_UART_MODE,                                  // UART mode
    EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION       // Oversampling
};


extern const eUSCI_SPI_MasterConfig spiMasterConfig =
{
        EUSCI_B_SPI_CLOCKSOURCE_SMCLK,             // SMCLK Clock Source
        3000000,                                   // SMCLK = DCO/3 = 4 MHz
        3000000,                                    // SPICLK = 3MHz
        EUSCI_B_SPI_MSB_FIRST,                     // MSB First
        EUSCI_B_SPI_PHASE_DATA_CAPTURED_ONFIRST_CHANGED_ON_NEXT,    // Phase
        EUSCI_B_SPI_CLOCKPOLARITY_INACTIVITY_LOW, // High polarity
        EUSCI_B_SPI_3PIN,        // 3Wire SPI Mode - this means we will set/clear the CS line manually via GPIO

};

// Timer to control UART messages and peripheral function execution
extern const Timer_A_UpModeConfig upConfig_3 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK frequency = 3 MHz
        TIMER_A_CLOCKSOURCE_DIVIDER_10,         // 3 MHz / 10 = 300 kHz
        600,                                  // 3750 * 1 / 75 kHz = 0.05 s (20 Hz) period  // 600 / 300 KHz = 0.002 s (500 Hz)
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

extern const Timer_A_UpModeConfig upConfig_0 =
{
        TIMER_A_CLOCKSOURCE_SMCLK,              // SMCLK frequency = 3 MHz
        TIMER_A_CLOCKSOURCE_DIVIDER_3,         // 3 MHz / 3 = 1 MHz
        30000,                                  // 30000 * 1 / 1 MHz = 0.03 s (33.3 Hz) period
        TIMER_A_TAIE_INTERRUPT_DISABLE,         // Disable timer interrupt
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE ,    // Enable CCR0 interrupt
        TIMER_A_DO_CLEAR                        // Clear value
};

// Array of slave select lines connected to quadrature counters
extern int ss[3][2] = {
    {GPIO_PORT_P1, GPIO_PIN5},
    {GPIO_PORT_P1, GPIO_PIN6},
    {GPIO_PORT_P1, GPIO_PIN7}
};
