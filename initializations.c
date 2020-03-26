#include "msp.h"
#include "driverlib.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "globals.h"
#include "initializations.h"

void init_timers(void){
    // Initialize timer that controls UART transmission and capture conversion, ADC, and SPI updates
    Timer_A_configureUpMode(TIMER_A3_BASE, &upConfig_3);
    Interrupt_enableInterrupt(INT_TA3_0);
    Timer_A_configureUpMode(TIMER_A1_BASE, &upConfig_0);
    Interrupt_enableInterrupt(INT_TA1_0);
}

void init_pins(void){
    // Configure P6.0 to trigger an interrupt that starts clocks when set high (from end of homing signal of the other MSP)
    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P6, GPIO_PIN0);
    GPIO_clearInterruptFlag(GPIO_PORT_P6, GPIO_PIN0);
    GPIO_enableInterrupt(GPIO_PORT_P6, GPIO_PIN0);
    Interrupt_enableInterrupt(INT_PORT6);
}

void init_clocks(void){
    // Configure clocks
    CS_setDCOFrequency(24.0E+6); // DCO = 24 MHz
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_8); // SMCLK = 3 MHz
    CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_4); // HSMCLK = 6 MHz

    // Configure Pin 4.4 to output HSMCLK to be connected to quadrature counters
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN4, GPIO_PRIMARY_MODULE_FUNCTION);
}

void init_spi(void){
    // Configure SPI Pins
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN5, GPIO_PRIMARY_MODULE_FUNCTION); //3.5    // 10.1 = EUSCI_B SCK
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN6, GPIO_PRIMARY_MODULE_FUNCTION); //3.6    // 10.2 = EUSCI_B MOSI/SIMO
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION); //3.7

    int ii;
    for(ii = 0; ii < 3; ii++){
        GPIO_setAsOutputPin(ss[ii][0], ss[ii][1]);       // SSi
        GPIO_setOutputHighOnPin(ss[ii][0], ss[ii][1]);   // Initialize SSi as high
    }

    // Configure SPI
    SPI_initMaster(EUSCI_B2_BASE, &spiMasterConfig);    // Initialize SPI
    SPI_enableModule(EUSCI_B2_BASE);                    // Enable SPI

    UCB2CTLW0 |= UCSYNC;
    UCB2CTLW0 |= UCMST;
    UCB2CTLW0 |= UCMM;
    UCB2CTLW0 |= UCMST;
    UCB2CTLW0 |= BIT6;
    UCB2CTLW0 |= BIT7;
    UCB2CTLW0 |= BIT1;
    UCB2CTLW0 &= ~ BIT0;

}
