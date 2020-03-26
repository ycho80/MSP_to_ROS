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
#include "peripheralFunctions.h"
#include "utilityFunctions.h"

// Damaged pins: 5.5, 5.0, 5.1, 3.5, 3.7, 2.4, 2.5 (old MSP)
// Damaged pins: 5.5?? 6.0, 6.1, 4.6 (new MSP)

void main(void)
{
    WDT_A_holdTimer();              // Stop watchdog timer
    MAP_FPU_enableModule();
    MAP_Interrupt_disableMaster();

    init_clocks();
    init_uart();
    init_spi();
    init_adc();
    init_capture();
    init_timers();

    MAP_Interrupt_enableSleepOnIsrExit();
    MAP_Interrupt_setPriority(INT_TA3_0, 0);
    MAP_Interrupt_setPriority(INT_TA0_N, 1);
    MAP_Interrupt_setPriority(INT_TA1_N, 2);
    MAP_Interrupt_enableMaster();

    // Start Counters/ISRs
    MAP_Timer_A_startCounter(TIMER_A0_BASE, TIMER_A_UP_MODE);
    MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);
    MAP_Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);

    while(1){
        MAP_PCM_gotoLPM0();
    }
}

void TA0_N_IRQHandler(void)
{
    //printf("\nTA0_N ISR\n");

    if(TIMER_A0->CCTL[1] & 1 == 1){
        MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
        rise_count[0] = MAP_Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
    }
    if(TIMER_A0->CCTL[2] & 1 == 1){
        MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
        fall_count[0] = MAP_Timer_A_getCaptureCompareCount(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
    }

    pulse_length[0] = fall_count[0] - rise_count[0];
    if(pulse_length[0] < 0){
        pulse_length[0] += 4061;
    }
}

void TA1_N_IRQHandler(void)
{
    //printf("\nTA1 ISR\n");
    if(TIMER_A1->CCTL[1] & 1 == 1){
        MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
        rise_count[1] = MAP_Timer_A_getCaptureCompareCount(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1);
    }
    if(TIMER_A1->CCTL[2] & 1 == 1){
        MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
        fall_count[1] = MAP_Timer_A_getCaptureCompareCount(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2);
    }

    if(TIMER_A1->CCTL[3] & 1 == 1){
        MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3);
        rise_count[2] = MAP_Timer_A_getCaptureCompareCount(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3);
    }
    if(TIMER_A1->CCTL[4] & 1 == 1){
        MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4);
        fall_count[2] = MAP_Timer_A_getCaptureCompareCount(TIMER_A1_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4);
    }

    pulse_length[1] = fall_count[1] - rise_count[1];
    if(pulse_length[1] < 0){
        pulse_length[1] += 4061;
    }

    pulse_length[2] = fall_count[2] - rise_count[2];
    if(pulse_length[2] < 0){
        pulse_length[2] += 4061;
    }
}

// ISR to get joint angles, currents, and forces then transmit them via UART
void TA3_0_IRQHandler(void){
    //printf("\nTA3 ISR\n");
    MAP_Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    MAP_ADC14_toggleConversionTrigger();

    //delay_ms(100);        // Delay to allow time for ADC conversions

    read_spi();
    read_capture();
    read_adc();

    int ii;
    for(ii = 0; ii < 6; ii++){
        write_uart(q_s[ii]);
    }
    for(ii = 0; ii < 6; ii++){
        write_uart(qd_s[ii]);
    }
    for(ii = 0; ii < 3; ii++){
        write_uart(i_s[ii]);
    }
    for(ii = 0; ii < 4; ii++){
        write_uart(F_s[ii]);
    }

    MAP_UART_transmitData(EUSCI_A0_BASE, carriage_return);
    MAP_UART_transmitData(EUSCI_A0_BASE, new_line);
}
