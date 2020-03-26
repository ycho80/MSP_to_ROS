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
#include "project.h"
#include "serial_ros.h"

volatile uint16_t  IR_reading, FSR_reading;
#define PWM1  500  // open gripper
#define PWM2  6000 // close gripper

// Controller declarations
volatile float reference[3]= {30, 45.0, 90.0};
volatile float reference_2[3] = {30.0, -45, 70.0};
volatile float int_error[3] = {0.0, 0.0, 0.0} ;   // Integrated error
volatile float int_error_2[3] = {0.0, 0.0, 0.0} ;   // Integrated error
volatile float prev_error[3] = {0.0, 0.0, 0.0};
volatile float prev_error_2[3] = {0.0, 0.0, 0.0} ;  // Previous error (for derivative calc)
volatile float PIDOutput[3] = {0.0, 0.0, 0.0} ;
volatile float PIDOutput_2[3] = {0.0, 0.0, 0.0};
volatile float error[3] = {0, 0, 0};
volatile float error_2[3] = {0, 0, 0};
volatile float error_dot[3] = {0, 0, 0} ;

volatile float kp1=15;
volatile float ki1=3;
volatile float kp2=27;
volatile float ki2=8;
volatile float kp3=7.5;
volatile float ki3=0;

volatile float kp1_2=10;
volatile float ki1_2=0;
volatile float kp2_2=12;
volatile float ki2_2=0;
volatile float kp3_2=15;
volatile float ki3_2=0;
void main(void)
 {
    WDT_A_holdTimer();              // Stop watchdog timer
    FPU_enableModule();
    Interrupt_disableMaster();

    //Set up LED toggle
    GPIO_setAsOutputPin(GPIO_PORT_P2,GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0|GPIO_PIN1|GPIO_PIN2);

    /* Initializing ADC (MCLK/1/4) and using the 10-bit */
       ADC14_enableModule();
       ADC14_setResolution(ADC_10BIT);
       ADC14_initModule(ADC_CLOCKSOURCE_SMCLK, ADC_PREDIVIDER_4, ADC_DIVIDER_4, 0);

       /* Configuring ADC Memory */
       ADC14_configureMultiSequenceMode(ADC_MEM1, ADC_MEM2, true);
       ADC14_configureConversionMemory(ADC_MEM1, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A1, false);
       ADC14_configureConversionMemory(ADC_MEM2, ADC_VREFPOS_AVCC_VREFNEG_VSS, ADC_INPUT_A3, false);

       /* Configuring GPIOs (5.5 A0) */
       GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION); // IR Seneor
       GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION); // FSR


       /* Enabling sample timer in manual iteration mode */
       ADC14_enableSampleTimer(ADC_MANUAL_ITERATION);                  // Had here issue due to being in manual iteration
       ADC14_enableConversion();                                       // Start conversion
       ADC14_toggleConversionTrigger();
       IR_reading = ADC14_getResult(ADC_MEM1);

       /* Set up Timer A Module 0 */
       P2SEL0 |= 0x10;                             // Set bit 4 of P2SEL0 to enable TA0.1 functionality on pin 2.4
       P2SEL1 &= ~0x10;                            // Clear bit 4 of P2SEL1 to enable TA0.1 functionality on pin 2.4
       P2DIR |= 0x10;                              // Set pin 2.4 as output pin
       TA0CCR0 = 7200;                             // Set Timer A Module 0 Period (PWM Signal Period
       TA0CCR1 = PWM1;                             // Set Duty Cycle to zero initially
       TA0CCTL1 = 0x00e0;                          // Set output mode to Reset/Set
       TA0CTL = TASSEL__SMCLK | MC__UP | TACLR;    // Tie Timer A Module 0 to SMCLK, use up mode, and clear TA0R

    GPIO_setAsOutputPin(GPIO_PORT_P6,GPIO_PIN6); //Setting output pin for P6.6
    GPIO_setAsOutputPin(GPIO_PORT_P6,GPIO_PIN7); //Setting output pin for P6.7
    GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN6); // INA CW
    GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN7); // INB CCW

    //PWM signal on P2.7
    P2SEL0 |= BIT7 ; // Set bit 7 of P2SEL0 to enable TA0.4 functionality on P2.7
    P2SEL1 &= ~BIT7 ; // Clear bit 7 of P2SEL1 to enable TA0.4 functionality on P2.7
    P2DIR |= BIT7 ; // Set pin 2.7 as an output pin

    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN2); //Setting output pin for P3.2
    GPIO_setAsOutputPin(GPIO_PORT_P3,GPIO_PIN3); //Setting output pin for P3.3
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2); // INA CW
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN3); // INB CCW

    //PWM signal on P2.5
    P2SEL0 |= BIT5;     // Set bit 6 of P2SEL0 to enable TA0.2 functionality on P2.5
    P2SEL1 &= ~BIT5;    // Clear bit 6 of P2SEL1 to enable TA0.2 functionality on P2.5
    P2DIR |= BIT5;      // Set pin 2.6 as an output pin

    GPIO_setAsOutputPin(GPIO_PORT_P4,GPIO_PIN5); //Setting output pin for P4.5
    GPIO_setAsOutputPin(GPIO_PORT_P4,GPIO_PIN6); //Setting output pin for P4.6
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5); // INA CW
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6); // INB CCW

    //PWM signal on P2.6
    P2SEL0 |= BIT6;     // Set bit 6 of P2SEL0 to enable TA0.3 functionality on P2.6
    P2SEL1 &= ~BIT6;    // Clear bit 6 of P2SEL1 to enable TA0.3 functionality on P2.6
    P2DIR |= BIT6;      // Set pin 2.6 as an output pin

    init_clocks();
    init_pins();
    init_spi();
    init_timers();

    Interrupt_enableSleepOnIsrExit();
    Interrupt_setPriority(INT_TA3_0, 0);        // UART message transmission has the highest priority

    Interrupt_enableMaster();

    // Start counters/ISRs (Done in P1 ISR)
    Timer_A_startCounter(TIMER_A3_BASE, TIMER_A_UP_MODE);

    while(1){
        PCM_gotoLPM0();
    }
}

// ISR to get joint angles, velocities, motor currents, and FSR forces then transmit them via UART
void TA3_0_IRQHandler(void){

    Timer_A_clearCaptureCompareInterrupt(TIMER_A3_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_0);
    read_spi();


        error[0] = reference[0] - qp[0];
        error[1] = reference[1] - qp[1];
        error[2] = reference[2] - qp[2];
        int_error[0] = int_error[0] + dt*absVal(error[0]);
        int_error[1] = int_error[1] + dt*absVal(error[1]);
        int_error[2] = int_error[2] + dt*absVal(error[2]);
   //     error_dot[0] = (error[0] - prev_error[0])/dt;

        PIDOutput[0] = (kp1*absVal(error[0]) + ki1*absVal(int_error[0]) ) ; // + kd1*absVal(error_dot[0]));
        PIDOutput[1] = (kp2*absVal(error[1]) + ki2*absVal(int_error[1]) ) ;
        PIDOutput[2] = (kp3*absVal(error[2]) + ki3*absVal(int_error[2]) ) ;

  //      prev_error[0] = error[0];

        error_2[0] = reference_2[0] - qp[0];
        error_2[1] = reference_2[1] - qp[1];
        error_2[2] = reference_2[2] - qp[2];
        int_error_2[0] = int_error_2[0] + dt*error_2[0];
        int_error_2[1] = int_error_2[1] + dt*error_2[1];
        int_error_2[2] = int_error_2[2] + dt*error_2[2];

        PIDOutput_2[0] = (kp1_2*absVal(error_2[0]) + ki1_2*absVal(int_error_2[0]) ) ;
        PIDOutput_2[1] = (kp2_2*absVal(error_2[1]) + ki2_2*absVal(int_error_2[1]) ) ;
        PIDOutput_2[2] = (kp3_2*absVal(error_2[2]) + ki3_2*absVal(int_error_2[2]) ) ;
        /*
        PIDOutput_2[0] = kp1_2*absVal(error_2[0]);
        PIDOutput_2[1] = kp2_2*absVal(error_2[1]);
        PIDOutput_2[2] = kp3_2*absVal(error_2[2]);
*/
        //First Motor
        if (error[0] > 0 & IR_reading < 700)
        {
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5); // INA CW
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6); // INB CCW
            // Set Timer A period (PWM signal period)
            TA0CCR0 = 7200;
            // Set Duty cycle
            TA0CCR3 = PIDOutput[0];
            // Set output mode to Reset/Set
            TA0CCTL3 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
            // Initialize Timer A
            TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R

        }
        else if(error[0] < 0 & IR_reading < 700)
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5); // INA CW
            GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6); // INB CCW
            // Set Timer A period (PWM signal period)
            TA0CCR0 = 7200;
            // Set Duty cycle
            TA0CCR3 = PIDOutput[0];
            // Set output mode to Reset/Set
            TA0CCTL3 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
            // Initialize Timer A
            TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R

        }
        //Second Motor
        if(error[1] > 0 & IR_reading < 700)
        {
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2); // INA CW
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN3); // INB CCW
            // Set Timer A period (PWM signal period)
            TA0CCR0 = 7200;
            // Set Duty cycle
            TA0CCR2 = PIDOutput[1] ;
            // Set output mode to Reset/Set
            TA0CCTL2 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
            // Initialize Timer A
            TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R


        }
        else if(error[1] < 0 & IR_reading < 700)
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2); // INA CW
            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN3); // INB CCW
            // Set Timer A period (PWM signal period)
            TA0CCR0 = 7200;
            // Set Duty cycle
            TA0CCR2 = PIDOutput[1] ;
            // Set output mode to Reset/Set
            TA0CCTL2 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
            // Initialize Timer A
            TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R

        }
        //Third Motor
        if(error[2] > 0 & IR_reading < 700)
        {
            GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN6); // INA CW
            GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN7); // INB CCW
            // Set Timer A period (PWM signal period)
            TA0CCR0 = 7200;
            // Set Duty cycle
            TA0CCR4 = PIDOutput[2] ;
            // Set output mode to Reset/Set
            TA0CCTL4 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
            // Initialize Timer A
            TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R

        }
        else if(error[2] < 0 & IR_reading < 700)
        {
            GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN6); // INA CW
            GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN7); // INB CCW
            // Set Timer A period (PWM signal period)
            TA0CCR0 = 7200;
            // Set Duty cycle
            TA0CCR4 = PIDOutput[2] ;
            // Set output mode to Reset/Set
            TA0CCTL4 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
            // Initialize Timer A
            TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R

        }
        if (IR_reading < 700) // OPEN
               {
                   GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); // green LED on
                   GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0); // red LED off
                   ADC14_toggleConversionTrigger(); // Start ADC Conversion

                   while (ADC14_isBusy()){} // pause while data is being converted

                   IR_reading = ADC14_getResult(ADC_MEM1); // IR sensor reading
                   FSR_reading = ADC14_getResult(ADC_MEM2); // FSR sensor reading

                   printf("IR Reading = %d\r\n FSR Reading = %d\r\n", IR_reading, FSR_reading ); // print values to make sure that ADC is working properly
                   TA0CCR1 = 0; // Set Duty Cycle using ADC14_getResult

               } // end if

         if (IR_reading >= 700) // CLOSE
               {
                   GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0); // red LED on
                   GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1); // green LED off
                   ADC14_toggleConversionTrigger(); // Start ADC Conversion

                   while (ADC14_isBusy()){} // pause while data is being converted

                   IR_reading = ADC14_getResult(ADC_MEM1); // IR sensor reading
                   FSR_reading = ADC14_getResult(ADC_MEM2); // FSR sensor reading

                   printf("IR Reading = %d\r\n FSR Reading = %d\r\n", IR_reading, FSR_reading ); // print values to make sure that ADC is working properly


                   if (FSR_reading > 1000)
                   {
                       ADC14_toggleConversionTrigger(); // Start ADC Conversion

                       while (ADC14_isBusy()){} // pause while data is being converted
                       FSR_reading = ADC14_getResult(ADC_MEM1); // get conversion results

                       GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2); // blue LED on
                       GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0); // red LED off

                   //    TA0CCR1 = 0;
                       //First Motor
                       if (error_2[0] > 0 )
                       {
                           GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN5); // INA CW
                           GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN6); // INB CCW
                           // Set Timer A period (PWM signal period)
                           TA0CCR0 = 7200;
                           // Set Duty cycle
                           TA0CCR3 = PIDOutput_2[0];
                           // Set output mode to Reset/Set
                           TA0CCTL3 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
                           // Initialize Timer A
                           TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R
                           if (absVal(error_2[1]) < 20)
                           {
                               TA0CCR1 = PWM1;
                               TA0CCR3 = 0;
                               TA0CCR2 = 0;
                               TA0CCR4 = 0;
                           }
                       }
                       else if(error_2[0] < 0 )
                       {
                           GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN5); // INA CW
                           GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN6); // INB CCW
                           // Set Timer A period (PWM signal period)
                           TA0CCR0 = 7200;
                           // Set Duty cycle
                           TA0CCR3 = PIDOutput_2[0];
                           // Set output mode to Reset/Set
                           TA0CCTL3 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
                           // Initialize Timer A
                           TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R
                           if (absVal(error_2[1]) < 20)
                           {
                               TA0CCR1 = PWM1;
                               TA0CCR2 = 0;
                               TA0CCR3 = 0;
                               TA0CCR4 = 0;



                           }
                       }
                       //Second Motor
                        if(error_2[1] > 0 )
                        {
                            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2); // INA CW
                            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN3); // INB CCW
                            // Set Timer A period (PWM signal period)
                            TA0CCR0 = 7200;
                            // Set Duty cycle
                            TA0CCR2 = PIDOutput_2[1] ;
                            // Set output mode to Reset/Set
                            TA0CCTL2 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
                            // Initialize Timer A
                            TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R


                        }
                        else if(error_2[1] < 0)
                        {
                            GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2); // INA CW
                            GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN3); // INB CCW
                            // Set Timer A period (PWM signal period)
                            TA0CCR0 = 7200;
                            // Set Duty cycle
                            TA0CCR2 = PIDOutput_2[1] ;
                            // Set output mode to Reset/Set
                            TA0CCTL2 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
                            // Initialize Timer A
                            TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R

                        }
                        //Third Motor
                        if(error_2[2] > 0)
                        {
                            GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN6); // INA CW
                            GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN7); // INB CCW
                            // Set Timer A period (PWM signal period)
                            TA0CCR0 = 7200;
                            // Set Duty cycle
                            TA0CCR4 = PIDOutput_2[2] ;
                            // Set output mode to Reset/Set
                            TA0CCTL4 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
                            // Initialize Timer A
                            TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R

                        }
                        else if(error_2[2] < 0)
                        {
                            GPIO_setOutputHighOnPin(GPIO_PORT_P6, GPIO_PIN6); // INA CW
                            GPIO_setOutputLowOnPin(GPIO_PORT_P6, GPIO_PIN7); // INB CCW
                            // Set Timer A period (PWM signal period)
                            TA0CCR0 = 7200;
                            // Set Duty cycle
                            TA0CCR4 = PIDOutput_2[2] ;
                            // Set output mode to Reset/Set
                            TA0CCTL4 = OUTMOD_7 ; // Macro which is equal to 0x00e0, defined in msp432p401r.h
                            // Initialize Timer A
                            TA0CTL = TASSEL__SMCLK | MC__UP | TACLR ; // Tie Timer A to SMCLK, use Up mode, and clear TA0R
                        }
                   } // end if
                   else if(IR_reading >= 700 & FSR_reading < 1000)
                   {

                       TA0CCR1 = PWM2; // Set Duty Cycle using ADC14_getResult
                   }
                   GPIO_setOutputLowOnPin(GPIO_PORT_P2,GPIO_PIN2); // blue LED off
               }


}

