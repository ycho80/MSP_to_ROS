#include "msp.h"
#include "driverlib.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "globals.h"
#include "peripheralFunctions.h"
#include "utilityFunctions.h"


// Gets motor angle and velocity from the quadrature counter
void read_spi(void){
    //printf("\nSPI\n");

    const float enc_res[3] = {0.055104, 0.080357, 0.080357}; // Encoder resolution = 360deg / CPR (0.0551048 for 100:1, 0.080357 for 70:1)
    //const float enc_res[3] = {0.91837, 0.91837, 0.91837}; // Encoder resolution = 360deg / CPR
    static int initializations[3] = {0, 0, 0};           // Keeps track of whether each counter has been initialized
    int n = 3;               // Number of quadrature counters
    int ii = 0;              // Index of current quadrature counter
    uint8_t byte_1, byte_2;  // Temporary variables to hold the bytes sent by the counter
    uint16_t count = 0;

    // Addresses and commands for the quadrature counter (16-bit CNTR and counts all rising and falling edges of channels A and B)
    volatile uint8_t tx_dummy = 0x00;                       // this clears the flag when you read it.
    const uint8_t MDR0_address = 0b10001000;                // MDR0 Address
    const uint8_t MDR0_config = 0b00000011;                 // MDR0 Configuration
    const uint8_t MDR1_address = 0b10010000;                // MDR1 Address
    const uint8_t MDR1_config = 0b000000010;// 0b00000011;  // data to write to MDR1
    const uint8_t clear_CNTR = 0b00100000; //0b10010000;    // clears the counter
    const uint8_t read_CNTR = 0b01100000;                   // mask to read the counter


    // Configure/Initialize quadrature counters
    for(ii = 0; ii < n; ii++){
        if(initializations[ii] == 0){
            initializations[ii] = 1;
            GPIO_setOutputLowOnPin(ss[ii][0], ss[ii][1]);

            // MDR0 Configuration
            while (!(SPI_getInterruptStatus(EUSCI_B2_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));
            SPI_transmitData(EUSCI_B2_BASE, MDR0_address);         // Transmit MDR0 address
            delay_ms(10);

            while (!(SPI_getInterruptStatus(EUSCI_B2_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));
            SPI_transmitData(EUSCI_B2_BASE, MDR0_config);          // Transmit MDR0 configuration
            delay_ms(10);

            GPIO_setOutputHighOnPin(ss[ii][0], ss[ii][1]);

            // MDR1 Configuration
            GPIO_setOutputLowOnPin(ss[ii][0], ss[ii][1]);

            while (!(SPI_getInterruptStatus(EUSCI_B2_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));
            SPI_transmitData(EUSCI_B2_BASE, MDR1_address);         // Transmit MDR1 address
            delay_ms(10);

            while (!(SPI_getInterruptStatus(EUSCI_B2_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));
            SPI_transmitData(EUSCI_B2_BASE, MDR1_config);          // Transmit MDR1 configuration
            delay_ms(10);

            GPIO_setOutputHighOnPin(ss[ii][0], ss[ii][1]);

            // Clear CNTR
            GPIO_setOutputLowOnPin(ss[ii][0], ss[ii][1]);

            while (!(SPI_getInterruptStatus(EUSCI_B2_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));
            SPI_transmitData(EUSCI_B2_BASE, clear_CNTR);           // Clears the counter
            delay_ms(10);

            GPIO_setOutputHighOnPin(ss[ii][0], ss[ii][1]);
        }

        // Read CNTR
        GPIO_setOutputLowOnPin(ss[ii][0], ss[ii][1]);

        //Make sure transmit buffer is clear
        while (!(SPI_getInterruptStatus(EUSCI_B2_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));
        // Mask to read the counter
        SPI_transmitData(EUSCI_B2_BASE, read_CNTR);
        delay_ms(10);

        //This will push slave to send data to master
        while (!(SPI_getInterruptStatus(EUSCI_B2_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));
        // Clears the counter
        SPI_transmitData(EUSCI_B2_BASE, tx_dummy);
        delay_ms(10);

        //Make sure receive interrupt flag is set
        while (!(SPI_getInterruptStatus(EUSCI_B2_BASE,EUSCI_B_SPI_RECEIVE_INTERRUPT)));
        //Receiving data
        byte_1 = SPI_receiveData(EUSCI_B2_BASE);
        delay_ms(10);

        //This will push slave to send data to master
        while (!(SPI_getInterruptStatus(EUSCI_B2_BASE,EUSCI_B_SPI_TRANSMIT_INTERRUPT)));
        // Clears the counte
        SPI_transmitData(EUSCI_B2_BASE, tx_dummy);
        delay_ms(10);

        //Make sure receive interrupt flag is set
        while (!(SPI_getInterruptStatus(EUSCI_B2_BASE,EUSCI_B_SPI_RECEIVE_INTERRUPT)));
        //Receiving data
        byte_2 = SPI_receiveData(EUSCI_B2_BASE);
        delay_ms(10);

        count = ((uint16_t)byte_1 << 8) | byte_2;

        if(count > 10000){
            q[ii] = - enc_res[ii] * (float)(65535 - count);
        }
        else{
        q[ii] = enc_res[ii] * (float)count;   // (deg/count) * count -> angular position
        }

        GPIO_setOutputHighOnPin(ss[ii][0], ss[ii][1]); // Set SS1 line high to end transmission


        qp[ii] = q[ii];

    }
}
