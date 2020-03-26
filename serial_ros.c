/*
This function receives an ASCII character array and transmits the characters one by one followed by a
terminatic character that increases visibility or can be recognized to part a series of values send in
one line.

Author: Bryan Blaise
*/
#include "msp.h"
#include "driverlib.h"

#include <stdio.h>
#include <string.h>

#include "serial_ros.h"

// Terminating Characters
extern char carriage_return = '\x0D';           			// ASCII carriage return
extern char new_line = '\n';                     			// ASCII new line
extern char A = 'A';          								// ASCII A
extern char space = '\x20';             					// ASCII space
extern char tab = '\t';          							// ASCII tab

void write_uart(char* msg){
    int str_len;
    str_len = strlen(msg);									// gets the length of the ASCII character array
    int ii;													// looping variable
    for(ii = 0; ii < str_len; ii++){
        MAP_UART_transmitData(EUSCI_A1_BASE, msg[ii]);
    }

    MAP_UART_transmitData(EUSCI_A1_BASE, space);			// transmits a space at the end of the message
    //MAP_UART_transmitData(EUSCI_A0_BASE, A);				// transmits a 'A' at the end of the message
    //MAP_UART_transmitData(EUSCI_A0_BASE, tab);			// transmits a tab at the end of the message
    //MAP_UART_transmitData(EUSCI_A0_BASE, carriage_return);// transmits a carriage return at the end of the message
    //MAP_UART_transmitData(EUSCI_A0_BASE, new_line);		// transmits a new line at the end of the message
}
