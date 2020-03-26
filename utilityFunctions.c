#include "msp.h"
#include "driverlib.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "utilityFunctions.h"

// Converts a float to a string with as many decimal places as the variable "precision"
char * myFTOA(char *s, float n, float precision){
  // handle special cases
    int digit, m;
    char *c = s;
    int neg = (n < 0);

    if (neg)
      n = -n;
    // calculate magnitude
     m = log10(n);

    if (neg)
      *(c++) = '-';

    else
      // *(c++) = '+'; // To have a ‘+’ sign

    if (m < 1.0) {
      m = 0;
    }

    // convert the number

    while (n > precision || m >= 0){
      double weight = pow(10.0, m);

      if (weight > 0 && !isinf(weight)){
        digit = floor(n / weight);
        n -= (digit * weight);
        *(c++) = '0' + digit;
      }
      if (m == 0 && n > 0)
        *(c++) = '.';
      m--;
    }
    *(c) = '\0';

  return s;
}

// Returns the absolute value of a number
float absVal(float value){
    float absValue = 0.0;

    if(value < 0.0){
        absValue = - value;
    }
    else if(value >= 0.0){
        absValue = value;
    }
    return absValue;
}

// Implements a simple delay
void delay_ms(int n){
    // **I have not yet figured out how to make this precise so that n = ms
    int i; int j;
    for (j = 0;j < n;j++) {
        for (i = 250; i > 0; i--);
    }
}

int sgn(float value){
    int sign;
    if(value > 0.0){
        sign = 1;
    }
    else if(value < 0.0){
        sign = -1;
    }
    else if(value == 0.0){
        sign = 0;
    }

    return sign;
}
