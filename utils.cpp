#include "project.h"


// THIS IS SET FOR 3MHZ
void delayMs(int n) {
    int i,j;
    for (j = 0;j < n;j++) {
        for (i = 2500; i > 0; i--);
    }
}

void calVelocity() {

    for (int i = 0;i<3;i++) {
        // This is actually not implemented.
        actual_joint_vel[i] = (actual_joint_pos[i] - actual_joint_pos[i])*20;
    }
}

// SEND UPDATES THROUGH UART AT A FIXED RATE

