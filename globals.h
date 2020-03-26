#ifndef GLOBALS_H_
#define GLOBALS_H_

extern float dt;
extern char carriage_return;           // Indicates the end of a stream of numbers
extern char new_line;                     // For use with Putty
extern char A;          //Indicates the end of a number
extern char tab;          // To increase visibility of a stream of numbers

extern float q[3];               // Joint Angular Positions
//extern char q_s[1][8];
//extern float qd[2];           // Joint Angular Velocities
//extern char qd_s[1][8];
extern float qp[3];   // Variable to hold previous time step angular position

//extern volatile uint32_t rise_count[3];
//extern volatile uint32_t fall_count[3];
//extern volatile int32_t pulse_length[3];
//extern float pulse_duration[3];       // Already defined in peripheralFunctions.c

extern float q_precision;
extern float V_precision;

extern const eUSCI_UART_Config uartConfig;
extern const eUSCI_SPI_MasterConfig spiMasterConfig;
extern const Timer_A_UpModeConfig upConfig_0;
extern const Timer_A_UpModeConfig upConfig_1;
extern const Timer_A_UpModeConfig upConfig_3;
extern const Timer_A_CaptureModeConfig captureModeConfig_q4_rising;
extern const Timer_A_CaptureModeConfig captureModeConfig_q4_falling;
extern const Timer_A_CaptureModeConfig captureModeConfig_q5_rising;
extern const Timer_A_CaptureModeConfig captureModeConfig_q5_falling;
extern const Timer_A_CaptureModeConfig captureModeConfig_q6_rising;
extern const Timer_A_CaptureModeConfig captureModeConfig_q6_falling;

extern int ss[3][2];
extern int adc_pin[7][2];
extern int adc_input[7];
extern int adc_mem[7];
extern int capture_pin[6][2];

#endif /* GLOBALS_H_ */
