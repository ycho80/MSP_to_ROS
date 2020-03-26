#ifndef SERIAL_R0S_H_
#define SERIAL_ROS_H_

extern char carriage_return;			// Indicates the end of a stream of numbers
extern char new_line;                   // For use with Putty
extern char A;							//Indicates the end of a number
extern char space;             			// Indicates the end of a number
extern char tab;          				// To increase visibility of a stream of numbers

void write_uart(char* msg);

#endif /* SERIAL_ROS_H_ */
