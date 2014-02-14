#ifndef MOVEROBOT_H_
#define MOVEROBOT_H_

 /****************************************************************
 * Constants
 ****************************************************************/
#include <iostream>
#include <stdint.h>
#include "SimpleGPIO.h"
#include "SimplePWM.h"

#define RIGHT   0
#define LEFT    1
#define FORWARD 2
#define REVERSE 3
#define STOP    4
#define BRAKE   5

#define R_FWD   69   // GPIO1_13 = (1x32) + 13 = 45
#define R_REV   45   // GPIO2_5  = (2x32) +  5 = 69
#define R_PWM   23   // GPIO0_23 = (0x32) + 23 = 23
#define L_FWD   47   // GPIO0_27 = (0x32) + 27 = 27
#define L_REV   27   // GPIO1_15 = (1x32) + 17 = 47
#define L_PWM   22   // GPIO0_22 = (0x32) + 22 = 22

/****************************************************************
 * move_robot
 ****************************************************************/

int init_robot();
int close_robot();
int move_robot(int direction, float speed);
int move_robot_2WD(double left, double right);
int rotate_motor(int motor, int direction, float speed);

#endif /* MOVEROBOT_H_ */
