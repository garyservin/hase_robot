#ifndef SIMPLEPWM_H_
#define SIMPLEPWM_H_

 /****************************************************************
 * Constants
 ****************************************************************/
#include <iostream>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#define RIGHT_PWM_DIR "/sys/devices/ocp.3/right_pwm.17/"
#define LEFT_PWM_DIR "/sys/devices/ocp.3/left_pwm.18/"
#define MAX_BUF 64

/****************************************************************
 * simple_pwm
 ****************************************************************/

int set_duty_cycle(int pwm, int duty);
int pwm_init(int pwm);
int pwm_close(int pwm);

#endif /* SIMPLEPWM_H_ */
