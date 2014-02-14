#include "MoveRobot.h"

int init_robot(){
    // Right Motor
    gpio_export(R_REV);
    gpio_export(R_FWD);
    gpio_set_dir(R_REV, OUTPUT_PIN);
    gpio_set_dir(R_FWD, OUTPUT_PIN);
    pwm_init(R_PWM);

    //Left Motor
    gpio_export(L_REV);
    gpio_export(L_FWD);
    gpio_set_dir(L_REV, OUTPUT_PIN);
    gpio_set_dir(L_FWD, OUTPUT_PIN);
    pwm_init(L_PWM);

    return 0;
}

int close_robot(){
    // Right Motor
    gpio_unexport(R_REV);
    gpio_unexport(R_FWD);
    pwm_close(R_PWM);

    //Left Motor
    gpio_unexport(L_REV);
    gpio_unexport(L_FWD);
    pwm_close(L_PWM);

    return 0;
}

int move_robot(int direction, float speed){
    switch(direction){
        case FORWARD:
            rotate_motor(RIGHT, FORWARD, speed);
            rotate_motor(LEFT, FORWARD, speed);
            break;
        case REVERSE:
            rotate_motor(RIGHT, REVERSE, speed);
            rotate_motor(LEFT, REVERSE, speed);
            break;
        case LEFT:
            rotate_motor(RIGHT, FORWARD, speed);
            rotate_motor(LEFT, REVERSE, speed);
            break;
        case RIGHT:
            rotate_motor(RIGHT, REVERSE, speed);
            rotate_motor(LEFT, FORWARD, speed);
            break;
        case STOP:
            rotate_motor(RIGHT, STOP, speed);
            rotate_motor(LEFT, STOP, speed);
            break;
    }
    return 0;
}

int move_robot_2WD(double left, double right){
    PIN_VALUE l_forward = LOW;
    PIN_VALUE l_reverse = LOW;
    PIN_VALUE r_forward = LOW;
    PIN_VALUE r_reverse = LOW;

    uint16_t l_pwm = (int)(left * 50000);
    uint16_t r_pwm = (int)(right * 50000);

    if (right > 0){
        r_forward = HIGH;
        r_reverse = LOW;
    }else{
        r_forward = LOW;
        r_reverse = HIGH;
    }

    if (left > 0){
        l_forward = HIGH;
        l_reverse = LOW;
    }else{
        l_forward = LOW;
        l_reverse = HIGH;
    }

    gpio_set_value(R_FWD, l_forward);
    gpio_set_value(R_REV, l_reverse);
    set_duty_cycle(0, r_pwm);

    gpio_set_value(L_FWD, r_forward);
    gpio_set_value(L_REV, r_reverse);
    set_duty_cycle(1, l_pwm);

    return 0;
}

int rotate_motor(int motor, int direction, float speed){
    PIN_VALUE forward = LOW;
    PIN_VALUE reverse = LOW;
    int pwm = 0;

    switch(direction){
        case FORWARD:
            forward = HIGH;
            reverse = LOW;
            break;
        case REVERSE:
            forward = LOW;
            reverse = HIGH;
            break;
        case STOP:
            forward = LOW;
            reverse = LOW;
            break;
        case BRAKE:
            forward = HIGH;
            reverse = HIGH;
            break;
    }

    pwm = (int)(speed * 50000);

    if (motor == LEFT){
        gpio_set_value(L_FWD, forward);
        gpio_set_value(L_REV, reverse);
        set_duty_cycle(1, pwm);
    }else{
        gpio_set_value(R_FWD, forward);
        gpio_set_value(R_REV, reverse);
        set_duty_cycle(0, pwm);
    }
    return 0;
}
