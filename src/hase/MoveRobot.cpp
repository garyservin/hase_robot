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

int move_robot_2WD(double left, double right){
    PIN_VALUE l_forward = LOW;
    PIN_VALUE l_reverse = LOW;
    PIN_VALUE r_forward = LOW;
    PIN_VALUE r_reverse = LOW;

    int l_pwm = (int)(left * 50000);
    int r_pwm = (int)(right * 50000);

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

    gpio_set_value(R_FWD, r_forward);
    gpio_set_value(R_REV, r_reverse);
    set_duty_cycle(0, r_pwm);

    gpio_set_value(L_FWD, l_forward);
    gpio_set_value(L_REV, l_reverse);
    set_duty_cycle(1, l_pwm);

    return 0;
}
