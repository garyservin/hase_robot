#include "../../include/hase_driver/SimplePWM.hpp"

int pwm_init(int pwm){
    return 0;
}

int pwm_close(int pwm){
    return 0;
}

int set_duty_cycle(int pwm, int duty){
    int fd, len;
    char buf[MAX_BUF];

    if (pwm == 0){
        snprintf(buf, sizeof(buf), RIGHT_PWM_DIR "/duty");
    } else{
        snprintf(buf, sizeof(buf), LEFT_PWM_DIR "/duty");
    }

    fd = open(buf, O_WRONLY);
    if (fd < 0) {
        perror("pwm/set-dutycycle");
        return fd;
    }

    len = snprintf(buf, sizeof(buf), "%d", duty);

    write(fd, buf, len);
    close(fd);

    return 0;
}
