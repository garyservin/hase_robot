#include "../../include/hase_driver/hase.hpp"

namespace hase
{

Hase::Hase() : is_enabled(false)
{
}

Hase::~Hase(){
	disable();
	// Right Motor
	gpio_unexport(R_REV);
	gpio_unexport(R_FWD);
	pwm_close(R_PWM);

	//Left Motor
	gpio_unexport(L_REV);
	gpio_unexport(L_FWD);
	pwm_close(L_PWM);
}

void Hase::init(void)
{
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
}

bool Hase::enable()
{
	is_enabled = true;
	return true;
}

bool Hase::disable()
{
	drive_2wd(0.0f, 0.0f);
	is_enabled = false;
	return true;
}

void Hase::drive_2wd(double left, double right)
{
	PIN_VALUE r_forward = LOW, r_reverse = LOW;
	PIN_VALUE l_forward = LOW, l_reverse = LOW;

	int r_pwm = 0;
	int l_pwm = 0;

	if(right > 0){
		r_forward = HIGH;
		r_reverse = LOW;
	}else if(right < 0){
		r_forward = LOW;
		r_reverse = HIGH;
	}

	if(left > 0){
		l_forward = HIGH;
		l_reverse = LOW;
	}else if(left < 0){
		l_forward = LOW;
		l_reverse = HIGH;
	}

	if(is_enabled){
		r_pwm = (int)(right * 50000);
		l_pwm = (int)(left * 50000);
	}else{
		r_forward = r_reverse = LOW;
		l_forward = l_reverse = LOW;
	}

	gpio_set_value(R_FWD, r_forward);
	gpio_set_value(R_REV, r_reverse);
	set_duty_cycle(0, r_pwm);

	gpio_set_value(L_FWD, l_forward);
	gpio_set_value(L_REV, l_reverse);
	set_duty_cycle(1, l_pwm);
}

}// namespace hase
