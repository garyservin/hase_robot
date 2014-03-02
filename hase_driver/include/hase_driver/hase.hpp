#ifndef _HASE_HPP_
#define _HASE_HPP_

#include "SimpleGPIO.hpp"
#include "SimplePWM.hpp"

#define R_FWD   69   // GPIO1_13 = (1x32) + 13 = 45
#define R_REV   45   // GPIO2_5  = (2x32) +  5 = 69
#define R_PWM   23   // GPIO0_23 = (0x32) + 23 = 23
#define L_FWD   47   // GPIO0_27 = (0x32) + 27 = 27
#define L_REV   27   // GPIO1_15 = (1x32) + 17 = 47
#define L_PWM   22   // GPIO0_22 = (0x32) + 22 = 22

namespace hase {
	class Hase
	{
	public:
		Hase();
		~Hase();
		void init(void);
		bool isEnabled() const { return is_enabled; } /**< Whether the motor power is enabled or disabled. **/
		bool enable(); /**< Enable power to the motors. **/
		bool disable(); /**< Disable power to the motors. **/
		void drive_2wd(double left, double right);
		//void setDigitalOutput(const DigitalOutput &digital_output);
	private:
		bool is_enabled;
	};
}; // namespace hase
#endif /* _HASE_HPP_ */
