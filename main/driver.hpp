#ifndef _DRIVER_HPP_
#define _DRIVER_HPP_

#include <Arduino.h>

#define DAC_RANGE 4095

class Driver {
   public:
	Driver(pin_size_t pin, uint32_t frequency, uint32_t range);
	void write(int value);
	void write_duty_cycle(float value);
	float get_duty_cycle();
	void log(int id);
	void set_time(int time);

   private:
	int time;
	pin_size_t pin;
	float duty_cycle;
};

#endif