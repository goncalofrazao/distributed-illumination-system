#ifndef _LUXMETER_HPP_
#define _LUXMETER_HPP_

#include <Arduino.h>

#define WINDOW_SIZE 8

class Luxmeter {
   public:
	Luxmeter(pin_size_t pin, int bits);
	float lux(int adc);
	float read();
	void read_raw();
	float read_resistance();
	float lux2resistance(float lux);
	void log(void *communicator);
	void set_time(int time);

   private:
	int time;
	pin_size_t pin;
	int bits;
};

#endif