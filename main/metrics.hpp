#ifndef _METRICS_HPP_
#define _METRICS_HPP_

#define LAST_MINUTE_BUFFER_LEN 6000
#define MAX_POWER 0.110
#define TIME_INTERVAL 0.01

#include "driver.hpp"

class Metrics {
	float lux[LAST_MINUTE_BUFFER_LEN];
	float duty_cycle[LAST_MINUTE_BUFFER_LEN];
	Driver *driver;
	int idx;

	float energy;
	float visibility_error;
	float flicker;

	float last_duty_cycle_variation = 0;
	float last_duty_cycle = 0;

   public:
	Metrics();
	void push(float lux, float ref, float duty);

	void set_driver(Driver *driver);

	float get_energy();
	float get_visibility_error();
	float get_flicker();
	float get_instantaneous_power();
	void log_luxmeter(int id);
	void log_dutycycle(int id);
};

#endif
