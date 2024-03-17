#include "metrics.hpp"

#include "Arduino.h"

Metrics::Metrics() {
	idx = 0;

	for (int i = 0; i < LAST_MINUTE_BUFFER_LEN; i++) {
		lux[i] = 0;
		duty_cycle[i] = 0;
	}
}

void Metrics::set_driver(Driver *driver) { this->driver = driver; }

void Metrics::push(float lux, float ref, float duty) {
	this->lux[idx] = lux;
	this->duty_cycle[idx] = duty;

	this->energy += duty * TIME_INTERVAL;
	this->visibility_error += (ref - lux > 0) ? ref - lux : 0;

	float this_duty_cycle_variation = last_duty_cycle - duty;
	if (last_duty_cycle_variation * this_duty_cycle_variation < 0) {
		this->flicker += std::abs(last_duty_cycle_variation) + std::abs(this_duty_cycle_variation);
	}
	last_duty_cycle = duty;
	last_duty_cycle_variation = this_duty_cycle_variation;

	idx = (idx + 1) % LAST_MINUTE_BUFFER_LEN;
}

float Metrics::get_energy() { return this->energy * MAX_POWER; }

float Metrics::get_visibility_error() { return visibility_error; }

float Metrics::get_flicker() { return this->flicker; }

float Metrics::get_instantaneous_power() { return this->driver->get_duty_cycle() * MAX_POWER; }

void Metrics::log_luxmeter(int id) {
	Serial.print("b l ");
	Serial.println(id);
	for (int i = idx; i != (idx - 1 + LAST_MINUTE_BUFFER_LEN) % LAST_MINUTE_BUFFER_LEN;
		 i = (i + 1) % LAST_MINUTE_BUFFER_LEN) {
		Serial.print(lux[i]);
		Serial.print(",");
	}
	Serial.println(lux[(idx - 1 + LAST_MINUTE_BUFFER_LEN) % LAST_MINUTE_BUFFER_LEN]);
}

void Metrics::log_dutycycle(int id) {
	Serial.print("b d ");
	Serial.println(id);
	for (int i = idx; i != (idx - 1 + LAST_MINUTE_BUFFER_LEN) % LAST_MINUTE_BUFFER_LEN;
		 i = (i + 1) % LAST_MINUTE_BUFFER_LEN) {
		Serial.print(duty_cycle[i]);
		Serial.print(",");
	}
	Serial.println(duty_cycle[(idx - 1 + LAST_MINUTE_BUFFER_LEN) % LAST_MINUTE_BUFFER_LEN]);
}
