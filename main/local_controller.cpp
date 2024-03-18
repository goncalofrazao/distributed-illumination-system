#include "local_controller.hpp"

#include <Arduino.h>

#include <iostream>

#define sat(v, min, max) (v < min ? min : (v > max ? max : v))
#define R1 10000.0
#define Vcc 3.3
#define C1 0.00001

LocalController::LocalController(float _h, float _K, float b_, float _Ti, float _Td, float _Tt,
								 float N_)
	: h(_h), K(_K), b(b_), Ti(_Ti), Td(_Td), Tt(_Tt), N(N_), I(0.0), D(0.0), y_old(0.0) {
	bi = _K * _h / _Ti;
	ad = _Td / (_Td + N_ * _h);
	bd = _K * ad * N_;
	ao = _h / _Tt;
	anti_windup = true;
	feedback = true;
}

float LocalController::get_G() { return this->G; }

void LocalController::set_feedback(bool feedback) { this->feedback = feedback; }

bool LocalController::get_feedback() { return this->feedback; }

void LocalController::set_anti_windup(bool anti_windup) { this->anti_windup = anti_windup; }

bool LocalController::get_anti_windup() { return this->anti_windup; }

void LocalController::set_ref(float ref) { this->ref = ref; }

float LocalController::get_ref() { return this->ref; }

void LocalController::set_driver(Driver *driver) { this->driver = driver; }

void LocalController::set_luxmeter(Luxmeter *luxmeter) { this->luxmeter = luxmeter; }

void LocalController::set_metrics(Metrics *metrics) { this->metrics = metrics; }

void LocalController::control(float y) {
	P = K * b * ref;
	D = ad * D;
	if (feedback) {
		P = P - K * y;
		D = D - bd * (y - y_old);
	}

	v = P + I + D;
	u = sat(v, 0.0, 4095.0);

	driver->write((int)u);

	if (feedback) {
		I = I + bi * (ref - y);
		if (anti_windup) {
			I = I + ao * (u - v);
		}
	}
	y_old = y;

	metrics->push(y, ref, u / 4095.0);
}

void LocalController::log() {
	Serial.print(G);  // Actuator gain
	Serial.print(" ");
	Serial.print(ref);	// reference luminance
	Serial.print(" ");
	Serial.print(y_old);  // read luminance
	Serial.print(" ");
	Serial.print(u / 4095.0);  // actuation (in duty cycle)
	Serial.print(" ");
	Serial.print(v / 4095.0);  // PID output (in duty cycle)
	Serial.print(" ");
	Serial.print(I);  // integral term of the controller
	Serial.print(" ");
	Serial.print(P);  // proportional term of the controller
	Serial.print(" ");
	Serial.print(this->metrics->get_energy());	// cumulative energy
	Serial.print(" ");
	Serial.print(this->metrics->get_visibility_error());  // cumulative visibility error
	Serial.print(" ");
	Serial.print(this->metrics->get_flicker());	 // cumulative flicker
	Serial.print(" ");
	Serial.print(this->metrics->get_instantaneous_power());	 // instantaneous power
	Serial.print(" ");
	Serial.print(this->luxmeter->read() -
				 G * this->driver->get_duty_cycle());  // external luminance
	Serial.print(" ");
}

void LocalController::set_ideal_gains(float x) {
	float R2 = luxmeter->lux2resistance(this->ref);

	float tau = R1 * R2 * C1 / (R1 + R2);
	float H = Vcc * R1 / (x * (R1 + R2));
	Ti = tau;
	K = tau / (G * H * x);
	b = x / tau;

	bi = K * h / Ti;
	ad = Td / (Td + N * h);
	bd = K * ad * N;
	ao = h / Tt;

	// Serial.print("G ");
	// Serial.print(G);
	// Serial.print(" H ");
	// Serial.print(H);
	// Serial.print(" tau ");
	// Serial.print(tau);
	// Serial.print(" K ");
	// Serial.print(K);
	// Serial.print(" b ");
	// Serial.print(b);
	// Serial.print(" Ti ");
	// Serial.println(Ti);
}

void LocalController::calibrate() {
	driver->write_duty_cycle(0.0);
	delay(1000);
	float y1 = luxmeter->read();
	driver->write_duty_cycle(1.0);
	delay(1000);
	float y2 = luxmeter->read();

	G = (y2 - y1) / 1.0;
	driver->write_duty_cycle(0.0);
}
