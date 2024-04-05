#include "luxmeter.hpp"

#include <algorithm>

#include "communicator.hpp"
#include "driver.hpp"

// const float b = 6.3551;	 // rpi 1
// const float m = -0.614;
// const float b = 6.1319;	 // rpi 2
// const float m = -0.2706;
const float b = 5.7992;	 // rpi 3
const float m = -0.8267;

Luxmeter::Luxmeter(pin_size_t pin, int bits) {
	this->pin = pin;
	analogReadResolution(bits);
}

float Luxmeter::lux(int adc) { return std::pow(10, (std::log10(33000 / (adc * 3.3 / DAC_RANGE) - 10000) - b) / m); }

float Luxmeter::read() {
	// read 8 values and get the median
	int arr[WINDOW_SIZE];
	for (int i = 0; i < WINDOW_SIZE; i++) {
		arr[i] = analogRead(pin);
	}

	std::sort(arr, arr + WINDOW_SIZE);

	// get the median value
	return lux(arr[WINDOW_SIZE / 2]);
}

void Luxmeter::read_raw() {
	int arr[WINDOW_SIZE];
	for (int i = 0; i < WINDOW_SIZE; i++) {
		arr[i] = analogRead(pin);
	}

	std::sort(arr, arr + WINDOW_SIZE);

	int voltage_raw = arr[WINDOW_SIZE / 2];
	float voltage = voltage_raw * 3.3 / DAC_RANGE;
	float resistance = (33000 / voltage - 10000);
	Serial.print(voltage_raw);
	Serial.print(" ");
	Serial.print(voltage);
	Serial.print(" ");
	Serial.println(resistance);
}

float Luxmeter::read_resistance() {
	int arr[WINDOW_SIZE];
	for (int i = 0; i < WINDOW_SIZE; i++) {
		arr[i] = analogRead(pin);
	}

	std::sort(arr, arr + WINDOW_SIZE);

	int voltage_raw = arr[WINDOW_SIZE / 2];
	float voltage = voltage_raw * 3.3 / DAC_RANGE;
	return (33000 / voltage - 10000);
}

float Luxmeter::lux2resistance(float lux) { return (std::pow(10, (std::log10(lux) * m + b))); }

void Luxmeter::log(void *communicator) {
	Communicator *comm = (Communicator *)communicator;
	comm->stream_lux(this->read());
	comm->send(NULL);
}

void Luxmeter::set_time(int time) { this->time = time; }
