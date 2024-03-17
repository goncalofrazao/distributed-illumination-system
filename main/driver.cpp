#include "driver.hpp"

Driver::Driver(pin_size_t pin, uint32_t frequency, uint32_t range) {
	this->pin = pin;

	pinMode(pin, OUTPUT);
	analogWriteFreq(frequency);
	analogWriteRange(range);
}

void Driver::write(int value) {
	this->duty_cycle = (float)value / DAC_RANGE;
	analogWrite(this->pin, value);
}

void Driver::write_duty_cycle(float value) {
	int v = value * DAC_RANGE;
	this->duty_cycle = value;
	analogWrite(this->pin, v);
}

float Driver::get_duty_cycle() { return this->duty_cycle; }

void Driver::log(int id) {
	Serial.print("s d ");
	Serial.print(id);
	Serial.print(" ");
	Serial.print(this->duty_cycle);
	Serial.print(" ");
	Serial.println(millis() - this->time);
}

void Driver::set_time(int time) { this->time = time; }
