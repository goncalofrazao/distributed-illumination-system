#include "communicator.hpp"
#include "driver.hpp"
#include "hardware/flash.h"
#include "interface.hpp"
#include "local_controller.hpp"
#include "luxmeter.hpp"
#include "mcp2515.h"
#include "metrics.hpp"

#define LED_PIN 15
#define FREQUENCY 60000
#define RANGE 4095

#define ID 1

// control
Luxmeter luxmeter(A0, 12);
LocalController local_controller(1, 1, 1, 1, 0, 1, 10);
Driver driver(LED_PIN, FREQUENCY, RANGE);
Metrics metrics;
Status status;
Communicator communicator;
Interface interface(&local_controller, &driver, &luxmeter, &metrics, &status, (void*)&communicator);

MCP2515 can0{spi0, 17, 19, 16, 18, 10000000};

unsigned long last_time = micros();
unsigned long sample_time = 10000;

unsigned long time_to_write;
unsigned long write_delay = 1000;

unsigned long timer;
unsigned long jitter;
unsigned long control_time;

void setup() {
	Serial.begin(115200);

	metrics.set_driver(&driver);

	local_controller.set_ref(1.0);
	local_controller.set_driver(&driver);
	local_controller.set_luxmeter(&luxmeter);
	local_controller.set_metrics(&metrics);
	local_controller.set_communicator((void*)&communicator);
	// local_controller.calibrate();

	time_to_write = millis() + write_delay;

	communicator.set_can0(&can0);
	communicator.set_interface(&interface);
	communicator.set_id();

	communicator.send_syn();
}

void setup1() {}

void loop() {
	unsigned long current_time = micros() - last_time;
	if (current_time > sample_time && status.controllerOn()) {
		jitter = current_time - sample_time;

		timer = micros();

		local_controller.control(luxmeter.read());

		control_time = micros() - timer;

		if (status.logOn()) {
			Serial.print(jitter);
			Serial.print(" ");

			Serial.print(control_time);
			Serial.print(" ");

			local_controller.log();

			Serial.print(interface.get_time());
			Serial.print(" ");
			Serial.println();
		}

		last_time = micros();
	}

	if (status.luxmeterLogOn() && current_time > sample_time) {
		luxmeter.log(&communicator);
	}

	if (status.dutycycleLogOn() && current_time > sample_time) {
		driver.log(&communicator);
	}

	// if (millis() >= time_to_write) {
	// 	communicator.send_id();
	// 	time_to_write = millis() + write_delay;
	// }

	if (communicator.recv()) {
		// uint8_t id = communicator.get_recv_id();
		// Serial.print("RX ");
		// Serial.println(id);
		communicator.process();
	}
}

void loop1() {
	if (interface.available()) {
		rp2040.idleOtherCore();
		String command = Serial.readStringUntil('\n');
		interface.process(command);
		rp2040.resumeOtherCore();
	}
	delay(1000);
}
