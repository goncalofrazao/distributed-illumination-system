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
Interface interface(ID, &local_controller, &driver, &luxmeter, &metrics, &status);

// can
// uint8_t this_pico_flash_id[8], node_address;
// struct can_frame can_msg_rx, can_msg_tx;
// unsigned long counter_tx = 0, counter_rx = 0;
// MCP2515::ERROR err;
// unsigned long time_to_write;
// unsigned long write_delay = 1000;
// const byte interruptPin = 20;
// volatile byte data_available = false;

// MCP2515 can0{spi0, 17, 19, 16, 18, 10000000};

// void read_interrupt(uint gpio, uint32_t events) { data_available = true; }

unsigned long last_time = micros();
unsigned long sample_time = 10000;

void setup() {
	Serial.begin(115200);

	metrics.set_driver(&driver);

	local_controller.set_ref(1.0);
	local_controller.set_driver(&driver);
	local_controller.set_luxmeter(&luxmeter);
	local_controller.set_metrics(&metrics);
	local_controller.calibrate();

	// setup can
	// flash_get_unique_id(this_pico_flash_id);
	// node_address = this_pico_flash_id[7];
	// can0.reset();
	// can0.setBitrate(CAN_1000KBPS);
	// can0.setNormalMode();
	// gpio_set_irq_enabled_with_callback(interruptPin, GPIO_IRQ_EDGE_FALL, true, &read_interrupt);
	// time_to_write = millis() + write_delay;
}

unsigned long timer;
unsigned long jitter;
unsigned long control_time;

void loop() {
	if (interface.available()) {
		timer = micros();

		interface.process();

		// Serial.print(millis() - timer);
		// Serial.print(" ");
	}

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

	// timer = millis();

	// if (status.luxmeterLogOn()) {
	// 	luxmeter.log(ID);
	// }

	// if (status.dutycycleLogOn()) {
	// 	driver.log(ID);
	// }

	// Serial.print(millis() - timer);
	// Serial.print(" ");

	// timer = millis();

	// if (millis() >= time_to_write) {
	// 	can_msg_tx.can_id = node_address;
	// 	can_msg_tx.can_dlc = 8;
	// 	unsigned long div = counter_tx * 10;
	// 	for (int i = 0; i < 8; i++) {
	// 		can_msg_tx.data[7 - i] = '0' + ((div /= 10) % 10);
	// 	}
	// 	err = can0.sendMessage(&can_msg_tx);
	// 	Serial.print("TX ");
	// 	Serial.print(counter_tx);
	// 	Serial.print(" from node ");
	// 	Serial.println(node_address, HEX);
	// 	counter_tx++;
	// 	time_to_write = millis() + write_delay;
	// }

	// if (data_available) {
	// 	can0.readMessage(&can_msg_rx);
	// 	Serial.print("RX ");
	// 	Serial.print(counter_rx++);
	// 	Serial.print(" from node ");
	// 	Serial.print(can_msg_rx.can_id, HEX);
	// 	Serial.print(" : ");
	// 	for (int i = 0; i < can_msg_rx.can_dlc; i++) {
	// 		Serial.print((char)can_msg_rx.data[i]);
	// 	}
	// 	Serial.println();
	// 	data_available = false;
	// }

	// Serial.print(millis() - timer);
	// Serial.println();
}
