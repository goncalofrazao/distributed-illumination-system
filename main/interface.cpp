#include "interface.hpp"

Interface::Interface(LocalController *local_controller, Driver *driver, Luxmeter *luxmeter, Metrics *metrics, Status *status) {
	this->local_controller = local_controller;
	this->driver = driver;
	this->luxmeter = luxmeter;
	this->metrics = metrics;
	this->status = status;
	this->restart_time = millis();
}

void Interface::set_id(int _id) { id = _id; }

bool Interface::available() {
	if (Serial.available()) {
		return true;
	} else {
		return false;
	}
}

void Interface::process(String command) {
	int i, occupancy, anti_windup, feedback;
	float duty_cycle, lux;

	if (command.startsWith("d")) {
		sscanf(command.c_str(), "d %d %f", &i, &duty_cycle);
		if (i == this->id) {
			this->driver->write_duty_cycle(duty_cycle);
			this->status->setControllerOff();
		}
		Serial.println("ack");
	} else if (command.startsWith("g d")) {
		sscanf(command.c_str(), "g d %d", &i);
		if (i == this->id) {
			Serial.print("d ");
			Serial.print(i);
			Serial.print(" ");
			Serial.println(this->driver->get_duty_cycle());
		}
	} else if (command.startsWith("r")) {
		sscanf(command.c_str(), "r %d %f", &i, &lux);
		if (i == this->id) {
			this->local_controller->set_ref(lux);
			this->status->setControllerOn();
		}
		Serial.println("ack");
	} else if (command.startsWith("g r")) {
		sscanf(command.c_str(), "g r %d", &i);
		if (i == this->id) {
			Serial.print("r ");
			Serial.print(i);
			Serial.print(" ");
			Serial.println(this->local_controller->get_ref());
		}
	} else if (command.startsWith("g l")) {
		sscanf(command.c_str(), "g l %d", &i);
		if (i == this->id) {
			Serial.print("l ");
			Serial.print(i);
			Serial.print(" ");
			Serial.println(this->luxmeter->read());
		}
	} else if (command.startsWith("o")) {
		sscanf(command.c_str(), "o %d %d", &i, &occupancy);
		if (i == this->id) {
			this->local_controller->set_ref(occupancy ? 100 : 10);
		}
		Serial.println("ack");
	} else if (command.startsWith("g o")) {
		sscanf(command.c_str(), "g o %d", &i);
		if (i == this->id) {
			Serial.print("o ");
			Serial.print(i);
			Serial.print(" ");
			Serial.println(this->local_controller->get_ref() == 100 ? 1 : 0);
		}
	} else if (command.startsWith("a")) {
		sscanf(command.c_str(), "a %d %d", &i, &anti_windup);
		if (i == this->id) {
			this->local_controller->set_anti_windup(anti_windup);
		}
		Serial.println("ack");
	} else if (command.startsWith("g a")) {
		sscanf(command.c_str(), "g a %d", &i);
		if (i == this->id) {
			Serial.print("a ");
			Serial.print(i);
			Serial.print(" ");
			Serial.println(this->local_controller->get_anti_windup());
		}
	} else if (command.startsWith("k")) {
		sscanf(command.c_str(), "k %d %d", &i, &feedback);
		if (i == this->id) {
			this->local_controller->set_feedback(feedback);
		}
		Serial.println("ack");
	} else if (command.startsWith("g k")) {
		sscanf(command.c_str(), "g k %d", &i);
		if (i == this->id) {
			Serial.print("k ");
			Serial.print(i);
			Serial.print(" ");
			Serial.println(this->local_controller->get_feedback());
		}
	} else if (command.startsWith("g x")) {
		sscanf(command.c_str(), "g x %d", &i);
		if (i == this->id) {
			Serial.print("x ");
			Serial.print(i);
			Serial.print(" ");
			Serial.println(this->luxmeter->read() - this->driver->get_duty_cycle() * this->local_controller->get_G());
		}
	} else if (command.startsWith("g p")) {
		sscanf(command.c_str(), "g p %d", &i);
		if (i == this->id) {
			Serial.print("p ");
			Serial.print(i);
			Serial.print(" ");
			Serial.println(this->metrics->get_instantaneous_power());
		}
	} else if (command.startsWith("g t")) {
		sscanf(command.c_str(), "g t %d", &i);
		if (i == this->id) {
			Serial.print("t ");
			Serial.print(i);
			Serial.print(" ");
			Serial.println((millis() - this->restart_time) / 1000);
		}
	} else if (command.startsWith("s l")) {
		sscanf(command.c_str(), "s l %d", &i);
		if (i == this->id) {
			this->status->setLuxmeterLogOn();
			this->luxmeter->set_time(millis());
		}
	} else if (command.startsWith("s d")) {
		sscanf(command.c_str(), "s d %d", &i);
		if (i == this->id) {
			this->status->setDutycycleLogOn();
			this->driver->set_time(millis());
		}
	} else if (command.startsWith("S l")) {
		sscanf(command.c_str(), "S l %d", &i);
		if (i == this->id) {
			this->status->setLuxmeterLogOff();
		}
		Serial.println("ack");
	} else if (command.startsWith("S d")) {
		sscanf(command.c_str(), "S d %d", &i);
		if (i == this->id) {
			this->status->setDutycycleLogOff();
		}
		Serial.println("ack");
	} else if (command.startsWith("g b l")) {
		sscanf(command.c_str(), "g b l %d", &i);
		if (i == this->id) {
			this->metrics->log_luxmeter(i);
		}
	} else if (command.startsWith("g b d")) {
		sscanf(command.c_str(), "g b d %d", &i);
		if (i == this->id) {
			this->metrics->log_dutycycle(i);
		}
	} else if (command.startsWith("g e")) {
		sscanf(command.c_str(), "g e %d", &i);
		if (i == this->id) {
			Serial.print("e ");
			Serial.print(i);
			Serial.print(" ");
			Serial.println(this->metrics->get_energy());
		}
	} else if (command.startsWith("g v")) {
		sscanf(command.c_str(), "g v %d", &i);
		if (i == this->id) {
			Serial.print("v ");
			Serial.print(i);
			Serial.print(" ");
			Serial.println(this->metrics->get_visibility_error());
		}
	} else if (command.startsWith("g f")) {
		sscanf(command.c_str(), "g f %d", &i);
		if (i == this->id) {
			Serial.print("f ");
			Serial.print(i);
			Serial.print(" ");
			Serial.println(this->metrics->get_flicker());
		}
	} else if (command.startsWith("get id")) {
		Serial.print("id: ");
		Serial.println(this->id);
	} else if (command.startsWith("log off")) {
		this->status->setLogOff();
	} else if (command.startsWith("log on")) {
		this->status->setLogOn();
	} else {
		Serial.println("err");
	}
}

float Interface::get_time() { return (float)(millis() - this->restart_time) / 1000.0; }

Status::Status() {
	this->_controllerOn = true;
	this->_logOn = false;
	this->_luxmeterLogOn = false;
	this->_dutycycleLogOn = false;
}

bool Status::controllerOn() { return this->_controllerOn; }

bool Status::logOn() { return this->_logOn; }

bool Status::luxmeterLogOn() { return this->_luxmeterLogOn; }

bool Status::dutycycleLogOn() { return this->_dutycycleLogOn; }

void Status::setControllerOn() { this->_controllerOn = true; }

void Status::setControllerOff() { this->_controllerOn = false; }

void Status::setLogOn() { this->_logOn = true; }

void Status::setLogOff() { this->_logOn = false; }

void Status::setLuxmeterLogOn() { this->_luxmeterLogOn = true; }

void Status::setLuxmeterLogOff() { this->_luxmeterLogOn = false; }

void Status::setDutycycleLogOn() { this->_dutycycleLogOn = true; }

void Status::setDutycycleLogOff() { this->_dutycycleLogOn = false; }
