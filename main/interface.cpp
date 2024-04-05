#include "interface.hpp"

#include "communicator.hpp"
#include "controller.hpp"

Interface::Interface(LocalController *local_controller, Driver *driver, Luxmeter *luxmeter, Metrics *metrics, Status *status, void *communicator,
					 void *controller) {
	this->local_controller = local_controller;
	this->driver = driver;
	this->luxmeter = luxmeter;
	this->metrics = metrics;
	this->status = status;
	this->communicator = communicator;
	this->controller = controller;
	this->restart_time = millis();

	this->neighbour_count = 0;
	for (int i = 0; i < 2; i++) {
		this->neighbours[i] = 0;
	}
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
	Communicator *comm = (Communicator *)communicator;
	Controller *ctrl = (Controller *)controller;

	if (command.startsWith("d")) {
		sscanf(command.c_str(), "d %d %f", &i, &duty_cycle);
		if (i == this->id) {
			this->driver->write_duty_cycle(duty_cycle);
			this->status->setControllerOff();
			Serial.println("ack");
		} else {
			comm->set_duty(i, duty_cycle);
			comm->default_push();
		}
	} else if (command.startsWith("g d")) {
		sscanf(command.c_str(), "g d %d", &i);
		if (i == this->id) {
			serial_convert_float('d', i, this->driver->get_duty_cycle());
		} else {
			comm->get_duty(i);
			comm->default_push();
		}
	} else if (command.startsWith("r")) {
		sscanf(command.c_str(), "r %d %f", &i, &lux);
		if (i == this->id) {
			this->local_controller->set_ref(lux);
			this->status->setControllerOn();
			Serial.println("ack");
		} else {
			comm->set_ref(i, lux);
			comm->default_push();
		}
	} else if (command.startsWith("g r")) {
		sscanf(command.c_str(), "g r %d", &i);
		if (i == this->id) {
			serial_convert_float('r', i, this->local_controller->get_ref());
		} else {
			comm->get_ref(i);
			comm->default_push();
		}
	} else if (command.startsWith("g l")) {
		sscanf(command.c_str(), "g l %d", &i);
		if (i == this->id) {
			serial_convert_float('l', i, this->luxmeter->read());
		} else {
			comm->get_lux(i);
			comm->default_push();
		}
	} else if (command.startsWith("o")) {
		sscanf(command.c_str(), "o %d %d", &i, &occupancy);
		if (i == this->id) {
			this->status->setControllerOn();
			if (occupancy)
				ctrl->set_occupied();
			else
				ctrl->set_unoccupied();
			ctrl->consensus_iterate();
			Serial.println("ack");
		} else {
			comm->set_occ(i, occupancy);
			comm->default_push();
		}
	} else if (command.startsWith("g o")) {
		sscanf(command.c_str(), "g o %d", &i);
		if (i == this->id) {
			serial_convert_int('o', i, this->local_controller->get_occupancy());
		} else {
			comm->get_occ(i);
			comm->default_push();
		}
	} else if (command.startsWith("a")) {
		sscanf(command.c_str(), "a %d %d", &i, &anti_windup);
		if (i == this->id) {
			this->local_controller->set_anti_windup(anti_windup);
			Serial.println("ack");
		} else {
			comm->set_anti(i, anti_windup);
			comm->default_push();
		}
	} else if (command.startsWith("g a")) {
		sscanf(command.c_str(), "g a %d", &i);
		if (i == this->id) {
			serial_convert_int('a', i, this->local_controller->get_anti_windup());
		} else {
			comm->get_anti(i);
			comm->default_push();
		}
	} else if (command.startsWith("k")) {
		sscanf(command.c_str(), "k %d %d", &i, &feedback);
		if (i == this->id) {
			this->local_controller->set_feedback(feedback);
			Serial.println("ack");
		} else {
			comm->set_fb(i, feedback);
			comm->default_push();
		}
	} else if (command.startsWith("g k")) {
		sscanf(command.c_str(), "g k %d", &i);
		if (i == this->id) {
			serial_convert_int('k', i, this->local_controller->get_feedback());
		} else {
			comm->get_fb(i);
			comm->default_push();
		}
	} else if (command.startsWith("g x")) {
		sscanf(command.c_str(), "g x %d", &i);
		if (i == this->id) {
			serial_convert_float('x', i, this->luxmeter->read() - this->driver->get_duty_cycle() * this->local_controller->get_G());
		} else {
			comm->get_ext_lux(i);
			comm->default_push();
		}
	} else if (command.startsWith("g p")) {
		sscanf(command.c_str(), "g p %d", &i);
		if (i == this->id) {
			serial_convert_float('p', i, this->metrics->get_instantaneous_power());
		} else {
			comm->get_power(i);
			comm->default_push();
		}
	} else if (command.startsWith("g t")) {
		sscanf(command.c_str(), "g t %d", &i);
		if (i == this->id) {
			serial_convert_float('t', i, this->get_time());
		} else {
			comm->get_time(i);
			comm->default_push();
		}
	} else if (command.startsWith("s l")) {
		sscanf(command.c_str(), "s l %d", &i);
		if (i == this->id) {
			this->status->setLuxmeterLogOn();
		} else {
			comm->start_lux_stream(i);
			comm->default_push();
		}
	} else if (command.startsWith("s d")) {
		sscanf(command.c_str(), "s d %d", &i);
		if (i == this->id) {
			this->status->setDutycycleLogOn();
		} else {
			comm->start_duty_stream(i);
			comm->default_push();
		}
	} else if (command.startsWith("S l")) {
		sscanf(command.c_str(), "S l %d", &i);
		if (i == this->id) {
			this->status->setLuxmeterLogOff();
			Serial.println("ack");
		} else {
			comm->stop_lux_stream(i);
			comm->default_push();
		}
	} else if (command.startsWith("S d")) {
		sscanf(command.c_str(), "S d %d", &i);
		if (i == this->id) {
			this->status->setDutycycleLogOff();
			Serial.println("ack");
		} else {
			comm->stop_duty_stream(i);
			comm->default_push();
		}
	} else if (command.startsWith("g b l")) {
		sscanf(command.c_str(), "g b l %d", &i);
		if (i == this->id) {
			this->metrics->log_luxmeter(i);
		} else {
			comm->get_last_min_lux(i);
			comm->default_push();
		}
	} else if (command.startsWith("g b d")) {
		sscanf(command.c_str(), "g b d %d", &i);
		if (i == this->id) {
			this->metrics->log_dutycycle(i);
		} else {
			comm->get_last_min_duty(i);
			comm->default_push();
		}
	} else if (command.startsWith("g e")) {
		sscanf(command.c_str(), "g e %d", &i);
		if (i == this->id) {
			serial_convert_float('e', i, this->metrics->get_energy());
		} else {
			comm->get_energy(i);
			comm->default_push();
		}
	} else if (command.startsWith("g v")) {
		sscanf(command.c_str(), "g v %d", &i);
		if (i == this->id) {
			serial_convert_float('v', i, this->metrics->get_visibility_error());
		} else {
			comm->get_visibility(i);
			comm->default_push();
		}
	} else if (command.startsWith("g f")) {
		sscanf(command.c_str(), "g f %d", &i);
		if (i == this->id) {
			serial_convert_float('f', i, this->metrics->get_flicker());
		} else {
			comm->get_flicker(i);
			comm->default_push();
		}
	} else if (command.startsWith("g O")) {
		sscanf(command.c_str(), "g O %d", &i);
		if (i == this->id) {
			serial_convert_float('O', i, ctrl->get_occupied());
		} else {
			comm->get_o_bound(i);
			comm->default_push();
		}
	} else if (command.startsWith("O")) {
		sscanf(command.c_str(), "O %d %f", &i, &lux);
		if (i == this->id) {
			ctrl->set_occupied_bound(lux);
			comm->consensus();
			comm->default_push();
			ctrl->consensus_iterate();
			Serial.println("ack");
		} else {
			comm->set_o_bound(i, lux);
			comm->default_push();
		}
	} else if (command.startsWith("g U")) {
		sscanf(command.c_str(), "g U %d", &i);
		if (i == this->id) {
			serial_convert_float('U', i, ctrl->get_unoccupied());
		} else {
			comm->get_u_bound(i);
			comm->default_push();
		}
	} else if (command.startsWith("U")) {
		sscanf(command.c_str(), "U %d %f", &i, &lux);
		if (i == this->id) {
			ctrl->set_unoccupied_bound(lux);
			Serial.println("ack");
			comm->consensus();
			comm->default_push();
			ctrl->consensus_iterate();
		} else {
			comm->set_u_bound(i, lux);
			comm->default_push();
		}
	} else if (command.startsWith("g L")) {
		sscanf(command.c_str(), "g L %d", &i);
		if (i == this->id) {
			serial_convert_float('L', i, ctrl->get_L());
		} else {
			comm->get_l_bound(i);
			comm->default_push();
		}
	} else if (command.startsWith("g c")) {
		sscanf(command.c_str(), "g c %d", &occupancy);
		i = occupancy == id ? 2 : get_neighbour_index(occupancy);
		serial_convert_float('c', occupancy, ctrl->get_c(i));
	} else if (command.startsWith("c")) {
		sscanf(command.c_str(), "c %d %f", &occupancy, &lux);
		i = occupancy == id ? 2 : get_neighbour_index(occupancy);
		ctrl->set_c(i, lux);
		comm->set_e_cost(occupancy, lux);
		comm->default_push();
		comm->consensus();
		comm->default_push();
		ctrl->consensus_iterate();
	} else if (command.startsWith("R")) {
		this->restart_time = millis();
		ctrl->calib();
		comm->consensus();
		comm->default_push();
		ctrl->consensus_iterate();
		Serial.println("ack");
	} else if (command.startsWith("get id")) {
		Serial.print("id: ");
		Serial.println(this->id);
	} else if (command.startsWith("log off")) {
		this->status->setLogOff();
	} else if (command.startsWith("log on")) {
		this->status->setLogOn();
	} else if (command.startsWith("get neighbours")) {
		this->print_neighbours();
	} else if (command.startsWith("get ks")) {
		ctrl->log();
	} else {
		Serial.println("err");
	}
}

float Interface::get_time() { return (float)(millis() - this->restart_time) / 1000.0; }

bool exists(int *arr, int size, int val) {
	for (int i = 0; i < size; i++) {
		if (arr[i] == val) {
			return true;
		}
	}
	return false;
}

int *Interface::get_neighbours(int *n) {
	*n = neighbour_count;
	return neighbours;
}

void Interface::push_neighbour(int id) {
	if (!exists(neighbours, neighbour_count, id)) {
		neighbours[neighbour_count] = id;
		neighbour_count++;
	}
}

void Interface::print_neighbours() {
	for (int i = 0; i < neighbour_count; i++) {
		Serial.print(neighbours[i]);
		Serial.print(" ");
	}
	Serial.println();
}

int Interface::get_neighbour_index(int id) {
	for (int i = 0; i < neighbour_count; i++) {
		if (neighbours[i] == id) {
			return i;
		}
	}
	return -1;
}

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
