#include "communicator.hpp"

#include "controller.hpp"

Communicator::Communicator() {}

void Communicator::default_push() { can_queue.push(can_msg_tx); }

void Communicator::set_id() {
	uint8_t pico_flash_id[8];
	flash_get_unique_id(pico_flash_id);
	id = pico_flash_id[6];
	interface->set_id(id);
}

uint8_t Communicator::get_id() { return id; }

void Communicator::set_can0(MCP2515* _can0) {
	can0 = _can0;
	can0->reset();
	can0->setBitrate(CAN_1000KBPS);
	can0->setNormalMode();
}

void Communicator::set_interface(Interface* _interface) { interface = _interface; }

void Communicator::set_controller(void* _controller) { controller = _controller; }

void Communicator::send(struct can_frame* msg) {
	if (msg)
		can0->sendMessage(msg);
	else
		can0->sendMessage(&can_msg_tx);
}

bool Communicator::recv() {
	auto err = can0->readMessage(&can_msg_rx);
	auto flags = can0->getErrorFlags();
	if (err == MCP2515::ERROR_OK) {
		return true;
	} else if (flags & MCP2515::EFLG_RX0OVR) {
		can0->clearRXnOVR();
		Serial.println("RX0OVR");
	}
	return false;
}

void Communicator::send_syn() {
	can_msg_tx.can_id = id;
	can_msg_tx.can_dlc = 1;
	can_msg_tx.data[0] = 0;
	can0->sendMessage(&can_msg_tx);
}

uint8_t Communicator::get_recv_id() { return can_msg_rx.can_id; }

void send_ack(MCP2515* can0, uint8_t id) {
	struct can_frame can_msg_tx;
	can_msg_tx.can_id = id;
	can_msg_tx.can_dlc = 1;
	can_msg_tx.data[0] = 1;
	can0->sendMessage(&can_msg_tx);
}

void serial_convert_float(char c, int id, float data) {
	Serial.print(c);
	Serial.print(" ");
	Serial.print(id);
	Serial.print(" ");
	Serial.println(data);
}

void serial_convert_int(char c, int id, int data) {
	Serial.print(c);
	Serial.print(" ");
	Serial.print(id);
	Serial.print(" ");
	Serial.println(data);
}

void stream_float(char c, int id, float val, float time) {
	Serial.print("s ");
	Serial.print(c);
	Serial.print(" ");
	Serial.print(id);
	Serial.print(" ");
	Serial.print(val);
	Serial.print(" ");
	Serial.println(time);
}

void fullfill_msg(struct can_frame* msg, int id, int code, void* data) {
	msg->can_id = id;
	msg->data[0] = code;
	if (data) {
		msg->can_dlc = 5;
		memcpy(&msg->data[1], data, 4);
	} else {
		msg->can_dlc = 1;
	}
}

void Communicator::consensus_update() {
	Controller* ctrl = (Controller*)controller;
	float d;
	can_msg_tx.can_id = id;
	can_msg_tx.can_dlc = 6;
	can_msg_tx.data[0] = 100;

	for (int i = 0; i < interface->neighbour_count; i++) {
		can_msg_tx.data[1] = interface->neighbours[i];
		memcpy(&can_msg_tx.data[2], &d, 4);
		can_queue.push(can_msg_tx);
	}

	can_msg_tx.data[1] = id;
	memcpy(&can_msg_tx.data[2], &d, 4);
	can_queue.push(can_msg_tx);
}

void Communicator::process() {
	int aux_int;
	float aux_float;
	Controller* ctrl = (Controller*)controller;
	int i, j;

	// Serial.print(can_msg_rx.can_id);
	// Serial.print(" - ");
	// Serial.println(can_msg_rx.data[0]);

	if (can_msg_rx.data[0] > 1 && can_msg_rx.data[0] < 32 && can_msg_rx.can_id != id) {
		return;
	}

	switch (can_msg_rx.data[0]) {
		case 0:
			interface->push_neighbour(can_msg_rx.can_id);
			send_ack(can0, id);
			return;
		case 1:
			interface->push_neighbour(can_msg_rx.can_id);
			return;
		case 2:
			// interface->process("d " + String(can_msg_rx.can_id) + " " + String(*(float*)&can_msg_rx.data[2]));
			interface->status->setControllerOff();
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			interface->driver->write_duty_cycle(aux_float);
			ack();
			break;
		case 3:
			ack_duty(interface->driver->get_duty_cycle());
			break;
		case 4:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			interface->local_controller->set_ref(aux_float);
			interface->status->setControllerOn();
			ack();
			break;
		case 5:
			ack_ref(interface->local_controller->get_ref());
			break;
		case 6:
			ack_lux(interface->luxmeter->read());
			break;
		case 7:
			interface->status->setControllerOn();
			memcpy(&aux_int, &can_msg_rx.data[1], 4);
			if (aux_int)
				ctrl->set_occupied();
			else
				ctrl->set_unoccupied();
			consensus();
			ctrl->consensus_iterate();
			break;
		case 8:
			ack_occ(ctrl->is_occupied());
			break;
		case 9:
			memcpy(&aux_int, &can_msg_rx.data[1], 4);
			interface->local_controller->set_anti_windup(aux_int);
			ack();
			break;
		case 10:
			ack_anti(interface->local_controller->get_anti_windup());
			break;
		case 11:
			memcpy(&aux_int, &can_msg_rx.data[1], 4);
			interface->local_controller->set_feedback(aux_int);
			ack();
			break;
		case 12:
			ack_fb(interface->local_controller->get_feedback());
			break;
		case 13:
			ack_ext_lux(interface->luxmeter->read() - interface->driver->get_duty_cycle() * interface->local_controller->get_G());
			break;
		case 14:
			ack_power(interface->metrics->get_instantaneous_power());
			break;
		case 15:
			ack_time(interface->get_time());
			break;
		case 16:
			interface->status->setDutycycleLogOn();
			break;
		case 17:
			interface->status->setLuxmeterLogOn();
			break;
		case 18:
			interface->status->setDutycycleLogOff();
			ack();
			break;
		case 19:
			interface->status->setLuxmeterLogOff();
			ack();
			break;
		case 20:
			break;
		case 21:
			break;
		case 22:
			ack_energy(interface->metrics->get_energy());
			break;
		case 23:
			ack_visibility(interface->metrics->get_visibility_error());
			break;
		case 24:
			ack_flicker(interface->metrics->get_flicker());
			break;
		case 25:
			ack_o_bound(ctrl->get_occupied());
			break;
		case 26:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			ctrl->set_occupied_bound(aux_float);
			consensus();
			ctrl->consensus_iterate();
			break;
		case 27:
			ack_u_bound(ctrl->get_unoccupied());
			break;
		case 28:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			ctrl->set_unoccupied_bound(aux_float);
			consensus();
			ctrl->consensus_iterate();
			break;
		case 29:
			ack_l_bound(ctrl->get_L());
			break;
		case 30:
			ack_e_cost(ctrl->get_c(2));
			break;
		case 31:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			ctrl->set_c(2, aux_float);
			ack();
			break;

		case 254:
			Serial.println("ack");
			return;
		case 255:
			Serial.println("err");
			return;
		case 33:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			serial_convert_float('d', can_msg_rx.can_id, aux_float);
			return;
		case 35:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			serial_convert_float('r', can_msg_rx.can_id, aux_float);
			return;
		case 36:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			serial_convert_float('l', can_msg_rx.can_id, aux_float);
			return;
		case 38:
			memcpy(&aux_int, &can_msg_rx.data[1], 4);
			serial_convert_int('o', can_msg_rx.can_id, aux_int);
			return;
		case 40:
			memcpy(&aux_int, &can_msg_rx.data[1], 4);
			serial_convert_int('a', can_msg_rx.can_id, aux_int);
			return;
		case 42:
			memcpy(&aux_int, &can_msg_rx.data[1], 4);
			serial_convert_int('k', can_msg_rx.can_id, aux_int);
			return;
		case 43:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			serial_convert_float('x', can_msg_rx.can_id, aux_float);
			return;
		case 44:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			serial_convert_float('p', can_msg_rx.can_id, aux_float);
			return;
		case 45:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			serial_convert_float('t', can_msg_rx.can_id, aux_float);
			return;
		case 46:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			stream_float('d', can_msg_rx.can_id, aux_float, interface->get_time());
			return;
		case 47:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			stream_float('l', can_msg_rx.can_id, aux_float, interface->get_time());
			return;
		case 50:
			return;
		case 51:
			return;
		case 52:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			serial_convert_float('e', can_msg_rx.can_id, aux_float);
			return;
		case 53:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			serial_convert_float('v', can_msg_rx.can_id, aux_float);
			return;
		case 54:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			serial_convert_float('f', can_msg_rx.can_id, aux_float);
			return;
		case 55:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			serial_convert_float('O', can_msg_rx.can_id, aux_float);
			return;
		case 57:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			serial_convert_float('U', can_msg_rx.can_id, aux_float);
			return;
		case 59:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			serial_convert_float('L', can_msg_rx.can_id, aux_float);
			return;
		case 60:
			memcpy(&aux_float, &can_msg_rx.data[1], 4);
			serial_convert_float('c', can_msg_rx.can_id, aux_float);
			return;
		case 61:
			ctrl->measure_o();
			return;
		case 62:
			memcpy(&aux_int, &can_msg_rx.data[1], 4);
			i = aux_int == id ? 2 : interface->get_neighbour_index(aux_int);
			ctrl->measure_k(i);
			return;
		case 63:
			ctrl->calculate();
			return;
		case 64:
			ctrl->consensus_iterate();
			return;
		case 100:
			i = interface->get_neighbour_index(can_msg_rx.can_id);
			j = can_msg_rx.data[1] == id ? 2 : interface->get_neighbour_index(can_msg_rx.data[1]);
			memcpy(&aux_float, &can_msg_rx.data[2], 4);
			ctrl->update_d_recv(i, j, aux_float);
			return;
	}
	can_queue.push(can_msg_tx);
}

void Communicator::set_duty(int i, float data) { fullfill_msg(&can_msg_tx, i, 2, &data); }		// 2
void Communicator::get_duty(int i) { fullfill_msg(&can_msg_tx, i, 3, NULL); }					// 3
void Communicator::set_ref(int i, float data) { fullfill_msg(&can_msg_tx, i, 4, &data); }		// 4
void Communicator::get_ref(int i) { fullfill_msg(&can_msg_tx, i, 5, NULL); }					// 5
void Communicator::get_lux(int i) { fullfill_msg(&can_msg_tx, i, 6, NULL); }					// 6
void Communicator::set_occ(int i, int data) { fullfill_msg(&can_msg_tx, i, 7, &data); }			// 7
void Communicator::get_occ(int i) { fullfill_msg(&can_msg_tx, i, 8, NULL); }					// 8
void Communicator::set_anti(int i, int data) { fullfill_msg(&can_msg_tx, i, 9, &data); }		// 9
void Communicator::get_anti(int i) { fullfill_msg(&can_msg_tx, i, 10, NULL); }					// 10
void Communicator::set_fb(int i, int data) { fullfill_msg(&can_msg_tx, i, 11, &data); }			// 11
void Communicator::get_fb(int i) { fullfill_msg(&can_msg_tx, i, 12, NULL); }					// 12
void Communicator::get_ext_lux(int i) { fullfill_msg(&can_msg_tx, i, 13, NULL); }				// 13
void Communicator::get_power(int i) { fullfill_msg(&can_msg_tx, i, 14, NULL); }					// 14
void Communicator::get_time(int i) { fullfill_msg(&can_msg_tx, i, 15, NULL); }					// 15
void Communicator::start_duty_stream(int i) { fullfill_msg(&can_msg_tx, i, 16, NULL); }			// 16
void Communicator::start_lux_stream(int i) { fullfill_msg(&can_msg_tx, i, 17, NULL); }			// 17
void Communicator::stop_duty_stream(int i) { fullfill_msg(&can_msg_tx, i, 18, NULL); }			// 18
void Communicator::stop_lux_stream(int i) { fullfill_msg(&can_msg_tx, i, 19, NULL); }			// 19
void Communicator::get_last_min_duty(int i) { fullfill_msg(&can_msg_tx, i, 20, NULL); }			// 20
void Communicator::get_last_min_lux(int i) { fullfill_msg(&can_msg_tx, i, 21, NULL); }			// 21
void Communicator::get_energy(int i) { fullfill_msg(&can_msg_tx, i, 22, NULL); }				// 22
void Communicator::get_visibility(int i) { fullfill_msg(&can_msg_tx, i, 23, NULL); }			// 23
void Communicator::get_flicker(int i) { fullfill_msg(&can_msg_tx, i, 24, NULL); }				// 24
void Communicator::get_o_bound(int i) { fullfill_msg(&can_msg_tx, i, 25, NULL); }				// 25
void Communicator::set_o_bound(int i, float data) { fullfill_msg(&can_msg_tx, i, 26, &data); }	// 26
void Communicator::get_u_bound(int i) { fullfill_msg(&can_msg_tx, i, 27, NULL); }				// 27
void Communicator::set_u_bound(int i, float data) { fullfill_msg(&can_msg_tx, i, 28, &data); }	// 28
void Communicator::get_l_bound(int i) { fullfill_msg(&can_msg_tx, i, 29, NULL); }				// 29
void Communicator::get_e_cost(int i) { fullfill_msg(&can_msg_tx, i, 30, NULL); }				// 30
void Communicator::set_e_cost(int i, float data) { fullfill_msg(&can_msg_tx, i, 31, &data); }	// 31

void Communicator::ack() { fullfill_msg(&can_msg_tx, id, 254, NULL); }							// 254
void Communicator::err() { fullfill_msg(&can_msg_tx, id, 255, NULL); }							// 255
void Communicator::ack_duty(float data) { fullfill_msg(&can_msg_tx, id, 33, &data); }			// 33
void Communicator::ack_ref(float data) { fullfill_msg(&can_msg_tx, id, 35, &data); }			// 35
void Communicator::ack_lux(float data) { fullfill_msg(&can_msg_tx, id, 36, &data); }			// 36
void Communicator::ack_occ(int data) { fullfill_msg(&can_msg_tx, id, 38, &data); }				// 38
void Communicator::ack_anti(int data) { fullfill_msg(&can_msg_tx, id, 40, &data); }				// 40
void Communicator::ack_fb(int data) { fullfill_msg(&can_msg_tx, id, 42, &data); }				// 42
void Communicator::ack_ext_lux(float data) { fullfill_msg(&can_msg_tx, id, 43, &data); }		// 43
void Communicator::ack_power(float data) { fullfill_msg(&can_msg_tx, id, 44, &data); }			// 44
void Communicator::ack_time(float data) { fullfill_msg(&can_msg_tx, id, 45, &data); }			// 45
void Communicator::stream_duty(float data) { fullfill_msg(&can_msg_tx, id, 46, &data); }		// 46
void Communicator::stream_lux(float data) { fullfill_msg(&can_msg_tx, id, 47, &data); }			// 47
void Communicator::ack_last_min_duty(float data) { fullfill_msg(&can_msg_tx, id, 50, &data); }	// 50
void Communicator::ack_last_min_lux(float data) { fullfill_msg(&can_msg_tx, id, 51, &data); }	// 51
void Communicator::ack_energy(float data) { fullfill_msg(&can_msg_tx, id, 52, &data); }			// 52
void Communicator::ack_visibility(float data) { fullfill_msg(&can_msg_tx, id, 53, &data); }		// 53
void Communicator::ack_flicker(float data) { fullfill_msg(&can_msg_tx, id, 54, &data); }		// 54
void Communicator::ack_o_bound(float data) { fullfill_msg(&can_msg_tx, id, 55, &data); }		// 55
void Communicator::ack_u_bound(float data) { fullfill_msg(&can_msg_tx, id, 57, &data); }		// 57
void Communicator::ack_l_bound(float data) { fullfill_msg(&can_msg_tx, id, 59, &data); }		// 59
void Communicator::ack_e_cost(float data) { fullfill_msg(&can_msg_tx, id, 60, &data); }			// 60

void Communicator::measure_o() { fullfill_msg(&can_msg_tx, id, 61, NULL); }			  // 61
void Communicator::measure_k(int data) { fullfill_msg(&can_msg_tx, id, 62, &data); }  // 62
void Communicator::calculate() { fullfill_msg(&can_msg_tx, id, 63, NULL); }			  // 63
void Communicator::consensus() { fullfill_msg(&can_msg_tx, id, 64, NULL); }			  // 64
