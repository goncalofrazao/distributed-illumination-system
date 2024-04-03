#include "communicator.hpp"

Communicator::Communicator() {}

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

void Communicator::send(struct can_frame* msg) {
	can0->sendMessage(msg);
	return;
}

bool Communicator::recv() {
	auto err = can0->readMessage(&can_msg_rx);
	if (err == MCP2515::ERROR_OK) {
		return true;
	} else {
		return false;
	}
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

void Communicator::process() {
	if (can_msg_rx.can_dlc == 1 && can_msg_rx.data[0] == 0) {
		interface->push_neighbour(can_msg_rx.can_id);
		send_ack(can0, id);
	} else if (can_msg_rx.can_dlc == 1 && can_msg_rx.data[0] == 1) {
		interface->push_neighbour(can_msg_rx.can_id);
	}
}
