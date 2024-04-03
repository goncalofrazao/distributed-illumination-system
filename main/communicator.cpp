#include "communicator.hpp"

Communicator::Communicator() {}

void Communicator::set_id(uint8_t _id) { id = _id; }

uint8_t Communicator::get_id() { return id; }

void Communicator::set_can0(MCP2515* _can0) {
	can0 = _can0;
	can0->reset();
	can0->setBitrate(CAN_1000KBPS);
	can0->setNormalMode();
}

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

void Communicator::send_id() {
	can_msg_tx.can_id = id;
	can_msg_tx.can_dlc = 1;
	can_msg_tx.data[0] = 0x00;
	can0->sendMessage(&can_msg_tx);
}

uint8_t Communicator::get_recv_id() { return can_msg_rx.can_id; }
