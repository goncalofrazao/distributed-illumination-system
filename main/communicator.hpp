#ifndef _COMMUNICATOR_HPP_
#define _COMMUNICATOR_HPP_

#include <Arduino.h>

#include "hardware/flash.h"
#include "interface.hpp"
#include "mcp2515.h"

class Communicator {
   public:
	Communicator();

	void set_id();
	uint8_t get_id();

	void set_can0(MCP2515 *_can0);
	void set_interface(Interface *_interface);

	void send(struct can_frame *msg);
	bool recv();
	void send_syn();
	uint8_t get_recv_id();

	void process();

   private:
	uint8_t id;
	MCP2515 *can0;
	struct can_frame can_msg_rx;
	struct can_frame can_msg_tx;
	Interface *interface;
};

#endif