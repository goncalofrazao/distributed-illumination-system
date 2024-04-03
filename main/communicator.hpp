#ifndef _COMMUNICATOR_HPP_
#define _COMMUNICATOR_HPP_

#include <Arduino.h>

#include "hardware/flash.h"
#include "mcp2515.h"

class Communicator {
   public:
	Communicator();
	void set_can0(MCP2515 *_can0);
	void set_id();
	uint8_t get_id();
	void send(struct can_frame *msg);
	bool recv();
	void send_id();
	uint8_t get_recv_id();

   private:
	uint8_t id;
	MCP2515 *can0;
	struct can_frame can_msg_rx;
	struct can_frame can_msg_tx;
};

#endif