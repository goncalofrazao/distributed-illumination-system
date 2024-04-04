#ifndef _COMMUNICATOR_HPP_
#define _COMMUNICATOR_HPP_

#include <Arduino.h>

#include "hardware/flash.h"
#include "interface.hpp"
#include "mcp2515.h"

void serial_convert_float(char c, int id, float data);
void serial_convert_int(char c, int id, int data);

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

	void set_duty(int i, float duty);		 // 2
	void get_duty(int i);					 // 3
	void set_ref(int i, float ref);			 // 4
	void get_ref(int i);					 // 5
	void get_lux(int i);					 // 6
	void set_occ(int i, int occ);			 // 7
	void get_occ(int i);					 // 8
	void set_anti(int i, int anti);			 // 9
	void get_anti(int i);					 // 10
	void set_fb(int i, int fb);				 // 11
	void get_fb(int i);						 // 12
	void get_ext_lux(int i);				 // 13
	void get_power(int i);					 // 14
	void get_time(int i);					 // 15
	void start_duty_stream(int i);			 // 16
	void start_lux_stream(int i);			 // 17
	void stop_duty_stream(int i);			 // 18
	void stop_lux_stream(int i);			 // 19
	void get_last_min_duty(int i);			 // 20
	void get_last_min_lux(int i);			 // 21
	void get_energy(int i);					 // 22
	void get_visibility(int i);				 // 23
	void get_flicker(int i);				 // 24
	void get_o_bound(int i);				 // 25
	void set_o_bound(int i, float o_bound);	 // 26
	void get_u_bound(int i);				 // 27
	void set_u_bound(int i, float u_bound);	 // 28
	void get_l_bound(int i);				 // 29
	void get_e_cost(int i);					 // 30
	void set_e_cost(int i, float e_cost);	 // 31

	void ack();									  // 254
	void err();									  // 255
	void ack_duty(float duty);					  // 33
	void ack_ref(float ref);					  // 35
	void ack_lux(float lux);					  // 36
	void ack_occ(int occ);						  // 38
	void ack_anti(int anti);					  // 40
	void ack_fb(int fb);						  // 42
	void ack_ext_lux(float ext_lux);			  // 43
	void ack_power(float power);				  // 44
	void ack_time(float time);					  // 45
	void stream_duty(float duty);				  // 46
	void stream_lux(float lux);					  // 47
	void ack_last_min_duty(float last_min_duty);  // 50
	void ack_last_min_lux(float last_min_lux);	  // 51
	void ack_energy(float energy);				  // 52
	void ack_visibility(float visibility);		  // 53
	void ack_flicker(float flicker);			  // 54
	void ack_o_bound(float o_bound);			  // 55
	void ack_u_bound(float u_bound);			  // 57
	void ack_l_bound(float l_bound);			  // 59
	void ack_e_cost(float e_cost);				  // 60

   private:
	uint8_t id;
	MCP2515 *can0;
	struct can_frame can_msg_rx;
	struct can_frame can_msg_tx;
	Interface *interface;
};

#endif