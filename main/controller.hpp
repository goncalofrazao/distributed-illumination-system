#ifndef _CONTROLLER_HPP_
#define _CONTROLLER_HPP_

#include <vector>

#include "communicator.hpp"
#include "driver.hpp"
#include "interface.hpp"

#define N 3
#define ITER 70

class Controller {
	float d[N][N];
	float k[N];
	float d_av[N];
	float lambda[N];
	float c[N];
	float n;
	float m;
	float o;
	float L;
	float rho;

	float occupied;
	float unoccupied;

	Communicator *communicator;
	Interface *interface;

   public:
	int iter;
	int count_recvs;

	Controller();
	~Controller() = default;

	void set_communicator(Communicator *communicator);
	void set_interface(Interface *interface);

	void set_c(int pos, float value);
	float get_c(int pos);
	float get_d(int pos);
	void set_L(float value);
	float get_L();
	float get_occupied();
	float get_unoccupied();
	void set_rho(float value);
	int is_occupied();

	void set_occupied_bound(float value);
	void set_unoccupied_bound(float value);

	void set_occupied();
	void set_unoccupied();

	void measure_o();
	void measure_k(int i);
	void calculate_n();
	void calculate_m();
	void calculate();

	void consensus();
	void consensus_iterate();

	void update_d_recv(int i, int j, float value);
	float new_ref();

	void calib();

	void log();
};

#endif