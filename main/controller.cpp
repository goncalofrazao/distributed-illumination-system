#include "controller.hpp"

#include <cfloat>

float dot_product(float a[], float b[]) {
	float value = 0;
	for (int i = 0; i < N; ++i) {
		value += a[i] * b[i];
	}
	return value;
}

Controller::Controller() {
	count_recvs = 0;
	iter = 0;
	for (int i = 0; i < N; i++) {
		d[2][i] = 0;
		d_av[i] = 0;
		lambda[i] = 0;
		c[i] = 0;
	}
	c[2] = MAX_POWER;
	rho = 0.1;
}

void Controller::set_communicator(Communicator* communicator) { this->communicator = communicator; }
void Controller::set_interface(Interface* interface) { this->interface = interface; }

void Controller::set_c(int pos, float value) { c[pos] = value; }
float Controller::get_c(int pos) { return c[pos]; }
float Controller::get_d(int pos) { return d[2][pos]; }
void Controller::set_L(float value) { L = value; }
float Controller::get_L() { return L; }
float Controller::get_occupied() { return occupied; }
float Controller::get_unoccupied() { return unoccupied; }
void Controller::set_rho(float value) { rho = value; }

void Controller::set_occupied_bound(float value) { occupied = value; }
void Controller::set_unoccupied_bound(float value) { unoccupied = value; }

void Controller::set_occupied() { L = occupied; }
void Controller::set_unoccupied() { L = unoccupied; }

void Controller::measure_o() { o = interface->luxmeter->read(); }
void Controller::measure_k(int i) { k[i] = interface->luxmeter->read() - o; }
void Controller::calculate_n() { n = dot_product(k, k); }
void Controller::calculate_m() { m = n - k[2] * k[2]; }

void Controller::calculate() {
	calculate_n();
	calculate_m();

	occupied = 0.85 * (k[2] + o);
	unoccupied = 0.15 * (k[2] + o);

	set_unoccupied();
}

bool is_feasible(float d[], float k[], float L, float o) {
	float tol = 0.001;
	if (d[2] < 0 - tol || d[2] > 100 + tol || dot_product(d, k) < L - o - tol) {
		return false;
	}
	return true;
}

float compute_cost(float d[], float c[], float lambda[], float d_av[], float rho) {
	float diff[N];
	for (int i = 0; i < N; i++) {
		diff[i] = d[i] - d_av[i];
	}

	float cd = dot_product(c, d);
	float lambda_diff = dot_product(lambda, d);
	float diff_diff = dot_product(diff, diff);

	return cd + lambda_diff + rho / 2 * diff_diff;
}

void unconstrained(float d_u[], float y[], float rho) {
	for (int i = 0; i < N; i++) {
		d_u[i] = y[i] / rho;
	}
}

void ILB(float d_bl[], float y[], float k[], float L, float rho, float o, float n) {
	float yk = dot_product(y, k);
	for (int i = 0; i < N; i++) {
		d_bl[i] = y[i] / rho - k[i] / n * (o - L + yk / rho);
	}
}

void DLB(float d_b0[], float d_u[]) {
	for (int i = 0; i < N; i++) {
		d_b0[i] = d_u[i];
	}
	d_b0[2] = 0;
}

void DUB(float d_b1[], float d_u[]) {
	for (int i = 0; i < N; i++) {
		d_b1[i] = d_u[i];
	}
	d_b1[2] = 100;
}

void ILB_DLB(float d_l0[], float y[], float k[], float m, float o, float L, float rho) {
	float yk = dot_product(y, k);
	for (int i = 0; i < N; i++) {
		d_l0[i] = y[i] / rho - k[i] / m * (o - L) + k[i] / rho / m * (k[2] * y[2] - yk);
	}
	d_l0[2] = 0;
}

void ILB_DUB(float d_l1[], float y[], float k[], float m, float o, float L, float rho) {
	float yk = dot_product(y, k);
	for (int i = 0; i < N; i++) {
		d_l1[i] = y[i] / rho - k[i] / m * (o - L + 100 * k[2]) + k[i] / rho / m * (k[2] * y[2] - yk);
	}
	d_l1[2] = 100;
}

void _y(float y[], float d_av[], float rho, float c[], float lambda[]) {
	for (int i = 0; i < N; ++i) {
		y[i] = rho * d_av[i] - c[i] - lambda[i];
	}
}

void _lambda(float lambda[], float rho, float d[], float d_av[]) {
	for (int i = 0; i < N; ++i) {
		lambda[i] = lambda[i] + rho * (d[i] - d_av[i]);
	}
}

void Controller::consensus() {
	float d_u[N];

	float d_l[N];
	float l_cost;

	float d_b0[N];
	float b0_cost;

	float d_b1[N];
	float b1_cost;

	float d_l0[N];
	float l0_cost;

	float d_l1[N];
	float l1_cost;

	float y[N];
	float* d_best;
	float best_cost = FLT_MAX;

	_y(y, d_av, rho, c, lambda);

	unconstrained(d_u, y, rho);
	if (is_feasible(d_u, k, L, o)) {
		best_cost = compute_cost(d_u, c, lambda, d_av, rho);
		d_best = d_u;
	}

	ILB(d_l, y, k, L, rho, o, n);
	if (is_feasible(d_l, k, L, o)) {
		l_cost = compute_cost(d_l, c, lambda, d_av, rho);
		if (l_cost < best_cost) {
			best_cost = l_cost;
			d_best = d_l;
		}
	}

	DLB(d_b0, d_u);
	if (is_feasible(d_b0, k, L, o)) {
		b0_cost = compute_cost(d_b0, c, lambda, d_av, rho);
		if (b0_cost < best_cost) {
			best_cost = b0_cost;
			d_best = d_b0;
		}
	}

	DUB(d_b1, d_u);
	if (is_feasible(d_b1, k, L, o)) {
		b1_cost = compute_cost(d_b1, c, lambda, d_av, rho);
		if (b1_cost < best_cost) {
			best_cost = b1_cost;
			d_best = d_b1;
		}
	}

	ILB_DLB(d_l0, y, k, m, o, L, rho);
	if (is_feasible(d_l0, k, L, o)) {
		l0_cost = compute_cost(d_l0, c, lambda, d_av, rho);
		if (l0_cost < best_cost) {
			best_cost = l0_cost;
			d_best = d_l0;
		}
	}

	ILB_DUB(d_l1, y, k, m, o, L, rho);
	if (is_feasible(d_l1, k, L, o)) {
		l1_cost = compute_cost(d_l1, c, lambda, d_av, rho);
		if (l1_cost < best_cost) {
			best_cost = l1_cost;
			d_best = d_l1;
		}
	}

	for (int i = 0; i < N; ++i) {
		d[2][i] = d_best[i];
	}
}

void update_average(float d_av[], float d[][N]) {
	for (int j = 0; j < N; j++) {
		d_av[j] = 0;
		for (int i = 0; i < N; i++) {
			d_av[j] += d[i][j];
		}
		d_av[j] /= N;
	}
}

void Controller::consensus_iterate() {
	iter++;
	update_average(d_av, d);
	_lambda(lambda, rho, d[2], d_av);

	consensus();

	Serial.print(iter);
	Serial.print(" : d : ");
	for (int i = 0; i < N; i++) {
		Serial.print(d[2][i]);
		Serial.print(" ");
	}
	Serial.println();

	if (iter >= ITER) {
		iter = 0;
		interface->local_controller->set_ref(new_ref());
		interface->driver->write_duty_cycle(d[2][2]);
		interface->status->setControllerOn();
	} else {
		communicator->consensus_update();
	}
}

void Controller::update_d_recv(int i, int j, float value) {
	d[i][j] = value;
	count_recvs++;
	if (count_recvs == 2 * N) {
		count_recvs = 0;
		consensus_iterate();
	}
}

float Controller::new_ref() {
	float ref = 0;
	for (int i = 0; i < N; i++) {
		ref += d[2][i] * k[i];
	}
	ref += o;
	return ref;
}

void Controller::calib() {
	int neighbours[2] = {interface->neighbours[0], interface->neighbours[1]};

	// get o
	interface->driver->write_duty_cycle(0);
	communicator->set_duty(neighbours[0], 0);
	communicator->send(NULL);
	communicator->set_duty(neighbours[1], 0);
	communicator->send(NULL);
	delay(1000);
	communicator->measure_o();
	communicator->send(NULL);
	measure_o();
	delay(1000);

	// get k0
	communicator->set_duty(neighbours[0], 1);
	communicator->send(NULL);
	delay(1000);
	communicator->measure_k(neighbours[0]);
	communicator->send(NULL);
	measure_k(0);
	delay(1000);
	communicator->set_duty(neighbours[0], 0);
	communicator->send(NULL);

	// get k1
	communicator->set_duty(neighbours[1], 1);
	communicator->send(NULL);
	delay(1000);
	communicator->measure_k(neighbours[1]);
	communicator->send(NULL);
	measure_k(1);
	delay(1000);
	communicator->set_duty(neighbours[1], 0);
	communicator->send(NULL);

	// get k2
	interface->driver->write_duty_cycle(1);
	delay(1000);
	communicator->measure_k(communicator->get_id());
	communicator->send(NULL);
	measure_k(2);
	delay(1000);
	interface->driver->write_duty_cycle(0);

	communicator->calculate();
	communicator->send(NULL);
	calculate();
}

void Controller::log() {
	Serial.print("k0: ");
	for (int i = 0; i < N; i++) {
		Serial.print(k[i]);
		Serial.print(" ");
	}
	Serial.println();

	Serial.print("o: ");
	Serial.println(o);

	Serial.print("L: ");
	Serial.println(L);
}
