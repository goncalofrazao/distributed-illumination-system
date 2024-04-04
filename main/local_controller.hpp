#ifndef _LOCAL_CONTROLLER_HPP_
#define _LOCAL_CONTROLLER_HPP_

#include "driver.hpp"
#include "luxmeter.hpp"
#include "metrics.hpp"

class LocalController {
	float P, I, D;
	float K, Ti, Td, Tt;
	float b, h, y_old, N;
	float bi, ad, bd, ao;
	float u, v, y;
	float ref;
	float G;
	bool anti_windup, feedback;

	float gains[3][3];

	Driver *driver;
	Luxmeter *luxmeter;
	Metrics *metrics;
	void *communicator;

   public:
	explicit LocalController(float _h, float _K = 1, float b_ = 1, float _Ti = 1, float _Td = 0, float _Tt = 1, float N_ = 10);
	~LocalController() = default;

	void set_feedback(bool feedback);
	bool get_feedback();

	void set_anti_windup(bool anti_windup);
	bool get_anti_windup();

	void set_ref(float ref);
	float get_ref();

	void set_occupancy(int occupancy);
	int get_occupancy();

	float get_G();

	void set_driver(Driver *driver);
	void set_luxmeter(Luxmeter *luxmeter);
	void set_metrics(Metrics *metrics);
	void set_communicator(void *communicator);

	void control(float y);

	void set_ideal_gains(float x);
	void calibrate();
	void log();
};

#endif