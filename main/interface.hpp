#ifndef _INTERFACE_HPP_
#define _INTERFACE_HPP_

#include <Arduino.h>

#include "driver.hpp"
#include "local_controller.hpp"
#include "luxmeter.hpp"
#include "metrics.hpp"

class Status {
	bool _controllerOn;
	bool _logOn;
	bool _luxmeterLogOn;
	bool _dutycycleLogOn;

   public:
	Status();
	bool controllerOn();
	bool logOn();
	bool luxmeterLogOn();
	bool dutycycleLogOn();

	void setControllerOn();
	void setControllerOff();
	void setLogOn();
	void setLogOff();
	void setLuxmeterLogOn();
	void setLuxmeterLogOff();
	void setDutycycleLogOn();
	void setDutycycleLogOff();
};

class Interface {
	int id;
	LocalController *local_controller;
	Driver *driver;
	Luxmeter *luxmeter;
	Metrics *metrics;
	Status *status;
	void *communicator;
	int restart_time;

	int neighbours[2];
	int neighbour_count;

   public:
	Interface(LocalController *local_controller, Driver *driver, Luxmeter *Luxmeter, Metrics *metrics, Status *status, void *communicator);
	void set_id(int _id);
	bool available();
	void process(String command);
	float get_time();

	int *get_neighbours(int *n);
	void push_neighbour(int id);
	void print_neighbours();
};

#endif