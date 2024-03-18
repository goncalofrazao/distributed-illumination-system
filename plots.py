import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np

class Data:
    def __init__(self):
        self.jitter = []
        self.control_time = []
        self.G = 0
        self.ref = []
        self.lux = []
        self.u_duty_cycle = []
        self.v_duty_cycle = []
        self.integral_term = []
        self.propotional_term = []
        self.energy = []
        self.visibility_error = []
        self.flicker = []
        self.power = []
        self.external_lux = []
        self.time = []


    def push(self, arg):
        self.jitter.append(int(arg[0]))
        self.control_time.append(int(arg[1]))
        self.G = float(arg[2])
        self.ref.append(float(arg[3]))
        self.lux.append(float(arg[4]))
        self.u_duty_cycle.append(float(arg[5]))
        self.v_duty_cycle.append(float(arg[6]))
        self.integral_term.append(float(arg[7]))
        self.propotional_term.append(float(arg[8]))
        self.energy.append(float(arg[9]))
        self.visibility_error.append(float(arg[10]))
        self.flicker.append(float(arg[11]))
        self.power.append(float(arg[12]))
        self.external_lux.append(float(arg[13]))
        self.time.append(float(arg[14]))

    def scale_time(self):
        self.time = [x - self.time[0] for x in self.time]

    def __str__(self):
        return f'{self.jitter[-1]},{self.control_time[-1]},{self.G},{self.ref[-1]},{self.lux[-1]},{self.u_duty_cycle[-1]},{self.v_duty_cycle[-1]},{self.integral_term[-1]},{self.propotional_term[-1]},{self.energy[-1]},{self.visibility_error[-1]},{self.flicker[-1]},{self.power[-1]},{self.external_lux[-1]},{self.time[-1]}'

data = Data()

SIMULATION = 4

def read_from_serial(port, baudrate):
    ser = serial.Serial(port, baudrate)
    i = 0

    ser.write('log on'.encode('utf-8'))
    print(ser.readline())

    while True:
        if SIMULATION == 0:
            if i == 0:
                ser.write('r 1 0'.encode('utf-8'))
                print(ser.readline())
            if i == 400:
                ser.write('r 1 90'.encode('utf-8'))
                print(ser.readline())
            if i == 800:
                ser.write('r 1 10'.encode('utf-8'))
                print(ser.readline())
            if i == 1200:
                ser.write('r 1 80'.encode('utf-8'))
                print(ser.readline())
            if i == 1600:
                ser.write('r 1 20'.encode('utf-8'))
                print(ser.readline())
            if i == 2000:
                ser.write(f'r 1 {int(data.G)}'.encode('utf-8'))
                print(ser.readline())
            if i == 2400:
                break

        if SIMULATION == 1:
            if i == 0:
                ser.write('r 1 90'.encode('utf-8'))
                print(ser.readline())
            if i == 1500:
                ser.write('a 1 0'.encode('utf-8'))
                print(ser.readline())
            if i == 3000:
                ser.write('a 1 1'.encode('utf-8'))
                print(ser.readline())
                break

        if SIMULATION == 2:
            if i == 1500:
                break

        if SIMULATION == 3:
            if i == 0:
                ser.write('k 1 0'.encode('utf-8'))
                ser.write('r 1 20'.encode('utf-8'))
                print(ser.readline())

            if i == 300:
                ser.write('r 1 40'.encode('utf-8'))
                print(ser.readline())

            if i == 600:
                ser.write('r 1 60'.encode('utf-8'))
                print(ser.readline())

            if i == 900:
                ser.write('k 1 1'.encode('utf-8'))
                ser.write('r 1 20'.encode('utf-8'))
                print(ser.readline())

            if i == 1200:
                ser.write('r 1 40'.encode('utf-8'))
                print(ser.readline())

            if i == 1500:
                ser.write('r 1 60'.encode('utf-8'))
                print(ser.readline())

            if i == 1800:
                break

        if SIMULATION == 4:
            if i == 0:
                ser.write('r 1 50'.encode('utf-8'))
                print(ser.readline())
            if i == 750:
                ser.write('r 1 10'.encode('utf-8'))
                print(ser.readline())
            if i == 1500:
                break
            
        arguments = ser.readline().decode('utf-8').strip().split(' ')
        data.push(arguments)
        # print(data)

        i += 1

    ser.write('r 1 0'.encode('utf-8'))
    print(ser.readline())

def plot_ref_lux():
    # plot data.ref and data.lux in the same graph in function of time
    fig, ax = plt.subplots()
    ax.plot(data.time, data.ref, label='ref (lux)')
    ax.plot(data.time, data.lux, label='y (lux)')
    ax.legend()
    ax.set_xlabel('time (s)')
    ax.set_ylabel('illuminance (lux)')
    plt.show()

def plot_integral_term():
    # plot data.integral_term in function of time
    fig, ax = plt.subplots()
    ax.plot(data.time, data.integral_term, label='integral_term')
    ax.legend()
    ax.set_xlabel('time (s)')
    ax.set_ylabel('integral_term')
    plt.show()

def plot_ref_lux_external_lux():
    # plot data.ref, data.lux and data.external_duty_cycle in the same graph in function of time
    fig, ax = plt.subplots()
    ax.plot(data.time, data.ref, label='ref (lux)')
    ax.plot(data.time, data.lux, label='y (lux)')
    ax.plot(data.time, data.external_lux, label='external_luminance (lux)')
    ax.legend()
    ax.set_xlabel('time (s)')
    ax.set_ylabel('illuminance (lux)')
    plt.show()

def plot_jitter():
    # plot data.jitter in function of time
    fig, ax = plt.subplots()
    ax.plot(data.time, data.jitter, label='jitter (us)')
    ax.legend()
    ax.set_xlabel('time (s)')
    ax.set_ylabel('jitter (us)')
    plt.show()

def plot_flicker():
    # plot data.flicker in function of time
    fig, ax = plt.subplots()
    ax.plot(data.time, data.flicker, label='flicker (1/s)')
    ax.legend()
    ax.set_xlabel('time (s)')
    ax.set_ylabel('flicker (1/s)')
    plt.show()

if __name__ == "__main__":
    read_from_serial('/dev/tty.usbmodem112201', 115200)
    data.scale_time()
    if SIMULATION == 0:
        plot_ref_lux()

    if SIMULATION == 1:
        plot_ref_lux_external_lux()
        plot_integral_term()

    if SIMULATION == 2:
        plot_jitter()

    if SIMULATION == 3:
        plot_ref_lux()
        plot_integral_term()

    if SIMULATION == 4:
        plot_flicker()

