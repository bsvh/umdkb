import numpy as np
import gpiozero
import cv2
import threading
import time

class CameraThread(threading.Thread):
    def __init__(self, runtime = 20):
        super(CameraThread, self).__init__()
        self.postion = []
        self.timestamp = []
        self.runtime = runtime
        self.np_position = np.empty()
    def run(self):
        while(time.process_time() < self.start_time + self.runtime):
            self.position.append(get_camera_postion())
            self.timestamp.append(time.process_time())
        self.np_position = np.array([self.position,self.timestamp])

class CoilThread(threading.Thread):
    def __init__(self, runtime = 20):
        super(CoilThread, self).__init__()
        self.voltage = []
        self.timestamp = []
        self.runtime = runtime
        self.np_voltage = np.empty()
    def run(self):
        while(time.process_time() < self.start_time + self.runtime):
            self.voltage.append(get_coil_voltage())
            self.timestamp.append(time.process_time())
        self.np_voltage = np.array([self.voltage,self.timestamp])

class MotorThread(threading.Thread):
    def __init__(self, current_step, acceleration, velocity, buffer = 1, steps = 1600, runtime = 20):
        super(MotorThread, self).__init__()
        self.current_step = current_step
        self.acceleration = acceleration
        self.velocity = velocity
        self.steps = steps
        self.buffer = buffer
        self.step_list = []
        self.timestamp = []
        self.runtime = runtime
        self.np_steps = np.empty()
    def run(self):
        delay = (1/self.velocity) * 360/6400 * steps

        self.step_list.append(self.current_step)
        self.timestamp.append(time.process_time())

        time.sleep(self.buffer)
        jog_steps, jog_times = motor_jog(current_step, delay, steps, 'forward', acceleration)
        self.step_list.extend(jog_steps)
        self.timestamp.extend(jog_times)

        self.current_step = step_list[-1]

        self.step_list.append(self.current_step)
        self.timestamp.append(time.process_time())

        time.sleep(self.buffer)
        jog_steps, jog_times = motor_jog(current_step, delay, steps, 'reverse', acceleration)
        self.step_list.extend(jog_steps)
        self.timestamp.extend(jog_times)

        self.current_step = step_list[-1]

        self.step_list.append(self.current_step)
        self.timestamp.append(time.process_time())

        self.np_steps = np.array([self.step_list,self.timestamp])

def velocity_mode(current_step, acceleration, target_velocity, buffer = 1, steps = 1600, runtime = 20):
    motor  = MotorThread(current_step, acceleration, target_velocity, buffer, steps, runtime)
    coil   = CoilThread(runtime)
    camera = CameraThread(runtime)

    for thread in motor, coil, camera:
        thread.start()
    for thread in motor, coil, camera:
        thread.join()

    steps = motor.np_steps # 2xM numpy array, [step positions, times]
    # includes one upward and one downward movement with a pause in the beginning and in between

    pixel_positions = camera.np_position # 2xN numpy array, [pixel positions, times]
    voltages        = coil.np_voltage # 2xA numpy array, [voltages, times]

    # PUT VELOCITY AND VOLTAGE CALCULATION CODE HERE

def motor_jog(delay, steps, direction, acceleration):
    jog_steps = []
    jog_times = []
    return jog_steps, jog_times

def get_camera_postion():
    pixel_position = 0
    return pixel_position

def get_coil_voltage():
    coil_voltage = 0
    return coil_voltage
