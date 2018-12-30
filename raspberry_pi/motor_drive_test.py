import numpy as np
from gpiozero import DigitalOutputDevice
import time
import os

def motor_jog(current_step, velocity, acceleration, steps, direction): # velocity in steps/s, acceleration in steps/s^2
    # defines pin numbers
    pulse_pin     = DigitalOutputDevice(24)
    direction_pin = DigitalOutputDevice(25)
    enable_pin    = DigitalOutputDevice(22)
    
    # sets correct direction pin voltage and enables the enable pin
    enable_pin.on()
    if direction   == 'f':
        direction_pin.off()
        multiplier = 1
    elif direction == 'r':
        direction_pin.on()
        multiplier = -1
    
    # creates loop variables
    target_step = current_step + multiplier * steps
    jog_steps = []
    jog_times = []
    current_velocity = 100
    initial_step = current_step
    acceleration_steps = velocity**2/(2*acceleration) # approx. num. of steps taken during acceleration
    current_time = time.perf_counter()
    
    #steps forward at the appropriate time
    while current_step != target_step:
        #turn pulse pin on and get current time
        last_time = current_time
        current_time = time.perf_counter()
        pulse_pin.on()
        dt = current_time - last_time
        
        ## calculate delay time to next step
        half_over = np.absolute(current_step - initial_step) > np.absolute(target_step - current_step) # are we more than half done travelling?
        # if we are still accelerating, increase current_velocity
        if (current_velocity != velocity and not half_over):
            current_velocity = min(velocity, current_velocity + acceleration * max(dt,1E-7))
        # if we are decelerating, decrease current_velocity
        elif (np.absolute(target_step - current_step) < acceleration_steps != velocity and half_over):
            current_velocity = max(100, current_velocity - acceleration * dt)
        
        # each step takes 1/current_velocity time (in s)
        delay = 1/current_velocity
        
        # save values
        jog_times.append(current_time) # save current time
        current_step = current_step + multiplier # calculate current step
        jog_steps.append(current_step) # save current step
        
        # WAIT to turn pulse pin off, then wait again
        while delay/2 > (time.perf_counter() - current_time):
            pass
        # turn pulse pin off, WAIT
        pulse_pin.off()
        while delay > (time.perf_counter() - current_time):
            pass
        
    # done with movement, so we turn the enable pin off
    enable_pin.off()
            
    return jog_steps, jog_times, current_step


# run control
current_step = 0
while(True):
    # ask for step number and direction
    os.system('clear')
    steps = int(input('Steps: '))
    direction = input('Direction (f/r): ')

    # sets acceleration and velocities for motor
    velocity = 3200 # INPUT GOOD VALUE
    acceleration = 2000 # INPUT GOOD VALUE

    #filters bad choices
    if direction.lower() != 'f' and direction.lower() != 'r':
        print('come on bro')

    # moves motor
    else:
        jog_steps, jog_times, current_step = motor_jog(current_step, velocity, acceleration,
                                            steps, direction)
