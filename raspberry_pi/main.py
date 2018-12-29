#!/usr/bin/env python3
import numpy as np
import core
import os

tare_current = 0
bl_factor = 0
g = 0
current_step = 0

def main_menu():
    print('Operations:')
    print('-'*50)
    print('1. Position calibration')
    print('2. Motor adjustment')
    print('3. Gravity input')
    print('4. B/L constant calibration (velocity mode)')
    print('5. 0/Tare balance')
    print('6. Mass measurement (force mode)')
    print('-'*50)
    return input('Selection: ')

while(True):
    os.system('clear')
    print('INTRO\n')
    user_input = main_menu()

    if user_input == '1':
        # postion calibration
        core.display_tracker_box()

    elif user_input == '2':
        # motor adjustment
        steps = int(input('Steps: '))
        direction = input('Direction (forward/reverse): ')

        velocity = 60 # INPUT GOOD VALUE
        acceleration = 720 # INPUT GOOD VALUE

        if direction.lower() != 'forward' and direction.lower() != 'reverse':
            print('come on bro')

        else:
            _, _, current_step = core.motor_jog(current_step, velocity, acceleration,
                                                steps, direction)

    elif user_input == '3':
        # gravity input
        g = input('Total gravitational acceleration (m/s^2): ')
        g_err = input('Uncertainty in total gravitational acceleration (m/s^2): ')

    elif user_input == '4':
        # b/l constant calibration
        velocity, voltage, velocity_err, voltage_err, current_step = ...
        core.velocity_mode(current_step, acceleration, target_velocity,
                           buffer = 1, steps = 1600, runtime = 20)

        bl_factor, bl_factor_err = core.bl_factor_calc(velocity, voltage, velocity_err, voltage_err)

    elif user_input == '5':
        # tare balance
        tare_current, tare_current_err, current_step = core.force_mode(current_step, target_pixel)

    elif user_input == '6':
        # mass measurement
        if bl_factor == 0 or tare_current == 0 or g == 0:
            print('calibration not complete\n')

        else:
            current, current_err, current_step = core.force_mode(current_step, target_pixel)
            mass, mass_err = core.mass_calc(current, current_err, bl_factor,
                                            bl_factor_err, g, g_err, tare_current,
                                            tare_current_err)

            print('I = {:f} +/- {:f}\n'.format(current, current_err))
            print('B/L Factor = {:f} +/- {:f}\n'.format(bl_factor, bl_factor_err))
            print('g = {:f} +/- {:f}\n'.format(g, g_err))
            print('m = {:f} +/- {:f}\n'.format(mass, mass_err))

    else:
        print('make a valid selection\n')

    input("Press the <ENTER> key to continue...")
