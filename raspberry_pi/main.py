#!/usr/bin/env python3
import numpy as np
import core
import os
import cv2
import time
import dac8552

# file save locations
tare_current_filename = './calibration_values/tare_current.npy'
bl_factor_filename = './calibration_values/bl_factor.npy'
g_filename = './calibration_values/g_value.npy'
target_step_filename = './calibration_values/target_step.npy'
calibration_filename = './calibration_values/calibration_file.npy'

def main_menu():
    print('Operations:')
    print('-'*50)
    print('1. Position calibration')
    print('2. Motor adjustment')
    print('3. View Camera')
    print('4. Gravity input')
    print('5. B/L constant calibration (velocity mode)')
    print('6. 0/Tare balance')
    print('7. Mass measurement (force mode)')
    print('-'*50)
    return input('Selection: ')

try:
    # start the DAC
    dac = dac8552.DAC8552()
    while False:
        core.set_DAC_voltage(dac,3.3)
        time.sleep(1)
        core.set_DAC_voltage(dac,0)
        time.sleep(1)
    # open camera and set properties
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280);
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720);

    # load initial values
    tare_current = np.load(tare_current_filename)
    bl_factor = np.load(bl_factor_filename)
    g = np.load(g_filename)
    target_step = np.load(target_step_filename)
    calibration_array = np.load(calibration_filename)
    lower_step = calibration_array[0,0]
    upper_step = calibration_array[0,-1]
    lower_limit = int(calibration_array[1,0])
    upper_limit = int(calibration_array[1,-1])
    current_pixel = core.get_camera_position(cap)
    current_step = int(core.pixel_to_step(current_pixel, calibration_filename))
    
    #run watt balance
    while(True):
        os.system('clear')
        print('INTRO\n')
        user_input = main_menu()

        if user_input == '1':
            # postion calibration
            # align camera
            os.system('clear')
            print("First, align the rod with black tape with the red box shown on the screen,")
            print('ensuring that the tape is within the red box.')
            core.display_tracker_box(cap)

            #set upper and lower pixel limits
            limits_set = 0

            while limits_set == 0:
                os.system('clear')
                print('Now, enter values for the upper pixel limit (shown in black)')
                print('and the lower pixel limit (shown in blue). Limits must fall')
                print('between 1 and 719. 719 is the BOTTOM of the frame.\n')
                upper_limit = input('Upper pixel limit (minimum 1, default 230): ')
                if not upper_limit:
                    upper_limit = '230'
                upper_limit = int(upper_limit)
                lower_limit = input('Lower pixel limit (maximum 720, default 480): ')
                if not lower_limit:
                    lower_limit = '480'
                lower_limit = int(lower_limit)
                core.display_tracker_box(cap, limits = [lower_limit,upper_limit])
                user_input = input('Continue adjusting limits? (y/N)')
                if user_input.lower() == 'n' or not user_input.lower():
                    limits_set = 1

            # jogs motor to upper position and prompts for height
            os.system('clear')
            print('Jogging motor to upper limit pixel...')
            current_step = core.jog_to_pixel(cap, current_step, upper_limit, show_image = True)
            upper_step = current_step
            os.system('clear')
            upper_limit_height = int(input('Please enter mass pan height above base in mm: '))
            upper_limit_height_err = int(input('and the uncertainty in mm: '))

            # jogs motor to lower position and prompts for height
            os.system('clear')
            print('Jogging motor to lower limit pixel...')
            current_step = core.jog_to_pixel(cap, current_step, lower_limit, show_image = True)
            lower_step = current_step
            os.system('clear')
            lower_limit_height = int(input('Please enter mass pan height above base in mm: '))
            lower_limit_height_err = int(input('and the uncertainty in mm: '))

            # creates calibration file
            os.system('clear')
            print('Creating calibration file, please wait...')
            core.create_calibration_file(cap, current_step, calibration_filename,
                                          [lower_step,upper_step],
                                          [lower_limit,upper_limit],
                                          [lower_limit_height,upper_limit_height])
            callib_array = np.load(calibration_filename)
            print(callib_array)


        elif user_input == '2':
            # motor adjustment
            # ask for step number and direction
            os.system('clear')
            steps = int(input('Steps (3200 steps per revolution, SIGNED): '))

            # sets acceleration and velocities for motor
            velocity = 3200 # INPUT GOOD VALUE
            acceleration = 2000 # INPUT GOOD VALUE

            # moves motor
            jog_steps, jog_times, current_step = core.motor_jog(current_step, velocity, acceleration, steps)

        elif user_input == '3':
            # display camera
            core.display_tracker_box(cap, limits = [lower_limit,upper_limit])
            
        elif user_input == '4':
            # gravity input
            g[0] = float(input('Total gravitational acceleration (m/s^2): '))
            g[1] = float(input('Uncertainty in total gravitational acceleration (m/s^2): '))
            np.save(g_filename,g)

        elif user_input == '5':
            # b/l constant calibration
            acceleration = 4000
            target_velocity = 1000
            step_limits = [lower_step, upper_step]
            bl_factor, current_step, target_step = \
            core.velocity_mode(cap, current_step, step_limits, acceleration, target_velocity,
                               calibration_filename, buffer = 1)
            np.save(bl_factor_filename, bl_factor)
            np.save(target_step_filename, target_step)

        elif user_input == '6':
            # tare balance
            tare_current, current_step = core.force_mode(dac, cap, current_step, target_step, calibration_filename)
            np.save(tare_current_filename, tare_current)

        elif user_input == '7':
            current, current_step = core.force_mode(dac, cap, current_step, target_step, calibration_filename)
            mass, mass_err = core.mass_calc(current, bl_factor, g, tare_current)

            print('I = {:f} +/- {:f}\n'.format(current[0], current[1]))
            print('B/L Factor = {:f} +/- {:f}\n'.format(bl_factor[0], bl_factor[1]))
            print('g = {:f} +/- {:f}\n'.format(g[0], g[1]))
            print('m = {:f} +/- {:f}\n'.format(mass, mass_err))

        else:
            print('make a valid selection\n')

        input("Press the <ENTER> key to continue...")

except KeyboardInterrupt:
    # release camera
    cap.release()
    dac.power_down(DAC_A, MODE_POWER_DOWN_100K)
    dac.power_down(DAC_B, MODE_POWER_DOWN_100K)
cap.release()
dac.power_down(DAC_A, MODE_POWER_DOWN_100K)
dac.power_down(DAC_B, MODE_POWER_DOWN_100K)