#!/usr/bin/env python3
import numpy as np
import core
import os
import cv2

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
    # open camera and set properties
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280);
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720);

    # define initial values
    tare_current = 0
    bl_factor = 0
    g = 0
    calibration_filename = './calibration_file.npy'
    calibration_array = np.load(calibration_filename)
    lower_step = calibration_array[0,0]
    upper_step = calibration_array[0,-1]
    lower_limit = int(calibration_array[1,0])
    upper_limit = int(calibration_array[1,-1])
    current_pixel = core.get_camera_position(cap)
<<<<<<< HEAD
    current_step = int(core.pixel_to_step(current_pixel, calibration_filename))
    
=======
    current_step = core.pixel_to_step(current_pixel[0], calibration_filename)

>>>>>>> 240a5dc560f13cca1e15a688f631c20f745024f8
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
            jog_steps, jog_times, current_step = core.motor_jog(current_step, velocity, acceleration,
                                                    steps)

        elif user_input == '3':
            # display camera
            core.display_tracker_box(cap, limits = [lower_limit,upper_limit])
            
        elif user_input == '4':
            # gravity input
            g = input('Total gravitational acceleration (m/s^2): ')
            g_err = input('Uncertainty in total gravitational acceleration (m/s^2): ')

        elif user_input == '5':
            # b/l constant calibration
            acceleration = 2000
            target_velocity = 3200
            step_limits = [lower_step, upper_step]
            velocity, voltage, velocity_err, voltage_err, current_step = \
            core.velocity_mode(cap, current_step, step_limits, acceleration, target_velocity,
                               calibration_filename, buffer = 1, runtime = 10)

            bl_factor, bl_factor_err = core.bl_factor_calc(velocity, voltage, velocity_err, voltage_err)

        elif user_input == '6':
            # tare balance
            tare_current, tare_current_err, current_step = core.force_mode(current_step, target_pixel)

        elif user_input == '7':
            # mass measurement
            if bl_factor == 0 or tare_current == 0 or g == 0:
                print('calibration not complete:\n')
                if bl_factor == 0:
                    print('- run velocity mode\n')
                if tare_current == 0:
                    print('- tare the balance\n')
                if g == 0:
                    print('- input a value for g\n')

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

except KeyboardInterrupt:
    # release camera
    cap.release()
cap.release()
