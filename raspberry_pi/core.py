import numpy as np
from gpiozero import DigitalOutputDevice
import cv2
import multiprocessing
import time
import matplotlib.pyplot as plt
import sys, select, os
import dac8552
from ctypes import cdll
import ctypes
from scipy.optimize import curve_fit

adc_lib = cdll.LoadLibrary("./record_voltage.so")

init_adc = adc_lib.init_adc
init_adc.restype = ctypes.c_int

get_voltage = adc_lib.get_voltage
get_voltage.restype = ctypes.c_int
get_voltage.argtypes = [ctypes.c_int]

kill_adc = adc_lib.kill_adc
kill_adc.restype = ctypes.c_int

class CameraThread(multiprocessing.Process):
    def __init__(self, cap, pixels, timestamp, runtime = 20):
        super(CameraThread, self).__init__()
        self.start_time = time.perf_counter()
        self.runtime = runtime
        self.cap = cap
        self.pixels = pixels
        self.timestamp = timestamp
    def run(self):
        # for time runtime, gets the current position and time and adds them to the list
        while(time.perf_counter() < self.start_time + self.runtime):
            self.timestamp.append(time.perf_counter())
            self.pixels.append(get_camera_position(self.cap))

class CoilThread(multiprocessing.Process):
    def __init__(self, voltage, timestamp, runtime = 20, array_length = 20000):
        # builds CoilThread class properties
        super(CoilThread, self).__init__()
        self.start_time = time.perf_counter()
        self.voltage = voltage
        self.timestamp = timestamp
        self.runtime = runtime
        self.array_length = array_length
    def run(self):
        # for time runtime, gets the current voltage and time and adds them to the list
        init_adc()
        while(time.perf_counter() < self.start_time + self.runtime):
            self.voltage.append(get_voltage(0))
            self.timestamp.append(time.perf_counter())
        kill_adc()
        #self.voltage = list(np_volts)
        #self.timestamp = list(np_times)

class MotorThread(multiprocessing.Process):
    def __init__(self, current_step, step_limits, acceleration, velocity, step_list, timestamp, buffer = 1):
        # builds MotorThread class properties
        super(MotorThread, self).__init__()
        self.current_step = current_step
        self.acceleration = acceleration
        self.velocity = velocity
        self.step_limits = step_limits
        self.buffer = buffer
        self.step_list = step_list
        self.timestamp = timestamp
    def run(self):
        # waits a time buffer, then moves up and saves values
        current_step = self.current_step
        better_sleep(self.buffer)
        jog_steps, jog_times, current_step = motor_jog(current_step, self.velocity,
                                                       self.acceleration, self.step_limits[1], absolute = True)
        self.step_list.extend(jog_steps)
        self.timestamp.extend(jog_times)

        # waits a time buffer, then moves down and saves values
        better_sleep(self.buffer)
        jog_steps, jog_times, current_step = motor_jog(current_step, self.velocity,
                                                       self.acceleration, self.step_limits[0], absolute = True)
        self.step_list.extend(jog_steps)
        self.timestamp.extend(jog_times)

def velocity_mode(cap, current_step, step_limits, acceleration, target_velocity, calibration_filename, buffer = 1):
    # move motor to bottom of range
    os.system('clear')
    print('Jogging to initial position...')
    _, _, current_step = motor_jog(current_step, target_velocity, acceleration, step_limits[0], absolute = True)

    manager = multiprocessing.Manager()
    step_list = manager.list()
    motor_timestamp = manager.list()
    voltages = manager.list()
    coil_timestamp = manager.list()
    pixel_list = manager.list()
    camera_timestamp = manager.list()

    # starts motor, camera, and coil threads to run
    runtime = int(np.ceil(buffer*3 + 2*target_velocity/acceleration + 2*abs(step_limits[1] - step_limits[0])/target_velocity))
    motor  = MotorThread(current_step, step_limits, acceleration, target_velocity, step_list, motor_timestamp, buffer)
    coil   = CoilThread(voltages, coil_timestamp, runtime)
    camera = CameraThread(cap, pixel_list, camera_timestamp, runtime)
    
    print('Starting measurement...')
    for thread in motor, coil, camera:
        thread.start()
    for thread in motor, coil, camera:
        thread.join()
    
    # 2xM numpy array, [step positions, times]
    # includes one upward and one downward movement with a pause in the beginning and in between
    print('Beginning analysis...')
    steps           = np.array([step_list, motor_timestamp])
    current_step    = motor.current_step
    voltages        = np.array([voltages, coil_timestamp]) # 2xN numpy array, [pixel positions, times]
    pixel_positions = np.array([pixel_list, camera_timestamp]) # 2xA numpy array, [voltages, times]
    voltages[0,:] = voltages[0,:]/1E6
    
    start_time = np.amin([np.amin(steps[1,:]),np.amin(pixel_positions[1,:]),np.amin(voltages[1,:])])

    for times in steps[1,:],pixel_positions[1,:],voltages[1,:]:
        times = times - start_time

    z_positions = np.array([pixel_to_z(pixel_positions[0,:], calibration_filename), pixel_positions[1,:]])
    
    # plot measurement results
    print('Raw position data (must look good). Close figure to continue.')
    plot_two_yvals(voltages[1,:], voltages[0,:], z_positions[1,:], z_positions[0,:],['time (s)','voltage (V)','position (mm)'])
 
    bl_factor, vmax_step = velocity_calc(steps, z_positions, voltages, target_velocity, acceleration)

    return bl_factor, current_step, vmax_step

def velocity_calc(steps, z_positions, voltages, target_velocity, acceleration):
    ## f_velocity is parabola stitched to a line stitched to a parabola. THe slope is zero on both ends.
    # a2 and b2 are the slope and intercept of the line. x0 through x3 are the stitch points.
    def f_velocity(x, x0, x1, x2, x3, a2, b2): # x is the independent variable, the rest are parameters
        # DEFINES sub-parameters based on the six free parameters. These parameters
        # are fixed based on the boundary conditions (that the function be continuous,
        # once differentiable, and that its derivatives are zero at the ends)
        b1 = -x0
        b3 = -x3
        a1 = a2/(2*(x1+b1))
        a3 = a2/(2*(x2+b3))
        c1 = a2*x1+b2-a1*(x1+b1)**2
        c3 = a2*x2+b2-a3*(x2+b3)**2

        def f1(xp):
            return a1*(xp+b1)**2+c1
        def f2(xp):
            return a2*xp+b2
        def f3(xp):
            return a3*(xp+b3)**2+c3
        
        y = np.piecewise(x, [x<=x1, (x1<x)*(x<=x2), x2<x], [f1, f2, f3])
        return y
    
    ## simple linear function
    def lin_func(x, velocity, intercept):
        return velocity*x + intercept
    
    ## gaussian function
    def gaussian_func(x, height, center, width, background):
        return height*np.exp(-(x-center)**2/width) + background
    
    ### VELOCITY AND VOLTAGE CALCULATION ###
    # GETS the TIME STAMPS where motion starts and stops for the up and down motion
    print('Calculating velocities...')
    up_start_time = steps[1, 0] # up motion starts with motor motion starting
    down_end_time = steps[1, -1] # down motion ends with motor motion ending
    
    step_dTdN       = np.diff(steps[1, :]) # 1xM-1 numpy array of time delay between motor steps
    up_step_end_ind = np.argmax(step_dTdN) # index of the biggest time between steps
    up_end_time     = steps[1, up_step_end_ind] # up motion ends at this time
    down_start_time = steps[1, up_step_end_ind+1] # down motion starts at this time

    # SPLITS z_positions array into purely up and purely down motion (no stationary)
    up_start_ind     = np.argmin(np.absolute(z_positions[1, :] - up_start_time))
    up_end_ind       = np.argmin(np.absolute(z_positions[1, :] - up_end_time))
    down_start_ind   = np.argmin(np.absolute(z_positions[1, :] - down_start_time))
    down_end_ind     = np.argmin(np.absolute(z_positions[1, :] - down_end_time))
    up_z_positions   = z_positions[:, up_start_ind:up_end_ind]
    down_z_positions = z_positions[:, down_start_ind:down_end_ind]

    ## FITS upward motion
    # guesses initial parameters
    x0_guess = up_start_time
    x1_guess = up_start_time+target_velocity/acceleration
    x2_guess = up_end_time-target_velocity/acceleration
    x3_guess = up_end_time
    a2_guess = (up_z_positions[0,-1]-up_z_positions[0,0])/(up_end_time-up_start_time) # this and the b2 guess assume constant motion throughout the upward motion
    b2_guess = up_z_positions[0,0]-a2_guess*up_z_positions[1,0]
    # does preliminary fit to find beginning and end of linear motion
    popt_up, pcov_up = curve_fit(f_velocity, up_z_positions[1,:], up_z_positions[0,:],
                                    p0 = [x0_guess, x1_guess, x2_guess, x3_guess, a2_guess, b2_guess],
                                    bounds = ([up_start_time,up_start_time,up_start_time,up_start_time,-np.inf,-np.inf],
                                              [up_end_time,up_end_time,up_end_time,up_end_time,np.inf,np.inf]))
    perr_up = np.sqrt(np.diag(pcov_up))
    lin_up_start = popt_up[1] # beginning TIME STAMP of LINEAR UPWARD MOTION
    lin_up_stop = popt_up[2] # ending TIME STAMP of LINEAR UPWARD MOTION
    
    # plots fit for user OK
    print('Primary fit one (must look good). Close figure to continue.')
    y_vals_temp = np.empty(up_z_positions[1,:].size)
    for i,x in enumerate(up_z_positions[1,:]):
        y_vals_temp[i] = f_velocity(x,popt_up[0], popt_up[1], popt_up[2], popt_up[3], popt_up[4], popt_up[5])
    plot_two_yvals(up_z_positions[1,:], up_z_positions[0,:],up_z_positions[1,:],y_vals_temp,['time (s)','data','fit'])

    # refines the fit to a linear model
    up_lin_start_ind = np.argmin(np.absolute(up_z_positions[1, :] - lin_up_start))
    up_lin_stop_ind = np.argmin(np.absolute(up_z_positions[1, :] - lin_up_stop))
    popt_up_lin, pcov_up_lin = curve_fit(lin_func,
                                    up_z_positions[1, up_lin_start_ind:up_lin_stop_ind],
                                    up_z_positions[0, up_lin_start_ind:up_lin_stop_ind],
                                    p0 = [popt_up[4],popt_up[5]])
    perr_up_lin = np.sqrt(np.diag(pcov_up_lin))
    up_vel     = popt_up_lin[0] # average VELOCITY during UPWARD motion
    up_vel_err = perr_up_lin[0] # error on up_vel_err
    
    # plots fit for user OK
    print('Final fit one (must look good). Close figure to continue.')
    y_vals_temp = np.empty(up_z_positions[1, up_lin_start_ind:up_lin_stop_ind].size)
    for i,x in enumerate(up_z_positions[1, up_lin_start_ind:up_lin_stop_ind]):
        y_vals_temp[i] = lin_func(x,popt_up_lin[0], popt_up_lin[1])
    plot_two_yvals(up_z_positions[1, up_lin_start_ind:up_lin_stop_ind],up_z_positions[0, up_lin_start_ind:up_lin_stop_ind],
                   up_z_positions[1, up_lin_start_ind:up_lin_stop_ind],y_vals_temp,['time (s)','data','fit'])


    ## FITS downward motion
    # guesses initial fit parameters
    x0_guess = down_start_time
    x1_guess = down_start_time+target_velocity/acceleration
    x2_guess = down_end_time-target_velocity/acceleration
    x3_guess = down_end_time
    a2_guess = (down_z_positions[0,-1]-down_z_positions[0,0])/(down_end_time-down_start_time) # this and the b2 guess assume constant motion throughout the downward motion
    b2_guess = down_z_positions[0,0]-a2_guess*down_z_positions[1,0]
    # does preliminary fit to find beginning and end of linear motion
    popt_down, pcov_down = curve_fit(f_velocity, down_z_positions[1,:], down_z_positions[0,:],
                                    p0 = [x0_guess, x1_guess, x2_guess, x3_guess, a2_guess, b2_guess],
                                    bounds = ([down_start_time,down_start_time,down_start_time,down_start_time,-np.inf,-np.inf],
                                              [down_end_time,down_end_time,down_end_time,down_end_time,np.inf,np.inf]))
    perr_down = np.sqrt(np.diag(pcov_down))
    lin_down_start = popt_down[1] # beginning TIME STAMP of LINEAR DOWNWARD MOTION
    lin_down_stop = popt_down[2] # ending TIME STAMP of LINEAR DOWNWARD MOTION
    
    # plots fit for user OK
    print('Primary fit two (must look good). Close figure to continue.')
    y_vals_temp = np.empty(down_z_positions[1,:].size)
    for i,x in enumerate(down_z_positions[1,:]):
        y_vals_temp[i] = f_velocity(x,popt_down[0], popt_down[1], popt_down[2], popt_down[3], popt_down[4], popt_down[5])
    plot_two_yvals(down_z_positions[1,:], down_z_positions[0,:], down_z_positions[1,:],y_vals_temp,['time (s)','data','fit'])

    # refines the fit to a linear model
    down_lin_start_ind = np.argmin(np.absolute(down_z_positions[1, :] - lin_down_start))
    down_lin_stop_ind = np.argmin(np.absolute(down_z_positions[1, :] - lin_down_stop))
    popt_down_lin, pcov_down_lin = curve_fit(lin_func,
                                    down_z_positions[1, down_lin_start_ind:down_lin_stop_ind],
                                    down_z_positions[0, down_lin_start_ind:down_lin_stop_ind],
                                    p0 = [popt_down[4],popt_down[5]])
    perr_down_lin = np.sqrt(np.diag(pcov_down_lin))
    down_vel     = popt_down_lin[0] # average VELOCITY during DOWNWARD motion
    down_vel_err = perr_down_lin[0] # error on down_vel_err
    
    # plots fit for user OK
    print('Final fit two (must look good). Close figure to continue.')
    y_vals_temp = np.empty(down_z_positions[1, down_lin_start_ind:down_lin_stop_ind].size)
    for i,x in enumerate(down_z_positions[1, down_lin_start_ind:down_lin_stop_ind]):
        y_vals_temp[i] = lin_func(x,popt_down_lin[0], popt_down_lin[1])
    plot_two_yvals(down_z_positions[1, down_lin_start_ind:down_lin_stop_ind],down_z_positions[0, down_lin_start_ind:down_lin_stop_ind],
                   down_z_positions[1, down_lin_start_ind:down_lin_stop_ind],y_vals_temp,['time (s)','data','fit'])


    ### GETS VOLTAGE during LINEAR motion
    print('Calculating voltages...\n')
    up_V_start_ind = np.argmin(np.absolute(voltages[1, :] - up_start_time))
    up_V_end_ind = np.argmin(np.absolute(voltages[1, :] - up_end_time))
    down_V_start_ind = np.argmin(np.absolute(voltages[1, :] - down_start_time))
    down_V_end_ind = np.argmin(np.absolute(voltages[1, :] - down_end_time))

    up_voltages = voltages[:,up_V_start_ind:up_V_end_ind]
    down_voltages = voltages[:,down_V_start_ind:down_V_end_ind]

    # fits a gaussian peak over region of constant UPWARD motion
    up_voltages = up_voltages[:,up_voltages[0,:] > 0]
    p0 = [np.amax(up_voltages[0,:]),up_voltages[1,np.argmax(up_voltages[0,:])],0.5,np.amax(up_voltages[0,:])]
    bounds = ([0,np.amin(up_voltages[1,:]),0.1,0],[10,np.amax(up_voltages[1,:]),np.ptp(up_voltages[1,:]),5])
    
    popt_up_voltage, pcov_up_voltage = curve_fit(gaussian_func, up_voltages[1,:], up_voltages[0,:],p0 = p0,bounds = bounds)
    
    perr_up_voltage = np.sqrt(np.diag(pcov_up_voltage))
    up_maxV = popt_up_voltage[0]
    up_maxV_unc = perr_up_voltage[0]
    
    #calculates pixel of maximum velocity
    time_of_max = popt_up_voltage[1]
    vmax_step = steps[0,np.argmin(np.absolute(steps[1, :] - time_of_max))]
    
    # plots fit for user OK
    print('Up voltage fit (must look good). Close figure to continue.')
    y_vals_temp = np.empty(up_voltages[1,:].size)
    for i,x in enumerate(up_voltages[1,:]):
        y_vals_temp[i] = gaussian_func(x,popt_up_voltage[0],popt_up_voltage[1],popt_up_voltage[2],popt_up_voltage[3])
    plot_two_yvals(up_voltages[1,:],up_voltages[0,:],
                   up_voltages[1,:],y_vals_temp,['time (s)','data','fit'])

    if False:
        # fits a gaussian peak over region of constant DOWNWARD motion
        down_voltages = down_voltages[:,down_voltages[0,:] < 0]
        down_voltages[0,:] = np.abs(down_voltages[0,:])
        popt_down_voltage, pcov_down_voltage = curve_fit(gaussian_func, down_voltages[1,:], down_voltages[0,:],
                                        p0 = [np.amax(down_voltages[0,:]),down_voltages[1,np.argmax[down_voltages[0,:]]],
                                              0.5,np.amax(down_voltages[0,:])],
                                               bounds = ([0,np.argmin[down_voltages[1,:]],0.1,0],
                                                         [10,np.argmin[down_voltages[1,:]],np.ptp(down_voltages[1,:]),5]))
        perr_down_voltage = np.sqrt(np.diag(pcov_down_voltage))
        down_maxV = popt_down_voltage[0]
        down_meanV_unc = perr_down_voltage[0]
    

    #gives a WEIGHTED AVERAGE of the UPWARD and DOWNWARD bl_factors
    def weighted_average(val1,val2,err1,err2):
        weight1     = 1/err1**2
        weight2     = 1/err2**2
        average     = (val1*weight1+val2*weight2)/(weight1+weight2)
        uncertainty = 1/(weight1+weight2)
        return average,uncertainty
    
    bl_factor_up, bl_factor_up_err = bl_factor_calc(abs(up_vel), abs(up_maxV), up_vel_err, up_maxV_unc)
    #bl_factor_down, bl_factor_down_err = bl_factor_calc(abs(down_vel), abs(down_meanV), down_vel_err, down_meanV_unc)
    #bl_factor, bl_factor_err  = weighted_average(bl_factor_up,bl_factor_down,bl_factor_up_err,bl_factor_down_err)
    
    print('up velocity = {:f} +/- {:f} mm/s'.format(up_vel, up_vel_err))
    print('up voltage = {:f} +/- {:f} volts'.format(up_maxV, up_maxV_unc))
    print('up bl_factor = {:f} +/- {:f} volt*s/mm\n'.format(bl_factor_up, bl_factor_up_err))
    bl_factor = bl_factor_up
    bl_factor_err = bl_factor_up_err
    
    #print('down velocity = {:f} +/- {:f} mm/s'.format(down_vel, down_vel_err))
    #print('down voltage = {:f} +/- {:f} volts'.format(down_meanV, down_meanV_unc))
    #print('down bl_factor = {:f} +/- {:f} volt*s/mm\n'.format(bl_factor_down, bl_factor_down_err))
    
    #print('Total bl_factor = {:f} +/- {:f} volt*s/mm\n'.format(bl_factor, bl_factor_err))
    bl_factor = [bl_factor, bl_factor_err]

    return bl_factor, vmax_step

def motor_jog(current_step, velocity = 3200, acceleration = 2000, steps = 1600, absolute = False): # velocity in steps/s, acceleration in steps/s^2
    # defines pin numbers
    pulse_pin     = DigitalOutputDevice(24)
    direction_pin = DigitalOutputDevice(25)
    enable_pin    = DigitalOutputDevice(22)

    # sets correct direction pin voltage and enables the enable pin
    enable_pin.on()
    if absolute:
        target_step = steps
        multiplier = np.sign(steps - current_step)
    else:
        target_step = current_step + steps
        multiplier = np.sign(steps)
        steps = np.abs(steps)
    if multiplier   == 1:
        direction_pin.off()
    elif multiplier == -1:
        direction_pin.on()

    # creates loop variables
    jog_steps = []
    jog_times = []
    current_velocity = 100
    initial_step = current_step
    acceleration_steps = velocity**2/(2*acceleration) # approx. num. of steps taken during acceleration
    current_time = time.perf_counter()

    try:
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
            better_sleep(delay/2, current_time)
            # turn pulse pin off, WAIT
            pulse_pin.off()
            better_sleep(delay, current_time)

        # done with movement, so we turn the enable pin off
        enable_pin.off()

    except KeyboardInterrupt:
        print("\nUser exit during motor usage. Powering down.\n")

        pulse_pin.off()
        enable_pin.off()

        sys.exit(0)

    return jog_steps, jog_times, current_step

def get_pixel_position(cap, x_range, y_range, offset):
    # get current pixel location
    ret, frame = cap.read()
    bw     = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img    = bw[y_range[0]:y_range[1],x_range[0]:x_range[1]]
    lines  = np.average(img, axis=1)
    cutoff = np.amin(lines) + offset

    _, w = bw.shape

    current_pixel = np.where(lines < cutoff)[0][0] + y_range[0] # this value is the vertical position in pixels
    return current_pixel, w, frame

def add_range_boxes(frame, x_range, y_range):
    frame[y_range[0],x_range[0]:x_range[1],:] = [0,0,255]
    frame[y_range[1],x_range[0]:x_range[1],:] = [0,0,255]
    frame[y_range[0]:y_range[1],x_range[0],:] = [0,0,255]
    frame[y_range[0]:y_range[1],x_range[1],:] = [0,0,255]
    return frame

def get_camera_position(cap, bounds = [[720, 750],[125,510]], offset = 5,
                       output_to_file = False, filename_base = './camera'):
    x_range = (bounds[0][0],bounds[0][1])
    y_range = (bounds[1][0],bounds[1][1])

    pixel_position, w, frame = get_pixel_position(cap, x_range, y_range, offset)

    if output_to_file:
        frame = add_range_boxes(frame, x_range, y_range)

        plt.plot(range(*y_range),lines)
        plt.axhline(cutoff)
        plt.savefig(''.join([filename_base,'_lines.png']))

        frame[pixel_position + y_range[0],:,:] = [[0,0,255]]*w

        cv2.imwrite(''.join([filename_base,'_image.png']), frame)

    return pixel_position

def display_tracker_box(cap, bounds = [[720, 750],[125,510]], offset = 5, limits = None):
    x_range = (bounds[0][0],bounds[0][1])
    y_range = (bounds[1][0],bounds[1][1])

    print("Press q on camera display window to end")

    while (cap.isOpened()):
        position, w, frame = get_pixel_position(cap, x_range, y_range, offset) # this value is the vertical position in pixels

        frame[position,:,:] = [[0,0,255]]*w

        if limits is not None:
            frame[limits[0],:,:] = [[255,0,0]]*w
            frame[limits[1],:,:] = [[0,0,0]]*w

        frame = add_range_boxes(frame, x_range, y_range)
        cv2.imshow('q to quit', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break

    return

def jog_to_pixel(cap, current_step, target_pixel, bounds = [[720, 750],[125,510]], offset = 5, show_image = False):
    x_range = (bounds[0][0],bounds[0][1])
    y_range = (bounds[1][0],bounds[1][1])

    # set stepper motor control values
    velocity = 3200
    acceleration = 1000

    # jog motor until pixel number is correct
    num_hits = 0
    while (num_hits < 10):
        # get current pixel location
        current_pixel, w, frame = get_pixel_position(cap, x_range, y_range, offset) # this value is the vertical position in pixels

        # show image on screeen if requested
        if show_image:
            frame[current_pixel,:,:] = [[0,0,255]]*w # line for current position
            frame[target_pixel,:,:] = [[255,0,0]]*w # line for target position

            # creates frame around detection region
            frame = add_range_boxes(frame, x_range, y_range)
            cv2.imshow('q to quit', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                current_pixel = target_pixel

        # adjust position with stepper motor
        dstep = min(700,abs(current_pixel - target_pixel))
        if target_pixel != current_pixel:
            num_hits = 0
            if target_pixel > current_pixel:
                dstep = -dstep
            _, _, current_step = motor_jog(current_step, velocity, acceleration, dstep)
        else:
            num_hits = num_hits + 1

    # close up shop
    cv2.destroyAllWindows()
    return current_step

def create_calibration_file(cap, current_step, filename, step_limits, pixel_limits, height_limits,
                             bounds = [[720, 750],[125,510]], offset = 5, show_image = False):
    # motor constants
    velocity = 3200
    acceleration = 1000
    dStep = 50
    delay = 0.1 # delay time before measuring

    # create array with all steps, zeros for the pixel values, and their corresponding heights
    step_list = np.arange(step_limits[0],step_limits[1],dStep)
    nPoints = step_list.size
    pixel_list = np.zeros_like(step_list)
    dzdPixel = (height_limits[1]-height_limits[0])/(step_limits[1]-step_limits[0])
    height_list = height_limits[0] + dzdPixel * (step_list - step_list[0])

    # open camera and set properties
    x_range = (bounds[0][0],bounds[0][1])
    y_range = (bounds[1][0],bounds[1][1])

    # jog motor to each position and measure pixel number
    current_pixel = None
    for i in range(nPoints):
        # go to desired pixel
        _, _, current_step = motor_jog(current_step, velocity, acceleration, step_list[i], absolute = True)

        # get current pixel location (averages over 10 measurements)
        pixel_measurements = np.zeros(9)
        for k in range(9):
            pixel_measurements[k], w, frame = get_pixel_position(cap, x_range, y_range, offset) # this value is the vertical position in pixels
        pixel_list[i] = np.average(pixel_measurements)

        # show image on screeen if requested
        if show_image:
            frame[pixel_list[i],:,:] = [[0,0,255]]*w # line for current position

            # creates frame around detection region
            frame = add_range_boxes(frame, x_range, y_range)
            cv2.imshow('q to quit', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                cv2.destroyAllWindows()
                pixel_list[i] = target_pixel

    # close up shop
    cv2.destroyAllWindows()
    full_array = np.array([step_list, pixel_list, height_list])
    np.save(filename, full_array)
    return current_step

def pixel_to_z(pixel_list, calibration_filename):
    calibration_array = np.load(calibration_filename)

    z_list = np.interp(-pixel_list, -calibration_array[1,:], calibration_array[2,:])

    return z_list

def pixel_to_step(pixel_list, calibration_filename):
    calibration_array = np.load(calibration_filename)

    step_list = np.interp(-pixel_list, -calibration_array[1,:], calibration_array[0,:])

    return step_list

def step_to_pixel(step_list, calibration_filename):
    calibration_array = np.load(calibration_filename)

    pixel_list = int(np.interp(step_list, calibration_array[0,:], calibration_array[1,:]))

    return pixel_list

def force_mode(dac, cap, current_step, target_step, calibration_filename, pixel_err = 10, vref = 5,
               coil_resistance = 2040, amp_gain = 1, debug = False, coil_err = 0, gain_err = 0):

    target_pixel = step_to_pixel(target_step, calibration_filename)
    current_step = jog_to_pixel(cap, current_step, target_pixel)

    current, _ = set_coil_current(dac, vref = vref, coil_resistance = coil_resistance, amp_gain = amp_gain)
    input()

    _, _, current_step = motor_jog(current_step, steps = -800)

    current_pixel = get_camera_position(cap)

    currents = []
    pixels = []
    times = []

    # check to see if position has dropped more than 2 sigma from target
    if current_pixel > target_pixel + 2*pixel_err:
        print('more than 2 sigma below target pixel')
        return None, current_step

    elif current_pixel < target_pixel - 2*pixel_err:
        print('more than 2 sigma above target pixel')
        return None, current_step

    else:
        # drop current til the position starts to drop
        while(current_pixel < target_pixel + pixel_err):
            current -= 1e-4
            current_pixel = get_camera_position(cap)
            currents.append(current)
            pixels.append(current_pixel)
            times.append(time.perf_counter())
        counter = 0

        #balance current
        while(counter < 50):
            next_current, _ = set_coil_current(dac, current = current, vref = vref, coil_resistance = coil_resistance, amp_gain = amp_gain)
            if abs(current_pixel - target_pixel) > pixel_err/2:
                counter = 0
            else:
                counter += 1
            if abs(current_pixel - target_pixel) > pixel_err/4:
                current = next_current + (current_pixel - target_pixel)*1e-4
            current_pixel = get_camera_position(cap)

            currents.append(current)
            pixels.append(current_pixel)
            times.append(time.perf_counter())

            # check to see if position has dropped more than 2 sigma from target
            if current_pixel > target_pixel + 2*pixel_err:
                print('more than 2 sigma below target pixel')
                return None, current_step

            elif current_pixel < target_pixel - 2*pixel_err:
                print('more than 2 sigma above target pixel')
                return None, current_step

    # Average over the currents in the last 50 steps and get the error in the average
    np_currents = np.array(currents)
    average_current = np.average(currents[-50:])
    current_err = np.std(currents[-50:])
    average_current = [average_current, current_err]

    _, _, current_step = motor_jog(current_step, steps = 800)

    set_coil_current(dac, current = 0)

    return average_current, current_step

def set_coil_current(dac, current = None, vref = 5, coil_resistance = 2040, amp_gain = 1):
    try:
        if current is None:
            target_voltage = vref
            current = amp_gain*target_voltage/coil_resistance
        else:
            target_voltage = (1/amp_gain)*current*coil_resistance

        if target_voltage > vref:
            target_voltage = vref
            raise RuntimeWarning('Target voltage too high! Setting to max voltage')

    except RuntimeWarning as e:
        print(str(e))

    set_DAC_voltage(dac,target_voltage)
        
    return current, target_voltage

def set_DAC_voltage(dac,voltage,vref = 5):
    dac.v_ref = vref
    data = int(voltage*dac.digit_per_v)
    print(data)
    print(voltage)
    dac.write_dac(dac8552.DAC_A, data)
    dac.write_dac(dac8552.DAC_B, data)

def mass_calc(current, bl_factor, g, tare_current):
    norm_current = current[0] - tare_current[0]
    norm_current_err = np.sqrt(current[1]**2 + tare_current[1]**2)

    mass = norm_current*bl_factor[0]/g[0]
    mass_err = (1/g[0])*np.sqrt((bl_factor[0]*norm_current_err)**2 +
               (norm_current*bl_factor[0]*g[1]/g[0])**2 +
               (norm_current*bl_factor[1])**2)

    return mass, mass_err

def bl_factor_calc(velocity, voltage, velocity_err, voltage_err):
    bl_factor = voltage/velocity

    bl_factor_err = (1/velocity)*np.sqrt(voltage_err**2 +
                                         (voltage*velocity_err/velocity)**2)

    return bl_factor, bl_factor_err

def better_sleep(delay, initial_time = None):
    if not initial_time:
        initial_time = time.perf_counter()
    while time.perf_counter() < initial_time + delay:
        pass
    return

def plot_two_yvals(x1,y1,x2,y2,axis_labels):
    fig, ax1 = plt.subplots()

    color = 'tab:red'
    ax1.set_xlabel(axis_labels[0])
    ax1.set_ylabel(axis_labels[1], color=color)
    ax1.plot(x1,y1, color=color, marker = ".", markersize = 3)
    ax1.tick_params(axis='y', labelcolor=color)

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

    color = 'tab:blue'
    ax2.set_ylabel(axis_labels[2], color=color)  # we already handled the x-label with ax1
    ax2.plot(x2,y2, color=color, marker = ".", markersize = 3)
    ax2.tick_params(axis='y', labelcolor=color)

    fig.tight_layout()  # otherwise the right y-label is slightly clipped
    plt.show()
    return