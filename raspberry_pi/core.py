import numpy as np
from gpiozero import DigitalOutputDevice
import cv2
import threading
import time
import matplotlib.pyplot as plt
import sys, select, os
import dac8552# import DAC8552, DAC_A, DAC_B, MODE_POWER_DOWN_100K

class CameraThread(threading.Thread):
    def __init__(self, cap, runtime = 20):
        super(CameraThread, self).__init__()
        self.runtime = runtime
        self.np_position = None
    def run(self):
        self.np_position = get_camera_position(cap, run_time = self.runtime)

class CoilThread(threading.Thread):
    def __init__(self, runtime = 20):
        super(CoilThread, self).__init__()
        self.start_time = time.perf_counter()
        self.voltage = []
        self.timestamp = []
        self.runtime = runtime
        self.np_voltage = None
    def run(self):
        while(time.perf_counter() < self.start_time + self.runtime):
            self.voltage.append(get_coil_voltage())
            self.timestamp.append(time.perf_counter())
        self.np_voltage = np.array([self.voltage,self.timestamp])

class MotorThread(threading.Thread):
    def __init__(self, current_step, step_limits, acceleration, velocity, buffer = 1, runtime = 20):
        super(MotorThread, self).__init__()
        self.current_step = current_step
        self.acceleration = acceleration
        self.velocity = velocity
        self.step_limits = step_limits
        self.buffer = buffer
        self.step_list = []
        self.timestamp = []
        self.runtime = runtime
        self.np_steps = None
    def run(self):
        current_step = self.current_step
        self.step_list.append(self.current_step)
        self.timestamp.append(time.perf_counter())

        time.sleep(self.buffer) # to wait for camera
        jog_steps, jog_times, current_step = motor_jog(current_step, self.velocity,
                                                       self.acceleration, self.step_limits[1], absolute = True)
        self.step_list.extend(jog_steps)
        self.timestamp.extend(jog_times)

        self.current_step = current_step
        self.step_list.append(self.current_step)
        self.timestamp.append(time.perf_counter())

        time.sleep(self.buffer)
        jog_steps, jog_times, current_step = motor_jog(current_step, self.velocity,
                                                       self.acceleration, self.step_limits[0], absolute = True)
        self.step_list.extend(jog_steps)
        self.timestamp.extend(jog_times)

        self.current_step = current_step
        self.step_list.append(self.current_step)
        self.timestamp.append(time.perf_counter())

        self.np_steps = np.array([self.step_list,self.timestamp])

def velocity_mode(cap, current_step, step_limits, acceleration, target_velocity, calibration_filename, buffer = 1, runtime = 20):
    _, _, current_step = motor_jog(current_step, target_velocity, acceleration, step_limits[0], absolute = True)
    motor  = MotorThread(current_step, step_limits, acceleration, target_velocity, buffer, runtime)
    coil   = CoilThread(runtime)
    camera = CameraThread(cap, runtime)

    for thread in motor, coil, camera:
        thread.start()
    for thread in motor, coil, camera:
        thread.join()

    # 2xM numpy array, [step positions, times]
    # includes one upward and one downward movement with a pause in the beginning and in between
    steps        = motor.np_steps
    current_step = motor.current_step

    pixel_positions = camera.np_position # 2xN numpy array, [pixel positions, times]
    voltages        = coil.np_voltage # 2xA numpy array, [voltages, times]

    start_time = np.amin([steps[1,:],pixel_positions[1,:],voltages[1,:]])

    for times in steps[1,:],pixel_positions[1,:],voltages[1,:]:
        times = times - start_time

    z_positions = np.array([pixel_to_z(pixel_positions[0,:], calibration_filename), pixel_positions[1,:]])

    velocity, voltage, velocity_err, voltage_err = velocity_calc(steps, z_positions, voltages)

    return velocity, voltage, velocity_err, voltage_err, current_step

def velocity_calc(steps, z_positions, voltages):
    ### VELOCITY AND VOLTAGE CALCULATION ###
    # GETS the TIME STAMPS where motion starts and stops for the up and down motion
    up_start_time = steps[1, 0] # up motion starts with motor motion starting
    down_end_time = steps[1, -1] # down motion ends with motor motion ending

    step_dTdN       = np.diff(steps[1, :]) # 1xM-1 numpy array of time delay between motor steps
    up_step_end_ind = np.argmax(step_dTdN) # index of the biggest time between steps
    up_end_time     = steps[1, up_end_ind] # up motion ends at this time
    down_start_time = steps[1, up_end_ind+1] # down motion starts at this time

    # SPLITS z_positions array into purely up and purely down motion (no stationary)
    up_start_ind     = np.argmin(np.absolute(z_positions[1, :] - up_start_time))
    up_end_ind       = np.argmin(np.absolute(z_positions[1, :] - up_end_time))
    down_start_ind   = np.argmin(np.absolute(z_positions[1, :] - down_start_time))
    down_end_ind     = np.argmin(np.absolute(z_positions[1, :] - down_end_time))
    up_z_positions   = z_positions[:, up_start_ind:up_end_ind]
    down_z_positions = z_positions[:, down_start_ind:down_end_ind]

    ## FITS a line to the UP pixel positions to find the start and end of the linear motion
    def f_up(x, x1, x2, a2, b2): # x is the independent variable, the rest are parameters
        # DEFINES sub-parameters based on the four free parameters. These parameters
        # are fixed based on the boundary conditions (that the function be continuous,
        # once differentiable, and that its derivatives are zero at the ends)
        x0 = up_start_time
        x3 = up_end_time
        b1 = -x0
        b3 = -x3
        a1 = a2/(2*(x1+b1))
        a3 = a2/(2*(x2+b3))
        c1 = a2*x1+b2-a1*(x1+b1)**2
        c3 = a2*x2+b2-a3*(x2+b3)**2

        def f1(x,a1,b1,c1):
            return a1*(x+b1)**2+c1
        def f2(x,a2,b2):
            return a2*x+b2
        def f3(x,a3,b3,c3):
            return a3*(x+b3)**2+c3

        if x<=x1:
            y = f1(x)
        elif x1<x<=x2:
            y = f2(x)
        elif x2<x:
            y = f3(x)

        return y

    # GUESSES initial fit parameters from data
    x1_guess = up_start_time+target_velocity/acceleration
    x2_guess = up_end_time-target_velocity/acceleration
    a2_guess = (up_z_positions[0,-1]-up_z_positions[0,0])/(up_end_time-up_start_time) # this and the b2 guess assume constant motion throughout the upward motion
    b2_guess = up_z_positions[0,0]-a2_guess*up_z_positions[1,0]

    # FITS upward motion data to parabola, linear, parabola of f_up
    popt_up, pcov_up = np.curve_fit(f_up, up_z_positions[1,:], up_z_positions[0,:],
                                    p0 = [x1_guess, x2_guess, a2_guess, b2_guess],
                                    bounds = ([up_start_time,up_start_time,np.inf,np.inf],
                                              [up_end_time,up_end_time,np.inf,np.inf]))

    perr_up = np.sqrt(np.diag(pcov_up))
    lin_up_start = popt_up[0] # beginning TIME STAMP of LINEAR UPWARD MOTION
    lin_up_stop = popt_up[1] # ending TIME STAMP of LINEAR UPWARD MOTION

    ## GET average VELOCITY over UPWARD motion
    def lin_func(x, velocity, intercept):
        return velocity*x + intercept

    up_lin_start_ind = np.argmin(np.absolute(up_z_positions[1, :] - lin_up_start))
    up_lin_stop_ind = np.argmin(np.absolute(up_z_positions[1, :] - lin_up_stop))

    popt_up_lin, pcov_up_lin = np.curve_fit(lin_func,
                                    up_z_positions[1, up_lin_start_ind:up_lin_stop_ind],
                                    up_z_positions[0, up_lin_start_ind:up_lin_stop_ind],
                                    p0 = [popt_up[2],popt_up[3]])

    perr_up_lin = np.sqrt(np.diag(pcov_up_lin))
    up_vel     = popt_up_lin[0] # average VELOCITY during UPWARD motion
    up_vel_err = perr_up_lin[0] # error on up_vel_err


    ## FITS a line to the DOWN pixel positions to find the start and end of the linear motion
    def f_down(x, x1, x2, a2, b2): # x is the independent variable, the rest are parameters
        # DEFINES sub-parameters based on the four free parameters. These parameters
        # are fixed based on the boundary conditions (that the function be continuous,
        # once differentiable, and that its derivatives are zero at the ends)
        x0 = down_start_time
        x3 = down_end_time
        b1 = -x0
        b3 = -x3
        a1 = a2/(2*(x1+b1))
        a3 = a2/(2*(x2+b3))
        c1 = a2*x1+b2-a1*(x1+b1)**2
        c3 = a2*x2+b2-a3*(x2+b3)**2

        def f1(x,a1,b1,c1):
            return a1*(x+b1)**2+c1
        def f2(x,a2,b2):
            return a2*x+b2
        def f3(x,a3,b3,c3):
            return a3*(x+b3)**2+c3

        if x<=x1:
            y = f1(x)
        elif x1<x<=x2:
            y = f2(x)
        elif x2<x:
            y = f3(x)

        return y

    # GUESSES initial fit parameters from data
    x1_guess = down_start_time+target_velocity/acceleration
    x2_guess = down_end_time-target_velocity/acceleration
    a2_guess = (down_z_positions[0,-1]-down_z_positions[0,0])/(down_end_time-down_start_time) # this and the b2 guess assume constant motion throughout the downward motion
    b2_guess = down_z_positions[0,0]-a2_guess*down_z_positions[1,0]

    # FITS downward motion data to parabola, linear, parabola of f_down
    popt_down, pcov_down = np.curve_fit(f_down, down_z_positions[1,:], down_z_positions[0,:],
                                    p0 = [x1_guess, x2_guess, a2_guess, b2_guess],
                                    bounds = ([down_start_time,down_start_time,np.inf,np.inf],
                                              [down_end_time,down_end_time,np.inf,np.inf]))

    perr_down = np.sqrt(np.diag(pcov_down))
    lin_down_start = popt_down[0] # beginning TIME STAMP of LINEAR DOWNWARD MOTION
    lin_down_stop = popt_down[1] # ending TIME STAMP of LINEAR DOWNWARD MOTION

    ## GET average VELOCITY over DOWNWARD motion
    def lin_func(x, velocity, intercept):
        return velocity*x + interceptx

    down_lin_start_ind = np.argmin(np.absolute(down_z_positions[1, :] - lin_down_start))
    down_lin_stop_ind = np.argmin(np.absolute(down_z_positions[1, :] - lin_down_stop))

    popt_down_lin, pcov_down_lin = np.curve_fit(lin_func,
                                    down_z_positions[1, down_lin_start_ind:down_lin_stop_ind],
                                    down_z_positions[0, down_lin_start_ind:down_lin_stop_ind],
                                    p0 = [popt_down[2],popt_down[3]])

    perr_down_lin = np.sqrt(np.diag(pcov_down_lin))
    down_vel     = popt_down_lin[0] # average VELOCITY during DOWNWARD motion
    down_vel_err = perr_down_lin[0] # error on down_vel_err


    ### GETS VOLTAGE during LINEAR motion
    up_V_start_ind = np.argmin(np.absolute(voltages[1, :] - up_start_time))
    up_V_end_ind = np.argmin(np.absolute(voltages[1, :] - up_end_time))
    down_V_start_ind = np.argmin(np.absolute(voltages[1, :] - down_start_time))
    down_V_end_ind = np.argmin(np.absolute(voltages[1, :] - down_end_time))

    up_voltages = voltages[:,up_V_start_ind:up_V_end_ind]
    down_voltages = voltages[:,down_V_start_ind:down_V_end_ind]

    # AVERAGES voltage over region of constant UPWARD motion
    up_meanV = np.mean(up_voltages[0,:])
    up_meanV_unc = np.std(up_voltages[0,:])/np.sqrt(up_voltages[0,:].size)

    # AVERAGES voltage over region of constant DOWNWARD motion
    down_meanV = np.mean(down_voltages[0,:])
    down_meanV_unc = np.std(down_voltages[0,:])/np.sqrt(down_voltages[0,:].size)

    #gives a WEIGHTED AVERAGE of the UPWARD and DOWNWARD VOLTAGE and VELOCITY
    def weighted_average(val1,val2,err1,err2):
        weight1     = 1/err1**2
        weight2     = 1/err2**2
        average     = (val1*weight1+val2*weight2)/(weight1+weight2)
        uncertainty = 1/(weight1+weight2)
        return average,uncertainty

    voltage, voltage_err   = weighted_average(up_meanV,down_meanV,up_meanV_unc,down_meanV_unc)
    velocity, velocity_err = weighted_average(up_vel,down_vel,up_vel_err,down_vel_err)

    return velocity, voltage, velocity_err, voltage_err

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
            while delay/2 > (time.perf_counter() - current_time):
                pass
            # turn pulse pin off, WAIT
            pulse_pin.off()
            while delay > (time.perf_counter() - current_time):
                pass

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

def get_camera_position(cap, bounds = [[720, 750],[125,510]], offset = 5, run_time = 1,
                       output_to_file = False, filename_base = './camera'):
    x_range = (bounds[0][0],bounds[0][1])
    y_range = (bounds[1][0],bounds[1][1])

    start_time = time.perf_counter()
    timestamp = []
    pixel_list = []

    while(time.perf_counter() < start_time + run_time):
        this_pixel, w, frame = get_pixel_position(cap, x_range, y_range, offset)

        pixel_list.append(this_pixel)
        timestamp.append(time.perf_counter())

        if output_to_file:
            frame = add_range_boxes(frame, x_range, y_range)

            plt.plot(range(*y_range),lines)
            plt.axhline(cutoff)
            plt.savefig(''.join([filename_base,'_lines.png']))

            frame[pixel_position + y_range[0],:,:] = [[0,0,255]]*w

            cv2.imwrite(''.join([filename_base,'_image.png']), frame)

    pixel_position = np.array([pixel_list,timestamp])

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

def pixel_to_z(pixel_list, calibration_filename = './calibration.npy'):
    calibration_array = np.load(calibration_filename)

    z_list = np.interp(-pixel_list, -calibration_array[1,:], calibration_array[2,:])

    return z_list

def pixel_to_step(pixel_list, calibration_filename = './calibration.npy'):
    calibration_array = np.load(calibration_filename)

    step_list = np.interp(-pixel_list, -calibration_array[1,:], calibration_array[0,:])

    return step_list

def force_mode(current_step, target_pixel, pixel_err = 10, vref = 3.3,
               coil_resistance = 2040, amp_gain = 1, debug = False, coil_err = 0, gain_err = 0):

    current_step = jog_to_pixel(current_step, target_pixel)

    _, current, _ = set_coil_current(vref = vref, coil_resistance = coil_resistance, amp_gain = amp_gain)

    _, _, current_step = motor_jog(current_step, steps = -800)

    current_pixel = get_camera_position()

    currents = []
    pixels = []
    times = []

    # check to see if position has dropped more than 2 sigma from target
    if current_pixel > target_pixel + 2*pixel_err:
        print('more than 2 sigma below target pixel')
        return None, None, current_step

    elif current_pixel < target_pixel - 2*pixel_err:
        print('more than 2 sigma above target pixel')
        return None, None, current_step

    else:
        # drop current til the position starts to drop
        while(current_pixel < target_pixel + pixel_err):
            current -= 1e-4
            current_pixel = get_camera_position()
            currents.append(current)
            pixels.append(current_pixel)
            times.append(time.perf_counter())
        counter = 0

        #balance current
        while(counter < 50):
            _, next_current, _ = set_coil_current(current = current, vref = vref, coil_resistance = coil_resistance, amp_gain = amp_gain)
            if abs(current_pixel - target_pixel) > pixel_err/2:
                counter = 0
            else:
                counter += 1
            if abs(current_pixel - target_pixel) > pixel_err/4:
                current = next_current + (current_pixel - target_pixel)*1e-4
            current_pixel = get_camera_position()

            currents.append(current)
            pixels.append(current_pixel)
            times.append(time.perf_counter())

            # check to see if position has dropped more than 2 sigma from target
            if current_pixel > target_pixel + 2*pixel_err:
                print('more than 2 sigma below target pixel')
                return None, None, current_step

            elif current_pixel < target_pixel - 2*pixel_err:
                print('more than 2 sigma above target pixel')
                return None, None, current_step

    # Average over the currents in the last 50 steps and get the error in the average
    np_currents = np.array(currents)
    average_current = np.average(currents[-50:])
    current_err = np.std(currents[-50:])

    _, _, current_step = motor_jog(current_step, steps = 800)

    set_coil_current(current = 0)

    return average_current, current_err, current_step

def set_coil_current(current = None, vref = 3.3, coil_resistance = 2040, amp_gain = 1):
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

    dac = dac8552.DAC8552()

    try:
        dac.v_ref = vref
        data = int(target_voltage*dac.digit_per_v)
        dac.write_dac(dac8552.DAC_A, data)

    except KeyboardInterrupt:
        print("\nUser exit during DAC usage. Powering down.\n")
        # Put DAC to Power Down Mode:
        dac.power_down(dac8552.DAC_A, dac8552.MODE_POWER_DOWN_100K)
        dac.power_down(dac8552.DAC_B, dac8552.MODE_POWER_DOWN_100K)
        sys.exit(0)

    return data, current, target_voltage

def get_coil_voltage():
    # PUT COIL VOLTAGE CODE HERE
    coil_voltage = 0
    return coil_voltage

def mass_calc(current, current_err, bl_factor, bl_factor_err, g, g_err, tare_current, tare_current_err):
    norm_current = current - tare_current
    norm_current_err = np.sqrt(current_err**2 + tare_current**2)

    mass = norm_current*bl_factor/g
    mass_err = (1/g)*np.sqrt((bl_factor*norm_current_err)**2 +
               (norm_current*bl_factor*g_err/g)**2 +
               (norm_current*bl_factor_err)**2)

    return mass, mass_err

def bl_factor_calc(velocity, voltage, velocity_err, voltage_err):
    bl_factor = voltage/velocity

    bl_factor_err = (1/velocity)*np.sqrt(voltage_err**2 +
                                         (voltage*velocity_err/velocity)**2)

    return bl_factor, bl_factor_err
