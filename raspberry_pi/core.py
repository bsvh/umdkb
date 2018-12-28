import numpy as np
import gpiozero
import cv2
import threading
import time
import matplotlib.pyplot as plt

class CameraThread(threading.Thread):
    def __init__(self, runtime = 20):
        super(CameraThread, self).__init__()
        self.postion = []
        self.timestamp = []
        self.runtime = runtime
        self.np_position = np.empty()
    def run(self):
        while(time.perf_counter() < self.start_time + self.runtime):
            self.position.append(get_camera_postion())
            self.timestamp.append(time.perf_counter())
        self.np_position = np.array([self.position,self.timestamp])

class CoilThread(threading.Thread):
    def __init__(self, runtime = 20):
        super(CoilThread, self).__init__()
        self.voltage = []
        self.timestamp = []
        self.runtime = runtime
        self.np_voltage = np.empty()
    def run(self):
        while(time.perf_counter() < self.start_time + self.runtime):
            self.voltage.append(get_coil_voltage())
            self.timestamp.append(time.perf_counter())
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
        current_step = self.current_step
        self.step_list.append(self.current_step)
        self.timestamp.append(time.perf_counter())

        time.sleep(self.buffer)
        jog_steps, jog_times, current_step = motor_jog(current_step, self.velocity,
                                                       self.acceleration, self.steps, 'forward')
        self.step_list.extend(jog_steps)
        self.timestamp.extend(jog_times)

        self.current_step = current_step
        self.step_list.append(self.current_step)
        self.timestamp.append(time.perf_counter())

        time.sleep(self.buffer)
        jog_steps, jog_times, current_step = motor_jog(current_step, self.velocity,
                                                       self.acceleration, self.steps, 'reverse')
        self.step_list.extend(jog_steps)
        self.timestamp.extend(jog_times)

        self.current_step = current_step
        self.step_list.append(self.current_step)
        self.timestamp.append(time.perf_counter())

        self.np_steps = np.array([self.step_list,self.timestamp])

def velocity_mode(current_step, acceleration, target_velocity, buffer = 1, steps = 1600, runtime = 20):
    motor  = MotorThread(current_step, acceleration, target_velocity, buffer, steps, runtime)
    coil   = CoilThread(runtime)
    camera = CameraThread(runtime)

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

    z_positions = np.array([pixel_to_z(pixel_positions[0,:]), pixel_positions[1,:])

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

def force_mode(current_step, target_pixel):

    '''
    needs to move the motor just above the target pixel, then slowly slack it off
    while feeding increasing current until the coil is supported by the current alone.
    Then, needs to hover for a short amount of time so that a time average can be
    done.
    '''

    current_step = motor_pid(current_step, target_pixel)

    current, current_err = coil_pid(target_pixel)

    return current, current_err, current_step

def motor_step(delay, direction = 'forward'):
    pulse_pin     = DigitalOutputDevice(17)
    direction_pin = DigitalOutputDevice(27)
    enable_pin    = DigitalOutputDevice(22)

    if direction == 'forward':
        direction_pin.off()
    elif direction == 'reverse':
        direction_pin.on()

    enable_pin.on()

    pulse_pin.on()
    time.sleep(delay/2)
    pulse_pin.off()
    time.sleep(delay/2)

    enable_pin.off()
    return

def motor_jog(current_step, velocity, acceleration, steps, direction):

    degs_per_step    = 360/6400
    delay_at_given_v = (1/velocity)*degs_per_step

    accel_delays  = np.empty(steps)
    prev_velocity = 0
    next_velocity = 0
    v_sum_min = 1e-6 # THIS IS A GUESS. NEEDS TO BE REFINED

    for i, delay in enumerate(accel_delays):
        if i < steps/2:
            next_velocity = np.sqrt(prev_velocity**2 + 2*acceleration*degs_per_step)
        else:
            next_velocity = np.sqrt(prev_velocity**2 - 2*acceleration*degs_per_step)

        delay = (2*degs_per_step)/max(v_sum_min,(prev_velocity + next_velocity))

    delays = np.maximum(np.full(steps,delay_at_given_v),accel_delays)

    jog_steps = []
    jog_times = []

    multiplier = 1

    if direction == 'reverse': multiplier = -1

    for delay in delays:
        motor_step(delay, direction)
        current_step += multiplier
        jog_steps.append(current_step)
        jog_times.append(time.perf_counter())

    return jog_steps, jog_times, current_step

def get_camera_postion(bounds = [[720, 750],[125,510]], offset = 5, device = 1,
                       output_to_file = False, filename_base = './camera'):
    cap = cv2.VideoCapture(device)
    x_range = (bounds[0][0],bounds[0][1])
    y_range = (bounds[1][0],bounds[1][1])

    ret, frame = cap.read()

    bw     = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    img    = bw[y_range[0]:y_range[1],x_range[0]:x_range[1]]
    lines  = np.average(img, axis=1)
    cutoff = np.amin(lines) + offset

    _, w = bw.shape

    pixel_position = np.where(lines < cutoff)[0][0]

    if output_to_file:
        frame[y_range[0],x_range[0]:x_range[1],:] = [0,0,255]
        frame[y_range[1],x_range[0]:x_range[1],:] = [0,0,255]
        frame[y_range[0]:y_range[1],x_range[0],:] = [0,0,255]
        frame[y_range[0]:y_range[1],x_range[1],:] = [0,0,255]

        plt.plot(range(*y_range),lines)
        plt.axhline(cutoff)
        plt.savefig(''.join([filename_base,'_lines.png']))

        frame[pixel_position + y_range[0],:,:] = [[0,0,255]]*w

        cv2.imwrite(''.join([filename_base,'_image.png']), frame)

    cap.release()

    return pixel_position

def pixel_to_z(pixel, fitting_func = None, calibration_file = './calibration.npy'):
    calibration_params = np.load(calibration_file)

    if fitting_func = None:
        fitting_func = lin_fit

    z = fitting_func(pixel, *calibration_params['pixel_to_z_params'])

    return z

def lin_fit(pixel, m, b):
    return m*pixel + b

def motor_pid(current_step, target_pixel):
    # WRITE PID CONTROLLER FOR MOTOR HERE
    return current_step

def coil_pid(current_step, target_pixel):
    # WRITE PID CONTROLLER FOR COIL HERE
    return current, current_err, current_step

def get_coil_voltage():
    # PUT COIL VOLTAGE CODE HERE
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
