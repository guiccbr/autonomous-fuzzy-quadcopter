#! /Library/Frameworks/Python.framework/Versions/2.7/bin/python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

# ------------------------ Imports ----------------------------------#
from sys import argv
import time
import struct
import math
import pickle

import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import numpy as np

import sparc
from socket_server import serve_socket
from geometry import rotate

# Nav
import navigation_controller

# ------------------------ Helpers ----------------------------------#
def send_floats(client, data):
    client.send(struct.pack('%sf' % len(data), *data))


def unpack_floats(msg, nfloats):
    return struct.unpack('f' * nfloats, msg)


def receive_floats(client, nfloats):
    # We use 32-bit floats
    msgsize = 4 * nfloats

    # Implement timeout
    start_sec = time.time()
    remaining = msgsize
    msg = ''
    while remaining > 0:
        msg += client.recv(remaining)
        remaining -= len(msg)
        if (time.time() - start_sec) > TIMEOUT_SEC:
            return None

    return unpack_floats(msg, nfloats)


def receive_string(client):
    return client.recv(int(receive_floats(client, 1)[0]))

# ------------------------ Constants  -------------------------------#
RAD2DEG = 180.0/math.pi


# - Motor:
MOTOR_KV = 980
MOTOR_MAX_VOLTAGE = 11.1

# - Control Signal:
UPARKED = 1225.0

UMIN_ALT = -1225.0  # Range Min
UMAX_ALT = +1225.0  # Range Max

UMIN_PITCHROLL = -100.0
UMAX_PITCHROLL = +100.0

UMIN_YAW = -100.0
UMAX_YAW = +100.0

# Note that:
# UMAX_ALT + UMAX_YAW + UMAX_PITCHROLL <= 2000 (MAX ENGINE CONTROL SIGNAL)
#   UMIN_ALT + UMIN_YAW + UMIN_PITCHROLL >= 1000 (MIN ENGINE CONTROL SIGNAL)

# - Input Signal (Measured by sensors on the plant)
X_SIZE = 2  # Dimension of the input (measured by sensors of the plant)

# - Plant output reference (Measured by sensors on the plant)
REFMAX_ALT = 3.0  # Range Max
REFMIN_ALT = 0.0  # Range Min

REFMIN_PITCHROLL = -450.0
REFMAX_PITCHROLL = 450.0

REFMIN_YAW = -1000.0
REFMAX_YAW = +1000.0

# - Timeout for receiving data from client
TIMEOUT_SEC = 1.0

# XXX Unrealistic GPS simulation (perfect accuracy
GPS_NOISE_METERS = 0

# - Noise Percentual
NOISE = 0.0

CONTROL_INIT_PATH = '/Users/gcc/Dropbox/Projects/gitRepos/github-tfc-drone/python/controller/vrep/dumped_controllers/'

# Artifical Damping
#Kv_pr = 0.3 / (REFMAX_PITCHROLL - REFMIN_PITCHROLL)
# Kv_pr = 0.0 / (REFMAX_PITCHROLL - REFMIN_PITCHROLL)
# Kv_y = 0.7 / (REFMAX_YAW - REFMIN_YAW)
# Kv_h = 10.0 / (REFMAX_ALT - REFMIN_ALT
Kv_pr = 0.0 / (REFMAX_PITCHROLL - REFMIN_PITCHROLL)
Kv_y = 0.0 / (REFMAX_YAW - REFMIN_YAW)
Kv_h = 0.0 / (REFMAX_ALT - REFMIN_ALT)


# ------------------------ Main Program  ---------------------------#
def test_sparc_model(print_stuff=False, control=[True] * 4, readfiles=[True] * 4, recordfiles=[True] * 4):
    # Connect to Client (VREP)
    client = serve_socket(int(argv[1]))

    # Receive working directory path from client
    pyquadsim_directory = receive_string(client)

    # List for plotting time axis
    kpoints = []
    time_elapsed = 0.0

    # Instantiates figure for plotting motor angular velocities:
    fig_motors = plt.figure('Motors', figsize=(14,10))
    axes_motors = fig_motors.add_axes([0.1, 0.1, 0.8, 0.8])
    motor_points = [[], [], [], []]

    # Instantiates figure for plotting stabilization results:
    fig_control, (axes_alt, axes_pitch, axes_roll, axes_yaw) = plt.subplots(4, 1, sharex=True, sharey=False,
                                                                            figsize=(14,10))

    fig_control.canvas.set_window_title('Quadcopter Stability - Commanded(RED), Performed(BLUE)')
    axes_alt.set_title('Alt')
    axes_pitch.set_title('Pitch')
    axes_roll.set_title('Roll')
    axes_yaw.set_title('Yaw')

    ypoints_alt = []
    refpoints_alt = []

    ypoints_pitch = []
    refpoints_pitch = []

    ypoints_roll = []
    refpoints_roll = []

    ypoints_yaw = []
    refpoints_yaw = []

    # Instantiates figure for plotting navigation results:
    fig_nav = plt.figure('Quadcopter Navigation', figsize=(14,10))
    axes_x = fig_nav.add_subplot(221)
    axes_x.set_title('X')
    ypoints_x = []
    refpoints_x = []

    axes_y = fig_nav.add_subplot(222)
    axes_y.set_title('Y')
    ypoints_y = []
    refpoints_y = []

    axes_xy = fig_nav.add_subplot(212)
    axes_xy.set_title('XY')


    # Instantiates figure for plotting control signals:
    fig_control, (ax_c_alt, ax_c_yaw, ax_c_pitch, ax_c_roll) = plt.subplots(4, 1, sharex=True, sharey=False,
                                                                            figsize=(14,10))

    fig_control.canvas.set_window_title('Control Effort')

    ax_c_alt.set_title('Alt')
    ax_c_yaw.set_title('Yaw')
    ax_c_pitch.set_title('Pitch')
    ax_c_roll.set_title('Roll')
    c_alt_points = []
    c_pitch_points = []
    c_roll_points = []
    c_yaw_points = []

    # Instantiates figure for plotting clouds:
    fig_clouds = plt.figure('Data Clouds', figsize=(14,10))
    ax_cloud_alt = fig_clouds.add_subplot(231)
    ax_cloud_alt.set_title('Alt')

    ax_cloud_yaw = fig_clouds.add_subplot(232)
    ax_cloud_yaw.set_title('Yaw')

    ax_cloud_pitch = fig_clouds.add_subplot(233)
    ax_cloud_pitch.set_title('Pitch')

    ax_cloud_roll = fig_clouds.add_subplot(234)
    ax_cloud_roll.set_title('Roll')

    ax_cloud_nav_x = fig_clouds.add_subplot(235)
    ax_cloud_nav_x.set_title('Nav X')

    ax_cloud_nav_y = fig_clouds.add_subplot(236)
    ax_cloud_nav_y.set_title('Nav Y')

    # Reference
    new_reference = True

    # Forever loop will be halted by VREP client or by exception
    first_iteration = True
    k = 1  # Iterations Counter

    # Read Starting clouds from files:
    if readfiles[0]:
        f_controller_alt_init = open(CONTROL_INIT_PATH + 'controller_alt_init')
        controller_alt_init = pickle.load(f_controller_alt_init)
        if print_stuff: print 'SPARC: Alt Controller Loaded', f_controller_alt_init.name
    if readfiles[1]:
        f_controller_yaw_init = open(CONTROL_INIT_PATH + 'controller_yaw_init')
        controller_yaw_init = pickle.load(f_controller_yaw_init)
        if print_stuff: print 'SPARC: Yaw Controller Loaded', f_controller_yaw_init.name
    if readfiles[2]:
        f_controller_pitch_init = open(CONTROL_INIT_PATH + 'controller_pitch_init')
        controller_pitch_init = pickle.load(f_controller_pitch_init)
        if print_stuff: print 'SPARC: Pitch Controller Loaded', f_controller_pitch_init.name
    if readfiles[3]:
        f_controller_roll_init = open(CONTROL_INIT_PATH + 'controller_roll_init')
        controller_roll_init = pickle.load(f_controller_roll_init)
        if print_stuff: print 'SPARC: Roll Controller Loaded', f_controller_roll_init.name
    if readfiles[4]:
        f_controller_nav_y_init = open(CONTROL_INIT_PATH + 'controller_navy_init')
        controller_nav_y_init = pickle.load(f_controller_nav_y_init)
        if print_stuff: print 'SPARC: Roll Controller Loaded', f_controller_nav_y_init.name
    else:
        controller_nav_y_init = None
    if readfiles[5]:
        f_controller_nav_x_init = open(CONTROL_INIT_PATH + 'controller_navx_init')
        controller_nav_x_init = pickle.load(f_controller_nav_x_init)
        if print_stuff: print 'SPARC: Roll Controller Loaded', f_controller_nav_x_init.name
    else:
        controller_nav_x_init = None

    # Instantiate Navigation Controllers and References
    #x_reference = Reference([0.0, 0.0, 5., 5.], [10., 10., 10., 10.], 1)
    #y_reference = Reference([0.0, 5., 5., 0.], [10., 10., 10., 10.], 1)

    # x_reference = Reference([0.0, 5., 5., 0.], [5., 10., 5., 10.], 1)
    # y_reference = Reference([0.0, 5., 5., 0.], [5., 10., 5., 10.], 1)
    x_reference = Reference([0.0], [10.])
    y_reference = Reference([0.0], [10.])

    nav = navigation_controller.navigation_controller(np.array([0., 0.]), np.array([0., 0.]), np.array([0., 0.]),
                                                      np.array([0., 0.]), controller_nav_y_init, controller_nav_x_init)

    if control[0]:
        #alt_reference = Reference([1.0, 1.0, 4., 4.], [5., 5., 24., 5.], 1)
        alt_reference = Reference([3., 5.0, 4.0, 7.0, 2.0], [10., 5., .5, 5., 1.], mode=2)
    else:
        alt_reference = Reference([0.0], [10.])

    if control[1]:
        #yaw_reference = Reference([0.0, 0.0, 45., 45.], [5., 5., 24., 5.], 1)
        yaw_reference = Reference([0.0], [10.])
    else:
        yaw_reference = Reference([0.0], [10.])

    prev_yaw = 0.0
    prev_pitch = 0.0
    prev_roll = 0.0
    curr_yaw = 0.0
    curr_pitch = 0.0
    curr_roll = 0.0

    while True:
        # Get core data from client
        clientData = receive_floats(client, 7)

        if print_stuff: print 'SPARC: k=', k
        if print_stuff: print 'SPARC: ClientData Received:'

        #if not clientData:
        #   break

        # Quit on timeout
        if not clientData:
            break

        # Unpack IMU data
        timestepSeconds = clientData[0]
        positionXMeters = clientData[1]
        positionYMeters = clientData[2]
        positionZMeters = clientData[3]
        alphaRadians = clientData[4]
        betaRadians = clientData[5]
        gammaRadians = clientData[6]

        # Add some Guassian noise (example)
        # positionXMeters = random.gauss(positionXMeters, GPS_NOISE_METERS)
        # positionYMeters = random.gauss(positionYMeters, GPS_NOISE_METERS)

        # Convert Euler angles to pitch, roll, yaw
        # See http://en.wikipedia.org/wiki/Flight_dynamics_(fixed-wing_aircraft) for positive/negative orientation
        rollAngleRadians, pitchAngleRadians = rotate((alphaRadians, betaRadians), gammaRadians)
        pitchAngleRadians = -pitchAngleRadians
        yawAngleRadians = -gammaRadians

        yawAngleDegrees = yawAngleRadians*RAD2DEG
        pitchAngleDegrees = pitchAngleRadians*RAD2DEG
        rollAngleDegrees = rollAngleRadians*RAD2DEG

        dr = rollAngleDegrees - prev_roll
        dp = pitchAngleDegrees - prev_pitch
        dy = yawAngleDegrees - prev_yaw

        if dr >= 0:
            dr = math.fmod(dr + 180., 2 * 180.) - 180.
        else:
            dr = math.fmod(dr - 180., 2 * 180.) + 180.

        if dp >= 0:
            dp = math.fmod(dp + 180., 2 * 180.) - 180.
        else:
            dp = math.fmod(dp - 180., 2 * 180.) + 180.

        if dy >= 0:
            dy = math.fmod(dy + 180., 2 * 180.) - 180.
        else:
            dy = math.fmod(dy - 180., 2 * 180.) + 180.

        prev_yaw = yawAngleDegrees
        prev_pitch = pitchAngleDegrees
        prev_roll = rollAngleDegrees
        curr_yaw = curr_yaw + dy
        curr_pitch = curr_pitch + dp
        curr_roll = curr_roll + dr

        # Get altitude directly from position Z
        altitudeMeters = positionZMeters

        # y : [alt, yaw, pitch, roll]
        curr_y = [altitudeMeters, curr_yaw, curr_pitch, curr_roll]

        # Get current references for navigation:
        x_curr_ref = x_reference.get_next(timestepSeconds)
        y_curr_ref = y_reference.get_next(timestepSeconds)

        # Set References.
        curr_ref = [0., 0., 0., 0.]
        curr_ref[0] = alt_reference.get_next(timestepSeconds)
        curr_ref[1] = yaw_reference.get_next(timestepSeconds)
        curr_ref[2], curr_ref[3] = nav.update([positionXMeters, positionYMeters], [x_curr_ref, y_curr_ref])

        # Convert pitch/roll input from radians to degrees
        curr_ref[2] = -curr_ref[2]*RAD2DEG
        curr_ref[3] = -curr_ref[3]*RAD2DEG

        if print_stuff: print 'SPARC: curr_y =', curr_y
        if print_stuff: print 'SPARC: curr_ref =', curr_ref


        # Start prev_ values as the first values on the first iteration:
        if first_iteration:
            prev_y = curr_y[:]
            prev_ref = curr_ref[:]

        # If reference curve has changed, update C.
        if (not first_iteration) and new_reference:
            if control[0]:
                controller_alt.update_reference_range(REFMIN_ALT, REFMAX_ALT)
            if control[2]:
                controller_pitch.update_reference_range(REFMIN_PITCHROLL, REFMAX_PITCHROLL)
            if control[3]:
                controller_roll.update_reference_range(REFMIN_PITCHROLL, REFMAX_PITCHROLL)
            if control[1]:
                controller_yaw.update_reference_range(REFMIN_YAW, REFMAX_YAW)
            new_reference = False

        # Generate Input (Pack error and error derivative to form x)

        curr_x = [generate_input(curr_y[0], prev_y[0], curr_ref[0], prev_ref[0], timestepSeconds),
                  generate_input(curr_y[1], prev_y[1], curr_ref[1], prev_ref[1], timestepSeconds),
                  generate_input(curr_y[2], prev_y[2], curr_ref[2], prev_ref[2], timestepSeconds),
                  generate_input(curr_y[3], prev_y[3], curr_ref[3], prev_ref[3], timestepSeconds)]

        # Stores on list for plotting:
        ypoints_alt.append(curr_y[0])
        refpoints_alt.append(curr_ref[0])

        # Stores on list for plotting:
        ypoints_pitch.append(curr_y[2])
        refpoints_pitch.append(curr_ref[2])

        # Stores on list for plotting:
        ypoints_roll.append(curr_y[3])
        refpoints_roll.append(curr_ref[3])

        # Stores on list for plotting:
        ypoints_yaw.append(curr_y[1])
        refpoints_yaw.append(curr_ref[1])

        # Stores on list for plotting:
        ypoints_x.append(positionXMeters)
        refpoints_x.append(x_curr_ref)

        # Stores on list for plotting:
        ypoints_y.append(positionYMeters)
        refpoints_y.append(y_curr_ref)


        # if print_stuff: print result (curr_ref - curr_y)
        # if print_stuff: print "Step:", k, " | y:", curr_y, " | err:", np.subtract(curr_y,curr_ref).tolist(), " | u:", curr_u

        # if k % 100 == 0:
        #     if print_stuff: print 't[s]:', k*timestepSeconds
        #     if print_stuff: print '#clouds:', 'alt =', len(controller_alt.clouds),\
        #         'yaw =', len(controller_yaw.clouds),\
        #         'pitch =', len(controller_pitch.clouds),\
        #         'roll =', len(controller_roll.clouds)

        # On the first iteration, initializes the controller with the first values
        if first_iteration:

            curr_u = [0., 0., 0., 0.]
            prev_u = [0., 0., 0., 0.]

            # Instantiates Controller and does not update model:

            # Altitude Controller
            if readfiles[0]:
                controller_alt = controller_alt_init
                curr_u[0] = controller_alt.update(curr_x[0], curr_y[0], curr_ref[0], prev_u[0]) if control[0] else 0.0
            else:
                controller_alt = sparc.SparcController((UMIN_ALT, UMAX_ALT), (REFMIN_ALT, REFMAX_ALT),
                                                       X_SIZE, curr_x[0], curr_ref[0], curr_y[0])
                curr_u[0] = controller_alt.clouds[0].get_consequent() if control[0] else 0.0

            # Yaw Controller
            if readfiles[1]:
                controller_yaw = controller_yaw_init
                curr_u[1] = controller_yaw.update(curr_x[1], curr_y[1], curr_ref[1], prev_u[1]) if control[1] else 0.0
            else:
                controller_yaw = sparc.SparcController((UMIN_YAW, UMAX_YAW), (REFMIN_YAW, REFMAX_YAW),
                                                       X_SIZE, curr_x[1], curr_ref[1], curr_y[1])
                curr_u[1] = controller_yaw.clouds[0].get_consequent() if control[1] else 0.0

            # Pitch Controller
            if readfiles[2]:
                controller_pitch = controller_pitch_init
                curr_u[2] = controller_pitch.update(curr_x[2], curr_y[2], curr_ref[2], prev_u[2]) if control[2] else 0.0
            else:
                controller_pitch = sparc.SparcController((UMIN_PITCHROLL, UMAX_PITCHROLL),
                                                         (REFMIN_PITCHROLL, REFMAX_PITCHROLL),
                                                         X_SIZE, curr_x[2], curr_ref[2], curr_y[2])
                curr_u[2] = controller_pitch.clouds[0].get_consequent() if control[2] else 0.0

            # Roll Controller
            if readfiles[3]:
                controller_roll = controller_roll_init
                curr_u[3] = controller_roll.update(curr_x[3], curr_y[3], curr_ref[3], prev_u[3]) if control[3] else 0.0
            else:
                controller_roll = sparc.SparcController((UMIN_PITCHROLL, UMAX_PITCHROLL),
                                                        (REFMIN_PITCHROLL, REFMAX_PITCHROLL),
                                                        X_SIZE, curr_x[3], curr_ref[3], curr_y[3])
                curr_u[3] = controller_roll.clouds[0].get_consequent() if control[3] else 0.0

            first_iteration = False

        else:

            # if print_stuff: print tested inputs
            if control[0]:
                if print_stuff: print 'SPARC: Alt_input = ', curr_x[0], curr_y[0], curr_ref[0]
            if control[1]:
                if print_stuff: print 'SPARC: Yaw_input = ', curr_x[1], curr_y[1], curr_ref[1]
            if control[2]:
                if print_stuff: print 'SPARC: Pitch_input = ', curr_x[2], curr_y[2], curr_ref[2]
            if control[3]:
                if print_stuff: print 'SPARC: Roll_input = ', curr_x[3], curr_y[3], curr_ref[3]

            # Gets the output of the controller for the current input x
            if control[0]:
                alt_u = controller_alt.update(curr_x[0], curr_y[0], curr_ref[0], prev_u[0])
            if control[1]:
                yaw_u = controller_yaw.update(curr_x[1], curr_y[1], curr_ref[1], prev_u[1])
            if control[2]:
                pitch_u = controller_pitch.update(curr_x[2], curr_y[2], curr_ref[2], prev_u[2])
            if control[3]:
                roll_u = controller_roll.update(curr_x[3], curr_y[3], curr_ref[3], prev_u[3])

            # if print_stuff: print 'SPARC: Yaw_control = ', yaw_u

            # curr_u = [alt_u, yaw_u, pitch_u, roll_u]
            curr_u = [0.0, 0.0, 0.0, 0.0]
            damped_u = [0.0, 0.0, 0.0, 0.0]

            curr_u[0] = alt_u if control[0] else 0.0
            curr_u[1] = yaw_u if control[1] else 0.0
            curr_u[2] = pitch_u if control[2] else 0.0
            curr_u[3] = roll_u if control[3] else 0.0

            # Add artificial damping
            # print 'timestep: ', timestepSeconds
            damped_u[0] = curr_u[0] - Kv_h * (curr_y[0] - prev_y[0])/timestepSeconds
            damped_u[1] = curr_u[1] - Kv_y * (curr_y[1] - prev_y[1])/timestepSeconds
            damped_u[2] = curr_u[2] - Kv_pr * (curr_y[2] - prev_y[2])/timestepSeconds
            damped_u[3] = curr_u[3] - Kv_pr * (curr_y[3] - prev_y[3])/timestepSeconds

            # if print_stuff: print 'SPARC: Damping (undamped_u, damped_u) = ', curr_u[3], damped_u[3]

            curr_u[:] = damped_u[:]

        # Prevent Over Excursion
        if curr_u[0] > UMAX_ALT:
            curr_u[0] = UMAX_ALT
        if curr_u[0] < UMIN_ALT:
            curr_u[0] = UMIN_ALT

        if curr_u[1] > UMAX_YAW:
            curr_u[1] = UMAX_YAW
        if curr_u[1] < UMIN_YAW:
            curr_u[1] = UMIN_YAW

        if curr_u[2] > UMAX_PITCHROLL:
            curr_u[2] = UMAX_PITCHROLL
        if curr_u[2] < UMIN_PITCHROLL:
            curr_u[2] = UMIN_PITCHROLL

        if curr_u[3] > UMAX_PITCHROLL:
            curr_u[3] = UMAX_PITCHROLL
        if curr_u[3] < UMIN_PITCHROLL:
            curr_u[3] = UMIN_PITCHROLL

        # Speed on Engines:
        motors = [0.] * 4
        #motors[0] = float(UPARKED + curr_u[0] + curr_u[1] - curr_u[2] + curr_u[3])
        #motors[1] = float(UPARKED + curr_u[0] - curr_u[1] + curr_u[2] + curr_u[3])
        #motors[2] = float(UPARKED + curr_u[0] + curr_u[1] + curr_u[2] - curr_u[3])
        #motors[3] = float(UPARKED + curr_u[0] - curr_u[1] - curr_u[2] - curr_u[3])

        # In case the quadcopter is upside down, use another dynamic (shut down alt. controller)
        if curr_y[2] >= 90 or curr_y[3] >= 90:
            curr_u[0] = 0.0

        motors[0] = float((UPARKED + curr_u[0]) * (1 - curr_u[1]/100. + curr_u[2]/100. - curr_u[3]/100.))
        motors[1] = float((UPARKED + curr_u[0]) * (1 + curr_u[1]/100. + curr_u[2]/100. + curr_u[3]/100.))
        motors[2] = float((UPARKED + curr_u[0]) * (1 - curr_u[1]/100. - curr_u[2]/100. + curr_u[3]/100.))
        motors[3] = float((UPARKED + curr_u[0]) * (1 + curr_u[1]/100. - curr_u[2]/100. - curr_u[3]/100.))

        # Stores on list for plotting:
        motor_points[0].append(motors[0])
        motor_points[1].append(motors[1])
        motor_points[2].append(motors[2])
        motor_points[3].append(motors[3])

        c_alt_points.append(UPARKED + curr_u[0])
        c_yaw_points.append((UPARKED + curr_u[0])*curr_u[1]/100)
        c_pitch_points.append((UPARKED + curr_u[0])*curr_u[2]/100)
        c_roll_points.append((UPARKED + curr_u[0])*curr_u[3]/100)

        kpoints.append(time_elapsed)

        # Send speed to Engines
        send_floats(client, motors)
        if print_stuff: print 'SPARC: Control Signal Sent!'
        if print_stuff: print 'SPARC: Control Signal: ', curr_u

        # debug = 'T'
        # if debug == 'T':
        #     if print_stuff: print 'SPARC: #CloudsYaw= ', len(controller_yaw.clouds)

        # Update prev values
        prev_u = curr_u[:]
        prev_y = curr_y[:]
        prev_ref = curr_ref[:]


        # Increment K
        k += 1
        time_elapsed += timestepSeconds

    # Plotting and saving
    if k > 10:

        # Plot Motors
        axes_motors.plot(kpoints, motor_points[0], 'r')
        axes_motors.plot(kpoints, motor_points[1], 'y')
        axes_motors.plot(kpoints, motor_points[2], 'b')
        axes_motors.plot(kpoints, motor_points[3], 'g')

        # Plot Altitude (Reference and Result)
        axes_alt.plot(kpoints, refpoints_alt, 'r')
        axes_alt.plot(kpoints, ypoints_alt, 'b')

        # Plot Roll (Reference and Result)
        axes_roll.plot(kpoints, refpoints_roll, 'r')
        axes_roll.plot(kpoints, ypoints_roll, 'b')

        # Plot Pitch (Reference and Result)
        axes_pitch.plot(kpoints, refpoints_pitch, 'r')
        axes_pitch.plot(kpoints, ypoints_pitch, 'b')

        # Plot Yaw (Reference and Result)
        axes_yaw.plot(kpoints, refpoints_yaw, 'r')
        axes_yaw.plot(kpoints, ypoints_yaw, 'b')

        # Plot Y (Reference and Result)
        axes_y.plot(kpoints, refpoints_y, 'r')
        axes_y.plot(kpoints, ypoints_y, 'b')

        # Plot X (Reference and Result)
        axes_x.plot(kpoints, refpoints_x, 'r')
        axes_x.plot(kpoints, ypoints_x, 'b')

        # Plot XY (Reference and Result)
        axes_xy.plot(refpoints_x, refpoints_y, 'r')
        axes_xy.plot(ypoints_x, ypoints_y, 'b')

        # Reconfigure limits of XY to keep proportion.
        xlim_range = max(axes_xy.get_xlim()) - min(axes_xy.get_xlim())
        ylim_range = max(axes_xy.get_ylim()) - min(axes_xy.get_ylim())

        if xlim_range > ylim_range:
            axes_xy.set_xlim(axes_xy.get_xlim())
            axes_xy.set_ylim(axes_xy.get_xlim())
        else:
            axes_xy.set_xlim(axes_xy.get_ylim())
            axes_xy.set_ylim(axes_xy.get_ylim())

        # Plot Control Signals
        ax_c_alt.plot(kpoints, c_alt_points, 'r')
        ax_c_pitch.plot(kpoints, c_pitch_points, 'y')
        ax_c_roll.plot(kpoints, c_roll_points, 'b')
        ax_c_yaw.plot(kpoints, c_yaw_points, 'g')

        # Plot Clouds
        if control[0]:
            e, de, u, r = [], [], [], []
            for c in controller_alt.clouds:
                e.append(c.zf[0])
                de.append(c.zf[1])
                u.append(c.zf[2])
                r = c.r
                ax_cloud_alt.add_patch(Ellipse((e[-1], de[-1]), r[0], r[1], facecolor='none', linestyle='dashed'))
            im_alt = ax_cloud_alt.scatter(e, de, c=u, cmap=plt.cm.jet)
            ax_cloud_alt.set_xlabel('Error')
            ax_cloud_alt.set_ylabel('DError')
            plt.colorbar(im_alt, ax=ax_cloud_alt)
            if recordfiles[0]:
                f_alt = open(CONTROL_INIT_PATH + 'controller_alt_init', 'w')
                pickle.dump(controller_alt, f_alt)
                f_alt.close()
                if print_stuff: print 'SPARC: Controller Serialized', f_alt.name
        if control[1]:
            e, de, u, r = [], [], [], []
            for c in controller_yaw.clouds:
                e.append(c.zf[0])
                de.append(c.zf[1])
                u.append(c.zf[2])
                r = c.r
                ax_cloud_yaw.add_patch(Ellipse((e[-1], de[-1]), r[0], r[1], facecolor='none', linestyle='dashed'))
            im_yaw = ax_cloud_yaw.scatter(e, de, c=u, cmap=plt.cm.jet)
            ax_cloud_yaw.set_xlabel('Error')
            ax_cloud_yaw.set_ylabel('DError')
            plt.colorbar(im_yaw, ax=ax_cloud_yaw)
            if recordfiles[1]:
                f_yaw = open(CONTROL_INIT_PATH + 'controller_yaw_init', 'w')
                pickle.dump(controller_yaw, f_yaw)
                f_yaw.close()
                if print_stuff: print 'SPARC: Controller Serialized', f_yaw.name
        if control[2]:
            e, de, u, r = [], [], [], []
            for c in controller_pitch.clouds:
                e.append(c.zf[0])
                de.append(c.zf[1])
                u.append(c.zf[2])
                r = c.r
                ax_cloud_pitch.add_patch(Ellipse((e[-1], de[-1]), r[0], r[1], facecolor='none', linestyle='dashed'))
            im_pitch = ax_cloud_pitch.scatter(e, de, c=u, cmap=plt.cm.jet)
            ax_cloud_pitch.set_xlabel('Error')
            ax_cloud_pitch.set_ylabel('DError')
            plt.colorbar(im_pitch, ax=ax_cloud_pitch)
            if recordfiles[2]:
                f_pitch = open(CONTROL_INIT_PATH + 'controller_pitch_init', 'w')
                pickle.dump(controller_pitch, f_pitch)
                f_pitch.close()
                if print_stuff: print 'SPARC: Controller Serialized', f_pitch.name
        if control[3]:
            e, de, u, r = [], [], [], []
            for c in controller_roll.clouds:
                e.append(c.zf[0])
                de.append(c.zf[1])
                u.append(c.zf[2])
                r = c.r
                ax_cloud_roll.add_patch(Ellipse((e[-1], de[-1]), r[0], r[1], facecolor='none', linestyle='dashed'))
            im_roll = ax_cloud_roll.scatter(e, de, c=u, cmap=plt.cm.jet)
            ax_cloud_roll.set_xlabel('Error')
            ax_cloud_roll.set_ylabel('DError')
            plt.colorbar(im_roll, ax=ax_cloud_roll)
            if recordfiles[3]:
                f_roll = open(CONTROL_INIT_PATH + 'controller_roll_init', 'w')
                pickle.dump(controller_roll, f_roll)
                f_roll.close()
                if print_stuff: print 'SPARC: Controller Serialized', f_roll.name

        # Navigation Clouds (Y)
        e, de, u, r = [], [], [], []
        for c in nav.latitudeController.clouds:
            e.append(c.zf[0])
            de.append(c.zf[1])
            u.append(c.zf[2])
            r = c.r
            ax_cloud_nav_y.add_patch(Ellipse((e[-1], de[-1]), r[0], r[1], facecolor='none', linestyle='dashed'))
        im_nav_y = ax_cloud_nav_y.scatter(e, de, c=u, cmap=plt.cm.jet)
        ax_cloud_nav_y.set_xlabel('Error')
        ax_cloud_nav_y.set_ylabel('DError')
        plt.colorbar(im_nav_y, ax=ax_cloud_nav_y)
        if recordfiles[4]:
            f_nav_y = open(CONTROL_INIT_PATH + 'controller_navy_init', 'w')
            pickle.dump(nav.latitudeController, f_nav_y)
            f_nav_y.close()
            if print_stuff: print 'SPARC: Controller Serialized', f_nav_y.name


        # Navigation Clouds (X)
        e, de, u, r = [], [], [], []
        for c in nav.longitudeController.clouds:
            e.append(c.zf[0])
            de.append(c.zf[1])
            u.append(c.zf[2])
            r = c.r
            ax_cloud_nav_x.add_patch(Ellipse((e[-1], de[-1]), r[0], r[1], facecolor='none', linestyle='dashed'))
        im_nav_x = ax_cloud_nav_x.scatter(e, de, c=u, cmap=plt.cm.jet)
        ax_cloud_nav_x.set_xlabel('Error')
        ax_cloud_nav_x.set_ylabel('DError')
        plt.colorbar(im_nav_x, ax=ax_cloud_nav_x)
        if recordfiles[5]:
            f_nav_x = open(CONTROL_INIT_PATH + 'controller_navx_init', 'w')
            pickle.dump(nav.longitudeController, f_nav_x)
            f_nav_x.close()
            if print_stuff: print 'SPARC: Controller Serialized', f_nav_x.name

        plt.show()


# ------------------------ Global Methods  -------------------------#
class Reference:
    def __init__(self, list_amplitudes, list_times, mode=0, l_err=0.001):

        if len(list_amplitudes) != len(list_times):
            raise ValueError('Amplitudes and Times must have the same sizes')

        self.time_acumulated = 0.0
        self.time_passed = 0.0
        self.amplitudes = np.copy(list_amplitudes)
        self.times = np.copy(list_times)
        self.curr_index = 0
        self.mode = mode
        self.logistic_err = l_err

    def get_next(self, steptime):
        dy = (self.amplitudes[(self.curr_index + 1) % len(self.amplitudes)] - self.amplitudes[
            self.curr_index]) / float(self.times[self.curr_index])
        if self.mode == 0:
            # STEP MODE
            self.time_passed += steptime
            if self.time_passed > self.time_acumulated:
                self.curr_index = (self.curr_index + 1) % len(self.amplitudes)
                self.time_acumulated += self.times[self.curr_index]
            return self.amplitudes[self.curr_index]
        elif self.mode == 1:
            # RAMP MODE
            self.time_passed += steptime
            if self.time_passed > self.times[self.curr_index]:
                self.curr_index = (self.curr_index + 1) % len(self.amplitudes)
                self.time_passed = 0.0
            return self.amplitudes[self.curr_index] + dy*self.time_passed
        else:
            # LOGISTIC MODE
            if self.time_passed > self.times[self.curr_index]:
                self.curr_index = (self.curr_index + 1) % len(self.amplitudes)
                self.time_passed = 0.0
            if self.time_passed == 0.0:
                self.a = self.amplitudes[self.curr_index]
                self.k = self.amplitudes[(self.curr_index + 1) % len(self.amplitudes)]
                self.m = self.times[self.curr_index]/2.0
                # Calculate B for an error
                self.b = math.log(abs((self.k-self.a)/self.logistic_err - 1))/self.m
            curr_ref = self.a + (self.k-self.a)/(1 + math.exp(-self.b*(self.time_passed - self.m)))
            self.time_passed += steptime
            return curr_ref

def reference(k, t):
    # Exponencial
    #refk = A*(1-math.e**(-0.01*k))

    refk = 5 * math.cos((2 * 180. / t) * k * t) + 5 * math.sin((1.4 * 2 * 180. / t) * k * t) + 10

    return refk


def generate_input(y, yprev, ref, refprev, t, gain=1):
    # Calculate current error
    if math.isnan(y):
        curr_e = 0
    else:
        curr_e = ref - y

    # Calculate previous error
    prev_e = refprev - yprev

    # Generate Inputs
    x = np.array([curr_e, (curr_e - prev_e)])
    return x

# ------------------------ Run Main Program ------------------------#
test_sparc_model(print_stuff=False, control=[True, True, True, True], readfiles=[True, True, True, True, False, False],
                 recordfiles=[False, False, False, False, False, False])
