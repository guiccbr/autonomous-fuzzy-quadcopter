# ------------------------ Imports ----------------------------------#
from ...controller import sparc
from ...models.py import quadcopter as quad, model

import matplotlib.pyplot as plt
import numpy as np
import math

# ------------------------ Constants  -------------------------------#

# - Motor:
MOTOR_KV = 980
MOTOR_MAX_VOLTAGE = 11.1

# - Control Signal:
UPARKED = 1058.75  # Control signal sent to motors that is enough to balance

UMIN_ALT = -50  # Range Min
UMAX_ALT = +50  # Range Max

UMIN_PITCHROLL = -10
UMAX_PITCHROLL = +10

UMIN_YAW = -10
UMAX_YAW = +10

# Not that:
#   UMAX_ALT + UMAX_YAW + UMAX_PITCHROLL <= 2000 (MAX ENGINE CONTROL SIGNAL)
#   UMIN_ALT + UMIN_YAW + UMIN_PITCHROLL >= 1000 (MIN ENGINE CONTROL SIGNAL)

# - Input Signal (Measured by sensors on the plant)
X_SIZE = 2  # Dimension of the input (measured by sensors of the plant)

# - Plant output reference (Measured by sensors on the plant)
REFMAX_ALT = 8.0  # Range Max
REFMIN_ALT = 0.0  # Range Min

REFMIN_PITCHROLL = -45.0
REFMAX_PITCHROLL = +45.0

REFMIN_YAW = -150.0
REFMAX_YAW = +150.0

# - Time Step
STEPTIME = 0.1
MAXTIME = 2000

# - Noise Percent
NOISE = 0.0


# ------------------------ Main Program  ---------------------------#
def test_sparc_model(debug):
    # Instantiates figure for plotting motor angular velocities:
    fig_motors = plt.figure('Motors')
    axes_motors = fig_motors.add_axes([0.1, 0.1, 0.8, 0.8])
    motor_points = [np.array([]), np.array([]), np.array([]), np.array([])]
    # Instantiates figure for plotting alt results:
    fig_alt = plt.figure('Quadcopter alt')
    axes_alt = fig_alt.add_axes([0.1, 0.1, 0.8, 0.8])
    ypoints_alt = []
    refpoints_alt = []

    # Instantiates figure for plotting pitch results:
    fig_pitch = plt.figure('Quadcopter pitch')
    axes_pitch = fig_pitch.add_axes([0.1, 0.1, 0.8, 0.8])
    ypoints_pitch = []
    refpoints_pitch = []

    # Instantiates figure for plotting roll results:
    fig_roll = plt.figure('Quadcopter roll')
    axes_roll = fig_roll.add_axes([0.1, 0.1, 0.8, 0.8])
    ypoints_roll = []
    refpoints_roll = []

    # Instantiates figure for plotting yaw results:
    fig_yaw = plt.figure('Quadcopter yaw')
    axes_yaw = fig_yaw.add_axes([0.1, 0.1, 0.8, 0.8])
    ypoints_yaw = []
    refpoints_yaw = []

    # Instantiate Plant:
    quadcopter = quad.quadcopter(model.model())

    # Start prev_ values:
    prev_y = [0.0, 0.0, 0.0, 0.0]
    prev_ref = [0.0, 0.0, 0.0, 0.0]
    prev_u = [0.0, 0.0, 0.0, 0.0]

    # Reference
    new_reference = True

    # Run for k steps
    k = 1
    while k * STEPTIME < MAXTIME:

        # Get sample, and generates input

        quad_position = quadcopter.x
        quad_angles = quadcopter.theta  # angles: [pitch, roll, yaw]

        # y : [alt, yaw, pitch, roll]
        curr_y = [quad_position[2], quad_angles[2], quad_angles[0], quad_angles[1]]

        # Set references.
        curr_ref = [7.0, 0.0, 0.0, 0.0]

        # If reference curve has changed, update C.
        if k != 1 and new_reference:
            controller_alt.update_reference_range(REFMIN_ALT, REFMAX_ALT)
            controller_pitch.update_reference_range(REFMIN_PITCHROLL, REFMAX_PITCHROLL)
            controller_roll.update_reference_range(REFMIN_PITCHROLL, REFMAX_PITCHROLL)
            controller_yaw.update_reference_range(REFMIN_YAW, REFMAX_YAW)
            new_reference = False

        # Adding Noise:
        curr_y = curr_y * (1 + 2 * NOISE * np.random.rand(4, 1))

        curr_x = [generate_input(curr_y[0], prev_y[0], curr_ref[0], prev_ref[0]),
                  generate_input(curr_y[1], prev_y[1], curr_ref[1], prev_ref[1]),
                  generate_input(curr_y[2], prev_y[2], curr_ref[2], prev_ref[2]),
                  generate_input(curr_y[3], prev_y[3], curr_ref[3], prev_ref[3])]

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

        # Print result (curr_ref - curr_y)
        # print "Step:", k, " | y:", curr_y, " | err:", np.subtract(curr_y,curr_ref).tolist(), " | u:", curr_u

        prev_y = curr_y[:]
        prev_ref = curr_ref[:]

        if (k * STEPTIME) % (MAXTIME / 10) == 0:
            print 't[s]:', k * STEPTIME
            print '#clouds:', 'alt =', len(controller_alt.clouds), \
                'yaw =', len(controller_yaw.clouds), \
                'pitch =', len(controller_pitch.clouds), \
                'roll =', len(controller_roll.clouds)

        # On the first iteration, initializes the controller with the first values
        if k == 1:

            # Initial Control signal (defined as the error relative to the reference):
            e_alt = curr_x[0][0]
            e_yaw = curr_x[1][0]
            e_pitch = curr_x[2][0]
            e_roll = curr_x[3][0]

            curr_u = [e_alt, e_yaw, e_pitch, e_roll]

            # Instantiates Controller and does not update model:
            controller_alt = sparc.SparcController((UMIN_ALT, UMAX_ALT), (REFMIN_ALT, REFMAX_ALT), X_SIZE, curr_x[0],
                                                   curr_u[0], curr_ref[0], curr_y[0])
            controller_yaw = sparc.SparcController((UMIN_YAW, UMAX_YAW), (REFMIN_YAW, REFMAX_YAW), X_SIZE, curr_x[1],
                                                   curr_u[1], curr_ref[1], curr_y[1])
            controller_pitch = sparc.SparcController((UMIN_PITCHROLL, UMAX_PITCHROLL),
                                                     (REFMIN_PITCHROLL, REFMAX_PITCHROLL),
                                                     X_SIZE, curr_x[2], curr_u[2], curr_ref[2], curr_y[2])
            controller_roll = sparc.SparcController((UMIN_PITCHROLL, UMAX_PITCHROLL),
                                                    (REFMIN_PITCHROLL, REFMAX_PITCHROLL),
                                                    X_SIZE, curr_x[3], curr_u[3], curr_ref[3], curr_y[3])

        else:
            # Gets the output of the controller for the current input x
            alt_u = controller_alt.update(curr_x[0], curr_y[0], curr_ref[0], prev_u[0])
            # yaw_u = controller_yaw.update(curr_x[1], curr_y[1], curr_ref[1], prev_u[1])
            # pitch_u = controller_pitch.update(curr_x[2], curr_y[2], curr_ref[2], prev_u[2])
            # roll_u = controller_roll.update(curr_x[3], curr_y[3], curr_ref[3], prev_u[3])

            curr_u = [alt_u, 0.0, 0.0, 0.0]

        # Convert control signals to speed on engines:
        # Method I:
        m1 = UPARKED + curr_u[0] + curr_u[1] + curr_u[2]
        m2 = UPARKED + curr_u[0] - curr_u[1] + curr_u[3]
        m3 = UPARKED + curr_u[0] + curr_u[1] - curr_u[2]
        m4 = UPARKED + curr_u[0] - curr_u[1] - curr_u[3]

        # Method 2 (from V-REP quad model):
        # m1 = UPARKED + curr_u[0]*(1 + curr_u[1] + curr_u[2])
        # m2 = UPARKED + curr_u[0]*(1 - curr_u[1]             + curr_u[3])
        # m3 = UPARKED + curr_u[0]*(1 + curr_u[1] - curr_u[2])
        # m4 = UPARKED + curr_u[0]*(1 - curr_u[1]             - curr_u[3])

        # Stores on list for plotting:
        motor_points[0] = np.append(motor_points[0], [m1])
        motor_points[1] = np.append(motor_points[1], [m2])
        motor_points[2] = np.append(motor_points[2], [m3])
        motor_points[3] = np.append(motor_points[3], [m4])

        if debug == 'T':
            print '#Clouds Alt: ', len(controller_yaw.clouds)
            print 'u: ', curr_u
            print '(alt, yaw, pitch, roll): ', (curr_y[0], curr_y[1], curr_y[2], curr_y[3])
            print 'Engines: ', (m1, m2, m3, m4)

        # Updates the model
        quadcopter.update(STEPTIME, (conv_control_to_motor_speed(m1),
                                     conv_control_to_motor_speed(m2),
                                     conv_control_to_motor_speed(m3),
                                     conv_control_to_motor_speed(m4)))

        # Increment K
        k += 1

        # Store prev_u
        prev_u = curr_u

    # Plotting
    kpoints = [x * float(STEPTIME) for x in range(1, k)]

    axes_motors.plot(kpoints, motor_points[0], 'r')
    axes_motors.plot(kpoints, motor_points[1], 'y')
    axes_motors.plot(kpoints, motor_points[2], 'b')
    axes_motors.plot(kpoints, motor_points[3], 'g')

    axes_alt.plot(kpoints, refpoints_alt, 'r')
    axes_alt.plot(kpoints, ypoints_alt, 'b')

    axes_roll.plot(kpoints, refpoints_roll, 'r')
    axes_roll.plot(kpoints, ypoints_roll, 'b')

    axes_pitch.plot(kpoints, refpoints_pitch, 'r')
    axes_pitch.plot(kpoints, ypoints_pitch, 'b')

    axes_yaw.plot(kpoints, refpoints_yaw, 'r')
    axes_yaw.plot(kpoints, ypoints_yaw, 'b')

    plt.show()


# ------------------------ Global Methods  -------------------------#
def reference(a, k):
    """
    Outputs the desired output of the plant, on the time step k.
    Keyword arguments:
    k -- timestemp
    """

    # Exponential
    refk = a * (1 - math.e ** (-0.01 * k))

    # refk = 5*math.cos((2*math.pi/t)*k*STEPTIME) + 5*math.sin((1.4*2*math.pi/t)*k*STEPTIME) + 20

    return refk


def generate_input(y, yprev, ref, refprev):
    if math.isnan(y):
        curr_e = 0
    else:
        curr_e = ref - y

    prev_e = refprev - yprev

    # x = np.array([y, yprev])

    # x = np.array([curr_e, (curr_e-prev_e)/t])
    x = np.array([curr_e, (curr_e - prev_e)])
    return x


def conv_control_to_motor_speed(m):
    return ((m - 1000.) / 1000.) * (MOTOR_MAX_VOLTAGE * MOTOR_KV)

# ------------------------ Run Main Program ------------------------#
test_sparc_model(debug=0)
