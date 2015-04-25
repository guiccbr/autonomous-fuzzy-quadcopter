# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import sys
# Add path to controllers && quadcopters
# Please, run script in {tfc-drone}/simulator dir
sys.path.append("../model/")

# ------------------------ Imports ----------------------------------#
import matplotlib.pyplot as plt
import numpy as np
import tank_model
import sparc
import math
import quadcopter as quad
import model

# ------------------------ Constants  -------------------------------#
# - Control Signal:
UPARKED = 639.3    # Control signal sent to motors that is enough to balance 
UMAX_ALT = 100.0     # Range Max
UMIN_ALT = -100.0   # Range Min
UMIN_PITCHROLL = -0.001
UMAX_PITCHROLL = 0.001
UMIN_YAW = -10.0
UMAX_YAW = 10.0

U1 = 0.0  # Start control signal

# - Input Signal (Measured by sensors on the plant)
X_SIZE = 2  # Dimension of the input (measured by sensors of the plant)

# - Plant output reference (Measured by sensors on the plant)
REFMAX_ALT = 15.0     # Range Max
REFMIN_ALT = -15.0    # Range Min

REFMIN_PITCHROLL = -math.pi/10000.0
REFMAX_PITCHROLL = math.pi/10000.0

REFMIN_YAW = -math.pi/10
REFMAX_YAW = math.pi/10

# - Time Step
STEPTIME = 0.1
MAXTIME = 2000

# - Noise Percentual
NOISE = 0.10


# ------------------------ Main Program  ---------------------------#
def test_sparc_model(debug):
    # Instantiates figure for plotting motor angular velocities:
    fig_motors = plt.figure('Motors')
    axes_motors = fig_motors.add_axes([0.1, 0.1, 0.8, 0.8])
    motor_points = [[], [],[],[]] 
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

    # Instatiate Plant:
    quadcopter = quad.quadcopter(model.model())
   
    # Start prev_ values:
    prev_y = [0.0, 0.0, 0.0, 0.0]
    prev_ref = [0.0, 0.0, 0.0, 0.0]
    
    # Starting u value:
    curr_u = [U1, U1, U1, U1] 

    # Run for k steps
    k = 1
    while k*STEPTIME < MAXTIME:
        
        # Get sample, and generates input

        quad_position = quadcopter.x
        quad_angles = quadcopter.theta # angles: [pitch, roll, yaw]

        # y : [alt, yaw, pitch, roll]
        curr_y = [quad_position[2], quad_angles[2], quad_angles[0], quad_angles[1]]
        curr_ref = [reference(10, k, 20), 2.0, 0.0, 0.0]

        # Adding Noise:
        curr_y = curr_y*(1+2*NOISE*np.random.rand(4))

        curr_x = [generate_input(curr_y[0], prev_y[0], curr_ref[0], prev_ref[0], STEPTIME),
                  generate_input(curr_y[1], prev_y[1], curr_ref[1], prev_ref[1], STEPTIME),
                  generate_input(curr_y[2], prev_y[2], curr_ref[2], prev_ref[2], STEPTIME),
                  generate_input(curr_y[3], prev_y[3], curr_ref[3], prev_ref[3], STEPTIME)]

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

        if (k*STEPTIME) % (MAXTIME/10) == 0:
            print 't[s]:', k*STEPTIME
            print '#clouds:', 'alt =', len(controller_alt.clouds),\
                'yaw =', len(controller_yaw.clouds),\
                'pitch =', len(controller_pitch.clouds),\
                'roll =', len(controller_roll.clouds)

        # On the first iteration, initializes the controller with the first values
        if k == 1:
            # Instantiates Controller and does not update model:
            controller_alt = sparc.SparcController((UMIN_ALT, UMAX_ALT), (REFMIN_ALT, REFMAX_ALT), X_SIZE, curr_x[0],
                                                   curr_u[0], curr_ref[0], curr_y[0])
            controller_yaw = sparc.SparcController((UMIN_YAW, UMAX_YAW), (REFMIN_YAW, REFMAX_YAW), X_SIZE, curr_x[1],
                                                   curr_u[1], curr_ref[1], curr_y[1])
            controller_pitch = sparc.SparcController((UMIN_PITCHROLL, UMAX_PITCHROLL), (REFMIN_PITCHROLL, REFMAX_PITCHROLL), X_SIZE, curr_x[2], curr_u[2], curr_ref[2], curr_y[2])
            controller_roll = sparc.SparcController((UMIN_PITCHROLL, UMAX_PITCHROLL), (REFMIN_PITCHROLL, REFMAX_PITCHROLL), X_SIZE, curr_x[3], curr_u[3], curr_ref[3], curr_y[3])

        else:
            # Gets the output of the controller for the current input x
            alt_u = controller_alt.update(curr_x[0], curr_y[0], curr_ref[0])
            yaw_u = controller_yaw.update(curr_x[1], curr_y[1], curr_ref[1])
            pitch_u = controller_pitch.update(curr_x[2], curr_y[2], curr_ref[2])
            roll_u = controller_roll.update(curr_x[3], curr_y[3], curr_ref[3])

            curr_u = [alt_u, yaw_u, pitch_u, roll_u]

        # Speed on Engines:
        m1 = UPARKED + curr_u[0] + curr_u[1] + curr_u[2]
        m2 = UPARKED + curr_u[0] - curr_u[1]             + curr_u[3]
        m3 = UPARKED + curr_u[0] + curr_u[1] - curr_u[2]
        m4 = UPARKED + curr_u[0] - curr_u[1]             - curr_u[3]

        # Stores on list for plotting:
        motor_points[0].append(m1)
        motor_points[1].append(m2)
        motor_points[2].append(m3)
        motor_points[3].append(m4)

        if debug == 'T':
            print '#Clouds Alt: ', len(controller_yaw.clouds)
            print 'u: ', curr_u
            print '(alt, yaw, pitch, roll): ', (curr_y[0], curr_y[1], curr_y[2], curr_y[3])
            print 'Engines: ', (m1, m2,m3,m4)

        # Updates the model
        quadcopter.update(STEPTIME, (m1, m2, m3, m4))

        # Increment K
        k += 1

    # Plotting
    kpoints = [x*float(STEPTIME) for x in range(1, k)]

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
def reference(A, k, t):
    """
    Outputs the desired output of the plant, on the time step k.
    Keyword arguments:
    k -- timestemp
    """

    # Exponencial
    # refk = A*(1-math.e**(-0.01*k))

    refk = 5*math.cos((2*math.pi/t)*k*STEPTIME) + 5*math.sin((1.4*2*math.pi/t)*k*STEPTIME) + 20

    return refk


def generate_input(y, yprev, ref, refprev, t, gain=1):

    if math.isnan(y):
        curr_e = 0
    else:
        curr_e = ref-y

    prev_e = refprev-yprev

    # x = np.array([y, yprev])

    # x = np.array([curr_e, (curr_e-prev_e)/t])
    x = np.array([curr_e, (curr_e-prev_e)])
    return x

# ------------------------ Run Main Program ------------------------#
test_sparc_model(sys.argv[1])