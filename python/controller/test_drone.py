#vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

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
UMAX = 50.0     # Range Max 
UMIN = -50.0   # Range Min
UMIN_PITCHROLL = -5.0
UMAX_PITCHROLL = 5.0
UMIN_ANGLE = -1.0
UMAX_ANGLE = 1.0

U1 = 0.0 # Start control signal

# - Input Signal (Measured by sensors on the plant)
X_SIZE = 2  # Dimension of the input (measured by sensors of the plant)

# - Plant output reference (Measured by sensors on the plant)
REFMAX = 50     # Range Max
REFMIN = -50    # Range Min

REFMIN_ANGLE = -math.pi/4
REFMAX_ANGLE = math.pi/4

REFMIN_YAW_ANGLE = -math.pi
REFMAX_YAW_ANGLE = math.pi

# - Time Step
STEPTIME = 0.1
MAXSTEPS = 5000 

# ------------------------ Main Program  ---------------------------#
def test_sparc_model(debug):
    # Instantiates figure for plotting motor angular velocities:
    fig_motors = plt.figure('Motors')
    axes_motors = fig_motors.add_axes([0.1, 0.1, 0.8, 0.8])
    motor_points = [[], [],[],[]] 
    # Instantiates figure for plotting alt results:
    fig_alt = plt.figure('Quadcopter alt');
    axes_alt = fig_alt.add_axes([0.1, 0.1, 0.8, 0.8])
    ypoints_alt = []
    refpoints_alt = []

    # Instantiates figure for plotting pitch results:
    fig_pitch = plt.figure('Quadcopter pitch');
    axes_pitch = fig_pitch.add_axes([0.1, 0.1, 0.8, 0.8])
    ypoints_pitch = []
    refpoints_pitch = []
   
    # Instantiates figure for plotting roll results:
    fig_roll = plt.figure('Quadcopter roll');
    axes_roll = fig_roll.add_axes([0.1, 0.1, 0.8, 0.8])
    ypoints_roll = []
    refpoints_roll = []

    # Instantiates figure for plotting yaw results:
    fig_yaw = plt.figure('Quadcopter yaw');
    axes_yaw = fig_yaw.add_axes([0.1, 0.1, 0.8, 0.8])
    ypoints_yaw = []
    refpoints_yaw = []

    # Instatiate Plant:
    quadcopter = quad.quadcopter(model.model())
   
    # Start prev_ values:
    prev_y = [0.0, 0.0, 0.0, 0.0]
    prev_ref = [0.0,0.0,0.0,0.0]
    
    # Starting u value:
    curr_u = [U1, U1, U1, U1] 

    # Run for k steps
    for k in range(1, MAXSTEPS):
        
        # Get sample, and generates input
        quad_position = quadcopter.x
        quad_angles = quadcopter.theta # angles: [roll, pitch, yaw]

        # y : [alt, yaw, pitch, roll]
        curr_y = [quad_position[2], quad_angles[2], quad_angles[1], quad_angles[0]]
        curr_ref = [reference(10.0, k), reference(math.pi/6, k), 0.0, 0.0] 

        curr_x  = [generate_input(curr_y[0], prev_y[0], curr_ref[0], prev_ref[0], STEPTIME),
                generate_input(curr_y[1], prev_y[1], curr_ref[1], prev_ref[1],
                    STEPTIME),
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
        #print "Step:", k, " | y:", curr_y, " | err:", np.subtract(curr_y,curr_ref).tolist(), " | u:", curr_u

        prev_y = curr_y[:]
        prev_ref = curr_ref[:]

        if k%(MAXSTEPS/10) == 0:
            print 'k= ', k

        # On the first iteration, initializes the controller with the first values
        if k == 1:
            # Instantiates Controller and does not update model:
            controller_alt = sparc.SparcController((UMIN, UMAX), (REFMIN,
                REFMAX), X_SIZE, curr_x[0], curr_u[0], curr_ref[0], curr_y[0])
            controller_pitch = sparc.SparcController((UMIN_PITCHROLL,
                UMAX_PITCHROLL), (REFMIN_ANGLE, REFMAX_ANGLE), X_SIZE, curr_x[2], curr_u[2], curr_ref[2], curr_y[2])
            controller_roll = sparc.SparcController((UMIN_PITCHROLL,
                UMAX_PITCHROLL), (REFMIN_ANGLE, REFMAX_ANGLE), X_SIZE, curr_x[3], curr_u[3], curr_ref[3], curr_y[3])
            controller_yaw = sparc.SparcController((UMIN_ANGLE, UMAX_ANGLE), (REFMIN_YAW_ANGLE, REFMAX_YAW_ANGLE), X_SIZE, curr_x[1], curr_u[1], curr_ref[1], curr_y[1])

        else:
            # Gets the output of the controller for the current input x
            alt_u = controller_alt.update(curr_x[0], curr_y[0], curr_ref[0])
            yaw_u = controller_yaw.update(curr_x[1], curr_y[1], curr_ref[1])
            #pitch_u = controller_pitch.update(curr_x[2], curr_y[2], curr_ref[2])
            #roll_u = controller_roll.update(curr_x[3], curr_y[3], curr_ref[3]) 
            curr_u = [alt_u, yaw_u, 0.0, 0.0]

        # Speed on Engines:
        m1 = UPARKED + curr_u[0] + curr_u[1] + 0*curr_u[2] + curr_u[3]
        m2 = UPARKED + curr_u[0] - curr_u[1] + curr_u[2] + 0*curr_u[3]
        m3 = UPARKED + curr_u[0] + curr_u[1] - 0*curr_u[2] - curr_u[3]
        m4 = UPARKED + curr_u[0] - curr_u[1] - curr_u[2] - 0*curr_u[3]

        # Stores on list for plotting:
        motor_points[0].append(m1)
        motor_points[1].append(m2)
        motor_points[2].append(m3)
        motor_points[3].append(m4)

        if debug == 'T':
            print '#Clouds Alt: ', len(controller_alt.clouds)
            print 'u: ', curr_u
            print '(alt, yaw, pitch, roll): ', (curr_y[0], curr_y[1],
                    curr_y[2], curr_y[3])
            print 'Engines: ', (m1, m2,m3,m4)

        # Updates the model
        quadcopter.update(STEPTIME, (m1, m2, m3, m4))

    # Plotting
    kpoints = range(1, MAXSTEPS)

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
def reference(A, k):
    """
    Outputs the desired output of the plant, on the time step k.
    Keyword arguments:
    k -- timestemp
    """

    # Exponencial
    refk = A*(1-math.e**(-0.001*k)) 

    # Exponencial..Constant..Exponential..Constant..Exponential..
#   if(k<MAXSTEPS/6.0):
#       refk = 10*(1-math.e**(-0.0001*k))
#   if(k>=MAXSTEPS/6.0 and k<2*MAXSTEPS/6.0):
#       refk=10
#   if(k>=2*MAXSTEPS/6.0 and k<3*MAXSTEPS/6.0):
#       refk = 10*(1-math.e**(0.0001*(k-3*MAXSTEPS/6.0)))
#   if(k>=3*MAXSTEPS/6.0 and k<4*MAXSTEPS/6.0):
#       refk = 0 
#   if(k>=4*MAXSTEPS/6.0 and k<5*MAXSTEPS/6.0):
#       refk = 10*(1-math.e**(-0.0001*(k-5*MAXSTEPS/6.0)))
#   else:
#       refk = 10
        #refk = (math.pi)*math.cos(0.05*k) 

    return refk 

def generate_input(y, yprev, ref, refprev, t, gain=1):
    curr_e = ref-y
    prev_e = refprev-yprev

    #x = np.array([curr_e, (curr_e-prev_e)/t])
    x = np.array([curr_e, (curr_e-prev_e)])
    return x

# ------------------------ Run Main Program ------------------------#
test_sparc_model(sys.argv[1])
