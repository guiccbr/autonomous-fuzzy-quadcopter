# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import sys
# Add path to controllers && quadcopters
# Please, run script in {tfc-drone}/simulator dir
sys.path.append("../../model/")

# ------------------------ Imports ----------------------------------#
import matplotlib.pyplot as plt
import numpy as np
import tank_model
import sparc
from math import sin, cos
import quadcopter as quad
import model

# ------------------------ Constants  -------------------------------#
# - Control Signal:
UMAX = 1000000.0     # Range Max 
UMIN = 0    # Range Min
U1 = 0.0 # Start control signal

# - Input Signal (Measured by sensors on the plant)
X_SIZE = 2  # Dimension of the input (measured by sensors of the plant)

# - Plant output reference (Measured by sensors on the plant)
REFMAX = 100.0     # Range Max
REFMIN = -100.0    # Range Min

# - Time Step
STEPTIME = 0.50
MAXSTEPS = 3000

# ------------------------ Main Program  ---------------------------#
def test_sparc_model():
    
    # Instantiates figure for plotting HOVER results:
    fig_hover = plt.figure('Quadcopter Hover');
    axes_hover = fig_hover.add_axes([0.1, 0.1, 0.8, 0.8])
    ypoints_hover = []
    refpoints_hover = []

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
    prev_ref = [0.0, 0.0, 0.0, 0.0]
    
    # Starting u value:
    curr_u = [U1, U1, U1, U1] 

    # Run for k steps
    for k in range(1, MAXSTEPS):
        
        # Get sample, and generates input
        quad_position = quadcopter.x
        quad_angles = quadcopter.theta

        # y : [hover, yaw, pitch, roll]
        curr_y = [quad_position[2][0], quad_angles[2][0], quad_angles[1][0], quad_angles[0][0]]
        curr_ref = [0,0,0,0] 

        curr_x  = [generate_input(curr_y[0], prev_y[0], curr_ref[0], prev_ref[0], STEPTIME),
                generate_input(curr_y[1], prev_y[1], curr_ref[1], prev_ref[1], STEPTIME),
                generate_input(curr_y[2], prev_y[2], curr_ref[2], prev_ref[2], STEPTIME),
                generate_input(curr_y[3], prev_y[3], curr_ref[3], prev_ref[3], STEPTIME)]

        # Stores on list for plotting:
        ypoints_hover.append(curr_y[0])
        refpoints_hover.append(curr_ref[0])

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
        print "Step:", k, " | y:", curr_y, " | err:", (curr_y - curr_ref), " | u:", curr_u
        
        # On the first iteration, initializes the controller with the first values
        if k == 1:
            # Instantiates Controller and does not update model:
            controller_hover = sparc.SparcController((UMIN, UMAX), (REFMIN, REFMAX), X_SIZE,
                    curr_x, curr_u)
            controller_pitch = sparc.SparcController((UMIN, UMAX), (REFMIN, REFMAX), X_SIZE,
                    curr_x, curr_u)
            controller_roll = sparc.SparcController((UMIN, UMAX), (REFMIN, REFMAX), X_SIZE,
                    curr_x, curr_u)
            controller_yaw = sparc.SparcController((UMIN, UMAX), (REFMIN, REFMAX), X_SIZE,
                    curr_x, curr_u)
        else:
            # Gets the output of the controller for the current input x
            curr_u = [controller_hover.update(curr_x[0], curr_y[0], curr_ref[0]),
                    controller_yaw.update(curr_x[1], curr_y[1], curr_ref[1]),
                    controller_pitch.update(curr_x[2], curr_y[2], curr_ref[2]),
                    controller_roll.update(curr_x[3], curr_y[3], curr_ref[3]),
           
            # Speed on Engines:
            m1 = u[0] + u[1] + u[2] + u[3]
            m2 = u[0] - u[1] + u[2] - u[3]
            m3 = u[0] + u[1] - u[2] - u[3]
            m4 = u[0] - u[1] - u[2] + u[3]

            # Updates the model
            quadcopter.update(STEPTIME, (m1, m2, m3, m4)) 

    # Plotting
    kpoints = range(1, MAXSTEPS)

    axes_hover.plot(kpoints, refpoints_hover, 'r')
    axes_hover.plot(kpoints, ypoints_hover, 'b')
    
    axes_roll.plot(kpoints, refpoints_roll, 'r')
    axes_roll.plot(kpoints, ypoints_roll, 'b')

    axes_pitch.plot(kpoints, refpoints_pitch, 'r')
    axes_pitch.plot(kpoints, ypoints_pitch, 'b')

    axes_yaw.plot(kpoints, refpoints_yaw, 'r')
    axes_yaw.plot(kpoints, ypoints_yaw, 'b')

    plt.show() 

# ------------------------ Global Methods  -------------------------#
def reference(k):
    """
    Outputs the desired output of the plant, on the time step k.
    Keyword arguments:
    k -- timestemp
    """

    refk = 5.0
    #refk = cos(0.05*k) + sin(0.07*k) + 3.7     

    return refk 

def generate_input(y, yprev, ref, refprev, t):
    curr_e = y-ref
    prev_e = yprev - refprev

    x = np.array([curr_e, (curr_e-prev_e)/t])
    return x

# ------------------------ Run Main Program ------------------------#
test_sparc_model()
