# ------------------------ Imports ----------------------------------#
from ...controller import sparc
from ...models.py import tank_model

import matplotlib.pyplot as plt
import numpy as np
from math import sin, cos


# ------------------------ Constants  -------------------------------#
# - Control Signal:
UMAX = 50.0     # Range Max
UMIN = -50.0    # Range Min

# - Input Signal (Measured by sensors on the plant)
X_SIZE = 2  # Dimension of the input (measured by sensors of the plant)

# - Plant output reference (Measured by sensors on the plant)
REFMAX = 8.0     # Range Max
REFMIN = 0.0    # Range Min

# - Time Step
STEPTIME = 0.05
MAXTIME = 600


# ------------------------ Main Program  ---------------------------#
def test_sparc_tank_model():
    
    # Instantiates figure for plotting results:
    fig = plt.figure()
    axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    ypoints = []
    refpoints = []

    # Instantiate Plant:
    y1 = 0.
    plant = tank_model.WaterTank(y1, STEPTIME)

    prev_ref = 0.
    prev_y = 0.
    prev_u = 0.

    # Run for k steps
    k = 1
    while k*STEPTIME < MAXTIME:
        
        # Get sample, and generates input
        curr_y = plant.get_y()
        curr_ref = reference(k)

        if k == 1:
            # Start prev_ values same as first:
            prev_y = curr_y
            prev_ref = curr_ref

        curr_x = generate_input(curr_y, prev_y, curr_ref, prev_ref)

        # Stores on list for plotting:
        ypoints.append(curr_y)
        refpoints.append(curr_ref)
       
        # Print result (curr_ref - curr_y)
        # print "Step:", k, " | y:", curr_y, " | err:", (curr_y - curr_ref), " | u:", curr_u
        
        # Updates prev_values:
        prev_y = curr_y
        prev_ref = curr_ref

        # On the first iteration, initializes the controller with the first values
        if k == 1:
            # Instantiates Controller:
            controller = sparc.SparcController((UMIN, UMAX), (REFMIN, REFMAX), X_SIZE, curr_x, curr_ref, curr_y, 1.)
            curr_u = controller.clouds[0].get_consequent()

        # Else Gets the output of the controller for the current input x
        else:
            curr_u = controller.update(curr_x, curr_y, curr_ref, prev_u)

        # Prevent over-excursion of the control signal:
        if curr_u > UMAX:
            curr_u = UMAX
        if curr_u < UMIN:
            curr_u = UMIN

        # Then Update the model with the control signal
        plant.update(curr_u)

        if (k*STEPTIME) % (MAXTIME/10.) == 0.:
            print 'k :', k
            print '#clouds: ', len(controller.clouds)

        # Update prev values
        prev_u = curr_u

        k += 1

    # Plotting
    kpoints = range(1, k)
    axes.plot(kpoints, refpoints, 'r')
    axes.plot(kpoints, ypoints, 'b')
    plt.show() 


# ------------------------ Global Methods  -------------------------#
def reference(k):
    """
    Outputs the desired output of the plant, on the time step k.
    Keyword arguments:
    k -- timestep
    """

    refk = cos(0.005*k) + sin(0.007*k) + 3.7

    return refk 


def generate_input(y, yprev, ref, refprev):
    curr_e = ref-y
    prev_e = refprev-yprev

    x = np.array([curr_e, (curr_e-prev_e)])
    return x

# ------------------------ Run Main Program ------------------------#
test_sparc_tank_model()
