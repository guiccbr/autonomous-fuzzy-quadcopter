# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

# ------------------------ Imports ----------------------------------#
import matplotlib.pyplot as plt
import numpy as np
import tank_model
import sparc
from math import sin, cos
import math

# ------------------------ Constants  -------------------------------#
# - Control Signal:
UMAX = 100.0     # Range Max
UMIN = -100.0    # Range Min
U1 = 0.0 # Start control signal

# - Input Signal (Measured by sensors on the plant)
X_SIZE = 2  # Dimension of the input (measured by sensors of the plant)

# - Plant output reference (Measured by sensors on the plant)
REFMAX = 15.0     # Range Max
REFMIN = 0.0    # Range Min

# - Time Step
STEPTIME = 0.05
MAXTIME = 600

# ------------------------ Main Program  ---------------------------#
def test_sparc_model():
    
    # Instantiates figure for plotting results:
    fig = plt.figure();
    axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    ypoints = []
    refpoints = []

    # Instatiate Plant:
    y1 = 0.0
    plant = tank_model.WaterTank(y1, STEPTIME)
   
    # Start prev_ values:
    prev_y = 0.0
    prev_ref = 0.0
    
    # Starting u value:
    curr_u = U1 

    # Run for k steps
    k = 1
    while k*STEPTIME < MAXTIME:
        
        # Get sample, and generates input
        curr_y = plant.get_y()
        curr_ref = reference(k)
        curr_x = generate_input(curr_y, prev_y, curr_ref, prev_ref, STEPTIME)

        # Stores on list for plotting:
        ypoints.append(curr_y)
        refpoints.append(curr_ref)
       
        # Print result (curr_ref - curr_y)
        #print "Step:", k, " | y:", curr_y, " | err:", (curr_y - curr_ref), " | u:", curr_u
        
        # Updates prev_values:
        prev_y = curr_y
        prev_ref = curr_ref

        # On the first iteration, initializes the controller with the first values
        if k == 1:
            # Instantiates Controller and does not update model:
            controller = sparc.SparcController((UMIN, UMAX), (REFMIN, REFMAX), X_SIZE,
                    curr_x, curr_u, curr_ref, curr_y)
        
        else:
            # Gets the output of the controller for the current input x
            curr_u = controller.update(curr_x, curr_y, curr_ref)
            
            # Updates the model
            plant.update(curr_u) 

        k = k + 1
        if((k*STEPTIME)%(MAXTIME/10) == 0):
            print 'k :', k
            print '#clouds: ', len(controller.clouds)

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
    k -- timestemp
    """

    refk = 5.0
    #refk = cos(0.005*k) + sin(0.007*k) + 3.7

    return refk 

def generate_input(y, yprev, ref, refprev, t):
    curr_e = ref-y
    prev_e = refprev-yprev

    x = np.array([curr_e, (curr_e-prev_e)/t])
    return x

# ------------------------ Run Main Program ------------------------#
test_sparc_model()
