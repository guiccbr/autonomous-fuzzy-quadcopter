#! /Library/Frameworks/Python.framework/Versions/2.7/bin/python
# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

# ------------------------ Imports ----------------------------------#
import matplotlib.pyplot as plt
import numpy as np
import sparc
from math import sin, cos
import sys
# Add path to controllers && quadcopters
# Please, run script in {tfc-drone}/simulator dir
sys.path.append("../model/")
import time
import struct
from socket_server import serve_socket
from sys import argv


# ------------------------ Helpers ----------------------------------#
def send_floats(client, data):
    client.send(struct.pack('%sf' % len(data), *data))


def unpack_floats(msg, nfloats):

    return struct.unpack('f'*nfloats, msg)


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
        if (time.time()-start_sec) > TIMEOUT_SEC:
            return None

    return unpack_floats(msg, nfloats)


def receive_string(client):

    return client.recv(int(receive_floats(client, 1)[0]))


# - Timeout for receiving data from client
TIMEOUT_SEC = 1.0

# XXX Unrealistic GPS simulation (perfect accuracy
GPS_NOISE_METERS = 0

# - Noise Percentual
NOISE = 0.0

CONTROL_INIT_PATH = '/Users/gcc/Dropbox/Projects/gitRepos/github-tfc-drone/python/controller/vrep/dumped_controllers/'


# ------------------------ Constants  -------------------------------#
# - Control Signal:
UMAX = 50.0     # Range Max
UMIN = 0.0    # Range Min

# - Input Signal (Measured by sensors on the plant)
X_SIZE = 2  # Dimension of the input (measured by sensors of the plant)

# - Plant output reference (Measured by sensors on the plant)
REFMAX = 7.0     # Range Max
REFMIN = 0.0    # Range Min

# - Time Step
STEPTIME = 0.05
MAXTIME = 600


# ------------------------ Main Program  ---------------------------#
def test_sparc_model():

    # Instantiates figure for plotting results:
    fig = plt.figure()
    axes = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    ypoints = []
    refpoints = []

    # Instatiate Plant:
    y1 = 0.

    # Connect to Client (VREP)
    client = serve_socket(int(argv[1]))

    # Receive working directory path from client
    pyquadsim_directory = receive_string(client)

    prev_ref = 0.
    prev_y = 0.
    prev_u = 0.

    # Run for k steps
    k = 1
    while True:

        # Get core data from client
        clientData = receive_floats(client, 7)

        # Quit on timeout
        if not clientData:
            break

        timestepSeconds = clientData[0]
        curr_y = clientData[3]

        curr_ref = 5.0

        if k == 1:
            # Start prev_ values same as first:
            prev_y = curr_y
            prev_ref = curr_ref

        curr_x = generate_input(curr_y, prev_y, curr_ref, prev_ref, timestepSeconds)
        print curr_x

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


        print '(y, u) ', (curr_y, curr_u)

        # Prevent over-excursion of the control signal:
        if curr_u > UMAX:
            curr_u = UMAX
        if curr_u < UMIN:
            curr_u = UMIN

        # Send speed to Engines
        motors = (curr_u, curr_u, curr_u, curr_u)
        print motors
        send_floats(client, motors)

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
    k -- timestemp
    """

    refk = cos(0.005*k) + sin(0.007*k) + 3.7

    return refk


def generate_input(y, yprev, ref, refprev, t):
    curr_e = ref-y
    prev_e = refprev-yprev

    x = np.array([curr_e, (curr_e-prev_e)])
    return x

# ------------------------ Run Main Program ------------------------#
test_sparc_model()
