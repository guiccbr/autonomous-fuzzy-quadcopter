#vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import sys
import matplotlib.pyplot as plt

sys.path.append("../../model")
sys.path.append("../../Fuzzy_Classico")

import model
import fuzzy_control
from math import sin
import math
import vrep
from random import random
import time

STEPTIME = 0.05
MAXSTEPS = 1000

ALTITUDE_ERROR_RANGE=[-1,1]
ALTITUDE_DERROR_RANGE=[-0.5,0.5]
ALTITUDE_RANGE=[ALTITUDE_ERROR_RANGE,ALTITUDE_DERROR_RANGE]

YAW_ERROR_RANGE=[-1,1]
YAW_DERROR_RANGE=[-0.5,0.5]
YAW_RANGE=[YAW_ERROR_RANGE,YAW_DERROR_RANGE]

PITCH_ERROR_RANGE=[-2,2]
PITCH_DERROR_RANGE=[-0.5,0.5]
PITCH_RANGE=[PITCH_ERROR_RANGE,PITCH_DERROR_RANGE]

ROLL_ERROR_RANGE=[-2,2]
ROLL_DERROR_RANGE=[-0.5,0.5]
ROLL_RANGE=[ROLL_ERROR_RANGE,ROLL_DERROR_RANGE]

def main():

    # Initializes vrep api
    print ('Program started')

    vrep.simxFinish(-1)  # just in case, close all opened connections
    clientID = vrep.simxStart('127.0.0.1', 19999, True, True, 5000, 5)
    if clientID != -1:
        print ('Connected to remote API server')
        res, objs = vrep.simxGetObjects(clientID, vrep.sim_handle_all, vrep.simx_opmode_oneshot_wait)
        if res == vrep.simx_return_ok:
            print ('Number of objects in the scene: ',len(objs))
        else:
            print ('Remote API function call returned with error code: ',res)
            sys.exit()
    else:
        print ('Failed connecting to remote API server')

    # Get handler for quadcopter base (from where position and angles are retrieved)
    quadBase = vrep.simxGetObjectHandle(clientID, 'Quadricopter_base#0', vrep.simx_opmode_oneshot_wait)

    # Get handler for the floor
    vrepFloor = vrep.simxGetObjectHandle(clientID, 'ResizableFloor_5_25', vrep.simx_opmode_oneshot_wait)

    # References
    altitudeReference = 10
    yawReference = 0.0
    pitchReference = 0.0
    rollReference = 0.0

    # Last values
    lastAltitudeError = 0
    lastYawError = 0
    lastPitchError = 0
    lastRollError = 0

    controller = fuzzy_control.fuzzyController(ALTITUDE_RANGE,[-2,2])
    yawController = fuzzy_control.fuzzyController(YAW_RANGE,[-0.5,0.5])
    pitchController = fuzzy_control.fuzzyController(PITCH_RANGE,[-0.1,0.1])
    rollController = fuzzy_control.fuzzyController(ROLL_RANGE,[-0.1,0.1])

    altitudeHistory = []
    yawHistory = []
    pitchHistory = []
    rollHistory = []
    timeHistory = []

    for i in range(MAXSTEPS):

         # Get sample, and generates input
        error_code, quad_position = vrep.simxGetObjectPosition(clientID, quadBase[-1], -1, vrep.simx_opmode_streaming)
        error_code, quad_angles = vrep.simxGetObjectOrientation(clientID, quadBase[-1], vrepFloor[-1], vrep.simx_opmode_streaming) # angles: [pitch, roll, yaw]


        currentAltitude = quad_position[2]
        currentYaw = quad_angles[2]
        currentPitch = quad_angles[0]
        currentRoll = quad_angles[1]


        # Update reference
        altitudeReference=(10-10*math.exp(-i*0.005))

        # Evaluate input signals

        # Altitude Error
        altitudeError = currentAltitude - altitudeReference
        altitudeErrorDelta = altitudeError - lastAltitudeError

        # Yaw error
        yawError = currentYaw - yawReference
        yawErrorDelta = yawError - lastYawError

        # Pitch error
        pitchError = currentPitch - pitchReference
        pitchErrorDelta = pitchError - lastPitchError

        # Roll error
        rollError = currentRoll - rollReference
        rollErrorDelta = rollError - lastRollError

        # Update signals
        lastAltitudeError = altitudeError
        lastYawError = yawError
        lastPitchError = pitchError
        lastRollError = rollError

        # Control
        altitudeControl = controller.output([altitudeError,altitudeErrorDelta])
        yawControl = yawController.output([yawError,yawErrorDelta])
        pitchControl = pitchController.output([pitchError, pitchErrorDelta])
        rollControl = rollController.output([rollError, rollErrorDelta])

        motor1 = 5.335 + altitudeControl - yawControl - pitchControl + rollControl
        motor2 = 5.335 + altitudeControl + yawControl + pitchControl + rollControl
        motor3 = 5.335 + altitudeControl - yawControl + pitchControl - rollControl
        motor4 = 5.335 + altitudeControl + yawControl - pitchControl - rollControl

        print "Altitude=%s / Controller=%s / Error=%s / Derror=%s" % (currentAltitude, altitudeControl, altitudeError, altitudeErrorDelta)
        print "Yaw=%s / Controller=%s / Error=%s / Derror=%s" % (currentYaw, yawControl, yawError, yawErrorDelta)
        print "Pitch=%s / Controller=%s / Error=%s / Derror=%s" % (currentPitch, pitchControl, pitchError, pitchErrorDelta)
        print "Roll=%s / Controller=%s / Error=%s / Derror=%s" % (currentRoll, rollControl, rollError, rollErrorDelta)

        # Update quadcopter state
        vrep.simxSetFloatSignal(clientID, 'signal_m0', motor1, vrep.simx_opmode_oneshot)
        vrep.simxSetFloatSignal(clientID, 'signal_m1', motor2, vrep.simx_opmode_oneshot)
        vrep.simxSetFloatSignal(clientID, 'signal_m2', motor3, vrep.simx_opmode_oneshot)
        vrep.simxSetFloatSignal(clientID, 'signal_m3', motor4, vrep.simx_opmode_oneshot)

        # History maker
        altitudeHistory.append(currentAltitude)
        yawHistory.append(currentYaw)
        pitchHistory.append(currentPitch)
        rollHistory.append(currentRoll)
        timeHistory.append(i*STEPTIME)

        time.sleep(0.05)


    # Plot results
    plt.plot(timeHistory,altitudeHistory,'b')
    plt.plot(timeHistory, [altitudeReference]*(len(timeHistory)),'g--')
    plt.plot(timeHistory, yawHistory, 'r')
    plt.plot(timeHistory, pitchHistory,'k')
    plt.plot(timeHistory, rollHistory, 'g')
    plt.show()

if __name__ == '__main__':
    main()
