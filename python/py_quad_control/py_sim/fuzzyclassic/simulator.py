#vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4
import sys
import matplotlib.pyplot as plt

sys.path.append("../model")

import quadcopter as quad
import model
import fuzzy_control
from math import sin
import math
from random import random

STEPTIME = 0.1
MAXSTEPS = 10000

ALTITUDE_ERROR_RANGE=[-2,2]
ALTITUDE_DERROR_RANGE=[-0.5,0.5]
ALTITUDE_RANGE=[ALTITUDE_ERROR_RANGE,ALTITUDE_DERROR_RANGE]

YAW_ERROR_RANGE=[-2,2]
YAW_DERROR_RANGE=[-0.5,0.5]
YAW_RANGE=[YAW_ERROR_RANGE,YAW_DERROR_RANGE]

PITCH_ERROR_RANGE=[-2,2]
PITCH_DERROR_RANGE=[-0.5,0.5]
PITCH_RANGE=[PITCH_ERROR_RANGE,PITCH_DERROR_RANGE]

ROLL_ERROR_RANGE=[-2,2]
ROLL_DERROR_RANGE=[-0.5,0.5]
ROLL_RANGE=[ROLL_ERROR_RANGE,ROLL_DERROR_RANGE]

def main():

    quadcopter = quad.quadcopter(model.model())

    # References
    altitudeReference = 10
    yawReference = 2
    pitchReference = 0.0
    rollReference = 0.0

    # Last values
    lastAltitudeError = 0
    lastYawError = 0
    lastPitchError = 0
    lastRollError = 0

    controller = fuzzy_control.fuzzyController(ALTITUDE_RANGE,[0,100])
    yawController = fuzzy_control.fuzzyController(YAW_RANGE,[-50,50])
    pitchController = fuzzy_control.fuzzyController(PITCH_RANGE,[-50,50])
    rollController = fuzzy_control.fuzzyController(ROLL_RANGE,[-50,50])

    altitudeHistory = []
    yawHistory = []
    pitchHistory = []
    rollHistory = []
    timeHistory = []
    
    for i in range(MAXSTEPS):
        
        # Do noise :)
        quadcopter.x[2]+=random()/2
        quadcopter.theta[2]+=random()/10
        quadcopter.theta[0]+=random()/10
        quadcopter.theta[1]+=random()/10

        currentAltitude = quadcopter.x[2]
        currentYaw = quadcopter.theta[2]
        currentPitch = quadcopter.theta[0]
        currentRoll = quadcopter.theta[1]
        

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
        altitudeControl = 10*controller.output([altitudeError,altitudeErrorDelta])
        yawControl = yawController.output([yawError,yawErrorDelta])
        pitchControl = pitchController.output([pitchError, pitchErrorDelta])
        rollControl = rollController.output([rollError, rollErrorDelta])

        motor1 = altitudeControl + yawControl + pitchControl
        motor2 = altitudeControl - yawControl                + rollControl
        motor3 = altitudeControl + yawControl - pitchControl
        motor4 = altitudeControl - yawControl                - rollControl

        print "Altitude=%s / Controller=%s / Error=%s / Derror=%s" % (currentAltitude, altitudeControl, altitudeError, altitudeErrorDelta)
        print "Yaw=%s / Controller=%s / Error=%s / Derror=%s" % (currentYaw, yawControl, yawError, yawErrorDelta)
        print "Pitch=%s / Controller=%s / Error=%s / Derror=%s" % (currentPitch, pitchControl, pitchError, pitchErrorDelta)
        print "Roll=%s / Controller=%s / Error=%s / Derror=%s" % (currentRoll, rollControl, rollError, rollErrorDelta)

        # Update quadcopter state
        quadcopter.update(STEPTIME, [motor1,motor2,motor3,motor4])
        
        # History maker
        altitudeHistory.append(currentAltitude)
        yawHistory.append(currentYaw)
        pitchHistory.append(currentPitch)
        rollHistory.append(currentRoll)
        timeHistory.append(i*STEPTIME)
  

    # Plot results 
    plt.plot(timeHistory,altitudeHistory,'b')
    plt.plot(timeHistory, [altitudeReference]*(len(timeHistory)),'g--')
    plt.plot(timeHistory, yawHistory, 'r')
    plt.plot(timeHistory, pitchHistory,'k')
    plt.plot(timeHistory, rollHistory, 'g')
    plt.show()

if __name__ == '__main__':
    main() 
