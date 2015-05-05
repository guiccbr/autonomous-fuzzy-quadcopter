# Copyright 2006-2014 Coppelia Robotics GmbH. All rights reserved. 
# marc@coppeliarobotics.com
# www.coppeliarobotics.com
# 
# -------------------------------------------------------------------
# THIS FILE IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
# WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
# AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
# DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
# MISUSING THIS SOFTWARE.
# 
# You are free to use/modify/distribute this file for whatever purpose!
# -------------------------------------------------------------------
#
# This file was automatically created for V-REP release V3.2.0 on Feb. 3rd 2015

# Make sure to have the server side running in V-REP: 
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simExtRemoteApiStart(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

U_PARKED = 5.1

import vrep
import time

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
else:
    print ('Failed connecting to remote API server')

# Get handler for quadcopter base (from where position and angles are retrieved)
quadBase = vrep.simxGetObjectHandle(clientID, 'Quadricopter_base', vrep.simx_opmode_oneshot)

# Get handler for the floor
vrepFloor = vrep.simxGetObjectHandle(clientID, 'ResizableFloor_5_25', vrep.simx_opmode_oneshot)

for k in range(1,1000):
    # Get position
    errorcode, pos = vrep.simxGetObjectPosition(clientID, quadBase[-1], -1, vrep.simx_opmode_oneshot)

    # Get angle
    errorcode, angle = vrep.simxGetObjectOrientation(clientID, quadBase[-1], vrepFloor[-1], vrep.simx_opmode_oneshot)

    # Motor Velocities
    m = [U_PARKED for i in range(0, 4)]

    # Send Motor Velocities
    for i in range(0, 4):
        vrep.simxSetFloatSignal(clientID, 'signal_m'+str(i), m[i], vrep.simx_opmode_oneshot)

    time.sleep(0.01)


vrep.simxFinish(clientID)
print 'Finished'