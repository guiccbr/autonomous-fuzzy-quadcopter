import sys

# Add path to controllers && quadcopters
# Please, run script in {tfc-drone}/simulator dir
sys.path.append("../controller/src/")
sys.path.append("../model/")

from datetime import datetime
import model
import quadcopter as quad
import sparc
import quadcopter_control_system as qcs
import model_3d
import math
import numpy as np
from visual import *

MOTOR_MAX = 1000000
MOTOR_MIN = -1000000
MAX_STEP=1000

def simulator():

    # Instantiate quadcopter
    quadcopter = quad.quadcopter(model.model())
    
    # Instantiate controllers
    init_input = np.array([0,0])
    altitude_controller = sparc.SparcController((MOTOR_MIN,MOTOR_MAX),(-10,100),2,init_input,0)
    yaw_controller = sparc.SparcController((MOTOR_MIN,MOTOR_MAX),(-10,100),2,init_input,0)
    roll_controller = sparc.SparcController((MOTOR_MIN,MOTOR_MAX),(-10,100),2,init_input,0)
    pitch_controller = sparc.SparcController((MOTOR_MIN,MOTOR_MAX),(-10,100),2,init_input,0)

    # Main quadcopter controller
    quadcopter_control = qcs.quadcopter_control_system(altitude_controller, yaw_controller, roll_controller, pitch_controller)

    # Quadcopter 3d model
    scene = model_3d.create_3d_world()
    quad3d = model_3d.QuadcopterGraphic3d((0,0,0), 0, 0, 0, scene, 2, math.pi/2, True, True) 

    # run simulation
    i=0
    prev_time = datetime.now() 
    start_time = prev_time
    for i in range(MAX_STEP):
    
        # Needed for correctly temporizing the 3d_model
        rate(30) # Executes 30 times per second

        # Get the current state of the model:
        y = quadcopter.x

        theta = quadcopter.theta
        #theta = np.array([raw_theta[0][0], raw_theta[1][0], raw_theta[2][0]]) 

        # Send data to the 3d model (roll, pitch, yaw)
        quad3d.set_pos((y[0][0], y[1][0], y[2][0])) 
        quad3d.set_angles(theta[0][0], theta[1][0], theta[2][0])

        # Calculate the output and update model (alt, yaw, pitch, roll):
        state = [y[2][0], theta[2][0], theta[1][0], theta[0][0]]
	
        reference = [50, 0, 0, 0] 
        inputs = quadcopter_control.update(state, reference)
  
        # Timing
        # I can think of two approaches here. Syncronized and non-syncronized.
        # We should try both to see which one works better.
   
        # Asyncronized implementation
        curr_time = datetime.now() 
        timedelta = (curr_time-prev_time).total_seconds()
        time_passed = (curr_time-start_time).total_seconds()
        #quadcopter.update(timedelta, inputs)
        quadcopter.update(timedelta, (1000000,1000000,1000000,1000000)) # Test Max Thrust
        #quadcopter.update(timedelta, (0,0,0,0)) # Test Free-fall
        prev_time = curr_time        
        print 'time: ', "{:0.2f}".format(time_passed), ' | x:', y[0][0],y[1][0],y[2][0], ' | a:', theta[0][0],theta[1][0],theta[2][0], ' | i:', inputs

if __name__ == "__main__":
    simulator()
