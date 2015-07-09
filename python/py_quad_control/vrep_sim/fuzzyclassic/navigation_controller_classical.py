import numpy as np
import sparc
import fuzzy_control
import math

# LATITUDE_RANGE = [[-0.2,0.2], [-0.1, 0.1]]
# LONGITUDE_RANGE = [[-0.2,0.2],[-0.1, 0.1]]
# CONTROL_RANGE = [-23.,23.]
# VARIANCE = [0.5, 0.05]

LATITUDE_RANGE = [[-2.,2.], [-1., 1.]]
LONGITUDE_RANGE = LATITUDE_RANGE
CONTROL_RANGE = [-40.,40.]
VARIANCE = [1., 0.5]

class navigation_controller:

    def __init__(self, initial_longitude_input, initial_latitude_input, initial_position_reference, current_position):
        
        self.longitude_u = initial_longitude_input
        self.latitude_u = initial_latitude_input
        self.previous_position = [0,0]
        self.previous_position_reference = [0,0]

        self.latitudeController = fuzzy_control.fuzzyController(LATITUDE_RANGE,CONTROL_RANGE, variance=VARIANCE)
        self.longitudeController = fuzzy_control.fuzzyController(LONGITUDE_RANGE,CONTROL_RANGE,variance=VARIANCE)

    def update(self, current_position, current_position_reference):
        
        # Generate inputs
        longitude_input = self.generate_input_nav(current_position[0], self.previous_position[0], current_position_reference[0], self.previous_position_reference[0])
        latitude_input = self.generate_input_nav(current_position[1], self.previous_position[1], current_position_reference[1], self.previous_position_reference[1])

        self.longitude_u = self.longitudeController.output(longitude_input)
        self.latitude_u = self.latitudeController.output(latitude_input)
        
        # Update
        self.previous_position = current_position
        self.previous_position_reference = current_position_reference

        return (self.longitude_u, self.latitude_u)

    def generate_input_nav(self, y, yprev, ref, refprev):
        # Calculate current error
        if math.isnan(y):
            curr_e = 0
        else:
            curr_e = -ref + y

        # Calculate previous error
        prev_e = refprev - yprev

        # Generate Inputs
        x = np.array([curr_e, (curr_e - prev_e)])
        return x
