import numpy as np
import sparc
import math

UMIN_LAT = -0.4
UMAX_LAT = 0.4
REFMIN_LAT = -10.0
REFMAX_LAT = 10.0

UMIN_LON = -0.4
UMAX_LON = 0.4
REFMIN_LON = -10.0
REFMAX_LON = 10.0

X_SIZE = 2

class navigation_controller:



    def __init__(self, initial_longitude_input, initial_latitude_input, initial_position_reference, current_position,
                 controller_lat_init=None, controller_long_init=None):
        
        self.longitude_u = 0.0
        self.latitude_u = 0.0
        self.previous_position = np.array([0, 0])
        self.previous_position_reference = np.array([0, 0])

        if controller_long_init is None:
            self.longitudeController = sparc.SparcController((UMIN_LON,UMAX_LON), (REFMIN_LON, REFMAX_LON), X_SIZE,
                                                             initial_longitude_input, initial_position_reference[0],
                                                             current_position[0])
        else:
            self.longitudeController = controller_long_init

        if controller_long_init is None:
            self.latitudeController = sparc.SparcController((UMIN_LAT,UMAX_LAT), (REFMIN_LAT, REFMAX_LAT), X_SIZE,
                                                        initial_latitude_input, initial_position_reference[1],
                                                        current_position[1])
        else:
            self.latitudeController = controller_lat_init


    def update(self, current_position, current_position_reference):
        
        # Generate inputs
        longitude_input = self.generate_input_nav(current_position[0], self.previous_position[0],
                                                  current_position_reference[0], self.previous_position_reference[0])
        latitude_input = self.generate_input_nav(current_position[1], self.previous_position[1],
                                                 current_position_reference[1], self.previous_position_reference[1])

        self.longitude_u = self.longitudeController.update(longitude_input, current_position[0],
                                                           current_position_reference[0], self.longitude_u)
        self.latitude_u = self.latitudeController.update(latitude_input, current_position[1],
                                                         current_position_reference[1], self.latitude_u)
        
        # Update
        self.previous_position = current_position
        self.previous_position_reference = current_position_reference

        return self.longitude_u, self.latitude_u

    def generate_input_nav(self, y, yprev, ref, refprev):
        # Calculate current error
        if math.isnan(y):
            curr_e = 0
        else:
            curr_e = ref - y

        # Calculate previous error
        prev_e = refprev - yprev

        # Generate Inputs
        x = np.array([curr_e, (curr_e - prev_e)])
        return x
