import numpy as np
from ...controller import sparc
import math

UMIN_LAT = -0.7
UMAX_LAT = 0.7
REFMIN_LAT = -30.0
REFMAX_LAT = 30.0

UMIN_LON = -0.7
UMAX_LON = 0.7
REFMIN_LON = -30.0
REFMAX_LON = 30.0

X_SIZE = 2


def generate_input_nav(y, yprev, ref, refprev):
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


class NavigationController:

    def __init__(self, initial_longitude_input, initial_latitude_input, initial_position_reference, current_position,
                 controller_lat_init=None, controller_long_init=None):
        
        self.longitude_u = 0.0
        self.latitude_u = 0.0
        self.previous_position = np.array([0, 0])
        self.previous_position_reference = np.array([0, 0])

        if controller_long_init is None:
            self.longitudeController = sparc.SparcController((UMIN_LON, UMAX_LON), (REFMIN_LON, REFMAX_LON), X_SIZE,
                                                             initial_longitude_input, initial_position_reference[0],
                                                             current_position[0])
        else:
            self.longitudeController = controller_long_init

        if controller_long_init is None:
            self.latitudeController = sparc.SparcController((UMIN_LAT, UMAX_LAT), (REFMIN_LAT, REFMAX_LAT), X_SIZE,
                                                            initial_latitude_input, initial_position_reference[1],
                                                            current_position[1])
        else:
            self.latitudeController = controller_lat_init

    def update(self, current_position, current_position_reference):
        
        # Generate inputs
        longitude_input = generate_input_nav(current_position[0], self.previous_position[0],
                                             current_position_reference[0], self.previous_position_reference[0])
        latitude_input = generate_input_nav(current_position[1], self.previous_position[1],
                                            current_position_reference[1], self.previous_position_reference[1])

        self.longitude_u = self.longitudeController.update(longitude_input, current_position[0],
                                                           current_position_reference[0], self.longitude_u)
        self.latitude_u = self.latitudeController.update(latitude_input, current_position[1],
                                                         current_position_reference[1], self.latitude_u)
        
        # Update
        self.previous_position = current_position
        self.previous_position_reference = current_position_reference

        # Do not let the control signal exceed the highest value
        self.longitude_u = np.median(np.array([UMIN_LON, self.longitude_u, UMAX_LON]))
        self.latitude_u = np.median(np.array([UMIN_LAT, self.latitude_u, UMAX_LAT]))

        return self.longitude_u, self.latitude_u
