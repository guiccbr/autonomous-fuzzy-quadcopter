import numpy as np

class quadcopter_control_system:
    ''' Control system class
        initial_state = [initial_altitude, initial_yaw, initial_pitch, initial_roll]
        initial_control = [initial_altitude_control, initial_yaw_control, initial_pitch_control, initial_roll_control]
    '''


    def __init__(self, altitude_controller, yaw_controller, roll_controller, pitch_controller, initial_state=[0,0,0,0], initial_control = [0,0,0,0]):
        
        # Controllers
        self.altitude_controller = altitude_controller
        self.yaw_controller = yaw_controller
        self.roll_controller = roll_controller
        self.pitch_controller = pitch_controller
        self.prev_state = initial_state
        self.prev_control = initial_control

    def update(self, state, reference):

        # Altitude control
        altitude_input = np.array([state[0]-self.prev_state[0], state[0]])
        altitude_u = self.altitude_controller.update(altitude_input, self.prev_control[0], reference[0])


        # Yaw control
        yaw_input = np.array([state[1]-self.prev_state[1], state[1]])
        yaw_u = self.yaw_controller.update(yaw_input, self.prev_control[1], reference[1])

	# pitch control
        pitch_input = np.array([state[2]-self.prev_state[2], state[2]])
        pitch_u = self.pitch_controller.update(pitch_input, self.prev_control[2], reference[2])

	# roll control
        roll_input = np.array([state[3]-self.prev_state[3], state[3]])
        roll_u = self.roll_controller.update(roll_input, self.prev_control[3], reference[3])

        # Update data
        self.prev_control = [altitude_u, yaw_u, pitch_u, roll_u]
        self.prev_state = state

        m1 = altitude_u + yaw_u + pitch_u + roll_u
        m2 = altitude_u - yaw_u + pitch_u - roll_u
        m3 = altitude_u + yaw_u - pitch_u - roll_u
        m4 = altitude_u - yaw_u - pitch_u + roll_u

        return [m1,m2,m3,m4]
