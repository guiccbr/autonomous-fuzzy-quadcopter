from model_3d import *
# ------------------------ Functions ------------------------------------------#
def test():
    """
    A simple test routine to draw the quadcopter model
    """
    
    # Creates a VPython Scene
    scene = display(title='Quad_Test', x=0, y=0, width=800, height=600, center=(0,0,0),
            background=(0,0,0), forward=(-1,-1,-1))

    # Inertial Static Reference Frame
    pointer = arrow(pos=(0,0,0), axis=(10,0,0), shaftwidth=0.1)
    pointer = arrow(pos=(0,0,0), axis=(0,10,0), shaftwidth=0.1)
    pointer = arrow(pos=(0,0,0), axis= (0,0,10), shaftwidth=0.1)

    # Instantiate a quadcopter 3d model
    pos = vector(3,0,3)
    roll, pitch, yaw = 0., 0., 0.
    quad = QuadcopterGraphic3d(pos, roll, pitch, yaw, scene, frame_length=3, frame_angle=pi/2)
   
    # Run indefinitely
    while True:
        # Run 30 times per second
        rate(30)

        # Set yaw (a half turn per second)
        yaw = yaw + pi/30
        quad.set_yaw(yaw)

        # Set roll (a half turn per second)
        # roll = roll + pi/30
        # quad.set_roll(roll)

        # Set pitch (a half turn per second)
        # pitch = pitch + pi/100
        # quad.set_pitch(pitch)

        # Set position (moves on x direction)
        pos = pos + vector(0,0,0.04)
        quad.set_pos(pos)

if __name__ == "__main__":
    test()
