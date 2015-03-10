# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

# ------------------------ Imports --------------------------------------------#
from visual import *
import numpy as np
# ------------------------ Classes  -------------------------------------------#

def create_3d_world():
    
    # Creates a VPython Scene
    scene = display(title='Quad_Test', x=0, y=0, width=800, height=600, center=(0,0,0),
            background=(0,0,0), forward=(-1,-1,-1))

    # Inertial Static Reference Frame
    pointer = arrow(pos=(0,0,0), axis=(10,0,0), shaftwidth=0.1)
    pointer = arrow(pos=(0,0,0), axis=(0,10,0), shaftwidth=0.1)
    pointer = arrow(pos=(0,0,0), axis= (0,0,-10), shaftwidth=0.1)
    text(text='x', pos=(10,0,0), axis=(10,0,0), align='center', depth=-0.3, color=color.white)
    text(text='z', pos=(0,10,0), axis=(10,0,0), align='center', depth=-0.3, color=color.white)
    text(text='y', pos=(0,0,-10), axis=(0,0,-10), align='center', depth=-0.3, color=color.white)
    
    return scene

class QuadcopterGraphic3d:
    """
    A simple graphical representation of a quadcopter using vpython.
    """

    def __init__(self, init_pos, init_pitch, init_yaw, init_roll, scene, frame_length=1,
            frame_angle=pi/2, show_reference=True, show_motors=False):
        """
        Creates a quadcopter graphical representation with initial position and
        initial angles.

        Keyword arguments:
        init_pos -- initial position of the geometric center of the quadcopter
        init_pitch -- initial pitch angle of the quadcopter (rads)
        init_yaw -- initial yaw angle of the quadcopter (rads)
        init_roll -- initial roll angle of the quadcopter (rads)
        frame_length -- distance between two opposite engines
        frame_angle -- shortest frame angle (degrees (0-pi/2]) 
        show_reference -- if True, shows the quadcopter non-intertial frame of
        reference, that is fixed to the quadcopter frame.
        show_motors -- if True, shows the length of the force vector resulting from
        the propellers.
        scene -- scene where the quadcopter is to be drawn
        """

        scene.select()

        # Instantiate a frame for the quadcopter parts: 
        # 2 cylinders and 4 spheres. 
        self.f = frame(origin=(0,0,0))
        
        # Instantiate the quadcopter reference frame 
        self.reference = frame(origin=(0,0,0))

        # Draw the first frame bar (cylinder) and two engines on its extremities (spheres) 
        pos = vector(-frame_length/2., 0, 0)
        pos =  pos.rotate(axis=(0,0,1), angle=frame_angle/2.)
        axis = vector(1, 0, 0)
        axis = axis.rotate(axis=(0,0,1), angle=frame_angle/2.)
        cylinder(frame=self.f, pos=pos, axis=axis, radius=0.1,
                length=frame_length, color=color.cyan)
        sphere(frame=self.f, pos=pos, radius=0.2, color=color.red)
        m1_pointer = arrow(frame=self.f, pos=pos, axis=(0,0,-1), shaftwidth=0.05)
        sphere(frame=self.f, pos=rotate(pos, axis=(0,0,1), angle=pi), radius=0.2,
                color=color.red)
        m2_pointer = arrow(frame=self.f, pos=rotate(pos, axis=(0,0,1), angle=pi), axis=(0,0,-1), shaftwidth=0.05)

        # Draw the second frame bar (cylinder) and two engines on its extremities (spheres) 
        pos = vector(-frame_length/2., 0, 0)
        pos = pos.rotate(axis=(0,0,1), angle=-frame_angle/2.)
        axis = vector(1, 0, 0)
        axis = axis.rotate(axis=(0,0,1), angle=-frame_angle/2.)
        cylinder(frame=self.f, pos=pos, axis=axis, radius=0.1,
                length=frame_length, color=color.cyan)
        sphere(frame=self.f, pos=pos, radius=0.2, color=color.red)
        m3_pointer = arrow(frame=self.f, pos=pos, axis=(0,0,-1), shaftwidth=0.05)
        sphere(frame=self.f, pos=rotate(pos, axis=(0,0,1), angle=pi), radius=0.2,
            color=color.red)
        m4_pointer = arrow(frame=self.f, pos=rotate(pos, axis=(0,0,1), angle=pi), axis=(0,0,-1), shaftwidth=0.05)

        # Shows forces if requested 
        m1_pointer.visible=show_motors
        m2_pointer.visible=show_motors
        m3_pointer.visible=show_motors
        m4_pointer.visible=show_motors

        # Vector pointing to to the front of the quadcopter
        arrow(frame=self.f, pos=(0,0,0), axis=(0,1,0), shaftwidth=0.05,
                color=color.red)

        # Rotates so the quadcopter gets grounded (direction vector parallel to z-x) and
        # pointing on the direction of z.
        self.f.rotate(axis=(1,0,0), angle=pi/2)

        # Reference frame 
        self.ax_x = arrow(frame=self.reference, pos=(0,0,0), axis=vector(1,0,0), shaftwidth=0.05,
                color=color.blue)
        self.ax_y = arrow(frame=self.reference, pos=(0,0,0), axis=vector(0,1,0), shaftwidth=0.05,
                color=color.blue)
        self.ax_z = arrow(frame=self.reference,pos=(0,0,0), axis=vector(0,0,1), shaftwidth=0.05,
                color=color.blue)
        
        # Shows frame reference if requested
        visibility = show_reference 
        self.ax_x.visible = visibility
        self.ax_y.visible = visibility 
        self.ax_z.visible = False  # Always false because the pointing vector goes on its place.

      
        # Initial Coordinates
        self.pitch=0
        self.roll=0
        self.yaw=0
       
        # Set initial coordinates
        self.set_pitch(init_pitch)
        self.set_roll(init_roll)
        self.set_yaw(init_yaw)
        self.set_pos(init_pos) 

    def set_pitch(self, pitch):
        '''
        Set the pitch (rotation around the side-to-side axis) of the 3d model
        '''
        dpitch = pitch-self.pitch
        self.pitch = pitch

        self.f.rotate(axis=self.ax_x.axis, angle=dpitch)
        self.ax_z.axis = rotate(self.ax_z.axis, axis=self.ax_x.axis, angle=dpitch)
        self.ax_y.axis = rotate(self.ax_y.axis, axis=self.ax_x.axis, angle=dpitch)

    def set_yaw(self, yaw):
        '''
        Set the yaw (rotation around the vertical axis) of the 3d model
        '''
        dyaw = yaw - self.yaw
        self.yaw = yaw

        self.f.rotate(axis=self.ax_y.axis, angle=dyaw)
        self.ax_z.axis = rotate(self.ax_z.axis, axis=self.ax_y.axis, angle=dyaw)
        self.ax_x.axis = rotate(self.ax_x.axis, axis=self.ax_y.axis, angle=dyaw)

    def set_roll(self, roll):
        '''
        Set the roll (rotation around the front-to-back axis) of the 3d model
        '''
        droll = roll - self.roll
        self.roll = roll

        self.f.rotate(axis=self.ax_z.axis, angle=droll)
        self.ax_y.axis = rotate(self.ax_y.axis, axis=self.ax_z.axis, angle=droll)
        self.ax_x.axis = rotate(self.ax_x.axis, axis=self.ax_z.axis, angle=droll)

    def set_pos(self, pos):
        '''
        Set the position of the center of the 3d model.

        Keyword arguments:
        pos -- vector with the new position (x, y, z) 
        '''
        self.f.pos = (pos[0], pos[2], pos[1])
        self.reference.pos = (pos[0], pos[2], pos[1])

    def set_angles(self, roll, pitch, yaw):
        '''
        Set roll, pitch and yaw of the 3d model.

        Keyword arguments:
        roll, pitch, yaw -- Angle in radians
        '''
        self.set_roll(roll)
        self.set_yaw(yaw)
        self.set_pitch(pitch)
