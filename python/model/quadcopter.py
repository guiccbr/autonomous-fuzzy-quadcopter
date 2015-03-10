# Universidade Estadual de Campinas - UNICAMP
# Trabalho Final de Curso
# Diego Pereira Domingos RA 090923
# Guilherme Campos Camargo RA

import numpy as np
from numpy import array,transpose
from math import sin,cos

class quadcopter():

	def __init__(self, model=None):
		self.x = array([[.0],[.0],[.0]])
		self.xdot = array([[.0],[.0],[.0]])
		self.theta = array([[.0],[.0],[.0]])
		self.thetadot = array([[.0],[.0],[.0]])
		self.omega = array([[.0],[.0],[.0]])
		self.omegadot = array([[.0],[.0],[.0]])
		self.model = model
		self.motors = array([.0,.0,.0,.0])
	
	def update(self,dt,inputs):
		self.omega = self.thetadot2omega(self.thetadot, self.theta);
		a = self.acceleration(inputs, self.theta, self.xdot, self.model.m, self.model.g, self.model.k, self.model.kd);
		self.omegadot = self.angular_acceleration(inputs, self.omega, self.model.I, self.model.L, self.model.b, self.model.k);
		self.omega = self.omega + dt * self.omegadot;
		
		self.thetadot = self.omega2thetadot(self.omega, self.theta); 
		self.theta = self.theta + dt * self.thetadot;
		self.xdot = self.xdot + dt * a;
		self.x = self.x + dt * self.xdot;
    
		#print self.theta
		#print self.x
    
	# Set model of environment
	def setModel(self,newModel):
		self.model = newModel
	
	
	# Compute thrust given current inputs and thrust coefficient.
	def thrust(self,inputs, k):
		T = array([[0], [0], [k * np.sum([i**2 for i in inputs])]])
                return T
		
	# Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
	def torques(self,inputs, L, b, k):
                tau = array([.0,.0,.0])
                tau[0] = L*k*(inputs[0]**2-inputs[2]**2)
                tau[1] = L*k*(inputs[1]**2-inputs[3]**2)
                tau[2] = b*(inputs[0]**2-inputs[1]**2+inputs[2]**2-inputs[3]**2)
	#	tau = array([L*k*(inputs[0]-inputs[2]), L*k*(inputs[1]-inputs[3]), b*(inputs[0]-inputs[1]+inputs[2]-inputs[3])])
               
                return tau
		
	# Compute acceleration in inertial reference frame
	# Parameters:
	#   g: gravity acceleration
	#   m: mass of quadcopter
	#   k: thrust coefficient
	#   kd: global drag coefficient
	def acceleration(self,inputs, angles, vels, m, g, k, kd):
		gravity = array([[0.0], [0.0], [-g]])
		R = self.rotation(angles);
		T = np.dot(R, self.thrust(inputs, k));
		Fd = -kd * vels;
		a = gravity + (1. / m) * T + Fd;
		return a
		
	# Compute angular acceleration in body frame
	# Parameters:
	# I: inertia matrix
        # FIXME: np.dot(I, omega) is resulting in a matrix with a size different than omega,
        # and so, the cross product is not possible.
	def angular_acceleration(self,inputs, omega, I, L, b, k):
		tau = self.torques(inputs, L, b, k);
		omegad = np.dot(transpose(np.linalg.inv(I)), (tau - np.cross(transpose(omega)[0], np.dot(transpose(I), transpose(omega)[0]))));
	
                # Filtering
                omegad = self.filterArray(omegad, 1e-15) 

                return self.lineToColumn(omegad)
		
	# Convert derivatives of roll, pitch, yaw to omega.
	def thetadot2omega(self,thetadot, angles):
		phi_ = angles[0];
		theta_ = angles[1];
		psi_ = angles[2];
		W = array([[1.0, 0.0, -sin(theta_)],[0.0, cos(phi_), cos(theta_)*sin(phi_)],[0.0, -sin(phi_), cos(theta_)*cos(phi_)]]);
		omega = np.dot(W,thetadot);
		return omega
		
	# Convert omega to roll, pitch, yaw derivatives
	def omega2thetadot(self,omega, angles):
		phi_ = angles[0];
		theta_ = angles[1];
		psi_ = angles[2];
		W = array([[1.0, 0.0, -sin(theta_)],[0.0, cos(phi_), cos(theta_)*sin(phi_)],[0.0, -sin(phi_), cos(theta_)*cos(phi_)]]);
		thetadot = np.dot(np.linalg.inv(W), omega);
		return thetadot
		
		
	# Compute rotation matrix for a set of angles.
	def rotation(self,angles):
		phi_ = angles[0]
		theta_ = angles[1]
		psi_ = angles[2]

                R = array([[.0,.0,.0],[.0,.0,.0],[.0,.0,.0]])
                R[0,0] = cos(phi_)*cos(psi_) - cos(theta_)*sin(phi_)*sin(psi_)
                R[0,1] = -cos(psi_)*sin(phi_) - cos(phi_)*cos(theta_)*sin(psi_)
                R[0,2] = sin(theta_)*sin(psi_)
                R[1,0] = cos(theta_)*cos(psi_)*sin(phi_) + cos(phi_)*sin(psi_)
                R[1,1] = cos(phi_)*cos(theta_)*cos(psi_) - sin(phi_)*sin(psi_)
                R[1,2] = -cos(psi_)*sin(theta_)
                R[2,0] = sin(phi_)*sin(theta_)
                R[2,1] = cos(phi_)*sin(theta_)
                R[2,2] = cos(theta_) 

#		R = array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]);
#		R[:, 0] = [cos(phi_) * cos(theta_),cos(theta_) * sin(phi_),-sin(theta_)]
#		R[:, 1] = [cos(phi_) * sin(theta_) * sin(psi_) - cos(psi_) * sin(phi_),cos(phi_) * cos(psi_) + sin(phi_) * sin(theta_) * sin(psi_),cos(theta_) * sin(psi_)]
#		R[:, 2] = [sin(phi_) * sin(psi_) + cos(phi_) * cos(psi_) * sin(theta_),cos(psi_) * sin(phi_) * sin(theta_) - cos(phi_) * sin(psi_),cos(theta_) * cos(psi_)]
		return R

	def lineToColumn(self, vector):
		return np.array([[vector[0]],[vector[1]],[vector[2]]])

        
        def filterArray(self, array, threshold):
                '''
                Elements on the numpy array (1d or 2d), that have an absolute value less than threshold
                are changed to 0.0
                '''
                array[np.nonzero(abs(array) < abs(threshold))] = 0.0
                return array
