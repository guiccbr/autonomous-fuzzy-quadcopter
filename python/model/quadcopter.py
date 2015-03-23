# Universidade Estadual de Campinas - UNICAMP

# Trabalho Final de Curso
# Diego Pereira Domingos RA 090923
# Guilherme Campos Camargo RA

import numpy as np
from numpy import array,transpose
from math import sin,cos

class quadcopter():

    def __init__(self, model = None):
        self.x = array([.0,.0,.0]); # (x,y,z)
        self.xdot = array([.0,.0,.0]);
        self.theta = array([.0,.0,.0]); # (roll, pitch, yaw)
        self.thetadot = array([.0,.0,.0]);
        self.omega = array([.0,.0,.0]);
        self.omegadot = array([.0,.0,.0]);
        self.model = model
        
    def update(self,dt,inputs):
        self.omega = self.thetadot2omega(self.thetadot, self.theta);
        a = self.acceleration(inputs, self.theta, self.xdot, self.model.m, self.model.g, self.model.k, self.model.kd);
        self.omegadot = self.angular_acceleration(inputs, self.omega, self.model.I, self.model.L, self.model.b, self.model.k);
        self.omega = self.omega + dt * self.omegadot;
        
        self.thetadot = self.omega2thetadot(self.omega, self.theta); 
        self.theta = self.theta + dt * self.thetadot;
        self.xdot = self.xdot + dt * a;
        self.x = self.x + dt * self.xdot;
        
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
        return tau
    
    def acceleration(self,inputs, angles, vels, m, g, k, kd):
        gravity = array([0.0, 0.0, -g])
        R = self.rotation(angles);
        T = np.dot(R, self.thrust(inputs, k));
        Ta=np.array([0.0,0.0,0.0])
        Ta[0] = T[0][0]
        Ta[1] = T[1][0]
        Ta[2] = T[2][0]
        Fd = -kd * vels;
        a = gravity + (1. / m) * Ta + Fd;
        return a
    
    def angular_acceleration(self,inputs, omega, I, L, b, k):
        tau = self.torques(inputs, L, b, k)
        omegad = np.dot(np.linalg.inv(I), np.subtract(tau, np.cross(omega, np.dot(I, omega))))      
        # Filtering
        #omegad = self.filterArray(omegad, 1e-14)
        return omegad
        
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
        phi_ = angles[2] # dont change!!
        theta_ = angles[1] # dont change!!
        psi_ = angles[0] # dont change!!

        R = array([[0.0,0.0,0.0],[0.0,0.0,0.0],[0.0,0.0,0.0]]);
        R[:, 0] = [cos(phi_) * cos(theta_),cos(theta_) * sin(phi_),-sin(theta_)]
        R[:, 1] = [cos(phi_) * sin(theta_) * sin(psi_) - cos(psi_) * sin(phi_),cos(phi_) * cos(psi_) + sin(phi_) * sin(theta_) * sin(psi_),cos(theta_) * sin(psi_)]
        R[:, 2] = [sin(phi_) * sin(psi_) + cos(phi_) * cos(psi_) * sin(theta_),cos(psi_) * sin(phi_) * sin(theta_) - cos(phi_) * sin(psi_),cos(theta_) * cos(psi_)]
        
        return R