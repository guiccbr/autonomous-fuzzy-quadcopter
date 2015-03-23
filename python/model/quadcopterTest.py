#quadcopterTest.py

import unittest
import numpy as np
import quadcopter
import model

class quadcopterTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls._quadcopter = quadcopter.quadcopter(model.model())

    # Test rotation function
    def testQuadcopterRotation(cls):
        R = cls._quadcopter.rotation(np.array([5.4,3.7,2.2]))
        Rcorrect = np.array([[.49910782, -0.75410195, -0.42687426],[-0.68568583,-0.04248787,-0.72665654],[0.52983614,0.65538159,-0.53828305]])
        cls.assertAlmostEqual(R[0,0],Rcorrect[0,0],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(R[0,1],Rcorrect[0,1],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(R[0,2],Rcorrect[0,2],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(R[1,0],Rcorrect[1,0],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(R[1,1],Rcorrect[1,1],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(R[1,2],Rcorrect[1,2],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(R[2,0],Rcorrect[2,0],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(R[2,1],Rcorrect[2,1],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(R[2,2],Rcorrect[2,2],places=3,msg=None,delta=None)
        
    def testQuadcopterOmega2thetadot(cls):
        thetaDot = cls._quadcopter.omega2thetadot(np.array([5.4,7.4,8.8]),np.array([2,6,3]))
        thetaDotcorrect = np.array([4.5076,-11.0813,3.1939])
        cls.assertAlmostEqual(thetaDot[0],thetaDotcorrect[0],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(thetaDot[1],thetaDotcorrect[1],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(thetaDot[2],thetaDotcorrect[2],places=3,msg=None,delta=None)
    
    def testThetadot2Omega(cls):
        omega = cls._quadcopter.thetadot2omega(np.array([5.4,7.4,8.8]),np.array([2,6,3]))
        omegaCorrect = np.array([7.8589,4.6036,-10.2450])
        cls.assertAlmostEqual(omega[0],omegaCorrect[0],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(omega[1],omegaCorrect[1],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(omega[2],omegaCorrect[2],places=3,msg=None,delta=None)
        
    def testAngularAcceleration(cls):
        I = cls._quadcopter.model.I
        L = cls._quadcopter.model.L
        b = cls._quadcopter.model.b
        k = cls._quadcopter.model.k
        
        angAcc = cls._quadcopter.angular_acceleration([1200,1000,1200,1000],np.array([5.9,6.6,9.2]),I,L,b,k)
        angAccCorrect = np.array([-60.7200,54.2800,8.8000])
        cls.assertAlmostEqual(angAcc[0],angAccCorrect[0],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(angAcc[1],angAccCorrect[1],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(angAcc[2],angAccCorrect[2],places=3,msg=None,delta=None)
        
    def testTorques(cls):
    
        L = cls._quadcopter.model.L
        b = cls._quadcopter.model.b
        k = cls._quadcopter.model.k
        
        torques = cls._quadcopter.torques([1100.0,1200.0,1500.0,1200.0],L,b,k)
        torquesCorrect = np.array([-0.78,0.0,0.0580])
        cls.assertAlmostEqual(torques[0],torquesCorrect[0],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(torques[1],torquesCorrect[1],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(torques[2],torquesCorrect[2],places=3,msg=None,delta=None)
        
    def testThrust(cls):

        k = cls._quadcopter.model.k
        
        thrust = cls._quadcopter.thrust([1100.0,1200.0,1500.0,1200.0],k)
        thrustCorrect = np.array([0.0,0.0,19.02])
        cls.assertAlmostEqual(thrust[0],thrustCorrect[0],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(thrust[1],thrustCorrect[1],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(thrust[2],thrustCorrect[2],places=3,msg=None,delta=None)

    def testAcceleration(cls):
        m = cls._quadcopter.model.m
        g = cls._quadcopter.model.g
        k = cls._quadcopter.model.k
        kd = cls._quadcopter.model.kd
        
        acc = cls._quadcopter.acceleration([1100.0,1200.0,1500.0,1200.0],np.array([3.7,8.9,10.3]),np.array([3.4,4.5,3.9]),m,g,k,kd)
        accCorrect = np.array([24.9809,-1.6321,17.1354])
        
        cls.assertAlmostEqual(acc[0],accCorrect[0],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(acc[1],accCorrect[1],places=3,msg=None,delta=None)
        cls.assertAlmostEqual(acc[2],accCorrect[2],places=3,msg=None,delta=None)
        
        
    def testUpdateYaw(cls):
    
        _quadcopter = quadcopter.quadcopter(model.model())
        updt = _quadcopter.update(0.5,[1200,1000,1200,1000])
        
        cls.assertAlmostEqual(_quadcopter.x[0],0.0,places=3,msg=None,delta=None)
        cls.assertAlmostEqual(_quadcopter.x[1],0.0,places=3,msg=None,delta=None)
        cls.assertAlmostEqual(_quadcopter.x[2],4.8675,places=3,msg=None,delta=None)
        
        cls.assertAlmostEqual(_quadcopter.theta[0],0.0,places=3,msg=None,delta=None)
        cls.assertAlmostEqual(_quadcopter.theta[1],0.0,places=3,msg=None,delta=None)
        cls.assertAlmostEqual(_quadcopter.theta[2],2.2000,places=3,msg=None,delta=None)
        
    def testUpdatePitch(cls):
        
        _quadcopter = quadcopter.quadcopter(model.model())
        
        updt = _quadcopter.update(0.5,[950,1000,1050,1000])
        
        cls.assertAlmostEqual(_quadcopter.x[0],0.0,places=3,msg=None,delta=None)
        cls.assertAlmostEqual(_quadcopter.x[1],0.0,places=3,msg=None,delta=None)
        cls.assertAlmostEqual(_quadcopter.x[2],3.5550,places=3,msg=None,delta=None)
        
        cls.assertAlmostEqual(_quadcopter.theta[0],-7.5000,places=3,msg=None,delta=None)
        cls.assertAlmostEqual(_quadcopter.theta[1],0.0,places=3,msg=None,delta=None)
        cls.assertAlmostEqual(_quadcopter.theta[2],0.0125,places=3,msg=None,delta=None)
        
def main():
    unittest.main()

if __name__ == '__main__':
    main()