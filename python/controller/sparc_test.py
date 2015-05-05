#vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

import unittest
import sparc
import numpy as np

e = 100*np.random.rand()
de = 100*np.random.rand()
u = 100*np.random.rand()

class sparcTests(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        global e, de
        
        z = np.array([e, de, 0])
        sigma = np.array([0.0]*2)

        cls.datacloud = sparc.DataCloud(z, sigma)

    def test_update_variance_and_centroid(cls):
        global e,de
        inputs_z = 50*np.random.rand(100,3)
        for z in inputs_z:
            cls.datacloud.m+=1 
            cls.datacloud.__update_variance_and_centroid__(z)
        calc_variance = cls.datacloud.variance
        assert_variance_0 = np.var(np.concatenate([[e], inputs_z[:,0]]))
        assert_variance_1 = np.var(np.concatenate([[de], inputs_z[:,1]]))
        cls.assertAlmostEqual(calc_variance[0], assert_variance_0)
        cls.assertAlmostEqual(calc_variance[1], assert_variance_1) 

    def test_update_consequent(cls):

    def test_get_local_density(cls):

    def test_add_second_point(cls):
        global e, de, u
        z = np.array([e, de, u])  
        cls.datacloud.add_point(z)



def main():
    unittest.main()

if __name__ == '__main__':
    main()
