import sys
# sys.path.append("ukf_uav")
import ukf_uav
import numpy as np
import unittest


class TestUKF(unittest.TestCase):


    def test_ukf_init(self):
        dim_x = 12
        dim_z = 6
        test_ukf = ukf_uav.UnscentedKalmanFilter(dim_x, dim_z, 1)
        np.testing.assert_equal(test_ukf._dim_x, dim_x)
        np.testing.assert_equal(test_ukf._dim_z, dim_z)
        self._ukf = test_ukf

    def test_sigmaPoints(self):
        uav = ukf_uav.UnscentedKalmanFilter()
        x = np.ones(3)
        P = np.eye(3)
        c = np.sqrt(2)
        uav.sigmaPoints(x, P, c)
        uav.sigmaPoints(x.T, P, c)

    def test_ut(self):
        uav = ukf_uav.UnscentedKalmanFilter()

    def test_ukf(self):
        pass

    def test_sss(self):
        pass

    def test_update(self):
        pass



if __name__ == '__main__':
    unittest.main()
