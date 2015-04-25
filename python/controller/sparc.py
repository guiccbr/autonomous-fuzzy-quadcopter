# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

# ------------------------ Imports ----------------------------------#
from __future__ import division  # Force real division
import numpy as np  # Numpy Arrays
import math  # Square root


# ------------------------ Classes  ---------------------------------#
class SparcController:
    def __init__(self, control_range, ref_range, input_size, init_input, init_output, init_ref, init_y,
                 init_data_clouds=None, dc_radius_const=0.5, k_init=1):
        """
        Initiates a sparc controller using the first sample.

        Keyword arguments:
        control_range -- Tuple of two elements with the first element representing
                         the minimum value of the control signal, and the second
                         element representing its maximum. (float, float).
        ref_range -- Tuple of two elements with the first element representing
                     the minimum value of the reference (desired  plant state) the second
                     element representing its maximum. (float, float).
        input_size -- Size of the input x (int)
        init_input -- First input value (numpy array of size input_size)
        init_output -- First output signal (float)
        init_data_clouds -- Optional initialization data cloud (list of DataCloud objects)
        dc_radius_const -- DataCloud radius constant (see DataCloud class for more details)
        k_init -- The number where the iteration count starts
        """
        if not init_data_clouds:
            init_data_clouds = []

        # Set constant values
        self.umin, self.umax = control_range
        self.refmin, self.refmax = ref_range
        self.xsize = input_size
        self.radius_update_const = dc_radius_const
        self.clouds = list(init_data_clouds)
        self.k = k_init

        # Global density recursive values
        self.g_csi = np.array([0.0] * (self.xsize + 1))
        self.g_b = 0.0

        # Initial input and output:
        self.curr_x = init_input
        self.curr_u = init_output
        self.curr_z = np.append(self.curr_x, self.curr_u)

        # Start prev_ values:
        self.prev_y = 0.0
        self.prev_ref = 0.0
        self.prev_u = 0.0

        # - Consequents update constant
        self.c = float(self.umax - self.umin) / (self.refmax - self.refmin)

        if not self.clouds:
            # Initiates SPARC with the first cloud, with an initial
            # consequent given by U1, and updates the plant if first iteration.
            sigma1 = np.array([1.0] * self.xsize)
            self.clouds.append(DataCloud(self.curr_z, sigma1, self.radius_update_const))

            # Initializes array with membership degrees.
            # md[i] corresponds to the degree of membership of the sample xk to the Data Cloud i
            self.curr_md = np.array([1.0])

            # Update global density recursive values for the first cloud
            self.g_csi = np.add(self.g_csi, self.curr_z)
            self.g_b = self.g_b + np.dot(self.curr_z, self.curr_z)

            # Store last sample
            self.prev_u = self.curr_u
            self.prev_y = init_y
            self.prev_ref = init_ref
            self.prev_md = np.copy(self.curr_md)

            # Update k before next iteration
            self.k += 1

        else:
            # TODO: Functionality of starting with a non-empy list of clouds has been added,
            # but has not been implemented and tested at all. Need more time to test it.
            # g_csi and g_b may have to be given as input for it to work.
            pass

    def update(self, curr_x, curr_y, curr_ref):
        """
        Calculate the output given an input and a reference.

        Keyword arguments:
        curr_x -- current data sample of dimension XSIZE (numpy array of size self.xsize)
        curr_y -- current plant output value (float)
        curr_ref -- current reference value (float)

        Returns:
        u -- output respective to curr_x (float)
        """

        # Updates the consequents of all clouds 
        for i in range(len(self.clouds)):
            self.clouds[i].update_consequent(self.prev_md[i], self.prev_ref, curr_y,
                                             self.prev_u, self.c, self.umin, self.umax)

        # Calculate the new values of membership degrees, for the current sample.
        # Also finds out the data cloud that better describes the current sample. 
        ld_sum = 0.0
        curr_x_cloud = 0  # Index of the Best cloud.
        curr_x_cloud_ld = 0.0  # Best cloud local density.
        for i in range(len(self.clouds)):
            self.curr_md[i] = self.clouds[i].get_local_density(curr_x)
            if math.isnan(self.curr_md[i]):
                self.curr_md[i] = 0
            ld_sum = ld_sum + self.curr_md[i]
            if self.curr_md[i] > curr_x_cloud_ld:
                curr_x_cloud = i
                curr_x_cloud_ld = self.curr_md[i]

        if math.isnan(ld_sum):
            self.curr_md = 0
        else:
            self.curr_md /= float(ld_sum)

        # Generate control signal
        self.curr_u = 0.0
        for i in range(len(self.clouds)):
            self.curr_u = self.curr_u + self.curr_md[i] * self.clouds[i].get_consequent()

        # Prevent over-excursion
        if self.curr_u > self.umax:
            self.curr_u = self.umax
        if self.curr_u < self.umin:
            self.curr_u = self.umin

        # print 'u_not_truncated: ', self.curr_u

        # Concatenates x and u to form z:
        curr_z = np.append(curr_x, self.curr_u)

        # Calculate Global Density of curr_z
        curr_gd = self.get_global_density(self.g_csi, self.g_b, curr_z, self.k)

        # Tests to see if a new cloud is needed by doing the following (1) and (2):
        curr_sample_global_density_is_better = True
        curr_sample_is_distant_enough = True

        # (1) Compares Z Global Density with all data clouds focal points global densities
        for c in self.clouds:
            gdf = self.get_global_density(self.g_csi, self.g_b, c.zf, self.k)
            if curr_gd <= gdf:
                curr_sample_global_density_is_better = False
                break

        # (2) Compares distance of zk to focal points with data cloud radiuses
        for c in self.clouds:
            for r in c.r:
                if np.linalg.norm(curr_z - c.zf) <= r / 2.0:
                    curr_sample_is_distant_enough = False
                    break

        # If a new cloud is needed, creates a new cloud
        # Otherwise, adds the current point to the best matching cloud and checks
        # if the focal point has to be updated
        new_cloud_needed = curr_sample_global_density_is_better and curr_sample_is_distant_enough

        if new_cloud_needed:

            # Computes starting sigma (local scatter):
            sigma = np.array([0.0] * self.xsize)
            for i in range(0, self.xsize):
                for c in self.clouds:
                    sigma[i] += math.sqrt(c.variance[i])
                sigma[i] = float(sigma[i]) / len(self.clouds)

            # Creates new cloud with focal point zk and starting sigma
            self.clouds.append(DataCloud(curr_z, sigma, self.radius_update_const))
            self.curr_md = np.append(self.curr_md, 0.0)
        else:
            # Computes cxk focal point global density (to check if focal point has to change)
            gdf = self.get_global_density(self.g_csi, self.g_b, self.clouds[curr_x_cloud].zf, self.k)

            # Compute focal point local density
            curr_x_cloud_zf = self.clouds[curr_x_cloud].zf
            curr_x_cloud_xf = curr_x_cloud_zf[:self.xsize]
            ldf = self.clouds[curr_x_cloud].get_local_density(curr_x_cloud_xf)

            # Checks if the focal point needs to be updated:
            # curr_x_cloud_ld is the local density of curr_x relative to its cloud.
            # ldf is the local density of the focal point of curr_x's cloud.
            # curr_gd is the global density of curr_z
            # curr_x_cloud_gd is the global density of the focal point of curr_z's cloud.
            if curr_x_cloud_ld > ldf and curr_gd > gdf:
                self.clouds[curr_x_cloud].update_focal_point(curr_z)

            # Insert point to data cloud, updating radius, size, variance, etc.
            self.clouds[curr_x_cloud].add_point(curr_z)

        # Update Global Density values g_csi and g_b
        # Update global density recursive values
        self.g_csi = np.add(self.g_csi, curr_z)
        self.g_b = self.g_b + np.dot(curr_z, curr_z)

        # Store last sample
        self.prev_u = self.curr_u
        self.prev_md = np.copy(self.curr_md)
        self.prev_ref = curr_ref
        self.prev_y = curr_y

        # Update k before next iteration
        self.k += 1

        if math.isnan(self.curr_u):
            self.curr_u = 0

        # Return output u related to input curr_x
        return self.curr_u




    @staticmethod
    def get_global_density(g_csi, g_b, curr_z, k):
        """
        Calculates recursively the Global Density of point curr_z.

        Keyword arguments:
        g_csi -- current global csi (recursive calculation variable)
        cur_b -- current global b (recursive calculation variable)
        curr_z -- sample that will have its corresponding global density calculated.
        k -- current time step
        """

        # Computes global density of z and global density of f
        # TODO: There's something wrong here. It might result in division by zero in
        # some occasions if implemented like the paper.
        # Workaround is to result in zero in these cases.
        if k == 1:
            gd = 0.0
            return gd
        else:
            gd = float((k - 1)) / ((k - 1) * (np.dot(curr_z, curr_z) + 1.0) - 2 * (np.dot(curr_z, g_csi)) + g_b)
            return gd


class DataCloud:
    """
    Class that represents a data cloud.

    It stores the following information in the form of instance variables:
    zf -- Focal point, composed by xf (data sample) and q (consequent) 
    csi, betha -- parameters for recursively calculation of local density
    r -- array of radii, one for each dimension of X.
    sigma_sq -- parameter for recursively calculation of radii. (variance)
    m -- number of points added so far
    z -- Last point added.
    """

    def __init__(self, z, sigma, radius_update_const=0.5):
        """
        Initializes a DataCloud with one point z.
        Extracts x and u, setting u as the consequent q.

        Keyword arguments:
        z --
        sigma -- array containing the sigma starting value for the new DataCloud
        radius_update_const -- Radius constant, usually 0.5
        """

        # Set radius update constant
        self.radius_update_const = radius_update_const

        # Gets plant input (x) and control signal (u) 
        # from z where z = [x', u']', setting them
        # as focal point (xf) and consequent (q) respectively.
        self.zf = z

        self.xsize = len(z) - 1

        # Local density calculation values
        csi0 = np.array([0.0] * self.xsize)
        self.csi = np.add(csi0, self.zf[:self.xsize])

        betha0 = 0.0
        self.betha = betha0 + np.dot(self.zf[:self.xsize], self.zf[:self.xsize])

        # Data Cloud Size
        self.m = 1

        # Data Cloud Radius
        # Each data cloud has X_SIZE radiuses, one for each dimension of x.
        # By definition the initial radius r1 is 1 for each dimension.
        self.r = np.array([1.0] * self.xsize)

        # Local Scatter square (sigma_square), has to be stored for recursive calculation of
        # the radius. For each dimension of x, there's a sigma associated to it.
        # By definition the initial sigma sigma1 is 1 if not provided 
        self.variance = sigma[:]

        # Besides, for calculating sigma recursively, the centroid has to be
        # stored and calculated recursively as well. The centroid starts as
        # the first data sample (z).
        self.centroid = self.zf

    def update_focal_point(self, z):
        """
        Update focal point. Just updates self.zf. Does not increment the size of the data cloud, neither
        updates radius or variance. Usually add_point is called right after.

        Keyword arguments:
        z -- datacloud point composed by x (data sample) and u (control signal) 
        """
        self.zf = z

    def __update_radius__(self):
        """
        Update radius of the Data Cloud recursively.
        It needs to be called after a new point is added to the Cloud.

        """
        p = self.radius_update_const
        for i in range(0, len(self.r)):
            self.r[i] = p * self.r[i] + (1 - p) * math.sqrt(self.variance[i])

    def __update_variance_and_centroid__(self, curr_z):
        """
        Update the local scatter square of the Data Cloud recursively.
        The local scatter ( sigma ) is needed to update the radius.

        Keyword arguments:
        curr_z -- Last added sample 
        """
        # Extract centroid X
        x = curr_z[:self.xsize]
        new_centroid = self.centroid[:]

        # Calculate New Centroid
        for i in range(0, len(self.centroid)):
            new_centroid[i] = (self.centroid[i] * (self.m - 1) + curr_z[i]) / self.m

        # Calulate and Update New Variance
        for i in range(0, len(self.variance)):
            prev_variance = self.variance[i]
            self.variance[i] = (1.0 / self.m) * (
                (self.m - 1) * prev_variance + (x[i] - self.centroid[i]) * (x[i] - new_centroid[i]))

        # Update centroid
        self.centroid = new_centroid

    def add_point(self, curr_z):
        """
        Associates a new point to the data cloud, updating the number of points
        updating local density values, sigma and radius.

        Keyword arguments:
        curr_z -- datacloud point composed by x (data sample) and u (control signal) 
        """

        # Update number of points
        self.m += 1

        # Extract x
        x = curr_z[:self.xsize]

        # Update Variance
        self.__update_variance_and_centroid__(curr_z)

        # Update radius
        self.__update_radius__()

        # Update local density values 
        self.csi = np.add(self.csi, x)
        self.betha = self.betha + np.dot(x, x)

    def get_local_density(self, x):
        """
        Recursively calculate the local density relative to the sample input x

        Keyword arguments:
        x -- an input of dimension XSIZE
        """
        ld = float(self.m) / (self.m * (np.dot(x, x) + 1) - 2 * (np.dot(x, self.csi)) + self.betha)

        return ld

    def update_consequent(self, prev_md, prev_ref, curr_y, prev_u, c, umin, umax):
        """
        Updates consequent

        Keyword arguments:
        prev_md -- membership degree of the previous data sample related to this cloud.
        prev_ref -- previous reference value
        curr_y -- current plant output value
        prev_u -- previous control signal
        C -- Consequent constant calculated by: C = (UMAX - UMIN)/(REFMAX - REFMIN)
        umin, umax -- Control Signal range, use determine if the consequent should be penalized.
        """

        # Calculate relative error:
        e = prev_ref - curr_y

        # Calculate consequent differential
        dq = c * prev_md * e

        # Checks if control signal maximum or minimum has been reached
        # to prevent penalization on these cases
        if (prev_u <= umin) and (dq < 0):
            dq = 0
        if (prev_u >= umax) and (dq > 0):
            dq = 0

        # Updates consequent
        q = self.get_consequent() + dq
        self.zf[-1] = q

    def get_consequent(self):
        """
        Extract consequent value from the focal point of the data cloud (zf).
        """
        return self.zf[-1]
