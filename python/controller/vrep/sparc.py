# vim: tabstop=8 expandtab shiftwidth=4 softtabstop=4

# ------------------------ Imports ----------------------------------#
from __future__ import division  # Force real division
import numpy as np  # Numpy Arrays
import math  # Square root


# ------------------------ Classes  ---------------------------------#
class SparcController:
    def __init__(self, control_range, ref_range, input_size, init_input, init_ref, init_y, monotonicity=1,
                 dc_radius_const=0.5):
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
        init_ref
        init_y
        monotonicity
        dc_radius_const -- DataCloud radius constant (see DataCloud class for more details)
        """

        # Set constant values
        self.umin, self.umax = control_range
        self.refmin, self.refmax = ref_range
        self.xsize = input_size
        self.radius_update_const = dc_radius_const
        self.k = 1                                          # Initial step number

        # Global density recursive values Initial
        self.g_csi = np.array([0.0] * (self.xsize + 1))
        self.g_b = 0.0

        # - Consequents normalization constant (Changes according to the current reference curve)
        self.c = abs(float(self.umax - self.umin) / (self.refmax - self.refmin))

        # - C signal is the same as the monotonicity
        if monotonicity < 0:
            self.c = -self.c

        # Initial consequent will be proportinal to the error.
        q_init = self.c*(init_ref - init_y)

        # Initial input
        curr_x = np.copy(init_input)
        curr_z = np.append(curr_x, q_init)

        # Instantiate a list of clouds
        self.clouds = []

        # Initiates SPARC with the first cloud, with an initial
        # consequent given by q_init, and update the plant if first iteration.
        initial_variance = np.array([0.0] * self.xsize)
        self.clouds.append(DataCloud(curr_z, initial_variance, self.radius_update_const))

        # Initializes array with membership degrees.
        # md[i] corresponds to the degree of membership of the sample xk to the Data Cloud i
        curr_md = np.array([1.0])

        # Store last sample
        self.prev_y = init_y
        self.prev_ref = init_ref
        self.prev_md = np.copy(curr_md)
        self.prev_z = np.copy(curr_z)

        # Update k before next iteration
        self.k += 1

    def update_reference_range(self, refmin, refmax):
        """
        Update the Consequent normalization constant according to a new refmim, refmax.
        :param refmin: Minimum value of the current reference curve.
        :param refmax: Maximum value of the current reference curve.
        :return: void
        """

        self.refmax = refmax
        self.refmin = refmin

        self.c = float(self.umax - self.umin) / (self.refmax - self.refmin)

    def update(self, curr_x, curr_y, curr_ref, prev_u):
        """
        Calculate the output given an input and a reference.

        Keyword arguments:
        curr_x -- current data sample of dimension XSIZE (numpy array of size self.xsize)
        curr_y -- current plant output value (float)
        curr_ref -- current reference value (float)
        prev_u -- value of the input finally applied to the plant (Truncated if needed)

        Returns:
        u -- output respective to curr_x (float)
        """

        num_clouds = len(self.clouds)

        # (1) Updates the consequents of all clouds
        for i in range(num_clouds):
            self.clouds[i].update_consequent(self.prev_md[i], self.prev_ref, curr_y,
                                             prev_u, self.c, self.umin, self.umax)

        # (2) Find the the Data Cloud associated to the new sample

        # First, calculate the relative local density relative to each cloud
        relative_ld = [0.]*num_clouds
        for i in range(num_clouds):
            relative_ld[i] = self.clouds[i].get_local_density(curr_x)

        # Second, calculate the normalized relative densities (membership degrees)
        curr_md = [md/float(sum(relative_ld)) for md in relative_ld]

        # Third, find the data cloud that better describes the current sample.
        curr_x_associated_cloud = np.argmax(curr_md)

        # (3) Generate control signal
        curr_u = 0.0
        for i in range(num_clouds):
            curr_u += curr_md[i] * self.clouds[i].get_consequent()

        # (4) Compute Global Density

        # First, concatenates x and u to form z and compute global
        curr_z = np.append(curr_x, curr_u)

        # Second, calculate Global Density of curr_z
        curr_gd = self.get_global_density(curr_z)

        # (5) Perform two tests to check if a new cloud is needed or if it needs to be updated.

        # First, Calculate the global density of the focal points of every existing Data Cloud.
        focal_points_gd = np.array([0.]*num_clouds)
        for i in range(num_clouds):
            focal_points_gd[i] = self.get_global_density(self.clouds[i].zf)

        # Second, Calculate the distances from the current sample to every focal point
        focal_points_distances = np.array([0.]*num_clouds)
        for i in range(num_clouds):
            focal_points_distances[i] = np.linalg.norm(curr_x - self.clouds[i].zf[:self.xsize])

        # Third, Check if the Global Density of the current point is bigger than the Global Densities of all
        # the focal points.
        curr_sample_global_density_is_better = False
        if curr_gd > np.max(focal_points_gd):
            curr_sample_global_density_is_better = True

        # Fourth, Check if the point is far enough from every data cloud.
        curr_sample_is_distant_enough = True
        for i in range(num_clouds):
            if focal_points_distances[i] <= np.max(self.clouds[i].r)/2.:
                curr_sample_is_distant_enough = False

        # Inverse Alternative to FIFTH (Check if sample satisfies one sigma condition)
        # If it's satisfied, a new cloud is not created.
        #if np.max(relative_ld) > 1./math.e:
        #    curr_sample_is_distant_enough = False

        # Fifth, If a new cloud is needed, creates a new cloud
        # Otherwise, adds the current point to the best matching cloud and checks
        # if the focal point has to be updated
        new_cloud_needed = curr_sample_global_density_is_better and curr_sample_is_distant_enough

        # If both conditions are satisfied (global density is better and sample is distant enough), create a new cloud
        if new_cloud_needed:

            # Get variances of all clouds to get the local scatter for the new cloud.
            local_scatters = np.array([[0., 0.]]*num_clouds)
            for i in range(num_clouds):
                local_scatters[i][0] = math.sqrt(self.clouds[i].variance[0])
                local_scatters[i][1] = math.sqrt(self.clouds[i].variance[1])
            new_cloud_local_scatter = np.average(local_scatters, 0)
            new_cloud_variance = new_cloud_local_scatter**2

            # Creates new cloud with focal point zk and starting variance
            self.clouds.append(DataCloud(curr_z, new_cloud_variance, self.radius_update_const))

            # Update Membership degree to include this new cloud!
            relative_ld.append(self.clouds[num_clouds].get_local_density(curr_x))
            curr_md = [float(md)/sum(relative_ld) for md in relative_ld]

        # If a new cloud is not needed, a focal point update might still be needed. If the local density of the current
        # sample relative to the associated cloud is bigger than the local density of the focal point of the associated
        # cloud relative to itself, and also if the global density of the current sample is bigger than the global
        # density of the focal point of the associated cloud, than update the focal point.

        # TEST: Add data sample to data cloud before updating focal point
        # self.clouds[curr_x_associated_cloud].add_point(curr_z)

        if not new_cloud_needed:
            # Local density of the sample and the focal point relative to the associated cloud:
            associated_cloud_xf = self.clouds[curr_x_associated_cloud].zf[:self.xsize]
            associated_cloud_xf_ld = self.clouds[curr_x_associated_cloud].get_local_density(associated_cloud_xf)
            curr_x_ld = self.clouds[curr_x_associated_cloud].get_local_density(curr_x)

            # Global density of the sample and the focal point of the associated cloud:
            associated_cloud_zf = self.clouds[curr_x_associated_cloud].zf
            associated_cloud_zf_gd = self.get_global_density(associated_cloud_zf)

            if curr_x_ld > associated_cloud_xf_ld and curr_gd > associated_cloud_zf_gd:
                self.clouds[curr_x_associated_cloud].update_focal_point(curr_z)

        # TEST: Add data sample to data cloud after updating focal point
        self.clouds[curr_x_associated_cloud].add_point(curr_z)

        # Update Global Density values g_csi and g_b
        # Update global density recursive values
        prev_gcsi = self.g_csi
        prev_gb = self.g_b
        self.g_csi = prev_gcsi + self.prev_z
        self.g_b = prev_gb + np.dot(self.prev_z, self.prev_z)

        # Store last sample
        self.prev_md = np.copy(curr_md)
        self.prev_ref = curr_ref
        self.prev_y = curr_y
        self.prev_z = np.copy(curr_z)

        # Update k before next iteration
        self.k += 1

        # Return output u related to input curr_x
        return curr_u

    def get_global_density(self, z):
        """
        Calculates recursively the Global Density of point curr_z.

        Keyword arguments:
        curr_z -- sample that will have its corresponding global density calculated.
        """
        prev_z = self.prev_z
        prev_gcsi = self.g_csi
        prev_gb = self.g_b

        gcsi_k = prev_gcsi + prev_z
        ga_k = np.dot(z, gcsi_k)
        gb_k = prev_gb + np.dot(prev_z, prev_z)

        gd = float(self.k - 1) / ((self.k - 1) * (np.dot(z, z) + 1) - 2.*ga_k + gb_k)

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

    def __init__(self, z, initial_variance, radius_update_const=0.5):
        """
        Initializes a DataCloud with one point z.
        Extracts x and u, setting u as the consequent q.

        Keyword arguments:
        z --
        initial_variance -- array containing the variance starting value for the new DataCloud
        radius_update_const -- Radius constant, usually 0.5
        """

        # Set radius update constant
        self.radius_update_const = radius_update_const

        # Gets plant input (x) and control signal (u)
        # from z where z = [x', u']', setting them
        # as focal point (xf) and consequent (q) respectively.
        self.zf = np.copy(z)

        self.xsize = len(z) - 1

        # Local density calculation values
        self.csi = np.array([0.0] * self.xsize)

        self.betha = 0.0

        # Data Cloud Size
        self.m = 1

        # Data Cloud Radius
        # Each data cloud has X_SIZE radiuses, one for each dimension of x.
        # By definition the initial radius r1 is 1 for each dimension.
        self.r = np.array([1.0] * self.xsize)

        # Local Scatter square (sigma_square), has to be stored for recursive calculation of
        # the radius. For each dimension of x, there's a sigma associated to it.
        # By definition the initial sigma sigma1 is 1 if not provided
        self.variance = np.copy(initial_variance)

        # Save previous added point for next calculations
        self.prev_z = np.copy(z)

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
        # Extract X and centroid
        x = curr_z[:self.xsize]

        # Calculate New Centroid (OLD WAY)
        # for i in range(0, len(self.centroid)):
        #     new_centroid[i] = (self.centroid[i] * (self.m - 1) + curr_z[i]) / self.m

        # Calulate and Update New Variance (OLD WAY _ WITH CENTROID)
        # for i in range(0, len(self.variance)):
        #     prev_variance = self.variance[i]
        #     self.variance[i] = (1.0 / self.m) * (
        #         (self.m - 1) * prev_variance + (x[i] - self.centroid[i]) * (x[i] - new_centroid[i]))

        # Calulate and Update New Variance (NEW WAY _ WITH FOCAL POINT)
        # for i in range(0, len(self.variance)):
        #     # dist_x_f = self.zf[:self.xsize] - x
        #     dist_z_f = self.zf - curr_z
        #     self.variance[i] = self.variance[i]*float(self.m-1)/self.m + np.dot(dist_z_f, dist_z_f)/float(self.m-1)

        # Calulate and Update New Variance (NEW WAY _ WITH FOCAL POINT)
        for i in range(len(self.variance)):
            dist_x_f = self.zf[:self.xsize][i] - x[i]
            self.variance[i] = self.variance[i]*float(self.m-1)/self.m + (dist_x_f**2)/float(self.m-1)

        # Update centroid (OLD WAY)
        # self.centroid = new_centroid

    def add_point(self, curr_z):
        """
        Associates a new point to the data cloud, updating the number of points
        updating local density values, sigma and radius.

        Keyword arguments:
        curr_z -- datacloud point composed by x (data sample) and u (control signal)
        """

        # Update number of points
        self.m += 1

        # Update Variance
        self.__update_variance_and_centroid__(curr_z)

        # Update radius
        self.__update_radius__()

        # Update local density values
        prev_x = self.prev_z[:self.xsize]
        prev_csi = self.csi
        prev_b = self.betha

        self.csi = prev_csi + prev_x
        self.betha = prev_b + np.dot(prev_x, prev_x)

        # Update Prev values (last added point):
        self.prev_z = np.copy(curr_z)

    def get_local_density(self, x):
        """
        Recursively calculate the local density relative to the sample input x

        Keyword arguments:
        x -- an input of dimension XSIZE
        """
        prev_x = self.prev_z[:self.xsize]
        prev_csi = self.csi
        prev_b = self.betha

        csi_k = prev_csi + prev_x
        a_k = np.dot(x, csi_k)
        b_k = prev_b + np.dot(prev_x, prev_x)

        ld = float(self.m) / (self.m * (np.dot(x, x) + 1) - 2.*a_k + b_k)

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
            dq = 0.0
        if (prev_u >= umax) and (dq > 0):
            dq = 0.0

        # Get Consequent
        q = self.get_consequent()

        # Updates consequent
        self.set_consequent(q+dq)

    def set_consequent(self, new_consequent):
        self.zf[-1] = new_consequent

    def get_consequent(self):
        """
        Extract consequent value from the focal point of the data cloud (zf).
        """
        return self.zf[-1]