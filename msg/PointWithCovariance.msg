# This represents a point in 3D space with uncertainty.

geometry_msgs/Point point

# Row-major representation of the 2x2 covariance matrix
# In order, the parameters are:
# (sigma_x, sigma_xy, sigma_yx, sigma_y )
#float64[4] covariance
# (sigma_x, sigma_xy, sigma_xz, sigma_yx, sigma_y, sigma_yz, sigma_zx, sigma_zy, sigma_z)
float64[9] covariance
