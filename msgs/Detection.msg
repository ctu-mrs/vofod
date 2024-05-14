# unique ID of the detection
uint32              id
# confidence of the detection
float64             confidence
# number of LiDAR points corresponding to the detection
uint64              n_points

# estimate of the position
geometry_msgs/Point position
# uncertainty of the detection's position
float64[9]          covariance
float64             detection_probability
