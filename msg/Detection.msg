# unique ID of the detection
uint32              id
# confidence of the detection
float64             confidence

# estimate of the position
geometry_msgs/Point position
# uncertainty of the detection's position
float64[9]          covariance
float64             detection_probability
