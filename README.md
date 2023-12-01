# MAV detection and localization from depthmap

Intended to be used with Intel Realsense D435 depth camera sensor.

## The working principle

### Detection of blobs in a depthmap image
The algorithm is pretty self-explanatory. For each input depthmap image, do:
 1) The depthmap image is preprocessed based on the selected parameters (`dilate_iterations`, `erode_iterations`, `gaussianblur_size` etc.).
 2) The preprocessed image is thresholded by a certain distance to convert it to binary image (eg. all pixels further than x meters are black, rest is white).
 3) Blobs are found in the binarized image using contour detecton algorithm.
 4) The found blobs are filtered based on the selected parameters (`filter_by_area`, `filter_by_circularity`, `filter_by_orientation` etc.).
 5) Remaining blobs are added to a set of selected blobs.
 6) Steps 2) to 5) are repeated for the selected distance range with the specified distance step (eg. from 1 meter to 13 meters with a step of 0.5 meter).
 7) When the whole distance range has been scanned for blobs, blobs in the resulting set of selected blobs which are closer than `min_dist_between` are grouped together to form a single output blob, which is added to a set of output blobs.
 8) If the number of blobs, which were grouped together to form an output blob, is less than `min_repeatability`, the output blob is kicked out of the set of output blobs.
 9) The set of output blobs is published as the detections.

### 3D localization of a single UAV from detected blobs
Target of this algorithm is to filter out false detections from the blob detection algorithm and to provide a 3D position estimate of the current position of the UAV (a **single UAV is presumed**). A set of active Kalman Filters is kept and a prediction update is made for each KF in this set periodically. The set is initially empty and KFs are added to it with new detections. Each KF represents a hypothesis about the 3D position and velocity of the UAV. If a new set of detections comes in, the following procedure is executed:
 1) The detected blobs are recalculated to 3D locations in the world coordinate system based on the camera projection matrices and the transformation from sensor to world at the time of taking the image in which the blobs were detected.
 2) Covariance of each 3D blob location is calculated based on the set parameters (`xy_covariance_coeff` and `z_covariance_coeff`) and transformed to the world coordinate system (so that it is properly rotated).
 3) For each Kalman Filter in the set of currently active Kalman Filters the following steps are taken:
   1) The measurement with the smallest divergence (currently the Kullback-Leibler divergence is used) from the set of the latest measurements (blob 3D positions with covariances) is associated to the KF and used for a correction update unless the divergence exceeds `max_update_divergence`.
   2) The uncertainty of the KF is calculated (currently a determinant of the state covariance matrix) and if it is higher than `max_lkf_uncertainty`, the KF is kicked out from the pool of currently active KFs and is not considered further.
 4) The KF with the highest number of correction updates is found. If two KFs have the same number of correction updates, the one with the lower uncertainty is picked. If the number of correction updates of the resulting KF is higher than `min_corrs_to_consider` then the position estimation of this KF is published as the current position of the detected UAV.
 5) For each measurement which was not associated to any KF, a new KF is instantiated using the parameters `lkf_process_noise_pos` and `lkf_process_noise_vel` to initialize the process noise matrix and `init_vel_cov` to initialize covariance of the velocity states. The initial position estimate and its covariance are initialized based on the measurement. Initial velocity estimate is set to zero.


## Description of the provided interface and other info

### The following launchfiles are provided:
 * **depth_detect.launch**: Starts the detection of blobs from a depth image.
 * **display_detections.launch**: Starts visualization of the detections to OpenCV windows (raw depthmap, processed depthmap + detected blobs and raw RGB image are displayed).
 * **localize_single.launch**: Starts 3D localization and filtering of a single UAV position from the detected blobs using Kalman Filters.
 * **backproject_location.launch**: Starts visualization of the detected UAV 3D location. The location is backprojected to the RGB image and displayed in an OpenCV window.
 * Variants of the above launchfiles, starting with **sim_** prependix are provided for use with simulation.
 * **simulation.launch**: Starts the simulation world with some trees and a grass pane. Also starts static transform broadcasters for the Realsense camera coordinate frames!

### The following config files are used by the nodes:
 * **detection_params_F550.yaml:** Contains parameters for the blob detection, tuned to detect the F550 from real-world data. Parameters are documented in the file itself.
 * **localization_params_F550.yaml:** Contains parameters for the single UAV localization, tuned to detect the F550 from real-world data. Parameters are documented in the file itself.
 * Variants of the above files with the *sim_* prependix are provided for use with simulation.
Most parameters (those which make sense) from the above files are dynamically reconfigurable.

### To launch simulation, detection, localization and visualization:
 1) In folder **tmux_scripts/simulation** launch the tmuxinator session to start simulation with two drones: `uav1` and `uav2`. `uav1` is the interceptor with the Realsense sensor and `uav2` is an intruder with a random flier controlling it.
 2) In folder **tmux_scripts/detection_and_visualization** launch the tmuxinator session to start detection, localization and visualization nodes. It is recommended to use install Tomáš's layout manager before-hand (just update to latest linux-setup) to enable automatic application of a layout (otherwise the windows will be quite cluttered).
 3) You can adjust the detection and localization parameters according to Rviz or according to the OpenCV visualization using the **rqt_reconfigure** (which is automatically launched in the **detection_and_visualization** session).
