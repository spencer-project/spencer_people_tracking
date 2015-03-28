# Internal information on the
# fovis algorithm parameters
# and results

Header header

# True if in the next run the reference
# frame will be changed. This is the case
# when the number of inliers drops below
# a threshold or the previous motion estimate
# failed in last motion estimation.
bool change_reference_frame

# The threshold that is currently
# used for the FAST feature detector.
int32 fast_threshold

# total number of detected keypoints in raw image
int32 num_total_detected_keypoints

# same as above per pyramid level, starting at 0
int32[] num_detected_keypoints

# total number of keypoints after bucketing and
# edge and depth filter
int32 num_total_keypoints

# same as above per pyramid level, starting at 0
int32[] num_keypoints

# info from motion estimator
int32 motion_estimate_status_code
string motion_estimate_status
bool motion_estimate_valid
int32 num_matches
int32 num_inliers
int32 num_reprojection_failures

# runtime of last iteration in seconds
float64 runtime

# motion estimation - in matrix (4x4) form [R|t]
float64[] transformation_matrix
