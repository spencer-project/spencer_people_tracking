# The segment label assigned to this set of laser points, usually unique per sensor_msgs/Laserscan.
# If the LaserscanSegmentation message is used to annotate groundtruth person tracks, each label
# corresponds to the ID of a tracked person.
# Note that laser points which are considered as background are not part of any LaserscanSegment!
int16       label

# Indices of the measurements in the sensor_msgs/Laserscan ranges[] and intensities[] arrays
# that are part of this segment. For example, if measurement_indices contains the values
# 0, 1 and 2 and the value of 'label' is 16, then the first three laser measurements (usually the
# three left-most laser points in polar coordinates) are assigned to segment 16.
uint16[]    measurement_indices