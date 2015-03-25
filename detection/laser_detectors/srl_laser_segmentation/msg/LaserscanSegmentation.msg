# A LaserscanSegmentation divides the points of a sensor_msgs/Laserscan into segments of (usually neighbouring)
# points. Each segment is assigned a label, which uniquely identifies the segment. Laserscan points that
# are considered as scene background are not part of any segment.
# LaserscanSegmentation messages can be used as an input to a classifier such as a person detector, or for annotating
# sensor_msgs/Laserscan messages with groundtruth person tracks (in that case, a segment's label denotes a person track ID,
# that is used consistently across multiple frames).

# Standard message header
# The timestamp of this header must be equal to the timestamp of the corresponding Laserscan message,
# and can thus be used for synchronization of Laserscan and LaserscanSegmentation messages.
std_msgs/Header     header

# All segments of interest of the current laser scan. Anything that is not part of a LaserscanSegment is
# implicitly defined as scene background. 
LaserscanSegment[]  segments