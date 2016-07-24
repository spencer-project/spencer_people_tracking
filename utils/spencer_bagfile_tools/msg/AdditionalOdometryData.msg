Header header # message header

uint64 hardwareTimestamp # milliseconds in hardware time since power-on

float64 totalDistance # total absolute distance travelled so far (i.e. 10m forward, 10m backward => 20m)
float64 theta # current theta angle in radians

int64 ticksLeft # raw ticks of left encoder
int64 ticksRight # raw ticks of right encoder

float64 calibOverallMultiplier # calibration factor applied to both encoders' raw ticks
float64 calibLeftEncMultiplier # calibration factor applied to only the left encoders' raw ticks, in addition to calibOverallMultiplier