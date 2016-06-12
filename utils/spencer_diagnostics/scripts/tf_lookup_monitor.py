#! /usr/bin/python

# Software License Agreement (BSD License)
#
#  Copyright (c) 2015, Timm Linder, Social Robotics Lab, University of Freiburg
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#  * Neither the name of the copyright holder nor the names of its contributors
#    may be used to endorse or promote products derived from this software
#    without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

""" 
Subscribes to TF and publishes a DiagnosticStatus describing whether a particular set of TF transforms is received,
and at which rate.

(C)2015 Timm Linder, Social Robotics Lab, University of Freiburg
"""

import rospy, socket, importlib, sys, collections, tf
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from spencer_control_msgs.msg import ComponentStatus
from std_msgs.msg import Float32, Bool

rospy.init_node("tf_lookup__monitor")

hostname = socket.gethostname()

transform = rospy.get_param("~transform", "")  # syntax: "source_frame --> target_frame"
timeout = rospy.get_param("~timeout", 1.0)
maxAvgDelay = rospy.get_param("~max_avg_delay", 0.15)
minAvgRate = rospy.get_param("~min_avg_rate", 0.0)

windowSize = rospy.get_param("~window_size", 20)
timestampBuffer = collections.deque(maxlen=windowSize)

if not transform:
  rospy.logerr("_transform must be specified!")
  sys.exit(1)

if not "-->" in transform:
  rospy.logerr("_transform syntax: source_frame --> target_frame")
  sys.exit(1)

source_frame, target_frame = transform.split("-->")
source_frame = source_frame.strip()
target_frame = target_frame.strip()

rospy.loginfo("Starting transform lookup monitor for transform from %s to %s, timeout %.3f sec!" % (source_frame, target_frame, timeout))

publisher = rospy.Publisher("/diagnostics", DiagnosticArray, latch=True, queue_size=1)
componentStatusPublisher = rospy.Publisher("/spencer/diagnostics/tf_status", ComponentStatus, latch=True, queue_size=1)
tfListener = tf.TransformListener()


rate = rospy.Rate(1.0 / timeout)
while not rospy.is_shutdown():
  now = rospy.Time.now()
  try:
    lastTransformAt = tfListener.getLatestCommonTime(source_frame, target_frame)
    timestampBuffer.append( [ lastTransformAt, now ] )
  except tf.Exception:
    lastTransformAt = rospy.Time(0)
    
  elapsedSinceLastTransform = now - lastTransformAt
  hasTimeout = elapsedSinceLastTransform > rospy.Duration(timeout)
  reallyTimedOut = elapsedSinceLastTransform > rospy.Duration(3*timeout)

  # Compute average time delay and publish rate
  averageDelay = 0.0
  #averageInterval = 0.0
  maxDelay = 0.0
  #maxInterval = 0.0
    
  previousTimestamp = None
  for sentTimestamp, receivedTimestamp in timestampBuffer:
    delay = (receivedTimestamp - sentTimestamp).to_sec()
    averageDelay += delay
    maxDelay = max(delay, maxDelay)

    #if previousTimestamp is not None:
      # interval = (receivedTimestamp - previousTimestamp).to_sec()
      # averageInterval += interval
      # maxInterval = max(interval, maxInterval)
    previousTimestamp = receivedTimestamp

  # Prepare diagnostics message
  diagnosticArray = DiagnosticArray()
  diagnosticArray.header.stamp = rospy.Time.now()

  tfStatus = DiagnosticStatus()
  tfStatus.name = "TF link status: %s --> %s" % (source_frame, target_frame)
  tfStatus.hardware_id = hostname
  tfStatus.level = DiagnosticStatus.WARN if lastTransformAt == rospy.Time(0) else DiagnosticStatus.ERROR if hasTimeout else DiagnosticStatus.OK

  averageRate = 0.0
  if len(timestampBuffer) > 1:
    # Finish averaging
    averageDelay /= float(len(timestampBuffer))
    # averageInterval /= float(len(timestampBuffer) - 1)
    # averageRate = 1.0 / averageInterval

    tfStatus.values.append(KeyValue('Last received at', str(lastTransformAt.to_sec())))
    tfStatus.values.append(KeyValue('Observation window size:', "%d messages" % len(timestampBuffer)))
    #tfStatus.values.append(KeyValue('Average interval between messages', "%.3f sec" % averageInterval))
    tfStatus.values.append(KeyValue('Maximum delay wrt. current ROS time', "%.3f sec" % maxDelay))
    tfStatus.values.append(KeyValue('Average delay wrt. current ROS time', "%.3f sec" % averageDelay))

  if lastTransformAt == rospy.Time(0):
    tfStatus.message = "Missing, never received any transform"
  elif hasTimeout:
    tfStatus.message = "Timed out %.2f sec ago" % (elapsedSinceLastTransform.to_sec() - timeout)
  else:
    tfStatus.message = "Active, receiving transforms (avg. delay %.3f sec)" % averageDelay # at %.1f Hz" % averageRate

  # Check if we have a low publish rate or high time delay, if configured
  if len(timestampBuffer) > 1:
    # if minAvgRate > 0.0:
    #   tfStatus.values.append(KeyValue('Min. allowed avg rate', "%.1f Hz" % minAvgRate))
    
    #   if averageRate < minAvgRate:
    #     tfStatus.message += ", low rate"
    #     if tfStatus.level == DiagnosticStatus.OK:
    #       tfStatus.level = DiagnosticStatus.WARN

    if maxAvgDelay > 0.0:
      tfStatus.values.append(KeyValue('Max. allowed avg delay', "%.3f sec" % maxAvgDelay))

      if averageDelay > maxAvgDelay and not hasTimeout:
        tfStatus.message += ", high delay"
        if tfStatus.level == DiagnosticStatus.OK:
          tfStatus.level = DiagnosticStatus.WARN

  # Publish diagnostics
  diagnosticArray.status.append(tfStatus)
  publisher.publish(diagnosticArray)

  # Publish component status
  componentStatus = ComponentStatus()
  componentStatus.name = "%s --> %s" % (source_frame, target_frame)
  componentStatus.alive = not reallyTimedOut
  componentStatus.detail = tfStatus.message
  componentStatusPublisher.publish(componentStatus)

  rate.sleep()



