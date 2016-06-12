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
Subscribes to a given ROS topic and publishes a DiagnosticStatus describing whether it is being received, and at which rate.
Use this only with low-bandwidth topics (e.g. use the associated CameraInfo topic instead of an Image topic in a camera stream)

(C)2015 Timm Linder, Social Robotics Lab, University of Freiburg
"""

import rospy, socket, importlib, sys, collections, threading
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from spencer_control_msgs.msg import ComponentStatus
from std_msgs.msg import Float32, Bool

rospy.init_node("topic_monitor")

lock = threading.Lock()
hostname = socket.gethostname()

topic = rospy.get_param("~topic", "")
msgType = rospy.get_param("~type", "")
timeout = rospy.get_param("~timeout", 1.0)
maxAvgDelay = rospy.get_param("~max_avg_delay", 0.1)
minAvgRate = rospy.get_param("~min_avg_rate", 0.0)

windowSize = rospy.get_param("~window_size", 20)
hasHeader = False
timestampBuffer = collections.deque(maxlen=windowSize)

if not topic:
  rospy.logerr("_topic must be specified!")
  sys.exit(1)

if not msgType:
  rospy.logerr("_type of message must be specified!")
  sys.exit(1)

if not "/" in msgType:
  rospy.logerr("_type attribute must be of form foo_msgs/Bar")
  sys.exit(1)

rospy.loginfo("Starting topic monitor for topic %s [%s], timeout %.3f sec!" % (topic, msgType, timeout))

def class_for_name(module_name, class_name):
  # load the module, will raise ImportError if module cannot be loaded
  m = importlib.import_module(module_name)
  # get the class, will raise AttributeError if class cannot be found
  c = getattr(m, class_name)
  return c

try:
  [moduleName, className] = msgType.split("/")
  msgClass = class_for_name(moduleName + ".msg", className)
except Exception as e:
  rospy.logerr("Failed to load message class for type %s: %s" % (msgType, str(e)))
  sys.exit(2)

lastMessageAt = rospy.Time(0)

def messageCallback(msg):
  with lock:
    global lastMessageAt, hasHeader
    lastMessageAt = rospy.Time.now()

    if msg._has_header:
      hasHeader = True
      timestampBuffer.append( [ msg.header.stamp, lastMessageAt] )
    else:
      timestampBuffer.append( [ lastMessageAt, lastMessageAt ] )


publisher = rospy.Publisher("/diagnostics", DiagnosticArray, latch=True, queue_size=1)
componentStatusPublisher = rospy.Publisher("/spencer/diagnostics/topic_status", ComponentStatus, latch=True, queue_size=1)
subscriber = rospy.Subscriber(topic, msgClass, messageCallback)

rate = rospy.Rate(1.0 / timeout)
while not rospy.is_shutdown():
  with lock:
    now = rospy.Time.now()
    elapsedSinceLastMsg = now - lastMessageAt
    hasTimeout = elapsedSinceLastMsg > rospy.Duration(timeout)
    reallyTimedOut = elapsedSinceLastMsg > rospy.Duration(3*timeout)

    # Compute average time delay and publish rate
    averageDelay = 0.0
    averageInterval = 0.0
    maxDelay = 0.0
    maxInterval = 0.0
    
    previousTimestamp = None
    for sentTimestamp, receivedTimestamp in timestampBuffer:
      delay = (receivedTimestamp - sentTimestamp).to_sec()
      averageDelay += delay
      maxDelay = max(delay, maxDelay)

      if previousTimestamp is not None:
        interval = (receivedTimestamp - previousTimestamp).to_sec()
        averageInterval += interval
        maxInterval = max(interval, maxInterval)

      previousTimestamp = receivedTimestamp
    
    # Prepare diagnostics message
    diagnosticArray = DiagnosticArray()
    diagnosticArray.header.stamp = rospy.Time.now()
    
    topicStatus = DiagnosticStatus()
    topicStatus.name = "Topic status: " + topic
    topicStatus.hardware_id = hostname
    topicStatus.level = DiagnosticStatus.WARN if lastMessageAt == rospy.Time(0) else DiagnosticStatus.ERROR if hasTimeout else DiagnosticStatus.OK
    
    averageRate = 0.0
    if len(timestampBuffer) > 1:
      # Finish averaging
      averageDelay /= float(len(timestampBuffer))
      averageInterval /= float(len(timestampBuffer) - 1)
      averageRate = 1.0 / max(averageInterval, 1.0 / 10000)

      topicStatus.values.append(KeyValue('Last received at', str(lastMessageAt.to_sec())))
      topicStatus.values.append(KeyValue('Observation window size:', "%d messages" % len(timestampBuffer)))
      topicStatus.values.append(KeyValue('Average interval between messages', "%.3f sec" % averageInterval))

      if hasHeader:
        topicStatus.values.append(KeyValue('Maximum delay wrt. current ROS time', "%.3f sec" % maxDelay))
        topicStatus.values.append(KeyValue('Average delay wrt. current ROS time', "%.3f sec" % averageDelay))

    if lastMessageAt == rospy.Time(0):
      topicStatus.message = "Missing, never received any messages"
    elif hasTimeout:
      topicStatus.message = "Timed out %.2f sec ago" % (elapsedSinceLastMsg.to_sec() - timeout)
    else:
      topicStatus.message = "Active, receiving messages at %.1f Hz" % averageRate

    # Check if we have a low publish rate or high time delay, if configured
    if len(timestampBuffer) > 1:
      if minAvgRate > 0.0:
        topicStatus.values.append(KeyValue('Min. allowed avg rate', "%.1f Hz" % minAvgRate))
      
        if averageRate < minAvgRate:
          topicStatus.message += ", low rate"
          if topicStatus.level == DiagnosticStatus.OK:
            topicStatus.level = DiagnosticStatus.WARN

      if maxAvgDelay > 0.0:
        topicStatus.values.append(KeyValue('Max. allowed avg delay', "%.3f sec" % maxAvgDelay))

        if averageDelay > maxAvgDelay:
          topicStatus.message += ", high delay"
          if topicStatus.level == DiagnosticStatus.OK:
            topicStatus.level = DiagnosticStatus.WARN

    # Publish diagnostics
    diagnosticArray.status.append(topicStatus)
    publisher.publish(diagnosticArray)

    # Publish component status
    componentStatus = ComponentStatus()
    componentStatus.name = topic
    componentStatus.alive = not reallyTimedOut
    componentStatus.detail = topicStatus.message
    componentStatusPublisher.publish(componentStatus)

  try:
    rate.sleep()
  except rospy.exceptions.ROSInterruptException:
    break



