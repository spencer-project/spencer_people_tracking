#! /usr/bin/python

from Los.Client import Connection
from Los.Types import *

import rospy, socket
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Float32

rospy.init_node("spencer_robot_battery_monitor")

rate = rospy.Rate(1.0 / 60.0)
publisher = rospy.Publisher("/diagnostics", DiagnosticArray, latch=True, queue_size=1)
batteryVoltagePublisher = rospy.Publisher("/spencer/sensors/battery/voltage", Float32, latch=True, queue_size=1)
batteryPercentagePublisher = rospy.Publisher("/spencer/sensors/battery/percentage", Float32, latch=True, queue_size=1)

rospy.loginfo("Starting robot battery monitor...")

while not rospy.is_shutdown():
  proxy = Connection(("ant", 1234))
  proxy.login("Master", "robox")
  data = proxy.inspect(StringArray(["Safety:batteryPower,batteryVoltage"]))
  
  diagnosticArray = DiagnosticArray()
  diagnosticArray.header.stamp = rospy.Time.now()
  
  batteryStatus = DiagnosticStatus()
  batteryStatus.name = "battery"
  batteryStatus.hardware_id = socket.gethostname()
  batteryStatus.message="%.3fV %.1f%%" % (data.Safety.batteryVoltage, data.Safety.batteryPower)
  
  if data.Safety.batteryPower < 10:
    batteryStatus.message += "; Critically low power, recharge NOW!"
    batteryStatus.level = DiagnosticStatus.ERROR
  elif data.Safety.batteryPower < 20:
    batteryStatus.message += "; Low power, recharge soon!"
    batteryStatus.level = DiagnosticStatus.WARN
  else:
    batteryStatus.message += "; Power level good"
    batteryStatus.level = DiagnosticStatus.OK

  diagnosticArray.status.append(batteryStatus)
  publisher.publish(diagnosticArray)
  batteryVoltagePublisher.publish(data.Safety.batteryVoltage)
  batteryPercentagePublisher.publish(data.Safety.batteryPower)
  rate.sleep()


print "Closing battery monitor"
proxy.close()
