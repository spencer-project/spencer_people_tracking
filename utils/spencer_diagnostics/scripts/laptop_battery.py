#! /usr/bin/python

import rospy, socket, subprocess, re, time
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Float32, Bool


def getBatteryState():
  cmd = 'upower -i $(upower -e | grep BAT) | grep --color=never -E "state|percentage|voltage"'
  p = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = True)
  (o,e) = p.communicate()
  if not p.returncode == 0:
    return []
  if not o:
    return []

  batteryVoltage = -1
  batteryPercentage = -1
  laptopPluggedIn = False

  lines = o.split('\n')

  for line in lines:
    if ":" in line:
      key, value = line.split(":", 1)
      key = key.strip().lower()
      value = value.strip().lower()

      if key == "state":
        laptopPluggedIn = value != "discharging"
      if key == "voltage":
        voltageString = re.search('([0-9.]+) *v', value).group(1)
        batteryVoltage = float(voltageString)
      if key == "percentage":
        percentageString = re.search('([0-9.]+) *\%', value).group(1)
        batteryPercentage = float(percentageString)

  return batteryVoltage, batteryPercentage, laptopPluggedIn


rospy.init_node("laptop_battery_monitor")

hostname = socket.gethostname()
rate = rospy.Rate(1.0 / 5.0)
publisher = rospy.Publisher("/diagnostics", DiagnosticArray, latch=True, queue_size=1)
batteryVoltagePublisher = rospy.Publisher("/spencer/computers/%s/battery/voltage" % hostname.replace('-', '_'), Float32, latch=True, queue_size=1)
batteryPercentagePublisher = rospy.Publisher("/spencer/computers/%s/battery/percentage" % hostname.replace('-', '_'), Float32, latch=True, queue_size=1)
laptopPluggedInPublisher = rospy.Publisher("/spencer/computers/%s/plugged_in" % hostname.replace('-', '_'), Bool, latch=True, queue_size=1)

rospy.loginfo("Starting laptop battery monitor...")
lastAntWarningAt = rospy.Time(0)

batteryVoltage = -1
batteryPercentage = -1
laptopPluggedIn = False

nextQueryAt = 0

while not rospy.is_shutdown():  
  if time.time() > nextQueryAt:
    try:
      batteryVoltage, batteryPercentage, laptopPluggedIn = getBatteryState()
      nextQueryAt = time.time() + 15.0 # query interval   
    except ValueError:
      rospy.loginfo("Waiting for laptop battery state to become available...")
      nextQueryAt = time.time() + 15.0
      continue

  diagnosticArray = DiagnosticArray()
  diagnosticArray.header.stamp = rospy.Time.now()
  
  batteryStatus = DiagnosticStatus()
  batteryStatus.name = hostname + " laptop battery power"
  batteryStatus.hardware_id = hostname
  batteryStatus.message="%.1f%% (%.3fV)" % (batteryPercentage, batteryVoltage)

  if batteryPercentage < 20:
    batteryStatus.message += "; Critically low power, recharge NOW!"
    batteryStatus.level = DiagnosticStatus.ERROR
  elif batteryPercentage < 40:
    batteryStatus.message += "; Low power, recharge soon!"
    batteryStatus.level = DiagnosticStatus.WARN
  else:
    batteryStatus.message += "; Power level good"
    batteryStatus.level = DiagnosticStatus.OK

  diagnosticArray.status.append(batteryStatus)

  laptopPluggedStatus = DiagnosticStatus()
  laptopPluggedStatus.name = hostname + " power supply"
  laptopPluggedStatus.hardware_id = hostname
  laptopPluggedStatus.message = "Plugged in" if laptopPluggedIn else "Disconnected!"
  laptopPluggedStatus.level = DiagnosticStatus.OK if laptopPluggedIn else DiagnosticStatus.WARN
  
  diagnosticArray.status.append(laptopPluggedStatus)
  
  publisher.publish(diagnosticArray)
  
  batteryVoltagePublisher.publish(batteryVoltage)
  batteryPercentagePublisher.publish(batteryPercentage)
  laptopPluggedInPublisher.publish(laptopPluggedIn)

  rate.sleep()

print "Closing laptop battery monitor"

