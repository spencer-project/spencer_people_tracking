#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##\author Kevin Watts

from __future__ import division

PKG = 'pr2_computer_monitor'
import roslib; roslib.load_manifest(PKG)

import rospy
import socket
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from pr2_msgs.msg import GPUStatus

import subprocess
import math

MAX_FAN_RPM = 4500

def _rads_to_rpm(rads):
    return rads / (2 * math.pi) * 60

def _rpm_to_rads(rpm):
    return rpm * (2 * math.pi) / 60

def gpu_status_to_diag(gpu_stat):
    stat = DiagnosticStatus()
    stat.name = socket.gethostname() + ' GPU Status'
    stat.message = ''
    stat.level = DiagnosticStatus.OK
    stat.hardware_id = socket.gethostname() + "/" + gpu_stat.product_name

    stat.values.append(KeyValue(key='Product Name',         value = gpu_stat.product_name))
    #stat.values.append(KeyValue(key='PCI Device/Vendor ID', value = gpu_stat.pci_device_id))
    #stat.values.append(KeyValue(key='PCI Location ID',      value = gpu_stat.pci_location))
    #stat.values.append(KeyValue(key='Display',              value = gpu_stat.display))
    stat.values.append(KeyValue(key='Driver Version',       value = gpu_stat.driver_version))
    stat.values.append(KeyValue(key='Temperature (C)',      value = '%.0f' % gpu_stat.temperature))
    #stat.values.append(KeyValue(key='Fan Speed (RPM)',      value = '%.0f' % _rads_to_rpm(gpu_stat.fan_speed)))
    #stat.values.append(KeyValue(key='Usage (%)',            value = '%.0f' % gpu_stat.gpu_usage))
    stat.values.append(KeyValue(key='Memory (%)',           value = '%.0f' % gpu_stat.memory_usage))

    errors = []

    # Check for valid data
    if not gpu_stat.product_name:
        stat.level = DiagnosticStatus.ERROR
        errors.append('No Device Data')
    else:
        # Check load
        if gpu_stat.gpu_usage > 95:
            stat.level = max(stat.level, DiagnosticStatus.WARN)
            errors.append('High Load')

        # Check thresholds
        if gpu_stat.temperature > 95:
            stat.level = max(stat.level, DiagnosticStatus.ERROR)
            errors.append('Temperature Alarm')
        elif gpu_stat.temperature > 90:
            stat.level = max(stat.level, DiagnosticStatus.WARN)
            errors.append('High Temperature')

        # Check memory usage
        if gpu_stat.memory_usage > 95:
            stat.level = max(stat.level, DiagnosticStatus.ERROR)
            errors.append('Memory critical')
        elif gpu_stat.memory_usage > 90:
            stat.level = max(stat.level, DiagnosticStatus.WARN)
            errors.append('Low Memory')

        # Check fan
        #if gpu_stat.fan_speed == 0:
        #    stat.level = max(stat.level, DiagnosticStatus.ERROR)
        #    stat.message = 'No Fan Speed'

    if not errors:
        stat.message = 'OK'
    else:
        stat.message = ', '.join(errors)

    return stat


def _find_val(output, word):
    lines = output.split('\n')
    for line in lines:
        tple = line.split(':')
        if not len(tple) > 1:
            continue

        name = tple[0].strip()
        val = ':'.join(tple[1:]).strip()

        if not name.lower() == word.lower():
            continue
        
        return val.strip()

    return ''

def parse_smi_output(output):
    gpu_stat = GPUStatus()

    
    gpu_stat.product_name   = _find_val(output, 'Product Name')
    gpu_stat.pci_device_id  = _find_val(output, 'PCI Device/Vendor ID')
    gpu_stat.pci_location   = _find_val(output, 'PCI Location ID')
    gpu_stat.display        = _find_val(output, 'Display')
    gpu_stat.driver_version = _find_val(output, 'Driver Version')
    
    temp_str = _find_val(output, 'GPU Current Temp')
    if temp_str and not "N/A" in temp_str:
        temp, units = temp_str.split()
        gpu_stat.temperature = int(temp)

    fan_str = _find_val(output, 'Fan Speed')
    if fan_str and not "N/A" in fan_str:
        # Fan speed in RPM
        fan_spd = float(fan_str.strip('\%').strip()) * 0.01 * MAX_FAN_RPM
        # Convert fan speed to Hz
        gpu_stat.fan_speed = _rpm_to_rads(fan_spd)
    else:
        gpu_stat.fan_speed = 0

    usage_str = _find_val(output, 'GPU')
    if usage_str and not "N/A" in usage_str:
        usage = usage_str.strip('\%').strip()
        gpu_stat.gpu_usage = int(usage)
    else:
        gpu_stat.gpu_usage = -1
        
    memUsed = 0
    mem_str = _find_val(output, 'Used')
    if mem_str and not "N/A" in mem_str:
        mem = mem_str.strip(' MiB').strip()
        memUsed = int(mem)

    memTotal = 0
    mem_str = _find_val(output, 'Total')
    if mem_str and not "N/A" in mem_str:
        mem = mem_str.strip(' MiB').strip()
        memTotal = int(mem)

    gpu_stat.memory_usage = int(100.0 * memUsed / memTotal)

    return gpu_stat
        
def get_gpu_status():
    p = subprocess.Popen('nvidia-smi -a', stdout = subprocess.PIPE, 
                         stderr = subprocess.PIPE, shell = True)
    (o, e) = p.communicate()

    if not p.returncode == 0:
        return ''

    if not o: return ''

    return o
