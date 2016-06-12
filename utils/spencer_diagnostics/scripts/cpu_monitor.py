#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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

from __future__ import with_statement
import roslib

import rospy

import traceback
import threading
from threading import Timer
import sys, os, time
from time import sleep
import subprocess
import string
from multiprocessing import cpu_count

import socket

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

##### monkey-patch to suppress threading error message in python 2.7.3
##### See http://stackoverflow.com/questions/13193278/understand-python-threading-bug
if sys.version_info[:3] == (2, 7, 3):
    import threading
    threading._DummyThread._Thread__stop = lambda x: 42
#####

stat_dict = { 0: 'OK', 1: 'Warning', 2: 'Error' }

# Output entire IPMI data set
def check_ipmi():
    diag_vals = []
    diag_msgs = []
    diag_level = DiagnosticStatus.OK

    try:
        p = subprocess.Popen('sudo ipmitool sdr',
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode
                        
        if retcode != 0:
            diag_level = DiagnosticStatus.ERROR
            diag_msgs = [ 'ipmitool Error' ]
            diag_vals = [ KeyValue(key = 'IPMI Error', value = stderr) ]
            return diag_vals, diag_msgs, diag_level

        lines = stdout.split('\n')
        if len(lines) < 2:
            diag_vals = [ KeyValue(key = 'ipmitool status', value = 'No output') ]

            diag_msgs = [ 'No ipmitool response' ]
            diag_level = DiagnosticStatus.ERROR

            return diag_vals, diag_msgs, diag_level

        for ln in lines:
            if len(ln) < 3:
                continue

            words = ln.split('|')
            if len(words) < 3:
                continue 

            name = words[0].strip()
            ipmi_val = words[1].strip()
            stat_byte = words[2].strip()

            # CPU temps
            if words[0].startswith('CPU') and words[0].strip().endswith('Temp'):
                if words[1].strip().endswith('degrees C'):
                    tmp = ipmi_val.rstrip(' degrees C').lstrip()
                    if unicode(tmp).isnumeric():
                        temperature = float(tmp)
                        diag_vals.append(KeyValue(key = name + ' (C)', value = tmp))

                        cpu_name = name.split()[0]
                        if temperature >= 80 and temperature < 89:
                            diag_level = max(diag_level, DiagnosticStatus.WARN)
                            if diag_msgs.count('CPU Hot') == 0:
                                diag_msgs.append('CPU Warm')

                        if temperature >= 89: # CPU should shut down here
                            diag_level = max(diag_level, DiagnosticStatus.ERROR)
                            diag_msgs.append('CPU Hot')                                
                            # Don't keep CPU Warm in list if CPU is hot
                            if diag_msgs.count('CPU Warm') > 0:
                                idx = diag_msgs.index('CPU Warm')
                                diag_msgs.pop(idx)
                else:
                    diag_vals.append(KeyValue(key = name, value = words[1]))


            # MP, BP, FP temps
            if name == 'MB Temp' or name == 'BP Temp' or name == 'FP Temp':
                if ipmi_val.endswith('degrees C'):
                    tmp = ipmi_val.rstrip(' degrees C').lstrip()
                    diag_vals.append(KeyValue(key = name + ' (C)', value = tmp))
                    # Give temp warning
                    dev_name = name.split()[0]
                    if unicode(tmp).isnumeric():
                        temperature = float(tmp)

                        if temperature >= 60 and temperature < 75:
                            diag_level = max(diag_level, DiagnosticStatus.WARN)
                            diag_msgs.append('%s Warm' % dev_name)

                        if temperature >= 75:
                            diag_level = max(diag_level, DiagnosticStatus.ERROR)
                            diag_msgs.append('%s Hot' % dev_name)
                    else:
                        diag_level = max(diag_level, DiagnosticStatus.ERROR)
                        diag_msgs.append('%s Error' % dev_name)
                else:
                    diag_vals.append(KeyValue(key = name, value = ipmi_val))
        
            # CPU fan speeds
            if (name.startswith('CPU') and name.endswith('Fan')) or name == 'MB Fan':
                if ipmi_val.endswith('RPM'):
                    rpm = ipmi_val.rstrip(' RPM').lstrip()
                    if unicode(rpm).isnumeric():
                        if int(rpm) == 0:
                            diag_level = max(diag_level, DiagnosticStatus.ERROR)
                            diag_msgs.append('CPU Fan Off')
                            
                        diag_vals.append(KeyValue(key = name + ' RPM', value = rpm))
                    else:
                        diag_vals.append(KeyValue(key = name, value = ipmi_val))

            # If CPU is hot we get an alarm from ipmitool, report that too
            # CPU should shut down if we get a hot alarm, so report as error
            if name.startswith('CPU') and name.endswith('hot'):
                if ipmi_val == '0x01':
                    diag_vals.append(KeyValue(key = name, value = 'OK'))
                else:
                    diag_vals.append(KeyValue(key = name, value = 'Hot'))
                    diag_level = max(diag_level, DiagnosticStatus.ERROR)
                    diag_msgs.append('CPU Hot Alarm')

    except Exception, e:
        diag_vals.append(KeyValue(key = 'Exception', value = traceback.format_exc()))
        diag_level = DiagnosticStatus.ERROR
        diag_msgs.append('Exception')

    return diag_vals, diag_msgs, diag_level
        

##\brief Check CPU core temps 
##
## Use 'find /sys -name temp1_input' to find cores
## Read from every core, divide by 1000
def check_core_temps(sys_temp_strings):
    diag_vals = []
    diag_level = 0
    diag_msgs = []
    
    for index, temp_str in enumerate(sys_temp_strings):
        if len(temp_str) < 5:
            continue
        
        cmd = 'cat %s' % temp_str
        p = subprocess.Popen(cmd, stdout = subprocess.PIPE, 
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            diag_level = DiagnosticStatus.ERROR
            diag_msg = [ 'Core Temp Error' ]
            diag_vals = [ KeyValue(key = 'Core Temp Error', value = stderr), 
                          KeyValue(key = 'Output', value = stdout) ]
            return diag_vals, diag_msgs, diag_level
  
        tmp = stdout.strip()
        if unicode(tmp).isnumeric():
            temp = float(tmp) / 1000
            diag_vals.append(KeyValue(key = 'Core %d Temp' % index, value = str(temp)))

            if temp >= 85 and temp < 90:
                diag_level = max(diag_level, DiagnosticStatus.WARN)
                diag_msgs.append('Warm')
            if temp >= 90:
                diag_level = max(diag_level, DiagnosticStatus.ERROR)
                diag_msgs.append('Hot')
        else:
            diag_level = max(diag_level, DiagnosticStatus.ERROR) # Error if not numeric value
            diag_vals.append(KeyValue(key = 'Core %s Temp' % index, value = tmp))

    return diag_vals, diag_msgs, diag_level

## Checks clock speed from reading from CPU info
def check_clock_speed(enforce_speed):
    vals = []
    msgs = []
    lvl = DiagnosticStatus.OK

    try:
        p = subprocess.Popen('cat /proc/cpuinfo | grep MHz', 
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            lvl = DiagnosticStatus.ERROR
            msgs = [ 'Clock speed error' ]
            vals = [ KeyValue(key = 'Clock speed error', value = stderr), 
                     KeyValue(key = 'Output', value = stdout) ]
            
            return (vals, msgs, lvl)

        for index, ln in enumerate(stdout.split('\n')):
            words = ln.split(':')
            if len(words) < 2:
                continue

            speed = words[1].strip().split('.')[0] # Conversion to float doesn't work with decimal
            vals.append(KeyValue(key = 'Core %d MHz' % index, value = speed))
            if unicode(speed).isnumeric():
                mhz = float(speed)
                
                if mhz < 2240 and mhz > 2150:
                    lvl = max(lvl, DiagnosticStatus.WARN)
                if mhz <= 2150:
                    lvl = max(lvl, DiagnosticStatus.ERROR)
            else:
                # Automatically give error if speed isn't a number
                lvl = max(lvl, DiagnosticStatus.ERROR)

        if not enforce_speed:
            lvl = DiagnosticStatus.OK

        if lvl == DiagnosticStatus.WARN and enforce_speed:
            msgs = [ 'Core slowing' ]
        elif lvl == DiagnosticStatus.ERROR and enforce_speed:
            msgs = [ 'Core throttled' ]

    except Exception, e:
        rospy.logerr(traceback.format_exc())
        lvl = DiagnosticStatus.ERROR
        msgs.append('Exception')
        vals.append(KeyValue(key = 'Exception', value = traceback.format_exc()))

    return vals, msgs, lvl
                    

# Add msgs output, too
##\brief Uses 'uptime' to see load average
def check_uptime(load1_threshold, load5_threshold):
    level = DiagnosticStatus.OK
    vals = []
    
    load_dict = { 0: 'OK', 1: 'High Load', 2: 'Very High Load' }

    try:
        p = subprocess.Popen('uptime', stdout = subprocess.PIPE, 
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            vals.append(KeyValue(key = 'uptime Failed', value = stderr))
            return DiagnosticStatus.ERROR, vals

        upvals = stdout.split()
        load1 = upvals[-3].rstrip(',').replace(',', '.')
        load5 = upvals[-2].rstrip(',').replace(',', '.')
        load15 = upvals[-1].replace(',', '.')
        num_users = upvals[-7]

        # Give warning if we go over load limit 
        if float(load1) > load1_threshold or float(load5) > load5_threshold:
            level = DiagnosticStatus.WARN

        vals.append(KeyValue(key = 'Load Average Status', value = load_dict[level]))
        vals.append(KeyValue(key = '1 min Load Average', value = load1))
        vals.append(KeyValue(key = '1 min Load Average Threshold', value = str(load1_threshold)))
        vals.append(KeyValue(key = '5 min Load Average', value = load5))
        vals.append(KeyValue(key = '5 min Load Average Threshold', value = str(load5_threshold)))
        vals.append(KeyValue(key = '15 min Load Average', value = load15))
        vals.append(KeyValue(key = 'Number of Users', value = num_users))

    except Exception, e:
        rospy.logerr(traceback.format_exc())
        level = DiagnosticStatus.ERROR
        vals.append(KeyValue(key = 'Load Average Status', value = traceback.format_exc()))
        
    return level, load_dict[level], vals

# Add msgs output
##\brief Uses 'free -m' to check free memory
def check_memory():
    values = []
    level = DiagnosticStatus.OK
    msg = ''

    mem_dict = { 0: 'OK', 1: 'Low Memory', 2: 'Very Low Memory' }

    try:
        p = subprocess.Popen('free -m',
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode
               
        if retcode != 0:
            values.append(KeyValue(key = "\"free -m\" Call Error", value = str(retcode)))
            return DiagnosticStatus.ERROR, values
 
        rows = stdout.split('\n')
        data = rows[1].split()
        total_mem = data[1]
        used_mem = data[2]
        free_mem = data[3]

        level = DiagnosticStatus.OK
        if float(free_mem) < 25:
            level = DiagnosticStatus.WARN
        if float(free_mem) < 1:
            level = DiagnosticStatus.ERROR

        values.append(KeyValue(key = 'Memory Status', value = mem_dict[level]))
        values.append(KeyValue(key = 'Total Memory', value = total_mem))
        values.append(KeyValue(key = 'Used Memory', value = used_mem))
        values.append(KeyValue(key = 'Free Memory', value = free_mem))

        msg = mem_dict[level]
    except Exception, e:
        rospy.logerr(traceback.format_exc())
        msg = 'Memory Usage Check Error'
        values.append(KeyValue(key = msg, value = str(e)))
        level = DiagnosticStatus.ERROR
    
    return level, mem_dict[level], values



##\brief Use mpstat to find CPU usage
##
usage_old = 0
has_warned_mpstat = False
has_error_core_count = False
def check_mpstat(core_count = -1):
    vals = []
    mp_level = DiagnosticStatus.OK
    
    load_dict = { 0: 'OK', 1: 'High Load', 2: 'Error' }

    try:
        p = subprocess.Popen('mpstat -P ALL 1 1',
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            global has_warned_mpstat
            if not has_warned_mpstat:
                rospy.logerr("mpstat failed to run for cpu_monitor. Return code %d.", retcode)
                has_warned_mpstat = True

            mp_level = DiagnosticStatus.ERROR
            vals.append(KeyValue(key = '\"mpstat\" Call Error', value = str(retcode)))
            return mp_level, 'Unable to Check CPU Usage', vals

        # Check which column '%idle' is, #4539
        # mpstat output changed between 8.06 and 8.1
        rows = stdout.split('\n')
        col_names = rows[2].split()
        idle_col = -1 if (len(col_names) > 2 and col_names[-1] == '%idle') else -2

        num_cores = 0
        cores_loaded = 0
        for index, row in enumerate(stdout.split('\n')):
            if index < 3:
                continue
            
            # Skip row containing 'all' data
            if row.find('all') > -1:
                continue

            lst = row.split()
            if len(lst) < 8:
                continue

            ## Ignore 'Average: ...' data
            if lst[0].startswith('Average'):
                continue

            cpu_name = '%d' % (num_cores)
            idle = lst[idle_col].replace(',', '.')
            user = lst[3].replace(',', '.')
            nice = lst[4].replace(',', '.')
            system = lst[5].replace(',', '.')
            
            core_level = 0
            usage = float(user) + float(nice)
            if usage > 1000: # wrong reading, use old reading instead
                rospy.logwarn('Read cpu usage of %f percent. Reverting to previous reading of %f percent'%(usage, usage_old))
                usage = usage_old
            usage_old = usage

            if usage > 90.0:
                cores_loaded += 1
                core_level = DiagnosticStatus.WARN
            if usage > 110.0:
                core_level = DiagnosticStatus.ERROR

            vals.append(KeyValue(key = 'CPU %s Status' % cpu_name, value = load_dict[core_level]))
            vals.append(KeyValue(key = 'CPU %s User' % cpu_name, value = user))
            vals.append(KeyValue(key = 'CPU %s Nice' % cpu_name, value = nice))
            vals.append(KeyValue(key = 'CPU %s System' % cpu_name, value = system))
            vals.append(KeyValue(key = 'CPU %s Idle' % cpu_name, value = idle))

            num_cores += 1
        
        # Warn for high load only if we have <= 2 cores that aren't loaded
        if num_cores - cores_loaded <= 2 and num_cores > 2:
            mp_level = DiagnosticStatus.WARN

        # Check the number of cores if core_count > 0, #4850
        if core_count > 0 and core_count != num_cores:
            mp_level = DiagnosticStatus.ERROR
            global has_error_core_count
            if not has_error_core_count:
                rospy.logerr('Error checking number of cores. Expected %d, got %d. Computer may have not booted properly.',
                              core_count, num_cores)
                has_error_core_count = True
            return DiagnosticStatus.ERROR, 'Incorrect number of CPU cores', vals
            
    except Exception, e:
        mp_level = DiagnosticStatus.ERROR
        vals.append(KeyValue(key = 'mpstat Exception', value = str(e)))

    return mp_level, load_dict[mp_level], vals

## Returns names for core temperature files
## Returns list of names, each name can be read like file
def get_core_temp_names():
    temp_vals = []
    try:
        p = subprocess.Popen('find /sys/devices -name temp1_input', 
                             stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell = True)
        stdout, stderr = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            rospy.logerr('Error find core temp locations: %s' % stderr)
            return []
        
        for ln in stdout.split('\n'):
            temp_vals.append(ln.strip())
        
        return temp_vals
    except:
        rospy.logerr('Exception finding temp vals: %s' % traceback.format_exc())
        return []

def update_status_stale(stat, last_update_time):
    time_since_update = rospy.get_time() - last_update_time

    stale_status = 'OK'
    if time_since_update > 20 and time_since_update <= 35:
        stale_status = 'Lagging'
        if stat.level == DiagnosticStatus.OK:
            stat.message = stale_status
        elif stat.message.find(stale_status) < 0:
            stat.message = ', '.join([stat.message, stale_status])
        stat.level = max(stat.level, DiagnosticStatus.WARN)
    if time_since_update > 35:
        stale_status = 'Stale'
        if stat.level == DiagnosticStatus.OK:
            stat.message = stale_status
        elif stat.message.find(stale_status) < 0:
            stat.message = ', '.join([stat.message, stale_status])
        stat.level = max(stat.level, DiagnosticStatus.ERROR)


    stat.values.pop(0)
    stat.values.pop(0)
    stat.values.insert(0, KeyValue(key = 'Update Status', value = stale_status))
    stat.values.insert(1, KeyValue(key = 'Time Since Update', value = str(time_since_update)))
    

class CPUMonitor():
    def __init__(self, hostname, diag_hostname):
        self._diag_pub = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=3)

        self._mutex = threading.Lock()

        self._check_ipmi = rospy.get_param('~check_ipmi_tool', True)
        self._enforce_speed = rospy.get_param('~enforce_clock_speed', True)

        self._check_core_temps = rospy.get_param('~check_core_temps', False)
        if self._check_core_temps:
            rospy.logwarn('Checking CPU core temperatures is deprecated. This will be removed in D-turtle')
        self._check_nfs = rospy.get_param('~check_nfs', False)
        if self._check_nfs:
            rospy.logwarn('NFS checking is deprecated for CPU monitor. This will be removed in D-turtle')

        # sane defaults according to http://blog.scoutapp.com/articles/2009/07/31/understanding-load-averages
        self._load1_threshold = rospy.get_param('~load1_threshold', 0.7*cpu_count())
        self._load5_threshold = rospy.get_param('~load5_threshold', 1.0*cpu_count())

        self._num_cores = rospy.get_param('~num_cores', 8.0)

        self._temps_timer = None
        self._usage_timer = None
        self._nfs_timer = None
        
        # Get temp_input files
        self._temp_vals = get_core_temp_names()

        # CPU stats
        self._temp_stat = DiagnosticStatus()
        self._temp_stat.name = '%s CPU Temperature' % diag_hostname
        self._temp_stat.level = 1
        self._temp_stat.hardware_id = hostname
        self._temp_stat.message = 'No Data'
        self._temp_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                   KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._usage_stat = DiagnosticStatus()
        self._usage_stat.name = '%s CPU Usage' % diag_hostname
        self._usage_stat.level = 1
        self._usage_stat.hardware_id = hostname
        self._usage_stat.message = 'No Data'
        self._usage_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                    KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._nfs_stat = DiagnosticStatus()
        self._nfs_stat.name = '%s NFS IO' % diag_hostname
        self._nfs_stat.level = 1
        self._nfs_stat.hardware_id = hostname
        self._nfs_stat.message = 'No Data'
        self._nfs_stat.values = [ KeyValue(key = 'Update Status', value = 'No Data' ),
                                  KeyValue(key = 'Time Since Last Update', value = 'N/A') ]

        self._last_temp_time = 0
        self._last_usage_time = 0
        self._last_nfs_time = 0
        self._last_publish_time = 0

        # Start checking everything
        self.check_temps()
        if self._check_nfs:
            self.check_nfs_stat()
        self.check_usage()

    # Restart temperature checking 
    def _restart_temp_check(self):
        rospy.logerr('Restarting temperature check thread in cpu_monitor. This should not happen')
        try:
            with self._mutex:
                if self._temps_timer:
                    self._temps_timer.cancel()
                
            self.check_temps()
        except Exception, e:
            rospy.logerr('Unable to restart temp thread. Error: %s' % traceback.format_exc())
            

    ## Must have the lock to cancel everything
    def cancel_timers(self):
        if self._temps_timer:
            self._temps_timer.cancel()

        if self._nfs_timer:
            self._nfs_timer.cancel()

        if self._usage_timer:
            self._usage_timer.cancel()

    def check_nfs_stat(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
                return

        nfs_level = 0
        msg = 'OK'
        vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                 KeyValue(key = 'Time Since Last Update', value = str(0) )]

        try:
            p = subprocess.Popen('iostat -n',
                                 stdout = subprocess.PIPE,
                                 stderr = subprocess.PIPE, shell = True)
            stdout, stderr = p.communicate()
            retcode = p.returncode
            
            if retcode != 0:
                nfs_level = DiagnosticStatus.ERROR
                msg = 'iostat Error'
                vals.append(KeyValue(key = '\"iostat -n\" Call Error', value = str(e)))
                stdout = ''
                

            for index, row in enumerate(stdout.split('\n')):
                if index < 3:
                    continue
                
                lst = row.split()
                if len(lst) < 7:
                    continue
                
                file_sys = lst[0]
                read_blk = lst[1]
                write_blk = lst[2]
                read_blk_dir = lst[3]
                write_blk_dir = lst[4]
                r_blk_srv = lst[5]
                w_blk_srv = lst[6]
                
                vals.append(KeyValue(
                        key = '%s Read Blks/s' % file_sys, value=read_blk))
                vals.append(KeyValue(
                        key = '%s Write Blks/s' % file_sys, value=write_blk))
                vals.append(KeyValue(
                        key = '%s Read Blk dir/s' % file_sys, value=read_blk_dir))
                vals.append(KeyValue(
                        key = '%s Write Blks dir/s' % file_sys, value=write_blk_dir))
                vals.append(KeyValue(
                        key = '%s Read Blks srv/s' % file_sys, value=r_blk_srv))
                vals.append(KeyValue(
                        key = '%s Write Blks srv/s' % file_sys, value=w_blk_srv))
                
        except Exception, e:
            rospy.logerr(traceback.format_exc())
            nfs_level = DiagnosticStatus.ERROR
            msg = 'Exception'
            vals.append(KeyValue(key = 'Exception', value = str(e)))
          
        with self._mutex:
            self._nfs_stat.level = nfs_level
            self._nfs_stat.message = msg
            self._nfs_stat.values = vals
            
            self._last_nfs_time = rospy.get_time()
            
            if not rospy.is_shutdown():
                self._nfs_timer = threading.Timer(5.0, self.check_nfs_stat)
                self._nfs_timer.start()
            else:
                self.cancel_timers()


    ## Call every 10sec at minimum
    def check_temps(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
            return

        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = str(0) ) ]
        diag_msgs = []
        diag_level = 0

        if self._check_ipmi:
            ipmi_vals, ipmi_msgs, ipmi_level = check_ipmi()
            diag_vals.extend(ipmi_vals)
            diag_msgs.extend(ipmi_msgs)
            diag_level = max(diag_level, ipmi_level)

        if self._check_core_temps:
            core_vals, core_msgs, core_level = check_core_temps(self._temp_vals)
            diag_vals.extend(core_vals)
            diag_msgs.extend(core_msgs)
            diag_level = max(diag_level, core_level)

        clock_vals, clock_msgs, clock_level = check_clock_speed(self._enforce_speed)
        diag_vals.extend(clock_vals)
        diag_msgs.extend(clock_msgs)
        diag_level = max(diag_level, clock_level)

        diag_log = set(diag_msgs)
        if len(diag_log) > 0:
            message = ', '.join(diag_log)
        else:
            message = stat_dict[diag_level]

        with self._mutex:
            self._last_temp_time = rospy.get_time()
            
            self._temp_stat.level = diag_level
            self._temp_stat.message = message
            self._temp_stat.values = diag_vals
            
            if not rospy.is_shutdown():
                self._temps_timer = threading.Timer(5.0, self.check_temps)
                self._temps_timer.start()
            else:
                self.cancel_timers()

    def check_usage(self):
        if rospy.is_shutdown():
            with self._mutex:
                self.cancel_timers()
            return 

        diag_level = 0
        diag_vals = [ KeyValue(key = 'Update Status', value = 'OK' ),
                      KeyValue(key = 'Time Since Last Update', value = 0 )]
        diag_msgs = []

        # Check mpstat
        mp_level, mp_msg, mp_vals = check_mpstat(self._num_cores)
        diag_vals.extend(mp_vals)
        if mp_level > 0:
            diag_msgs.append(mp_msg)
        diag_level = max(diag_level, mp_level)
            
        # Check uptime
        uptime_level, up_msg, up_vals = check_uptime(self._load1_threshold, self._load5_threshold)
        diag_vals.extend(up_vals)
        if uptime_level > 0:
            diag_msgs.append(up_msg)
        diag_level = max(diag_level, uptime_level)
        
        # Check memory
        mem_level, mem_msg, mem_vals = check_memory()
        diag_vals.extend(mem_vals)
        if mem_level > 0:
            diag_msgs.append(mem_msg)
        diag_level = max(diag_level, mem_level)

        if diag_msgs and diag_level > 0:
            usage_msg = ', '.join(set(diag_msgs))
        else:
            usage_msg = stat_dict[diag_level]

        # Update status
        with self._mutex:
            self._last_usage_time = rospy.get_time()
            self._usage_stat.level = diag_level
            self._usage_stat.values = diag_vals
            
            self._usage_stat.message = usage_msg
            
            if not rospy.is_shutdown():
                self._usage_timer = threading.Timer(5.0, self.check_usage)
                self._usage_timer.start()
            else:
                self.cancel_timers()

    def publish_stats(self):
        with self._mutex:
            # Update everything with last update times
            update_status_stale(self._temp_stat, self._last_temp_time)
            update_status_stale(self._usage_stat, self._last_usage_time)
            if self._check_nfs:
                update_status_stale(self._nfs_stat, self._last_nfs_time)

            msg = DiagnosticArray()
            msg.header.stamp = rospy.get_rostime()
            msg.status.append(self._temp_stat)
            msg.status.append(self._usage_stat)
            if self._check_nfs:
                msg.status.append(self._nfs_stat)

            if rospy.get_time() - self._last_publish_time > 0.5:
                self._diag_pub.publish(msg)
                self._last_publish_time = rospy.get_time()

        
        # Restart temperature checking if it goes stale, #4171
        # Need to run this without mutex
        if rospy.get_time() - self._last_temp_time > 90: 
            self._restart_temp_check()


if __name__ == '__main__':
    hostname = socket.gethostname().replace("spencer-", "")

    import optparse
    parser = optparse.OptionParser(usage="usage: cpu_monitor.py [--diag-hostname=cX]")
    parser.add_option("--diag-hostname", dest="diag_hostname",
                      help="Computer name in diagnostics output (ex: 'c1')",
                      metavar="DIAG_HOSTNAME",
                      action="store", default = hostname)
    options, args = parser.parse_args(rospy.myargv())

    try:
        rospy.init_node('cpu_monitor_%s' % hostname)
    except rospy.exceptions.ROSInitException:
        print >> sys.stderr, 'CPU monitor is unable to initialize node. Master may not be running.'
        sys.exit(0)

    cpu_node = CPUMonitor(hostname, options.diag_hostname)

    rate = rospy.Rate(1.0)
    try:
        while not rospy.is_shutdown():
            rate.sleep()
            cpu_node.publish_stats()
    except KeyboardInterrupt:
        pass
    except Exception, e:
        traceback.print_exc()
        rospy.logerr(traceback.format_exc())

    cpu_node.cancel_timers()
    sys.exit(0)
    


    

            

