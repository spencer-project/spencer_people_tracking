#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
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

'''
A node to check the TF tree

A big chunk of the code was copied from tfwtf, the wtf plugin for tf.
'''

import roslib; roslib.load_manifest('diagnostic_common_diagnostics')
import rospy
import diagnostic_updater as DIAG



#-------------------------------------------------------------------------------
# Copying from tfwtf.py begins here

import time
import tf.msg

# global list of messages received
_msgs = []

def rostime_delta(ctx):
    deltas = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            d = t.header.stamp - stamp
            secs = d.to_sec()
            if abs(secs) > 1.:
                if callerid in deltas:
                    if abs(secs) > abs(deltas[callerid]):
                        deltas[callerid] = secs
                else:
                    deltas[callerid]  = secs

    errors = []
    for k, v in deltas.iteritems():
        errors.append("receiving transform from [%s] that differed from ROS time by %ss"%(k, v))
    return errors

def reparenting(ctx):
    errors = []
    parent_id_map = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            frame_id = t.child_frame_id
            parent_id = t.header.frame_id
            if frame_id in parent_id_map and parent_id_map[frame_id] != parent_id:
                msg = "reparenting of [%s] to [%s] by [%s]"%(frame_id, parent_id, callerid)
                parent_id_map[frame_id] = parent_id
                if msg not in errors:
                    errors.append(msg)
            else:
                parent_id_map[frame_id] = parent_id
    return errors

def cycle_detection(ctx):
    max_depth = 100
    errors = []
    parent_id_map = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            frame_id = t.child_frame_id
            parent_id = t.header.frame_id
            parent_id_map[frame_id] = parent_id

    for frame in parent_id_map:
        frame_list = []
        current_frame = frame
        count = 0
        while count < max_depth + 2:
            count = count + 1
            frame_list.append(current_frame)
            try:
                current_frame = parent_id_map[current_frame]
            except KeyError:
                break
            if current_frame == frame:
                errors.append("Frame %s is in a loop. It's loop has elements:\n%s"% (frame, " -> ".join(frame_list)))
                break


    return errors

def multiple_authority(ctx):
    errors = []
    authority_map = {}
    for m, stamp, callerid in _msgs:
        for t in m.transforms:
            frame_id = t.child_frame_id
            parent_id = t.header.frame_id
            if frame_id in authority_map and authority_map[frame_id] != callerid:
                msg = "node [%s] publishing transform [%s] with parent [%s] already published by node [%s]"%(callerid, frame_id, parent_id, authority_map[frame_id])
                authority_map[frame_id] = callerid
                if msg not in errors:
                    errors.append(msg)
            else:
                authority_map[frame_id] = callerid
    return errors

def no_msgs(ctx):
    return not _msgs

# rospy subscriber callback for /tf
def _tf_handle(msg):
    _msgs.append((msg, rospy.get_rostime(), msg._connection_header['callerid']))


# Copying from tfwtf.py stops here
#-------------------------------------------------------------------------------


def make_diag_fn(fn, errlvl, errmsg, okmsg="OK"):
    '''A diagnostic function generator'''

    def diag_fn(stat):
        stat.summary(0, okmsg)

        res = fn(None)
        if isinstance(res, bool):
            if res:
                stat.summary(errlvl, errmsg)
        elif isinstance(res, list):
            if len(res)>0:
                stat.summary(errlvl, errmsg)
                for i,r in enumerate(res):
                    stat.add("Error %d" % (i+1), r)

        return stat

    return diag_fn


rospy.init_node('tf_monitor')


diag_updater = DIAG.Updater()
diag_updater.setHardwareID('none')
diag_updater.add('Messaging status', make_diag_fn(no_msgs, DIAG.WARN, 'No tf messages received') )
diag_updater.add('Time status', make_diag_fn(rostime_delta, DIAG.WARN, 'Received out-of-date/future transforms') )
diag_updater.add('Reparenting status', make_diag_fn(reparenting, DIAG.ERROR, 'TF re-parenting contention') )
diag_updater.add('Cycle status', make_diag_fn(cycle_detection, DIAG.ERROR, 'TF cycle detection') )
diag_updater.add('Multiple authority status', make_diag_fn(multiple_authority, DIAG.ERROR, 'TF multiple authority contention') )


while not rospy.is_shutdown():
    _msgs = []
    sub1 = rospy.Subscriber('/tf', tf.msg.tfMessage, _tf_handle)
    time.sleep(5.0)
    sub1.unregister()
    diag_updater.update()

