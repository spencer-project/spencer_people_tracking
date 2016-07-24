#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2013-2015, Timm Linder, Social Robotics Lab, University of Freiburg
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy, sys, track_annotation_tool

if __name__ == '__main__':
    arguments = rospy.myargv()
    rospy.init_node("track_simulator")

    # Create backend components
    transformer = track_annotation_tool.Transformer()
    database = track_annotation_tool.Database()
    simulator = track_annotation_tool.Simulator(database, None, transformer)

    database.registerLoadCallback(transformer.loadTransformCache)

    visualization = rospy.get_param("~visualization", False)  # publish waypoint markers?
    if visualization:
        visualizer = track_annotation_tool.Visualizer(database, None, transformer)

    # Load database
    filename = rospy.get_param("~filename", None)
    if filename is None:
        rospy.logerr("filename argument with track database must be specified!")
        sys.exit(1)

    try:
        database.load(filename)
    except IOError as e:
        rospy.logerr("Failed to load track database %s. Reason: %s" % (filename, e) )

    # Determine oldest and youngest timestamp in database
    oldestTimestamp = database.findGlobalOldestTimestamp()
    youngestTimestamp = database.findGlobalYoungestTimestamp()
    rospy.loginfo("Oldest timestamp in track database: %.1f, youngest timestamp: %.1f" % (oldestTimestamp, youngestTimestamp))

    # Read clock-related parameters
    publishClock = rospy.get_param("~clock", True)
    simulator.clockGenerator.setEnabled(publishClock)

    simulationLength = None
    if publishClock:
        simulationRate = rospy.get_param("~rate", 1.0)
        simulationLength = rospy.get_param("~duration", -1.0)
        prelude = rospy.get_param("~prelude", 5.0)
        loop = rospy.get_param("~loop", False)

        # Check that database is not zero-length
        if oldestTimestamp == float("inf") or youngestTimestamp == float("-inf"):
            rospy.logerr("Cannot play zero-length track database as it must contain at least two consecutive waypoints!")
            sys.exit(2)

        # Automatically determine simulation length
        if simulationLength <= 0:
            if oldestTimestamp != float("inf") and youngestTimestamp != float("-inf"):
                simulationLength = youngestTimestamp - oldestTimestamp
                rospy.loginfo("Playback duration (automatically determined): %.1f" % simulationLength)
            else:
                rospy.logerr("Cannot play zero-length track database as it must contain at least two consecutive waypoints!")
        else:
            rospy.loginfo("Playback duration (manually set): %.1f" % simulationLength)

        rospy.loginfo("Publishing clock on /clock topic, starting at time %.1f. Simulation rate: %.1fx" % (oldestTimestamp - prelude, simulationRate))

        # Apply parameters
        simulator.clockGenerator.setSimulationStart(oldestTimestamp - prelude)
        simulator.clockGenerator.setSimulationLength(simulationLength + prelude)
        simulator.clockGenerator.setSimulationRate(simulationRate)
        simulator.clockGenerator.setLoop(loop)
    else:
        rospy.loginfo("Clock publishing is disabled, assuming that someone else is publishing a correct clock")
        if rospy.Time.now().to_sec() > youngestTimestamp:
            rospy.logwarn("Youngest timestamp in track database is prior to current ROS time. No tracks will appear unless clock is reset to an earlier time.")


    # Start clock generator (must happen after all calls to rospy.get_param() due to thread synchronization issues)
    simulator.clockGenerator.start()

    # Run simulation
    rospy.loginfo("Starting simulation! Waiting to reach (in time) the first waypoint in the database...")
    firstTimestampReached = False
    lastTimestampReached = False

    updateHz = 30
    updateRate = track_annotation_tool.WallRate(updateHz)
    framesProcessed = 0

    while not rospy.is_shutdown():
        # Update track positions
        simulator.update()

        # Update waypoint visualization if enabled
        if visualization:
            visualizer.update()

        # Display progress
        framesProcessed += 1

        if rospy.Time.now().to_sec() < oldestTimestamp:
            firstTimestampReached = lastTimestampReached = False

        if not firstTimestampReached and rospy.Time.now().to_sec() >= oldestTimestamp:
            rospy.loginfo("Reached time of first waypoint in the database!")
            firstTimestampReached = True
            sys.stderr.write("\n")

        if firstTimestampReached and simulationLength is not None:
            if framesProcessed % updateHz == 0 or simulator.clockGenerator.simulationEnded:
                percent = (rospy.Time.now().to_sec() - oldestTimestamp) / simulationLength
                percent = int(100*max(0.0, min(percent, 1.0)))

                fullBlocks = int(percent/3)
                emptyBlocks = int(100/3 - fullBlocks)

                progressBar = u"[" + u"\u2588" * fullBlocks + u"\u2591" * emptyBlocks + "]   "
                sys.stderr.write("\rPlayback of track database: %s %d %% completed" % (progressBar, percent))
                sys.stderr.flush()

        if not lastTimestampReached and rospy.Time.now().to_sec() >= youngestTimestamp:
            sys.stderr.write("\n\n")
            rospy.loginfo("Reached time of last waypoint in the database!")
            lastTimestampReached = True

        # Check if we're done
        if simulator.clockGenerator.simulationEnded:
            rospy.loginfo("Simulation has ended after %.1f seconds (looping is disabled)!" % simulationLength)
            break

        # Sleep until next update
        try:
            updateRate.sleep()
        except rospy.exceptions.ROSInterruptException as e:
            rospy.logwarn(e)
