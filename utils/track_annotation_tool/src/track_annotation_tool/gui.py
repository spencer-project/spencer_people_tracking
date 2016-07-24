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

import os, rospy, threading, time, rospkg
import track_annotation_tool

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget, QMessageBox, QFileDialog, QInputDialog

from spencer_human_attribute_msgs.msg import CategoricalAttribute
from std_msgs.msg import Header
from geometry_msgs.msg import PointStamped


class TrackAnnotationPlugin(Plugin):
    def __init__(self, context):
        super(TrackAnnotationPlugin, self).__init__(context)
        self.setObjectName('TrackAnnotationPlugin')
        self.shutdownRequested = False

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()

        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true", dest="quiet", help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self.widget = QWidget()
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'gui.ui')
        loadUi(ui_file, self.widget)

        # Give QObjects reasonable names
        self.widget.setObjectName('TrackAnnotationPluginUi')

        if context.serial_number() > 1:
            self.widget.setWindowTitle(self.widget.windowTitle() + (' (%d)' % context.serial_number()))

        # Setup backend components
        self.setupBackend()

        # Populate inputs
        self.widget.genderCombo.addItem("")
        self.widget.genderCombo.addItem(CategoricalAttribute.GENDER_MALE)
        self.widget.genderCombo.addItem(CategoricalAttribute.GENDER_FEMALE)

        self.widget.ageGroupCombo.addItem("")
        self.widget.ageGroupCombo.addItem(CategoricalAttribute.AGE_GROUP_0_TO_2)
        self.widget.ageGroupCombo.addItem(CategoricalAttribute.AGE_GROUP_3_TO_7)
        self.widget.ageGroupCombo.addItem(CategoricalAttribute.AGE_GROUP_8_TO_12)
        self.widget.ageGroupCombo.addItem(CategoricalAttribute.AGE_GROUP_13_TO_19)
        self.widget.ageGroupCombo.addItem(CategoricalAttribute.AGE_GROUP_20_TO_36)
        self.widget.ageGroupCombo.addItem(CategoricalAttribute.AGE_GROUP_37_TO_65)
        self.widget.ageGroupCombo.addItem(CategoricalAttribute.AGE_GROUP_66_TO_99)

        # Register event handlers
        uiActions = UiActions(self.widget, self.database, self.editor, self.simulator, self.visualizer)
        self.uiActions = uiActions

        self.widget.loadDatabaseButton.clicked[bool].connect(uiActions.loadDatabase)
        self.widget.saveDatabaseButton.clicked[bool].connect(uiActions.saveDatabase)
        self.widget.saveDatabaseAsButton.clicked[bool].connect(uiActions.saveDatabaseAs)

        self.widget.newTrackButton.clicked[bool].connect(uiActions.newTrack)
        self.widget.previousTrackButton.clicked[bool].connect(uiActions.previousTrack)
        self.widget.trackIdText.returnPressed.connect(uiActions.trackIdTextChanged)
        self.widget.nextTrackButton.clicked[bool].connect(uiActions.nextTrack)
        self.widget.deleteTrackButton.clicked[bool].connect(uiActions.deleteTrack)
        self.widget.mergeTracksButton.clicked[bool].connect(uiActions.mergeTracks)

        self.widget.waypointFrameCombo.editTextChanged[str].connect(uiActions.waypointFrameChanged)
        self.widget.previousWaypointButton.clicked[bool].connect(uiActions.previousWaypoint)
        self.widget.waypointIndexText.returnPressed.connect(uiActions.waypointIndexTextChanged)
        self.widget.nextWaypointButton.clicked[bool].connect(uiActions.nextWaypoint)
        self.widget.deleteWaypointButton.clicked[bool].connect(uiActions.deleteWaypoint)

        self.widget.genderCombo.editTextChanged[str].connect(uiActions.genderChanged)
        self.widget.ageGroupCombo.editTextChanged[str].connect(uiActions.ageGroupChanged)

        self.widget.syncTracksWithDetectionsCheckbox.stateChanged[int].connect(uiActions.syncTracksWithDetectionsChanged)
        self.widget.publishClockCheckbox.stateChanged[int].connect(uiActions.publishClockChanged)
        self.widget.hideInactiveTrajectoriesCheckbox.stateChanged[int].connect(uiActions.hideInactiveTrajectoriesChanged)
        self.widget.pauseCheckbox.stateChanged[int].connect(uiActions.pauseChanged)

        self.widget.simulationLengthInput.valueChanged[float].connect(uiActions.simulationLengthChanged)
        self.widget.simulationRateInput.valueChanged[float].connect(uiActions.simulationRateChanged)

        # Set default values
        self.widget.syncTracksWithDetectionsCheckbox.setCheckState(0)
        self.widget.hideInactiveTrajectoriesCheckbox.setCheckState(0)
        self.widget.waypointFrameCombo.setEditText("laser_front_link")
        self.widget.publishClockCheckbox.setCheckState(0)

        # Add widget to the user interface
        context.add_widget(self.widget)

        # Initialization done, start update thread
        rospy.loginfo("Track annotation editor has finished loading!")
        self.startUpdateThread()

        # Load database if specified
        databaseToLoad = str(rospy.get_param("~filename", ""))
        if databaseToLoad:
            uiActions.loadGivenDatabase(databaseToLoad)


    def setupBackend(self):
        # Create backend components
        self.transformer = track_annotation_tool.Transformer()
        self.database = track_annotation_tool.Database()
        self.editor = track_annotation_tool.Editor(self.database, self.transformer)
        self.simulator = track_annotation_tool.Simulator(self.database, self.editor, self.transformer)
        self.visualizer = track_annotation_tool.Visualizer(self.database, self.editor, self.transformer)

        # Create ROS subscribers for communication with RViz plugin
        self.waypointSubscriber = WaypointSubscriber(self.editor)

        # Start clock generator (must happen after all calls to rospy.get_param() due to thread synchronization issues)
        self.simulator.clockGenerator.start()

    def startUpdateThread(self):
        # Start update loop
        self.thread = threading.Thread(target=self.updateLoop)
        self.thread.start()

    def updateLoop(self):
        # Run simulation
        lastTime = rospy.Time.now()
        rate = track_annotation_tool.WallRate(30)  # in hertz

        while not self.shutdownRequested:
            currentTime = rospy.Time.now()
            if currentTime.to_sec() < lastTime.to_sec():
                rospy.logwarn("*** TIME MOVED BACKWARDS! RESETTING TRACK POSITIONS! ***")
            lastTime = currentTime

            self.simulator.update()
            self.visualizer.update()

            self.updateAvailableFrames()

            try:
                rate.sleep()
            except rospy.exceptions.ROSInterruptException as e:
                rospy.logwarn(e)

    def updateAvailableFrames(self):
        selectedFrame = self.editor.activeFrameId
        existingFrames = [self.widget.waypointFrameCombo.itemText(i) for i in range(self.widget.waypointFrameCombo.count())]
        for frameId in sorted(self.simulator.transformer.tfListener.getFrameStrings()):
            if not frameId in existingFrames:
                self.widget.waypointFrameCombo.addItem(frameId)
                #rospy.loginfo("New TF frame available: " + frameId)
        self.widget.waypointFrameCombo.setEditText(selectedFrame)

    def shutdown_plugin(self):
        self.shutdownRequested = True
        if self.database.hasUnsavedChanges:
            reply = QMessageBox.warning(self.widget, "Close track annotation tool", "You have unsaved changes. Do you want to save these changes before closing?", QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.uiActions.saveDatabase()

    def save_settings(self, plugin_settings, instance_settings):
        plugin_settings.set_value('default_waypoint_frame', self.editor.activeFrameId)
        plugin_settings.set_value('hide_inactive_trajectories', self.visualizer.hideInactiveTrajectories)

    def restore_settings(self, plugin_settings, instance_settings):
        self.widget.hideInactiveTrajectoriesCheckbox.setCheckState(2 if plugin_settings.value('hide_inactive_trajectories') else 0)
        defaultWaypointFrame = plugin_settings.value('default_waypoint_frame')
        if defaultWaypointFrame:
            self.widget.waypointFrameCombo.setEditText(defaultWaypointFrame)


class UiActions(object):
    def __init__(self, widget, database, editor, simulator, visualizer):
        self.widget = widget
        self.database = database
        self.editor = editor
        self.simulator = simulator
        self.visualizer = visualizer

        rospack = rospkg.RosPack()
        self.defaultDatabaseDirectory = rospack.get_path('track_annotation_tool') + os.path.sep + "saved_data"

        self.database.registerLoadCallback(self._onDatabaseLoaded)
        self.editor.registerActiveTrackChangedCallback(self._onActiveTrackChanged)
        self.editor.registerActiveWaypointChangedCallback(self._onActiveWaypointChanged)

    def loadDatabase(self, unused=None):
        # Load database
        filename = QFileDialog.getOpenFileName(self.widget, caption="Load track database", filter="*.tracks", directory=self.defaultDatabaseDirectory)[0]
        self.loadGivenDatabase(filename)

    def loadGivenDatabase(self, filename):
        try:
            self.database.load(filename)
            self.databaseFilename = filename
            self._onDatabaseFilenameChanged()
        except IOError as e:
            rospy.logerr("Failed to load track database %s. Reason: %s" % (filename, e) )

    def saveDatabase(self, unused=None):
        if not self.databaseFilename:
            self.saveDatabaseAs()
        else:
            rospy.loginfo(str(self.databaseFilename))
            self.database.save(self.databaseFilename)

    def saveDatabaseAs(self, unused=None):
        self.databaseFilename = QFileDialog.getSaveFileName(self.widget, caption="Save track database", filter="*.tracks", directory=self.defaultDatabaseDirectory)[0]
        if self.databaseFilename:
            if not self.databaseFilename.endswith(".tracks"):
                self.databaseFilename += ".tracks"
            self._onDatabaseFilenameChanged()
            self.saveDatabase()

    def trackIdTextChanged(self):
        trackId = int(self.widget.trackIdText.text())
        self.editor.selectActiveTrack(trackId)

    def previousTrack(self, unused=None):
        trackIds = self.database.getTrackIds()
        self.editor.selectActiveTrack( trackIds[trackIds.index(self.editor.activeTrackId)-1] )

    def nextTrack(self, unused=None):
        trackIds = self.database.getTrackIds()
        self.editor.selectActiveTrack( trackIds[min(len(trackIds)-1, trackIds.index(self.editor.activeTrackId)+1)] )

    def newTrack(self, unused=None):
        self.editor.addNewTrack()
        self.updateTrackCount()

    def deleteTrack(self, unused=None):
        trackId = self.editor.activeTrackId
        reply = QMessageBox.warning(self.widget, "Delete track %d" % trackId, "Do you really want to delete the track with ID %d? This cannot be undone." % trackId, QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.editor.deleteActiveTrack()
        self.updateTrackCount()

    def verifyTrackExists(self, trackId):
        if not trackId in self.database.getTrackIds():
            QMessageBox.critical(self.widget, "Wrong track ID", "Person track with ID %d does not exist!" % trackId)
            return False
        return True

    def mergeTracks(self, unused=None):
        (track1Id, ok) = QInputDialog.getInteger(self.widget, "Merge two tracks", "Please enter the ID of the 1st person track you want to merge.")
        if not ok:
            return

        (track2Id, ok) = QInputDialog.getInteger(self.widget, "Merge two tracks", "Please enter the ID of the 2nd person track you want to merge.")
        if not ok:
            return

        if track1Id == track2Id:
            QMessageBox.critical(self.widget, "Merge two tracks", "Track IDs cannot be identical!")
            return

        if self.verifyTrackExists(track1Id) and self.verifyTrackExists(track2Id):
            self.editor.mergeTracks(track1Id, track2Id)
            self.updateTrackCount()
            QMessageBox.information(self.widget, "Merge two tracks", "Person tracks %d and %d have been merged!" % (track1Id, track2Id))

    def waypointFrameChanged(self, stringValue):
        self.editor.activeFrameId = stringValue.encode('ascii','ignore')

    def waypointIndexTextChanged(self):
        waypointIndex = int(self.widget.waypointIndexText.text())
        self.editor.selectActiveWaypoint(waypointIndex)

    def previousWaypoint(self, unused=None):
        self.editor.selectActiveWaypoint(self.editor.activeWaypointIndex-1)

    def nextWaypoint(self, unused=None):
        self.editor.selectActiveWaypoint(self.editor.activeWaypointIndex+1)

    def deleteWaypoint(self, unused=None):
        self.editor.deleteActiveWaypoint()

    def genderChanged(self, stringValue):
        categoricalAttributes = self.database.getCategoricalAttributes(self.editor.getActiveTrack())
        categoricalAttributes[CategoricalAttribute.GENDER] = stringValue.encode('ascii', 'ignore')

    def ageGroupChanged(self, stringValue):
        categoricalAttributes = self.database.getCategoricalAttributes(self.editor.getActiveTrack())
        categoricalAttributes[CategoricalAttribute.AGE_GROUP] = stringValue.encode('ascii', 'ignore')

    def syncTracksWithDetectionsChanged(self, state):
        self.simulator.synchronizeWithDetections = state > 0

    def hideInactiveTrajectoriesChanged(self, state):
        self.visualizer.hideInactiveTrajectories = state > 0

    def publishClockChanged(self, state):
        if state > 0:
            self.updateClockGeneratorTimes()

        self.simulator.clockGenerator.enabled = state > 0
        self.widget.clockSettingsFrame.setEnabled(state > 0)
        self.widget.simulationLengthInput.setEnabled(state > 0)
        self.widget.simulationRateInput.setEnabled(state > 0)
        self.widget.pauseCheckbox.setEnabled(state > 0)

    def pauseChanged(self, state):
        self.simulator.clockGenerator.paused = state > 0

    def updateClockGeneratorTimes(self):
        oldestTimestamp = self.database.findGlobalOldestTimestamp()
        if oldestTimestamp != float("inf"):
            self.prelude = 5.0  # time in seconds before 1st waypoint
            simulationStart = oldestTimestamp
            rospy.loginfo("Simulated time will start %.1f seconds before first waypoint which is at %.1f" % (self.prelude, simulationStart))
        else:
            self.prelude = simulationStart = 0.0
            rospy.logwarn("No waypoints exist. Simulated time will start at 0.0!")

        self.simulator.clockGenerator.setSimulationStart(simulationStart - self.prelude)
        self.simulationLengthChanged(self.widget.simulationLengthInput.value())

    def determineRequiredSimulationLength(self):
        length = self.widget.simulationLengthInput.value()

        oldestTimestamp = self.database.findGlobalOldestTimestamp()
        youngestTimestamp = self.database.findGlobalYoungestTimestamp()
        rospy.loginfo("Oldest timestamp in track database: %.1f, youngest timestamp: %.1f" % (oldestTimestamp, youngestTimestamp))
        if oldestTimestamp != float("inf") and youngestTimestamp != float("-inf"):
            length = max(length, youngestTimestamp - oldestTimestamp)

        return length + self.prelude

    def simulationLengthChanged(self, floatValue):
        requiredLength = self.determineRequiredSimulationLength()
        if requiredLength != floatValue:
            rospy.logwarn("Overriding specified simulation length %.1f with length %.1f because waypoint timestamps exist beyond that point!" % (floatValue, requiredLength))
        else:
            rospy.loginfo("Setting simulation length to %.1f!" % requiredLength)

        self.simulator.clockGenerator.setSimulationLength(requiredLength)

    def simulationRateChanged(self, floatValue):
        self.simulator.clockGenerator.setSimulationRate(floatValue)

    def waypointIndexTextChanged(self):
        waypointIndex = int(self.widget.waypointIndexText.text())
        self.editor.selectActiveWaypoint(waypointIndex)

    def updateTrackCount(self):
        trackCount = len(self.database.getTracks())
        self.widget.trackCountLabel.setText(str(trackCount))

    def _onDatabaseLoaded(self, filename):
        self.updateTrackCount()
        if self.widget.publishClockCheckbox.isChecked():
            self.updateClockGeneratorTimes()

    def _onDatabaseFilenameChanged(self):
        filename = self.databaseFilename[:]
        if len(filename) > 80:
            filename = "..." + filename[len(filename)-80:]
        self.widget.databaseFilenameLabel.setText(filename)

    def _onActiveTrackChanged(self):
        trackId = self.editor.activeTrackId
        self.widget.trackIdText.setText(str(trackId))

        activeTrack = self.editor.getActiveTrack()
        waypoints = self.database.getWaypoints(activeTrack)
        waypointCount = len(waypoints)
        self.widget.waypointCountLabel.setText(str(waypointCount))

        firstWaypointAt = float("nan")
        lastWaypointAt = float("nan")

        if waypointCount > 0:
            firstWaypointAt = waypoints[0]["timestamp"]
            lastWaypointAt = waypoints[-1]["timestamp"]

        self.widget.trackAppearsAtLabel.setText("%.2f s" % firstWaypointAt)
        self.widget.trackDisappearsAfterLabel.setText("%.2f s" % lastWaypointAt)

        categoricalAttributes = self.database.getCategoricalAttributes(self.editor.getActiveTrack())
        self.widget.genderCombo.setEditText(categoricalAttributes[CategoricalAttribute.GENDER] if CategoricalAttribute.GENDER in categoricalAttributes else "")
        self.widget.ageGroupCombo.setEditText(categoricalAttributes[CategoricalAttribute.AGE_GROUP] if CategoricalAttribute.AGE_GROUP in categoricalAttributes else "")

    def _onActiveWaypointChanged(self):
        activeTrack = self.editor.getActiveTrack()

        waypoints = self.database.getWaypoints(activeTrack)
        waypointIndex = float("nan")
        waypointTimestamp = float("nan")

        if waypoints:
            waypointIndex = self.editor.activeWaypointIndex
            waypointTimestamp = waypoints[waypointIndex]["timestamp"]

        self.widget.waypointIndexText.setText(str(waypointIndex))
        self.widget.waypointTimestampText.setText(str(waypointTimestamp))


class WaypointSubscriber(object):
    """ Subscribes to add / delete waypoint messages from track_annotation_tool Rviz plugin """
    def __init__(self, editor):
        self.editor = editor

        newWaypointTopic = "track_annotation_tool/new_waypoints"
        self.newWaypointSubscriber = rospy.Subscriber(newWaypointTopic, PointStamped, self.newWaypointCallback)

        deleteActiveWaypointTopic = "track_annotation_tool/delete_active_waypoint"
        self.deleteActiveWaypointSubscriber = rospy.Subscriber(deleteActiveWaypointTopic, Header, self.deleteActiveWaypointCallback)

        rospy.loginfo("Listening for new waypoints on %s and for deleted waypoints on %s" % (newWaypointTopic, deleteActiveWaypointTopic) )

    def newWaypointCallback(self, pointStamped):
        self.editor.addWaypoint(pointStamped)

    def deleteActiveWaypointCallback(self, header):
        self.editor.deleteActiveWaypoint()
