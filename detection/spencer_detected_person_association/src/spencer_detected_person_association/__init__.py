# Software License Agreement (BSD License)
#
#  Copyright (c) 2014-2015, Timm Linder, Social Robotics Lab, University of Freiburg
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
The TrackAssociation class subscribes to /spencer/perception/tracked_persons and provides a lookup from any past detection ID to the associated track ID, if any.
Note that this module cannot look into the future -- if the person tracker is lagging behind the detections, the detection-to-track lookup will not return anything useful for the latest detections!
You can use the TrackSynchronizer class to buffer incoming ROS messages until person tracks are available for the timestamps of these messages.
"""

import collections, rospy, threading, message_filters
from spencer_tracking_msgs.msg import CompositeDetectedPersons, TrackedPersons


""" Subscribes to /spencer/perception/tracked_persons and invokes the registered callback(s). """
class TrackSubscriber(object):
    def __init__(self):
        self.topicName = "/spencer/perception/tracked_persons"
        self.subscriber = rospy.Subscriber(self.topicName, TrackedPersons, self.newMessage)
        self.callbacks = []

    def addCallback(self, newTracksCallback):
        self.callbacks.append(newTracksCallback)

    def newMessage(self, msg):
        for callback in self.callbacks:
            callback(msg)

"""
Provides a lookup for track IDs, given a detection ID, by monitoring /spencer/perception/tracked_persons and /spencer/perception_internal/fused_person_detections.
This is the main class to be used when associating frame-by-frame detections (e.g. human attributes such as age, gender for a given person detection ID) with longer-lasting person tracks.
This class is thread-safe.

Example of usage:
trackAssociation = spencer_detected_person_association.TrackAssociation()
trackId = trackAssociation.lookupTrackId(someDetectionId)
"""
class TrackAssociation(object):
    """
    Constructor, automatically subscribes to /spencer/perception/tracked_persons to
    :param maxDetectionsToRemember: Maximum number of detections to remember.
    """
    def __init__(self, maxFusedDetectionsToRemember = 1000, maxOriginalDetectionsToRemember = 10000):
        self.lock = threading.Lock()

        self.trackLookup = dict()
        self.oldDetections = collections.deque() # used to efficiently remove old detections from the dictionary if max length is exceeded

        self.trackSubscriber = TrackSubscriber()
        self.trackSubscriber.addCallback(self.newTracksAvailable)

        self.maxFusedDetectionsToRemember = maxFusedDetectionsToRemember
        self.fusedDetectionIdMemory = FusedDetectionIdMemory(maxOriginalDetectionsToRemember)

    """
    Looks up the track ID with which the given detection ID is associated, if any. The detection ID might be a fused detection ID
    from the /spencer/perception/detected_persons topic (after merging detections of different detectors), or an original detection ID
    that directly comes from any of the detectors in the /spencer/perception_internal/ namespace.
    Can be None if no track is associated with the given detection ID (e.g. due to it being considered as a false alarm).
    """
    def lookupTrackId(self, detectionId):
        with self.lock:
            # In the normal case, we should know which track this detection belongs to, if it was not considered as a false alarm by the tracker.
            trackId = self.trackLookup.get(detectionId)

            if trackId is None:
                # Do we know any tracks at all?
                if not self.trackLookup:
                    rospy.logwarn("Lookup from detections to tracks is empty, make sure %s is being published!" % self.trackSubscriber.topicName)
                else:
                    # Maybe the provided detection ID is an original detection ID (before combining different sensor modalities).
                    # Lookup the ID into which this original detection was fused, and then try again to find the associated track.
                    fusedDetectionId = self.fusedDetectionIdMemory.lookupFusedDetectionId(detectionId)
                    if fusedDetectionId is not None:
                        trackId = self.trackLookup.get(fusedDetectionId)

            return trackId

    """ Internal callback which processes new tracks. """
    def newTracksAvailable(self, trackedPersons):
        with self.lock:
            for trackedPerson in trackedPersons.tracks:
                trackId = trackedPerson.track_id
                detectionId = trackedPerson.detection_id

                if not trackedPerson.is_occluded:
                    self.trackLookup[detectionId] = trackId
                    self.oldDetections.append(detectionId)

                while(len(self.oldDetections) > self.maxFusedDetectionsToRemember):
                    del self.trackLookup[ self.oldDetections.popleft() ]


"""
Remembers which original detection IDs were merged into which fused detection IDs by monitoring /spencer/perception_internal/fused_person_detections.
Used by TrackAssociation. This class is thread-safe.
"""
class FusedDetectionIdMemory(object):
    def __init__(self, maxOriginalDetectionsToRemember):
        self.lock = threading.Lock()

        self.fusedDetectionLookup = dict()
        self.oldDetections = collections.deque() # used to efficiently remove old detections from the dictionary if max length is exceeded

        self.fusedDetectionsTopic = "/spencer/perception/detected_persons_composite"
        self.fusedDetectionSubscriber = rospy.Subscriber(self.fusedDetectionsTopic, CompositeDetectedPersons, self.newFusedDetectionsAvailable)
        self.maxOriginalDetectionsToRemember = maxOriginalDetectionsToRemember

        self.noDataWarningShown = False

    """
    For a given original detection ID (directly from a person detector, e.g. in RGB-D), lookup the associated fused detection ID after fusion of detections.
    The fused detection ID is the one referenced in spencer_tracking_msgs/TrackedPerson messages.
    """
    def lookupFusedDetectionId(self, originalDetectionId):
        with self.lock:
            if not self.fusedDetectionLookup and not self.noDataWarningShown:
                self.noDataWarningShown = True
                rospy.logwarn("Lookup from original detection IDs to fused detection IDs is empty, make sure %s is being published!" % self.fusedDetectionsTopic)

            return self.fusedDetectionLookup.get(originalDetectionId)

    """ Internal callback which processes new CompositeDetectedPersons. """
    def newFusedDetectionsAvailable(self, fusedDetections):
        with self.lock:
            for fusedDetection in fusedDetections.elements:
                fusedId = fusedDetection.composite_detection_id
                for originalDetection in fusedDetection.original_detections:
                    originalDetectionId = originalDetection.detection_id
                    self.fusedDetectionLookup[originalDetectionId] = fusedId
                    self.oldDetections.append(originalDetectionId)

                while(len(self.oldDetections) > self.maxOriginalDetectionsToRemember):
                    del self.fusedDetectionLookup[ self.oldDetections.popleft() ]


"""
Buffers messages of the given inputFilter until person tracks are available with the
same or a newer timestamp. This is useful for associating frame-by-frame detection information
(e.g. person age, gender) with longer-lasting tracks using the embedded TrackAssociation instance. As the TrackAssociation
cannot look into the future, all ROS messages which shall be associated with a person track ID need
to be witheld until person tracks for the particular timestamp have been output by the tracker.

The required ``queue size`` parameter specifies how many messages shall be buffered while waiting
for person tracks to arrive. If no person tracks arrive in time, the oldest messages will be deleted
first. If the messages are large (e.g. image messages), a low queue_size (not larger than 10) should be used.

Example of usage:
def callback(trackAssociation, humanAttributes):
    # Do something useful with the message...
    # Use trackAssociation.lookupTrackId(detectionId) to look up the track associated with a particular detection (may be None)

human_attribute_sub = message_filters.Subscriber('/spencer/perception_internal/human_attributes/rgbd_front_top', HumanAttributes)
track_sync = spencer_detected_person_association.TrackSynchronizer(human_attribute_sub, 100)
track_sync.registerCallback(callback)
"""
class TrackSynchronizer(message_filters.SimpleFilter):
    def __init__(self, inputFilter, queue_size):
        message_filters.SimpleFilter.__init__(self)
        self.trackAssociation = TrackAssociation()
        self.trackAssociation.trackSubscriber.addCallback(self.newTracksAvailable)
        self.latestTrackTimestamp = rospy.Time(0);
        self.connectInput(inputFilter)
        self.queue_size = queue_size
        self.lock = threading.Lock()

    def signalMessage(self, *msg):
        for (cb, args) in self.callbacks.values():
            cb(*( (self.trackAssociation,) + msg + args) )

    def connectInput(self, inputFilter):
        self.queue = collections.deque()
        self.input_connection = inputFilter.registerCallback(self.newMessage)

    def newMessage(self, msg):
        with self.lock:
            # Temporarily store the message in the queue
            self.queue.append(msg)

            # Invoke callback for all queued messages where track information is already available
            while self.queue and self.queue[0].header.stamp <= self.latestTrackTimestamp:
                publishMsg = self.queue.popleft()
                self.signalMessage(publishMsg)

            # Remove old messages for which no track information was received, if queue gets too long
            numMessagesDeleted = 0

            while len(self.queue) > self.queue_size:
                self.queue.popleft()
                numMessagesDeleted +=1

            if numMessagesDeleted > 0:
                rospy.loginfo("TrackSynchronizer: %d buffered message(s) had to be deleted because no recent tracked person information was available!")

    def newTracksAvailable(self, trackedPersons):
        self.latestTrackTimestamp = trackedPersons.header.stamp
