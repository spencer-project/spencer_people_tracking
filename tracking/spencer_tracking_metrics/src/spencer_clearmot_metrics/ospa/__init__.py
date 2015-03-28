import numpy, rospy
from spencer_tracking_msgs.msg import TrackedPersons
import ospa as ospa_implementation
from std_msgs.msg import Float32 as ResultMsg



class OspaAnalysis(ospa_implementation.OSPADistance):
    def __init__(self, p=2.0, c=10.0, a=1.0, publishResult=True):
        ospa_implementation.OSPADistance.__init__(self,p=p,c=c, a=a)
        self.ospaResultList = []
        self.publishResultFlag = publishResult
        if self.publishResultFlag:
            self.Publisher = rospy.Publisher("/ospa_distance", ResultMsg, queue_size=1)



    def doOspaAnalysis(self,trackedPersons, groundTruth):
        ospa_tracks = []
        groundtruth_labels = []
        track_labels = []
        for track in trackedPersons.tracks:
            ospa_tracks.append((track.pose.pose.position.x , track.pose.pose.position.y))
            track_labels.append(track.track_id)

        ospa_gt = []
        for track in groundTruth.tracks:
            ospa_gt.append((track.pose.pose.position.x , track.pose.pose.position.y))
            groundtruth_labels.append(track.track_id)

        results = self.evaluateFrame(numpy.asarray(ospa_tracks), numpy.asarray(ospa_gt), track_labels, groundtruth_labels)
        # rospy.loginfo("OSPA metric results:  OSPA distance %f, Location Error %f, Cardinality Error %f", results[0], results[1], results[2])
        timed_results = results + (trackedPersons.header.stamp.to_sec(),) 
        self.publishResult(timed_results)
        self.ospaResultList.append(timed_results)
        return timed_results


    def publishResult(self, result):
        if self.publishResultFlag:
            self.Publisher.publish(data=result[0])


    def writeResultsToFile(self, filename):
        with open(filename, 'w') as f:
            # OSPA results
            f.write("OSPA distance ; Localization Error ; Cardinalization Error ; Labeling Error ;  Timestamp [s] \n")
            for result in self.ospaResultList:
                f.write(' ; '.join(map(str, result)) + "\n")

        rospy.loginfo("Results have been written to file " + filename)