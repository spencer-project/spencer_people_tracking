import numpy, rospy
from spencer_tracking_msgs.msg import TrackedPersons
import ospa as ospa_implementation
from std_msgs.msg import Float32 as ResultMsg



class OspaAnalysis(ospa_implementation.OSPADistance):
    def __init__(self, p=1.0, c=10.0, a=1.0, publishResult=True):
        ospa_implementation.OSPADistance.__init__(self,p=p,c=c, a=a)
        self.ospaResultList = []
        self.publishResultFlag = publishResult
        if self.publishResultFlag:
            self.Publisher = rospy.Publisher("/ospa_distance", ResultMsg, queue_size=1)



    def doOspaAnalysis(self,trackedPersons, groundTruth):
        ospa_tracks = []
        for track in trackedPersons.tracks:
            if not track.is_occluded:
                ospa_track = {'position':(track.pose.pose.position.x , track.pose.pose.position.y),
                              'id': track.track_id}
                ospa_tracks.append(ospa_track)

        ospa_gt = []
        for track in groundTruth.tracks:
            if not track.is_occluded:
                ospa_groundtruth = {'position':(track.pose.pose.position.x , track.pose.pose.position.y),
                              'id': track.track_id}
                ospa_gt.append(ospa_groundtruth)

        results = self.evaluateFrame(ospa_tracks, ospa_gt)
        # rospy.loginfo("OSPA metric results:  OSPA distance %f, Location Error %f, Cardinality Error %f", results[0], results[1], results[2])
#         rospy.loginfo(results)
        timed_results = results + (float(trackedPersons.header.stamp.to_sec()),) 
        self.publishResult(timed_results)
        self.ospaResultList.append(timed_results)
        return timed_results


    def publishResult(self, result):
        if self.publishResultFlag:
            self.Publisher.publish(data=result[0])
            
    def printAverageResults(self):
        array = numpy.asarray(self.ospaResultList)
        averageResults = numpy.mean(array, axis=0)
        rospy.loginfo("############## Average OSPA Results #####################\nOSPA distance={}; \nLocalization Error={}; \nCardinalization Error={}; \nLabeling Error={}, \nIDswitches ={}, \nFragments={}".format(averageResults[0],averageResults[1],averageResults[2],averageResults[3], averageResults[4], averageResults[5]))

    def printTotalResults(self):
        array = numpy.asarray(self.ospaResultList)
        averageResults = numpy.sum(array, axis=0)
        rospy.loginfo("############## Total OSPA Results #####################\nOSPA distance={}; \nLocalization Error={}; \nCardinalization Error={}; \nLabeling Error={}, \nIDswitches ={}, \nFragments={}".format(averageResults[0],averageResults[1],averageResults[2],averageResults[3], averageResults[4], averageResults[5]))

    def writeResultsToFile(self, filename):
        with open(filename, 'w') as f:
            # OSPA results
            f.write("OSPA distance ; Localization Error ; Cardinalization Error ; Labeling Error ; IDswitches ; Fragments ;  Timestamp [s] \n")
            for result in self.ospaResultList:
                f.write(' ; '.join(map(str, result)) + "\n")

        rospy.loginfo("Results have been written to file " + filename)