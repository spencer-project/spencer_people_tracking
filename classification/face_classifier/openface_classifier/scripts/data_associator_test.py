#!/usr/bin/env python2

import sys
import rospy
import yaml
from openface_classifier.srv import (
    TrackLabelAssociationService,
    TrackLabelAssociationServiceRequest,
    TrackLabelAssociationServiceResponse,
)

def label_track_client(request):
    service_name = "/spencer/classification_internal/classified_tracks/track_label"
    rospy.wait_for_service(service_name)
    try:
        label_track = rospy.ServiceProxy(name=service_name, service_class=TrackLabelAssociationService)
        response = label_track(request)
        return response
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    #  OR use rosservice CLI
    # $ rosservice call /spencer/classification_internal/classified_tracks/track_label '{track_id: 1, min_embeddings: 5, track_label: 'Jaskaran', reset_label: False}'
    return "%s \"{track_id: 1, min_embeddings: 5, track_label: 'Jaskaran', reset_label: False}\""%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 2:
        arg_dict = yaml.load(sys.argv[1])
        request = TrackLabelAssociationServiceRequest()
        request.track_id = arg_dict['track_id']
        request.track_label = arg_dict['track_label']
        request.min_embeddings = arg_dict['min_embeddings']
        request.reset_label = arg_dict['reset_label']
    else:
        print usage()
        sys.exit(1)
    print "Requesting: "
    print request
    print "response: %s"%(label_track_client(request))
