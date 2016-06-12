# -*- coding: utf-8 -*-
# code based on github implemenation https://github.com/cheesinglee/cuda-PHDSLAM/blob/master/python/ospa.py

from munkres import Munkres
import rospy, numpy, time


class OSPADistance():
    def __init__(self, p=2.0, c=10.0, a=5.0):
        # Weighting for labeling error
        self.a = a
        # Order of metric
        self.p = p
        # Cut off distannce
        self.c = c
        #mapping
        self.gt_mappings = dict()
        #use lapjv instead of munkres? (requires library to be installed from http://www.starklab.org/members/kazmar/software/)
        self.use_lapjv = rospy.get_param("~use_lapjv", True)

        if self.use_lapjv:
            try:
                import lapjv
            except ImportError:
                rospy.logerr("Setting use_lapjv in OSPA to False because required library could not be found. Install from http://www.starklab.org/members/kazmar/software/!")
                self.use_lapjv = False
            else:
                self.lapjv = lapjv

    def calculateCostMatrix(self,X,Y):   
        
        # compute the cost matrix using Euclidean distance
        # rospy.logdebug("Size of X %i and Size of Y %i",numpy.size(X,0),numpy.size(Y,0) )
        cost_matrix = numpy.empty([max(len(X),len(Y)),max(len(X),len(Y))])
        cost_matrix.fill(self.c)
        idx_x = 0
        idx_y = 0
        found_possible_match = False
        for i in X:
            for j in Y:
                distance = numpy.linalg.norm(numpy.asarray(i['position']) - numpy.asarray(j['position']))
                if distance <= self.c:
                    found_possible_match = True
                    cost_matrix[idx_x][idx_y] = distance
                idx_y += 1
            idx_x +=1
            idx_y = 0
        
        return cost_matrix
            

           

    def evaluateFrame(self,X_Tracks,Y_GT):
        """ Compute the OSPA metric between two sets of points. """
            
        # check for empty sets
        if len(X_Tracks) == 0 and len(Y_GT) == 0:
            return (0,0,0,0,0,0)
        elif len(X_Tracks) == 0 or len(Y_GT) == 0 :
            return (self.c,0,self.c, 0, 0, 0)
            
        # we assume that Y_GT is the larger set
        m = len(X_Tracks) 
        n = len(Y_GT)
        switched = False
        if m > n:
            m,n = n,m 
            switched = True 
            
        existing_X_ids = []
        existing_Y_ids = []
        total_loc = 0
        
        for gt_id in self.gt_mappings.keys():
            groundtruth = filter(lambda g: g['id'] == gt_id, Y_GT) # Get ground truths with given ground truth id in current frame
            if len(groundtruth) > 1:
                rospy.loginfo("found %d > 1 ground truth tracks for id %s", len(groundtruth), gt_id)
            elif len(groundtruth) < 1:
                continue
            
            hypothesis = filter(lambda h: h['id'] == self.gt_mappings[gt_id], X_Tracks) # Get hypothesis with hypothesis id according to mapping
            assert len(hypothesis) <= 1
            if len(hypothesis) != 1:
                continue
            
            # Hypothesis found for known mapping
            # Check hypothesis for overlap
            distance = numpy.linalg.norm(numpy.asarray(groundtruth[0]['position']) - numpy.asarray(hypothesis[0]['position']))
            if distance <= self.c:
                rospy.logdebug("Keeping correspondence between %s and %s" % (gt_id,self.gt_mappings[gt_id] ))
#               print "DIFF Keep corr %s %s %.2f" % (groundtruth[0]["id"], hypothesis[0]["id"], Rect(groundtruth[0]).overlap(Rect(hypothesis[0])))
                #listofprints.append("DIFF Keep corr %s %s %.2f" % (groundtruth[0]["id"], hypothesis[0]["id"], Rect(groundtruth[0]).overlap(Rect(hypothesis[0]))))
                total_loc += distance **self.p
                existing_X_ids.append(hypothesis[0]['id'])
                existing_Y_ids.append(groundtruth[0]['id'])
                #Removing found correspondences from SETS
        if len(existing_X_ids) > 0:
            X_Tracks = filter(lambda h: h['id'] not in existing_X_ids, X_Tracks) 
        
        if len(existing_Y_ids) > 0:
            Y_GT = filter(lambda g: g['id'] not in existing_Y_ids, Y_GT) 



        # check for empty sets
        wrong_labels = 0
        fragments = 0
        if len(X_Tracks) > 0 or len(Y_GT) > 0:
                               
            #Find not yet existing correspondences
            dists = self.calculateCostMatrix(X_Tracks,Y_GT)
            # Copy cost matrix for munkres module
            munkres_matrix = numpy.copy(dists)
            # Only run munkres on non-empty matrix
            if munkres_matrix.min() == self.c:
                indices = zip(range(max(len(X_Tracks),len(Y_GT))), range(max(len(X_Tracks),len(Y_GT))))
            elif len(munkres_matrix) > 1:
                if self.use_lapjv:
                    lap_result = self.lapjv.lap(numpy.array(munkres_matrix, dtype=numpy.float32 ))
                    indices = zip(range(max(len(X_Tracks),len(Y_GT))), lap_result[1])
                else:
                    munkres = Munkres()
                    #rospy.logdebug(munkres_matrix)
                    indices = munkres.compute(munkres_matrix)
            elif len(munkres_matrix) == 1:
                indices = []
                indices.append((0,0))
            else:
                indices = []
                    
            # compute the OSPA metric
            
            new_mappings = dict(self.gt_mappings)
            for [t,g] in indices:
                total_loc += min(dists[t][g],self.c)**self.p
                if dists[t][g] < self.c:
                    if Y_GT[g]['id'] in self.gt_mappings.keys() and self.gt_mappings[Y_GT[g]['id']] != X_Tracks[t]['id']:
                        rospy.logdebug("Detected id switch for ground truth track {} assigned before to {} now assigned to {}".format(Y_GT[g]['id'], self.gt_mappings[Y_GT[g]['id']],X_Tracks[t]['id'] ))
                        fragments += 1
                        if X_Tracks[t]['id'] in self.gt_mappings.values():
                            wrong_labels += 1
                    new_mappings[Y_GT[g]['id']] = X_Tracks[t]['id']
                    rospy.logdebug("Found new mapping for gt_id {} to track_id {}".format(Y_GT[g]['id'], X_Tracks[t]['id']))
            
            self.gt_mappings = dict(new_mappings)
        # calculate cardinalization error
        err_cn = (float((self.c)**(self.p)*(n-m))/n)**(1/float(self.p))
        # calculate localization error
        err_loc = (float(total_loc)/n)**(1/float(self.p)) 

        # Not contained in version from github implemented 
        # acc. to paper "A Metric for Performance Evaluation of Multi-Target Tracking Algorithms"
        # Implementation of labeling error
        # Penalizes ID switches from frame to frame
        #rospy.loginfo(wrong_labels)
        err_label = float(((self.a**self.p))/float(n)*float(fragments))**(1/float(self.p)) 
        # store current assignments of labels to track
        #ospa_err = ( float(total_loc + (n-m)*self.c**self.p) / n)**(1/float(self.p))
        ospa_err = ( float(total_loc + self.a*wrong_labels + (n-m)*self.c**self.p) / n)**(1/float(self.p))
        ospa_tuple = (float(ospa_err),float(err_loc),float(err_cn), float(err_label),float(wrong_labels),float(fragments)) 

#         rospy.loginfo(ospa_tuple)

        return ospa_tuple
    
if __name__ == '__main__':
    # test routine
    ospa = OSPADistance()
    X = numpy.arange(6,dtype='float')
    Y = numpy.array([0,-3,6],dtype='float')
    d = ospa.evaluateFrame(X,Y)
    
    
    
            
        
        