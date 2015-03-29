# -*- coding: utf-8 -*-
# code based on github implemenation https://github.com/cheesinglee/cuda-PHDSLAM/blob/master/python/ospa.py

from munkres import Munkres
import rospy, numpy, time


class OSPADistance():
    def __init__(self, p=2.0, c=10.0, a=1.0):
        # Weighting for labeling error
        self.a = a
        # Order of metric
        self.p = p
        # Cut off distannce
        self.c = c
        # old asssignments
        self.old_assignments = []


    def calculateCostMatrix(self,X,Y):   
        
        # compute the cost matrix using Euclidean distance
        # rospy.logdebug("Size of X %i and Size of Y %i",numpy.size(X,0),numpy.size(Y,0) )
        cost_matrix = numpy.empty([numpy.size(X,0) ,numpy.size(Y,0)])
        idx_x = 0
        idx_y = 0
        for i in X:
            for j in Y:
                distance = numpy.linalg.norm(i - j)
                if distance > self.c:
                    distance = self.c
                cost_matrix[idx_x][idx_y] = distance
                idx_y += 1
            idx_x +=1
            idx_y = 0

        return cost_matrix

           

    def evaluateFrame(self,X,Y, X_labels=None, Y_labels=None):
        """ Compute the OSPA metric between two sets of points. """
            
        # check for empty sets
        if numpy.size(X) == 0 and numpy.size(Y) == 0:
            return (0,0,0,0)
        elif numpy.size(X) == 0 or numpy.size(Y) == 0 :
            return (self.c,0,self.c, 0)
            
        # we assume that Y is the larger set
        m = numpy.size(X,0) 
        n = numpy.size(Y,0)
        switched = False
        if m > n:
            X,Y = Y,X
            m,n = n,m 
            switched = True       

        dists = self.calculateCostMatrix(X,Y)
        # Copy cost matrix for munkres module
        munkres_matrix = numpy.copy(dists)
        # Only run munkres on non-empty matrix
        if len(munkres_matrix) > 0:
            munkres = Munkres()
            indices = munkres.compute(munkres_matrix)
        else:
            indices = []
                
        # compute the OSPA metric
        total = 0 
        total_loc = 0
        for [i,j] in indices:
            total_loc += dists[i][j]**self.p
        # calculate cardinalization error
        err_cn = (float((self.c)**(self.p)*(n-m))/n)**(1/float(self.p))
        # calculate localization error
        err_loc = (float(total_loc)/n)**(1/float(self.p)) 

        # Not contained in version from github implemented 
        # acc. to paper "A Metric for Performance Evaluation of Multi-Target Tracking Algorithms"
        # Implementation of labeling error
        # Penalizes ID switches from frame to frame
        new_assignments = []
        for [i,j] in indices:
            if (switched):
                i,j = j,i
            new_assignments.append((X_labels[i], Y_labels[j]))
        wrong_labels =len(self.old_assignments) - len(set(new_assignments) & set(self.old_assignments))
        rospy.loginfo(wrong_labels)
        err_label = (float((float(self.a)**float(self.p))/n)*wrong_labels) **(1/float(self.p)) 
        # store current assignments of labels to track
        self.old_assignments = new_assignments
        #ospa_err = ( float(total_loc + (n-m)*self.c**self.p) / n)**(1/float(self.p))
        ospa_err = ( float(total_loc + self.a*wrong_labels + (n-m)*self.c**self.p) / n)**(1/float(self.p))
        ospa_tuple = (ospa_err,err_loc,err_cn, err_label) 

        rospy.loginfo(ospa_tuple)

        return ospa_tuple
    
if __name__ == '__main__':
    # test routine
    ospa = OSPADistance()
    X = numpy.arange(6,dtype='float')
    Y = numpy.array([0,-3,6],dtype='float')
    print X
    d = ospa.evaluateFrame(X,Y)
    print(d)
    
    
    
            
        
        