#!/usr/bin/env python2

import time

start = time.time()

import argparse
import cv2
import os
import pickle

from operator import itemgetter

import numpy as np
np.set_printoptions(precision=2)
import pandas as pd

import openface
import dlib

from sklearn.preprocessing import LabelEncoder
from sklearn.svm import SVC
from sklearn.grid_search import GridSearchCV
from sklearn.mixture import GMM
from sklearn.tree import DecisionTreeClassifier
from sklearn.naive_bayes import GaussianNB

modelDir = os.path.join('/home/jaskaran/openface', 'models')
dlibModelDir = os.path.join(modelDir, 'dlib')
openfaceModelDir = os.path.join(modelDir, 'openface')

class OpenFaceArgParser(argparse.ArgumentParser):
    """Argument parser class"""

    def set(self):
        """Setup parser"""

        self.add_argument(
            '--dlibFacePredictor',
            type=str,
            help="Path to dlib's face predictor.",
            default=os.environ.get(
                'DLIB_FACE_PREDICTOR_PATH',
                 os.path.join(
                    dlibModelDir,
                    "shape_predictor_68_face_landmarks.dat")))
        self.add_argument(
            '--networkModel',
            type=str,
            help="Path to Torch network model.",
            default=os.environ.get(
                'NETWORK_MODEL_PATH',
                 os.path.join(
                    openfaceModelDir,
                    'nn4.small2.v1.t7')))
        self.add_argument(
            '--classifierModel',
            type=str,
            help="Path to Classifier Model.",
            default=os.environ.get(
                'CLASSIFIER_MODEL_PATH',
                 os.path.join(
                    '/home/jaskaran/train_images',
                    'classifier.pkl')))
        self.add_argument(
            '--imgDim',
            type=int,
            help="Default image dimension.",
            default=os.environ.get(
                'IMG_DIM',
                 96))
        self.add_argument(
            '--cuda',
            action='store_true')
        self.add_argument(
            '--verbose',
        action='store_true')
        self.add_argument(
            '--features',
            type=str,
            help="Path to Features numpy.",
            default='/home/jaskaran/train_images/features.npy')

class OpenFaceAnotater(object):

    def __init__(self, argv):
        start = time.time()
        self.arg_parser = OpenFaceArgParser(
            prog='openface',
            description='OpenFace')
        self.arg_parser.set()
        self.args, argv = self.arg_parser.parse_known_args(argv)
        if self.args.verbose:
            print("Argument parsing and import libraries took {} seconds.".format(
                time.time() - start))

        start = time.time()
        self.align = openface.AlignDlib(self.args.dlibFacePredictor)
        self.net = openface.TorchNeuralNet(
            self.args.networkModel,
            imgDim=self.args.imgDim,
            cuda=self.args.cuda)

        try:
            self.load_model(self.args.classifierModel)
        except:
            print("No classifierModel loaded")
        try:
            self.load_features(self.args.features)
        except:
            print("No features loaded. Necessary for Nearest Neighbor")

        if self.args.verbose:
            print("Loading the dlib and OpenFace models took {} seconds.".format(
                time.time() - start))
            start = time.time()

    def load_model(self, classifierModel):
        with open(classifierModel, 'r') as f:
            (le, clf) = pickle.load(f)
        self.le = le
        self.clf = clf

    def load_features(self, feature_path):
        try:
            self.features = np.load(feature_path)
        except:
            print('No feature file specified')

    def predictWithLabel(self, rgbImg, bbs, multiple=False, scale=None):
        annotatedImg = np.copy(rgbImg)
        dot_prod_threshold_unknown = 0.6

        # try:
        reps = self.getRep(rgbImg, bbs, multiple, scale) #bbs is passed by reference
        # print("reps: ", reps)
        # if len(reps) > 1:
            # print("List of faces in image from left to right")
        identities = []
        for r in reps:
            rep = r[1].reshape(1, -1)
            bbx = r[0]
            bb = r[2] #bounding box
            landmarks = r[3] #landmarks
            start = time.time()
            dist,ind = self.clf.kneighbors(rep)
            
            # Logic 1:
            # #threshold logic on nearest neigbor distance
            # if dist[0][0]>threshold_unknown:
            #     # print ('Unknown')
            #     person = 'Unknown'
            # else:
            #     person = self.le.inverse_transform(self.clf.predict(rep))[0]
            #     # print (person)

            #Logic
            nn_product = np.vdot(rep,self.features[ind[0]])
            print 'NN dot product',nn_product
            print 'Nearest neigbor is',self.le.inverse_transform(self.clf.predict(rep))[0]
            if nn_product>dot_prod_threshold_unknown:
                person = self.le.inverse_transform(self.clf.predict(rep))[0]
            else:
                person = 'Unknown'
            identities.append(person)

            if self.args.verbose:
                print("Prediction took {} seconds.".format(time.time() - start))
                if multiple:
                    print("Predict {} @ x={}".format(
                        person,
                        bbx))
                else:
                    print("Predict {}".format(
                        person))
            if isinstance(self.clf, GMM):
                dist = np.linalg.norm(rep - self.clf.means_[maxI])
                if self.args.verbose:
                    print("  + Distance from the mean: {}".format(dist))

            #code for annotated bounding box
            bl = (bb.left(), bb.bottom())
            tr = (bb.right(), bb.top())
            cv2.rectangle(annotatedImg, bl, tr, color=(153, 255, 204),thickness=3)
            for p in openface.AlignDlib.OUTER_EYES_AND_NOSE:
                cv2.circle(annotatedImg, center=landmarks[p], radius=3,
                               color=(102, 204, 255), thickness=-1)
            cv2.putText(annotatedImg, person, (bb.left(), bb.top() - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.75,
                            color=(152, 255, 204), thickness=2)
        # annotatedImgBgr = cv2.cvtColor(annotatedImg, cv2.COLOR_RGB2BGR)
        return annotatedImg,identities


    def predict(self, rgbImg, bbs, multiple=False, scale=None):
        # bgrImg = img
        # rgbImg = cv2.cvtColor(bgrImg, cv2.COLOR_BGR2RGB)
        annotatedImg = np.copy(rgbImg)
        #http://cmusatyalab.github.io/openface/demo-2-comparison/
        #threshold_unknown = 0.8
        dot_prod_threshold_unknown = 0.6

        # try:
        reps = self.getRep(rgbImg, bbs, multiple, scale) #bbs is passed by reference
        # print("reps: ", reps)
        # if len(reps) > 1:
            # print("List of faces in image from left to right")
        for r in reps:
            rep = r[1].reshape(1, -1)
            bbx = r[0]
            bb = r[2] #bounding box
            landmarks = r[3] #landmarks
            start = time.time()
            dist,ind = self.clf.kneighbors(rep)
            
            # Logic 1:
            # #threshold logic on nearest neigbor distance
            # if dist[0][0]>threshold_unknown:
            #     # print ('Unknown')
            #     person = 'Unknown'
            # else:
            #     person = self.le.inverse_transform(self.clf.predict(rep))[0]
            #     # print (person)

            #Logic
            nn_product = np.vdot(rep,self.features[ind[0]])
            print 'NN dot product',nn_product
            print 'Nearest neigbor is',self.le.inverse_transform(self.clf.predict(rep))[0]
            if nn_product>dot_prod_threshold_unknown:
                person = self.le.inverse_transform(self.clf.predict(rep))[0]
            else:
                person = 'Unknown'

            if self.args.verbose:
                print("Prediction took {} seconds.".format(time.time() - start))
                if multiple:
                    print("Predict {} @ x={}".format(
                        person,
                        bbx))
                else:
                    print("Predict {}".format(
                        person))
            if isinstance(self.clf, GMM):
                dist = np.linalg.norm(rep - self.clf.means_[maxI])
                if self.args.verbose:
                    print("  + Distance from the mean: {}".format(dist))

            #code for annotated bounding box
            bl = (bb.left(), bb.bottom())
            tr = (bb.right(), bb.top())
            cv2.rectangle(annotatedImg, bl, tr, color=(153, 255, 204),thickness=3)
            for p in openface.AlignDlib.OUTER_EYES_AND_NOSE:
                cv2.circle(annotatedImg, center=landmarks[p], radius=3,
                               color=(102, 204, 255), thickness=-1)
            cv2.putText(annotatedImg, person, (bb.left(), bb.top() - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.75,
                            color=(152, 255, 204), thickness=2)
        # annotatedImgBgr = cv2.cvtColor(annotatedImg, cv2.COLOR_RGB2BGR)
        return annotatedImg
        # except Exception as e:
            # print str(e)
            # return rgbImg, None

    def getRep(self, rgbImg, bbs, multiple=False, scale=None):
        # print ('getRep entered:')
        # print ('rgbimg shape:{}'.format(rgbImg.shape))
        start = time.time()

        reps = []
        #Possible performance optimization of grayscale. Didn't affect at all.
        bwImg = cv2.cvtColor(rgbImg, cv2.COLOR_RGB2GRAY)
        #Performance Optimization. Finding bounding boxes in scaled down image
        if scale is not None:
            bwImg = cv2.resize(bwImg, (0,0), fx=scale, fy=scale)
            scale_inv = 1.0 / scale

        #don't find faces if bounding boxes are provided. Performance Optimization.
        if len(bbs)==0:    
            if multiple:
                bbs.extend(self.align.getAllFaceBoundingBoxes(bwImg))
            else:
                bbs.append(self.align.getLargestFaceBoundingBox(bwImg))
            
            if len(bbs) == 0:
                if self.args.verbose:
                    print("Unable to find a face")
                # raise Exception("Unable to find a face")
            
            if self.args.verbose:
                print("Face detection took {} seconds.".format(time.time() - start))
            
            temp = []
            for bb in bbs:
                if bb is not None:
                    start = time.time()
                    if scale is not None:
                        bb = dlib.rectangle(
                            left=long(bb.left()*scale_inv),
                            top=long(bb.top()*scale_inv),
                            right=long(bb.right()*scale_inv),
                            bottom=long(bb.bottom()*scale_inv))
                    alignedFace = self.align.align(
                        self.args.imgDim,
                        rgbImg,
                        bb,
                        landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)

                    # print("alignedFace: ", alignedFace)
                    if alignedFace is None:
                        #raise Exception("Unable to align image")
                        continue
                    if self.args.verbose:
                        print("Alignment took {} seconds.".format(time.time() - start))
                        print("This bbox is centered at {}, {}".format(bb.center().x, bb.center().y))
                    landmarks = self.align.findLandmarks(rgbImg,bb)
                    start = time.time()
                    rep = self.net.forward(alignedFace)
                    if self.args.verbose:
                        print("Neural network forward pass took {} seconds.".format(
                            time.time() - start))
                    reps.append((bb.center().x, rep, bb, landmarks)) #added the bounding box and landmarks
                    temp.append(bb)
            
            #empty bbs as it currently contains bounding boxes on downscaled image and then add all boxes from temp
            del bbs[:]
            bbs.extend(temp)
        else:
            for bb in bbs:
                start = time.time()
                alignedFace = self.align.align(
                    self.args.imgDim,
                    rgbImg,
                    bb,
                    landmarkIndices=openface.AlignDlib.OUTER_EYES_AND_NOSE)

                # print("alignedFace: ", alignedFace)
                if alignedFace is None:
                    raise Exception("Unable to align image")
                if self.args.verbose:
                    print("Alignment took {} seconds.".format(time.time() - start))
                    print("This bbox is centered at {}, {}".format(bb.center().x, bb.center().y))
                landmarks = self.align.findLandmarks(rgbImg,bb)
                start = time.time()
                rep = self.net.forward(alignedFace)
                if self.args.verbose:
                    print("Neural network forward pass took {} seconds.".format(
                        time.time() - start))
                reps.append((bb.center().x, rep, bb, landmarks)) #added the bounding box and landmarks
                
        sreps = sorted(reps, key=lambda x: x[0])
        return sreps