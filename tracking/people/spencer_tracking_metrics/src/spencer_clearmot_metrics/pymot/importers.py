#!/usr/bin/env python

from collections import deque

def MOT_hypo_import(lines):
    
    frames = []
    
    for line in lines:
#        print line
        s = deque(line.strip().split()) # any whitespace

        # Skip empty lines
        if len(s) < 1:
            continue
        
        ts = float(s.popleft())
        
        # each hypothesis takes exactly 5 values
        assert((len(s)) % 5 == 0)
        nHypotheses = len(s) / 5
        
        hypotheses = []
        
        for i in range(nHypotheses):
            hypo = {
                'id':     s[5*i + 0],
                'x':      float(s[5*i + 1]),
                'y':      float(s[5*i + 2]),
                'width':  float(s[5*i + 3]) - float(s[5*i + 1]), # x - tl_x
                'height': float(s[5*i + 4]) - float(s[5*i + 2]), # y - tl_y
            }
            hypotheses.append(hypo)
    
        frameitem = {
            # no num
            'class':      'frame',
            'timestamp':  ts,
            'hypotheses': hypotheses,
        }
        frames.append(frameitem)
    
    fileitem = {
        'class': 'video',
        'frames': frames,
    }
    
    return fileitem


def MOT_groundtruth_import(lines): # TODO rename, since we do not create json
    
    frames = []
    
    for line in lines:
        s = deque(line.strip().split()) # any whitespace
        
        # Skip empty lines
        if len(s) < 1:
            continue

        ts = float(s.popleft())
        
        # each annotation takes exactly 13 values
        assert((len(s)) % 13 == 0)
        nAnnotations = len(s) / 13
        
        annotations = []
        for i in range(nAnnotations):
            id = s[13*i + 0]
            cx = float(s[13*i + 3])
            cy = float(s[13*i + 4])
            w  = float(s[13*i + 5])
            h  = float(s[13*i + 6])
            
            # Ignore annotations with negative center coordinates
            if cx < 0 and cy < 0:
                continue
            
            # Ground truth is considered a DCO object, if more than two features found
            # Silly, but thats how the perl script seems to do it
            nFeaturesGreaterThanZero = 0
            for j in range(7, 13):
                if float(s[13*i + j]) >= 0.0:
                    nFeaturesGreaterThanZero += 1
            
            dco = True
            if nFeaturesGreaterThanZero >= 2:
                dco = False
            
            annotation = {
                'id':     id,
                'x':      cx - w/2,
                'y':      cy - h/2,
                'width':  w,
                'height': h,
                'dco':    dco,                  
            }
            annotations.append(annotation)
        
        frameitem = {
            # no num
            'class': 'frame',
            'timestamp': ts,
            'annotations': annotations
        }
        frames.append(frameitem)
        
    fileitem = {
        'class': 'video',
        'frames': frames,
    }
    
    return fileitem
