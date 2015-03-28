/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

#ifndef _DENNIS_IDX_H
#define	_DENNIS_IDX_H

#include <stdlib.h>
#include <math.h>
#include "Vector.h"


using namespace std;

class FrameInlier
{

public:

    FrameInlier();
    FrameInlier(int frameO);
    ~FrameInlier();

    void addInlier(int inlierO);
    void addWeight(double weightO);
    void clearWeights();
    void setAllWeightsCoincident(Vector<double>& vecO);
    int getFrame() const;
    Vector<int> getInlier();
    Vector<double> getWeight();

    void showFrameInlier();

    int getNumberInlier();
    bool operator < (const FrameInlier& fO) const;


protected:

    int frameC;
    Vector<int> inlierC;
    Vector<double> weightsC;
    
};



#endif	/* _DENNIS_IDX_H */

