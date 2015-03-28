#include "FrameInlier.h"
#include <iostream>

using namespace std;

FrameInlier::FrameInlier()
{
}

FrameInlier::~FrameInlier()
{
     inlierC.clearContent();;
     weightsC.clearContent();;
}

FrameInlier::FrameInlier(int frameO)
{
    frameC = frameO;
}

void FrameInlier::addInlier(int inlierO)
{
    inlierC.pushBack(inlierO);
}

void FrameInlier::addWeight(double wO)
{
    weightsC.pushBack(wO);
}

int FrameInlier::getFrame() const
{
    return frameC;
}

Vector<int> FrameInlier::getInlier()
{
    return inlierC;
}

Vector<double> FrameInlier::getWeight()
{
    return weightsC;
}

void FrameInlier::clearWeights()
{
    weightsC.clearContent();
}

int FrameInlier::getNumberInlier()
{
    return inlierC.getSize();
}

bool FrameInlier::operator< ( const FrameInlier& fO) const
{
    return (this->getFrame() < fO.getFrame());
}

void FrameInlier::showFrameInlier()
{
    cout << "--------------------------------------------------------" << endl;
    cout << "Frame: " << frameC << endl;
    for(int i = 0; i < inlierC.getSize(); i++)
    {
        cout << "Inlier: " << inlierC(i) << endl;
        cout << "Weights: " << weightsC(i) << endl;
    }
    cout << "--------------------------------------------------------" << endl;

}

 void FrameInlier::setAllWeightsCoincident(Vector<double>& vecO)
 {
     weightsC = vecO;
 }
