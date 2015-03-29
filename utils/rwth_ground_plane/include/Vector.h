
#ifndef _DENNIS_VECTOR_H
#define	_DENNIS_VECTOR_H

#include <vector>
#include <string>
#include <cassert>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <algorithm>

using namespace std;

template <class T> class Vector {
public:

    //***********************************
    // Constructors
    //***********************************

    // standard constructor
    Vector();
    inline Vector(const int sizeO);
    // copy constructor
    inline Vector(const Vector<T>& copy);
    // filling constructor
    inline Vector(const int sizeO, const T& FillValO);
    // appending constructor
    Vector(const Vector<T>& vec1, const Vector<T>& vec2);
    inline Vector(const int xSizeO, T* data);
    // copy from STL vector constructor0
    Vector(const vector<T>& vecO);

    Vector(const T& x, const T& y, const T& z, const T& a);
    Vector(const T &x, const T &y, const T &z);

    //    ~Vector();
    //************************************
    // Operators
    //************************************

    inline T& operator()(const unsigned int aIndex);
    inline T operator()(const unsigned int aIndex) const;
    inline T& operator[](const unsigned int aIndex);
    inline T operator[](const unsigned int aIndex) const;

    inline Vector<T>& operator=(const Vector<T>& vecO);
    bool operator==(const Vector<T>& vec0);
    inline Vector<T>& operator=(const T& fillValO);

    inline Vector<T>& operator*=(const T& scalar);
    // Pointwise mult
    inline Vector<T>& operator*=(const Vector<T>& vecO);
    inline Vector<T>& operator+=(const Vector<T>& vecO);
    inline Vector<T>& operator+=(const T& value);

    inline Vector<T>& operator-=(const Vector<T>& vecO);
    inline Vector<T>& operator-=(const vector<T>& vecO);

    //**************************************
    // Determine the size of a vector
    //**************************************
    inline void setSize(int sizeO);
    inline void setSize(int sizeO, const T& fillValO);

    //**************************************
    // Cross Product
    //**************************************
    void cross(Vector<T>& src);

    //**************************************
    // Append an element to the vector
    //**************************************
    inline void pushBack(const T &value);

    //**************************************
    // show a vector on the console
    //**************************************
    void show() const;

    //**************************************
    // fill
    //**************************************
    void fill(const T& valO);

    //**************************************
    // reserveCapacity
    //**************************************
    void reserveCapacity(const int valO);

    //**************************************
    // change the order of vector values: v(0)->v(end), v(end)->v(0)
    //**************************************
    void swap();

    //**************************************
    // Read in data from a .txt File
    //**************************************
    void readTXT(const char *FileName, int sizeO);
    
    //**************************************
    // Smooth linear
    //**************************************
    void smooth(int smoothRange);

    //**************************************
    // Computer the intersection of two vectors
    //**************************************
    void intersection(Vector<T>& vec2, Vector<T>& intersect);

    //***************************************
    // Clear
    //***************************************

    void clearContent();

    //***************************************
    // Compare
    //***************************************
    
    bool compare(Vector<T>& vecO);

    //***************************************
    // Get Size
    //***************************************
    inline int getSize() const;

    //***************************************
    // Get min, max
    //***************************************
    pair<T, int>  minim();
    pair<T, int>  maxim();

    //***************************************
    // Compute the norm of a vector
    //***************************************
    T norm();

    //***************************************
    // Sum up all the elements and return the value
    //***************************************
    inline T sum();

    //***************************************
    // sort
    //***************************************

    void sortV();

    void permutate();

    //***************************************
    // Transfer my vec to stl
    //***************************************

    vector<T> getStl();

    //**********************************************
    // Append a vector to the end of another vector
    //**********************************************
    void append(Vector<T>& toInsert);

    //**********************************************
    // find if an entry exists and return pos else -1
    //**********************************************

    int findV(const T& value);

    //**********************************************
    // Resize
    //**********************************************

    void resize(int newSize);
    void resize_from_end(int newSize);

    //*******************************************************//
    // Get the pointer to the data                           //
    //*******************************************************//
    T* data();

    //*********************************************
    // Write the data to .txt
    //*********************************************
    void writeToTXT(char* filename);


    //*********************************************
    // Find local mamixa
    //*********************************************

    Vector<int> localMaxima();

    //*********************************************
    // Unique
    //*********************************************

    void make_unique();

    inline void erase_element(int index);


protected:
    vector<T> dataC;

};
// Smoothe linear
template <class T> Vector<T> smooth(int smoothRange, Vector<T>& vecToSmooth);
//Swap the elements of a vector
template <class T> Vector<T> swap(Vector<T>& vecToSwap);
// Return sorted vector;
template <class T> Vector<T> sortV(Vector<T>& vecToSort);
template <class T> T getMedian(const Vector<T>& vecToSort);
// Compute Cross product between two vectors
template <class T> Vector<T> cross(const Vector<T>& vec1, const Vector<T>& vec2);
// Adds two vectors
template <class T> Vector<T> operator+(const Vector<T>& vec1, const Vector<T>& vec2);
// Substracts two vectors
template <class T> Vector<T> operator-(const Vector<T>& vec1, const Vector<T>& vec2);
// Multiplies vector with a scalar
template <class T> Vector<T> operator*(const Vector<T>& aVector, const T& aValue);
template <class T> Vector<T> operator*(const T& aValue, const Vector<T>& aVector);
// Multiplies two vectors pointwise
template <class T> inline Vector<T> operator*(const Vector<T>& aVector, const Vector<T>& bVector);
// Dot product
template <class T> T DotProduct(const Vector<T>& vec1, const Vector<T>& vec2);

template <class T> void show_vec_vec(Vector<Vector<T> >& vec);

template <class T> Vector<T> ringOffset(const Vector<T>& vec, int offset);

template <class T> Vector<T> hist(Vector<T>& vec, int nrBins, bool normalize, T st, T en);

template <class T> T hist_bhatta(Vector<T>& h1, Vector<T>& h2);

//*****************************************************************************************************************************************************//
//-----------------------------------------------------------------------------------------------------------------------------------------------------//
//                                                                                                                                                     //
//                                                          IMPLEMENTATION                                                                             //
//                                                                                                                                                     //
//-----------------------------------------------------------------------------------------------------------------------------------------------------//
//*****************************************************************************************************************************************************//

//****************************************************************************
// Constructors
//****************************************************************************


// standard constructor
template <class T> Vector<T>::Vector()
{
}

template <class T> Vector<T>::Vector(int sizeO)
{
    dataC.resize(sizeO);
}

// copy constructor
template <class T> Vector<T>::Vector(const Vector<T>& copy)
{
    dataC = copy.dataC;
}

template <class T> Vector<int> Vector<T>::localMaxima()
{
    Vector<int> res;

    // Difference between subsequent elements:
    Vector<double> dx(dataC.size()-1, 0.0);
    for(int i = dataC.size()-1; i > 0; i--){
        dx(i-1) = dataC[i]-dataC[i-1];
    }

    // Is an horizontal line?
    for(int i = 0; i < dx.getSize(); i++){
        if(dx(i)!= 0)
            break;
        else if(i == dx.getSize()-1 && dx(i)== 0)
            return res;
    }

    // Flat peaks? Put the middle element:
    Vector<int> a;              // Indexes where x changes
    for(int i = 0; i < dx.getSize(); i++){
        if(dx(i) != 0)
            a.pushBack(i);
    }

//    cout << "a " << endl;
//    a.show();


    // Indexes where a do not changes
    // Difference between subsequent elements:
    Vector<double> da(a.getSize()-1);
    for(int i = a.getSize()-1; i > 0; i--)
        da(i-1) = a[i]-a[i-1];

//    cout << "da" << endl;
//    da.show();

    Vector<int> lm;
    for(int i=0; i < da.getSize(); i++)
    {
        if(da(i)!=1)
            lm.pushBack(i+1);
    }
//    cout << "lm" << endl;
//    lm.show();

    Vector<int> d(lm.getSize());
    for(int i = 0; i < lm.getSize(); i++)
    {
        d(i) = a(lm(i)) - a(lm(i)-1);
    }

//    cout << "d" << endl;
//    d.show();

    for(int i = 0; i < lm.getSize(); i++)
    {
        a(lm(i)) = a(lm(i)) - floor(d(i)/2.0);
    }

    a.pushBack(dataC.size());

//    cout << "a" << endl;
//    a.show();

    //Serie without flat peaks
    Vector<double> xa(a.getSize());
    for(int i = 0; i < a.getSize(); i++)
    {
        xa(i) = dataC[a(i)];
    }

//    cout << "xa" << endl;
//    xa.show();

    // Difference between subsequent elements:
    Vector<double> dxa(xa.getSize()-1);
    for(int i = xa.getSize()-1; i > 0; i--)
        dxa(i-1) = xa[i]-xa[i-1];

//    cout << "dxa" << endl;
//    dxa.show();

    Vector<int> b(dxa.getSize(), 0);
    for(int i = 0; i < dxa.getSize(); i++)
    {
        if(dxa(i) > 0)
            b(i) = 1;
    }

//    cout << "b" << endl;
//    b.show();

    Vector<int> xb(b.getSize()-1);
    for(int i = b.getSize()-1; i > 0; i--)
        xb(i-1) = b[i]-b[i-1];

//    cout << "xb" << endl;
//    xb.show();

    // maxima indexes
    Vector<int> imax;
    for(int i = 0; i < xb.getSize(); i++)
    {
        if(xb(i) == -1)
            imax.pushBack(a(i+1));
    }

//    cout << "imax" << endl;
//    imax.show();

    // minima indexes
    Vector<int> imin;
    for(int i = 0; i < xb.getSize(); i++)
    {
        if(xb(i) == 1)
            imin.pushBack(a(i+1));
    }

    Vector<double> xmax;
    Vector<double> xmin;

    // Maximum or minumim on a flat peak at the ends?
    if(imax.getSize() == 0 && imin.getSize() == 0)
    {
        if(dataC[0] > dataC[dataC.size()-1])
        {
            xmax.pushBack(dataC[0]);
            imax.pushBack(0);
            xmin.pushBack(dataC[dataC.size()-1]);
            imin.pushBack(dataC.size()-1);
        }
        else if(dataC[0] < dataC[dataC.size()-1])
        {
            xmax.pushBack(dataC[dataC.size()-1]);
            imax.pushBack(dataC[dataC.size()-1]);
            xmin.pushBack(dataC[0]);
            imin.pushBack(0);
        }

        return res;
    }

    // Maximum or minumim at the ends?
    if(imax.getSize() == 0)
    {
        imax.pushBack(0);
        imax.pushBack(dataC.size()-1);
    }
    else if(imin.getSize() == 0)
    {
        imin.pushBack(0);
        imin.pushBack(dataC.size()-1);
    }
    else
    {
        if(imax(0) < imin(0))
        {
            imin.resize(imin.getSize()+1);
            imin.swap();
            imin(0) = 0;
        }
        else
        {
            imax.resize(imin.getSize()+1);
            imax.swap();
            imax(0) = 0;
        }

        if(imax(imax.getSize()-1) > imin(imin.getSize()-1))
        {
            imin.pushBack(dataC.size()-1);
        }
        else
        {
            imax.pushBack(dataC.size()-1);
        }
    }

    for(int i = 0; i < imax.getSize(); i++)
    {
        xmax.pushBack(dataC[imax(i)]);
    }

    for(int i = 0; i < imin.getSize(); i++)
    {
        xmin.pushBack(dataC[imin(i)]);
    }

    return imax;
}

// filling constructor
template <class T> Vector<T>::Vector(const int sizeO, const T &FillValO) {

    dataC.resize(sizeO);

    std::fill(dataC.begin(), dataC.end(), FillValO);
}

// appending constructor
template <class T> Vector<T>::Vector(const Vector<T>& vec1, const Vector<T>& vec2) {

    int sizeC = vec1.size() + vec2.size();
    dataC.resize(sizeC);

    for(int i = 0; i < vec1.getSize(); i++)
        dataC[i] = vec1[i];

    for(int i = vec1.getSize(); i < sizeC; i++)
        dataC[i] = vec2[i - vec1.getSize()];
}

// copy from STL vector constructor0
template <class T> Vector<T>::Vector(const vector<T> &vecO) {
    dataC = vecO;
}

template <class T> Vector<T>::Vector(const int xSizeO, T* data)
{
    dataC.resize(xSizeO);
    for(int i = 0; i < xSizeO; i++)
        dataC[i] = data[i];
}

template <class T> Vector<T>::Vector(const T &x, const T& y, const T& z, const T& a) : dataC(4)
{
//    dataC.resize(4);
    dataC[0] = x; dataC[1] = y; dataC[2] = z; dataC[3] = a;
}

template <class T> Vector<T>::Vector(const T& x, const T& y, const T& z) : dataC(3)
{
//    dataC.resize(3);
    dataC[0] = x; dataC[1] = y; dataC[2] = z;
}


//*****************************************************************************
// size

template <class T> int  Vector<T>::getSize() const {
    return dataC.size();
}

//*****************************************************************************
// Operators
//*****************************************************************************


// operator ()
template <class T> inline T& Vector<T>::operator()(unsigned int indexO) {

    assert (indexO < dataC.size());
    return dataC[indexO];
}

// operator ()
template <class T> inline T Vector<T>::operator()(unsigned int indexO) const {

    assert (indexO < dataC.size());
    return dataC[indexO];
}

// operator []
template <class T> inline T& Vector<T>::operator[](unsigned int indexO) {

    assert (indexO < dataC.size());
    return dataC[indexO];
}

// operator []
template <class T> inline T Vector<T>::operator[](unsigned int indexO) const {

    assert (indexO < dataC.size());
    return dataC[indexO];
}

// operator +
template <class T> Vector<T>& Vector<T>::operator+=(const Vector<T>& vecO) {

    assert (dataC.size() == (unsigned int) vecO.getSize());

    for (unsigned int i = 0; i < dataC.size(); i++)
        dataC[i] += vecO[i];
    return *this;
}

// operator +
template <class T> Vector<T>& Vector<T>::operator+=(const T& value) {

    for (unsigned int i = 0; i < dataC.size(); i++)
        dataC[i] += value;
    return *this;
}

// operator -
template <class T> Vector<T>& Vector<T>::operator-=(const Vector<T>& vecO) {
    assert (dataC.size() == (unsigned int) vecO.getSize());

    for (unsigned int i = 0; i < dataC.size(); i++)
        dataC[i] -= vecO[i];
    return *this;
}

// operator -
template <class T> Vector<T>& Vector<T>::operator-=(const vector<T>& vecO)
{
    assert (dataC.size() == vecO.getSize());

    for (unsigned int i = 0; i < dataC.size(); i++)
        dataC[i] -= vecO.at(i);
    return *this;
}

// operator *
template <class T> Vector<T>& Vector<T>::operator*=(const T& scalar) {
    for (unsigned int i = 0; i < dataC.size(); i++)
        dataC[i] *= scalar;
    return *this;
}

// operator *
template <class T> Vector<T>& Vector<T>::operator*=(const Vector<T>& vecO)
{
    assert(vecO.getSize() == dataC.size());
    for(int i = 0; i < dataC.size(); i++)
    {
        dataC[i] *= vecO(i);
    }
    return *this;
}

// operator +
template <class T>
Vector<T> operator+(const Vector<T>& vec1, const Vector<T>& vec2)
{
    assert(vec1.getSize() == vec2.getSize());
    Vector<T> result(vec1.getSize());
    for (int i = 0; i < vec1.getSize(); i++)
        result(i) = vec1[i]+vec2[i];
    return result;
}

// operator -
template <class T>
Vector<T> operator-(const Vector<T>& vec1, const Vector<T>& vec2)
{
    assert(vec1.getSize() == vec2.getSize());
    Vector<T> result(vec1.getSize());
    for (int i = 0; i < vec1.getSize(); i++)
        result(i) = vec1(i)-vec2(i);
    return result;
}

// operator *
template <class T>
Vector<T> operator*(const T& aValue, const Vector<T>& aVector)
{
    Vector<T> result(aVector.getSize());
    for (int i = 0; i < aVector.getSize(); i++)
        result(i) = aValue*aVector(i);
    return result;
}

// operator *
template <class T>
Vector<T> operator*(const Vector<T>& aVector, const T& aValue)
{
    return operator*(aValue,aVector);
}

// operator *, pointwise multiplication
template <class T> Vector<T> operator*(const Vector<T>& aVector, const Vector<T>& bVector)
{
    assert(aVector.getSize() == bVector.getSize());
    Vector<T> result(aVector.getSize());
    for (int i = 0; i < aVector.getSize(); i++)
        result(i) = aVector(i)*bVector(i);
    return result;

}
// operator =
template <class T> Vector<T>& Vector<T>::operator=(const Vector<T>& vecO) {

    if(this == &vecO){return *this;}

    dataC = vecO.dataC;
    return *this;
}

template <class T> Vector<T>& Vector<T>::operator=(const T& fillValueO) {
    std::fill(dataC.begin(), dataC.end(), fillValueO);
    return *this;
}

template <class T> bool Vector<T>::operator==(const Vector<T>& vec0)
{
    assert(vec0.getSize()==dataC.size());
    for(int i = 0; i < dataC.size(); i++)
    {
        if(vec0(i) != dataC[i])
            return false;
    }
    return true;
}

//*************************************************************************************
// Determine the size of a Vector
//*************************************************************************************

template <class T> void Vector<T>::setSize(int sizeO) {
    dataC.resize(sizeO);
}

template <class T> void Vector<T>::setSize(int sizeO, const T &fillValO) {
    dataC.resize(sizeO);

    for(int i = 0; i < sizeO; i++) dataC[i] = fillValO;
}

//*************************************************************************************
// Cross Product
//*************************************************************************************


template <class T> void Vector<T>::cross(Vector<T>& src)
{

    assert(dataC.size() == 3 && src.getSize() == 3);

    Vector<T> newData(3);

    newData[0]=dataC[1]*src(2) - dataC[2]*src(1);
    newData[1]=dataC[2]*src(0) - dataC[0]*src(2);
    newData[2]=dataC[0]*src(1) - dataC[1]*src(0);

    dataC.clear();
    for(int i = 0; i < newData.getSize(); i++)
        dataC.push_back(newData(i));
}

template <class T> Vector<T> cross(Vector<T>& vec1, Vector<T>& vec2)
{
    assert(vec1.getSize() == 3 && vec2.getSize() == 3);

    Vector<T> result(3);

    result[0]=vec1[1]*vec2(2) - vec1[2]*vec2(1);
    result[1]=vec1[2]*vec2(0) - vec1[0]*vec2(2);
    result[2]=vec1[0]*vec2(1) - vec1[1]*vec2(0);

    return result;
}

template <class T> void Vector<T>::pushBack(const T& value)
{
    dataC.push_back(value);
}

//**************************************************************************************
// Show a vector on the console
//**************************************************************************************

template <class T> void Vector<T>::show() const
{

    for(unsigned int i = 0; i < dataC.size(); i++)
    {
        cout << dataC[i] << endl;
    }

    cout << "\n"<< endl;
}

//**************************************************************************************
// Fill
//**************************************************************************************

template <class T> void Vector<T>::fill(const T &valO)
{
    std::fill(dataC.begin(), dataC.end(), valO);
}

//**************************************************************************************
// Swap
//**************************************************************************************

template <class T> void Vector<T>::swap()
{
    std::reverse(dataC.begin(), dataC.end());
}

template <class T> Vector<T> swap(Vector<T>& vecToSwap)
{

    int size = vecToSwap.getSize();
    Vector<T> swapedOutput(size);

    std::reverse_copy(vecToSwap.begin(), vecToSwap.end(), swapedOutput.begin());

    return swapedOutput;

}

//**************************************************************************************
// Read text file
//**************************************************************************************

template <class T> void Vector<T>::readTXT(const char* aFilename, int sizeO)
{
    ifstream aStream(aFilename);

    dataC.resize(sizeO);
    for ( int i = 0; i < sizeO ; i++)
    {
        aStream >> dataC[i];
    }
}

//**************************************************************************************
// Clear
//**************************************************************************************

template <class T> void  Vector<T>::clearContent()
{
    dataC.clear();
}

//**************************************************************************************
// Intersection
//**************************************************************************************

template <class T> void Vector<T>::intersection(Vector<T>& vec2, Vector<T>& intersect)
{
    intersect.clearContent();

    intersect.clearContent();

    Vector<T> first(*this);
    Vector<T> second(vec2);

    first.sortV();
    second.sortV();


    int j = 0;
    for(unsigned int i = 0; i < dataC.size(); i++)
    {
        while (first[i] > second(j) && j < second.getSize()-1)
        {
            j++;
        }

        if(j == second.getSize())
            break;

        if(first[i] == second[j])
        {
            intersect.pushBack(first[i]);
        }
    }
}

//**************************************************************************************
// Compare if two vectors are equal or not
//**************************************************************************************

template <class T> bool Vector<T>::compare(Vector<T>& vecO)
{
    assert (dataC.size() == vecO.sizeC);

    for(int i = 0; i < dataC.size(); i++)
    {
        if(dataC[i] != vecO.dataC[i])
            return false;
    }
    return true;
}

//***************************************************************************************
// Min/Max return a pair containing the Min/Max value and the index
//***************************************************************************************

template <class T> pair<T, int> Vector<T>::maxim() {

    int pos = distance(dataC.begin(), max_element(dataC.begin(), dataC.end()));
    pair<T, int> maxValueWithPos(*max_element(dataC.begin(), dataC.end()), pos);
    return maxValueWithPos;
}

template <class T> pair<T, int>  Vector<T>::minim() {

    int pos = distance(dataC.begin(), min_element(dataC.begin(), dataC.end()));
    pair<T, int> minValueWithPos(*min_element(dataC.begin(), dataC.end()), pos);
    return minValueWithPos;
}

//***************************************************************************************
// Norm sqrt(xTy)
//***************************************************************************************

template <class T> T Vector<T>::norm() {
    T sumVec = 0.0;
    for (unsigned int i = 0; i < dataC.size(); i++)
        sumVec += dataC[i]*dataC[i];
    return sqrtf(sumVec);
}

//***************************************************************************************
// Sum up all Elements
//***************************************************************************************
template <class T> T Vector<T>::sum() {
    T result = 0.0;
    for (unsigned int i = 0; i < dataC.size(); i++)
        result += dataC[i];
    return result;
}


//***************************************************************************************
// Smooth linear
//***************************************************************************************

template <class T> void Vector<T>::smooth(int smoothRange)
{
    int width = floor(smoothRange/2.0f);
    int nrX = dataC.size();

    vector<T> smoothed;
    smoothed.resize(dataC.size());
    double mean;

    for(int i = 0; i < nrX; i++)
    {
        int cw = min(min(i, width),nrX-i-1);
        int upperCol = i - cw;
        int lowerCol = i + cw;

        mean = 0;
        for(int j = upperCol; j <= lowerCol; j++)
        {
            mean += this->dataC[j];
        }

        if((lowerCol - upperCol) > 0)
        {
            mean *=(1.0/((lowerCol - upperCol) + 1));
        }
        smoothed[i] = mean;
    }

    dataC = smoothed;
}

template <class T> Vector<T> smooth(int smoothRange, Vector<T>& vecToSmooth)
{
    int width = floor(smoothRange/2.0f);
    int nrX = vecToSmooth.size();

    Vector<T> smoothedOutput(vecToSmooth.getSize());
    double mean;

    for(int i = 0; i < nrX; i++)
    {
        int cw = min(min(i, width),nrX-i-1);
        int upperCol = i - cw;
        int lowerCol = i + cw;

        mean = 0;
        for(int j = upperCol; j <= lowerCol; j++)
        {
            mean += vecToSmooth[j];
        }

        if((lowerCol - upperCol) > 0)
        {
            mean *=(1.0/((lowerCol - upperCol) + 1));
        }
        smoothedOutput(i) = mean;
    }

    return smoothedOutput;
}

//***************************************************************************************
// Sort
//***************************************************************************************

template <class T> void Vector<T>::sortV()
{
    sort(dataC.begin(), dataC.end());
}

template <class T> T getMedian(const Vector<T> &vecToSort)
{
    vecToSort.sortV();
    return vecToSort(floor(vecToSort.getSize()/2.0));
}

template <class T> void Vector<T>::permutate()
{
    random_shuffle(dataC.begin(), dataC.end());
}

template <class T> Vector<T> sortV(Vector<T>& vecToSort)
{
    Vector<T> newV(vecToSort);
    newV.sortV();
    return newV;
}

//***************************************************************************************
// Dot Product
//***************************************************************************************

template <class T> T DotProduct(const Vector<T>& v1, const Vector<T>& v2)
{
    assert(v1.getSize() == v2.getSize());
    double result = 0;
    for (int i = 0; i < v1.getSize(); i++) {
        result += v1(i) * v2(i);
    }
    return result;
}

template <class T> void show_vec_vec(Vector<Vector<T> > &v)
{
    for(int i = 0; i < v.getSize(); i++)
    {
        for(int j = 0; j < v(i).getSize(); j++)
        {
            printf("%1.5f ", v(i)(j));
        }
        cout << endl;
    }
}

template <class T> Vector<T> ringOffset(const Vector<T>& vec, int offset)
{
    Vector<T> res(vec.getSize());
    for(int i = offset; i < res.getSize(); i++)
    {
        res(i) = vec(i-offset);
    }

    for(int i = 0; i < offset; i++)
    {
        res(i) = vec(vec.getSize()-offset+i);
    }

    return res;
}

template <class T> Vector<T> hist(Vector<T>& vec, int nrBins, bool normalize, T st, T en)
{
    Vector<T> histogram(nrBins, 0.0);

    int cnt = 0;
    T step = (en-st)/(double)(nrBins);

//    cout << "step " << step << endl;
    for(T i = st; i < en; i+=step)
    {
//        cout << i << endl;
        for(int j = 0; j < vec.getSize(); j++)
        {
            if(vec(j) >= i && vec(j) < i+step)
            {
                histogram(cnt) += 1.0;
            }

            // Make sure if a element in the vector equals en we need to add to the last bin.
            if(vec(j) == en)
            {
                histogram(nrBins-1) += 1.0;
            }
        }
        cnt += 1;
    }

    if(normalize && histogram.sum()>0)
    {
        histogram *= 1.0/histogram.sum();
    }

    return histogram;
}


//***************************************************************************************
// Transfer my vec to stl
//***************************************************************************************

template <class T> vector<T> Vector<T>::getStl()
{
    vector<T> newStl;
    newStl = dataC;
    return newStl;
}


//***************************************************************************************
// Append
//***************************************************************************************

template <class T> void Vector<T>::append(Vector<T>& toInsert)
{
    for(int i = 0; i < toInsert.getSize(); i++)
    {
        dataC.push_back(toInsert[i]);
    }
}

template <class T> int Vector<T>::findV(const T &value)
{
    typename vector<T>::iterator it;
    it = find(dataC.begin(), dataC.end(), value);
    if(it == dataC.end())
    {
        return -1;
    }
    else
    {
        return distance(dataC.begin(), it);
    }
}
template <class T>  T* Vector<T>::data()
{
    return &dataC[0];
}



//***************************************************************************************
// Resize
//***************************************************************************************

template <class T> void Vector<T>::resize(int newSize)
{
    Vector<T> copyVec = dataC;

    dataC.resize(newSize);

    int nr = min(newSize, copyVec.getSize());
    for(int i = 0; i < nr; i++)
    {
        dataC[i] = copyVec(i);
    }
}

template <class T> void Vector<T>::resize_from_end(int newSize)
{
    Vector<T> copyVec = dataC;

    dataC.resize(newSize);

//    int nr = min(newSize, copyVec.getSize());
    int d = newSize - copyVec.getSize();
    int so,eo = copyVec.getSize(), sd;//,ed=newSize;
    if(d>0)
    {
        so = 0;
        sd = d;
    }
    else
    {
        so = -d;
        sd = 0;
    }
    for(int io = so, id = sd; io < eo; ++io,++id)
    {
        dataC[id] = copyVec(io);
    }
}

//**************************************
// reserveCapacity
//**************************************
template <class T> void Vector<T>::reserveCapacity(const int valO)
{
    dataC.reserve(valO);
}

//**************************************
// write the data to .txt
//**************************************

template <class T> void Vector<T>::writeToTXT(char* filename)
{
    ofstream aStream(filename);
    for (int i = 0; i < dataC.size(); i++)
    {
        aStream << dataC[i] << endl;
    }
}

template <class T> void Vector<T>::make_unique()
{
    using namespace std;
    vector<int>::iterator new_end;
    this->sortV();
    new_end = unique(dataC.begin(), dataC.end());
    // delete all elements past new_end
    dataC.erase(new_end, dataC.end());
}

template <class T> void Vector<T>::erase_element(int index)
{
    dataC.erase(dataC.begin()+ index);
}

template <class T> T hist_bhatta(Vector<T>& h1, Vector<T>& h2)
{
    assert(h1.getSize()==h2.getSize());
    T distance = 0.0;

    for(int i = 0; i < h1.getSize(); i++)
    {
        distance += sqrt(h1(i)*h2(i));
    }
    return distance;
}

#endif	/* _DENNIS_VECTOR_H */

