/*
 * Copyright 2012 Dennis Mitzel
 *
 * Authors: Dennis Mitzel
 * Computer Vision Group RWTH Aachen.
 */

#ifndef _VOLUME_DENNIS_H
#define	_VOLUME_DENNIS_H

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>

#include "Matrix.h"

//+++++++++++++++++++++++++++++++++ Definition ++++++++++++++++++++++++++++++++++++++

template <class T> class Volume
{

    //*****************************************************
    // Constructors
    //*****************************************************
public:
    // standard constructor
    Volume();
//    Volume(const int xSizeO, const int ySizeO, const int zSizeO);

    //  copy constructor
    Volume(const Volume<T>& Copy);

    // filling constructor
//    Volume(const int xSizeO, const int ySizeO, const int zSizeO , const T& fillValO);

    ~Volume();

    //*****************************************************
    // Operators
    //*****************************************************

    Volume<T>& operator+=(const Volume<T>& volO);
    Volume<T>& operator*=(const Volume<T>& volO);
    Volume<T>& operator*=(const T& value);

    inline T& operator()(const int xO, const int yO, const int zO);
    inline T operator()(const int xO, const int yO, const int zO ) const;
    Volume<T>& operator=(const Volume<T>& CopyO);
    Volume<T>& operator=(const T& fillValO);

    //*****************************************************
    // Size
    //*****************************************************
    void setSize( int xO,  int yO, int zO);
    void setSize(int xO,  int yO, int zO, const T &fillValO);

    //*****************************************************
    // Get / Set matrices into the volume
    //*****************************************************
    void getMatrix(Matrix<T>& MatO, const int zO);
    void insertMatrix(Matrix<T>& MatO, const int zO);

    //*****************************************************
    // Sum along axes z
    //*****************************************************
    void sumAlongAxisZ(Matrix<T>& result);

    //*****************************************************
    // Set all volume entries to a constant value
    //*****************************************************
    void fill(const T& fillValueO);

    //*****************************************************
    // Write
    //*****************************************************
    void writeToTXT(char* filename);



    int xSize();
    int ySize();
    int zSize();
    int size();
    T* data();

protected:

    int xSizeC;
    int ySizeC;
    int zSizeC;
    T *dataC;

};

//template <class T> Matrix<T> sumVolAlongAxisZ(Volume<T>& vol);
//template <class T> Matrix<T> getMatrix(Volume<T>& vol, const int zO);
//*****************************************************************************************************************************************************//
//-----------------------------------------------------------------------------------------------------------------------------------------------------//
//                                                                                                                                                     //
//                                                          IMPLEMENTATION                                                                             //
//                                                                                                                                                     //
//-----------------------------------------------------------------------------------------------------------------------------------------------------//
//*****************************************************************************************************************************************************//

template <class T>  Volume<T>::Volume(const Volume<T>& Copy)
{
    xSizeC = Copy.xSizeC;
    ySizeC = Copy.ySizeC;
    zSizeC = Copy.zSizeC;

    int whSize = xSizeC*ySizeC*zSizeC;

    dataC = new T[whSize];

    memcpy(dataC, Copy.dataC, sizeof(T)*whSize);

}

template <class T> void Volume<T>::fill(const T &fillValueO)
{
    int whSize = xSizeC * ySizeC * zSizeC;
    std::fill(dataC, dataC + whSize, fillValueO);
}

template <class T> Volume<T>::Volume()
{
    xSizeC = 0;
    ySizeC = 0;
    zSizeC = 0;
    dataC = 0;
}

//template <class T> Volume<T>::Volume(const int xSizeO, const int ySizeO, const int zSizeO)
//{
//    xSizeC = xSizeO;
//    ySizeC = ySizeO;
//    zSizeC = zSizeO;
//    dataC = new T[xSizeO*ySizeO*zSizeO];
//}

//template <class T> Volume<T>::Volume(const int xSizeO, const int ySizeO, const int zSizeO, const T &fillVal0)
//{
//    xSizeC = xSizeO;
//    ySizeC = ySizeO;
//    zSizeC = zSizeO;
//    int whSize = xSizeO*ySizeO*zSizeO;
//    dataC = new T[whSize];
//    for (int i = 0; i < whSize; i++)
//    {
//        dataC[i] = fillVal0;
//    }
//}

template <class T> Volume<T>::~Volume()
{
    if (dataC != 0) delete [] dataC;
}

template <class T> void Volume<T>::setSize( int xO,  int yO, int zO)
{
  if (dataC != 0) delete[] dataC;
  dataC = new T[xO*yO*zO];
  xSizeC = xO;
  ySizeC = yO;
  zSizeC = zO;
}


template <class T> void Volume<T>::setSize(int xO,  int yO, int zO,  const T &fillValO)
{
  if (dataC != 0) delete[] dataC;
  dataC = new T[xO*yO*zO];
  xSizeC = xO;
  ySizeC = yO;
  zSizeC = zO;

  int whSize = xO*yO*zO;

  for(int i = 0; i < whSize; i++)
  {
      dataC[i] = fillValO;
  }
}

template <class T> int Volume<T>::ySize(){
  return ySizeC;
}

template <class T> int Volume<T>::xSize(){
  return xSizeC;
}

template <class T> int Volume<T>::zSize(){
  return zSizeC;
}

template <class T> int Volume<T>::size()
{
  return ySizeC * xSizeC * zSizeC;
}

template <class T> T* Volume<T>::data(){
  return dataC;
}

template <class T> inline T& Volume<T>::operator()(const int xO, const int yO, const int zO)
{
    assert(xO < xSizeC && yO < ySizeC && zO < zSizeC && xO >= 0 && yO >= 0 && zO >= 0);
    return dataC[xSizeC*(ySizeC*zO+yO)+xO];
}

template <class T> inline T Volume<T>::operator()(const int xO, const int yO, const int zO) const
{
    assert(xO < xSizeC && yO < ySizeC && zO < zSizeC && xO >= 0 && yO >= 0 && zO >= 0);
    return dataC[xSizeC*(ySizeC*zO+yO)+xO];
}

template <class T> Volume<T>& Volume<T>::operator=(const T& fillValue)
{
    std::fill(dataC, dataC + (xSizeC*ySizeC*zSizeC), fillValue);
    return *this;

}

template <class T> Volume<T>& Volume<T>::operator=(const Volume<T>& CopyO) {

    if (this == &CopyO)
    {
        return *this;
    }

    if (dataC != 0) {
        delete[] dataC;
    }
    xSizeC = CopyO.xSizeC;
    ySizeC = CopyO.ySizeC;
    zSizeC = CopyO.zSizeC;
    int whSize = xSizeC * ySizeC*zSizeC;
    dataC = new T[whSize];
    for (int i = 0; i < whSize; i++)
    {
        dataC[i] = CopyO.dataC[i];
    }


    return *this;
}

template <class T> void Volume<T>::getMatrix(Matrix<T>& MatO, const int zO)
{

    MatO.set_size(xSizeC, ySizeC);
    T* ptr = dataC + (xSizeC*ySizeC*zO);
    for (int i = 0; i < xSizeC * ySizeC; i++)
    {
        MatO.data()[i] = ptr[i];
    }
}

template <class T> void Volume<T>::insertMatrix(Matrix<T>& MatO, const int zO)
{
    if (MatO.xSize() != xSizeC || MatO.ySize() != ySizeC)
    {
        cerr << "Make sure that the size of Matrix equals to xSize and ySize of Volume!!!" << endl;
    }
    T* ptr = dataC + (xSizeC*ySizeC*zO);
    for (int i = 0; i < xSizeC*ySizeC; i++)
    {
        ptr[i] = MatO.data()[i];
    }
}

template <class T> Volume<T>& Volume<T>::operator+=(const Volume<T>& volO) {

  if (xSizeC != volO.xSizeC || ySizeC != volO.ySizeC || zSizeC != volO.zSizeC)
      cerr << "Ensure the corrent size of both Volumes for add!!!" << endl;

  int whoSize = xSizeC*ySizeC*zSizeC;
  for (int i = 0; i < whoSize; i++)
    dataC[i] += volO.dataC[i];
  return *this;
}

template <class T> Volume<T>& Volume<T>::operator*=(const Volume<T>& volO)
{

    if (xSizeC != volO.xSizeC || ySizeC != volO.ySizeC || zSizeC != volO.zSizeC)
        cerr << "ERORR: Check the size of both Volumes for add!!!" << endl;

    int whoSize = xSizeC*ySizeC*zSizeC;
        for (int i = 0; i < whoSize; i++)
            dataC[i] *= volO.dataC[i];

    return *this;
}

template <class T> Volume<T>& Volume<T>::operator*=(const T& value)
{
    int whoSize = xSizeC*ySizeC*zSizeC;
        for (int i = 0; i < whoSize; i++)
            dataC[i] *= value;

    return *this;
}


template <class T> void Volume<T>::sumAlongAxisZ(Matrix<T>& result) {
    Matrix<T> aux;
    result.set_size(xSizeC, ySizeC, 0.0);
    aux.set_size(xSizeC, ySizeC);
    for (int i = 0; i < zSizeC; i++) {
        getMatrix(aux, i);
        result += aux;
    }
}

template <class T> void Volume<T>::writeToTXT(char* filename)
{
  ofstream aStream(filename);
  for(int i = 0; i < xSizeC*ySizeC*zSizeC; i++)
  {
      if ((i+1)%xSizeC != 0)
          aStream << dataC[i] << " ";
     else
        aStream << dataC[i] << ";"<< endl;
  }
}


//template <class T> Matrix<T> sumVolAlongAxisZ(Volume<T>& vol)
//{
//    Matrix<T> aux;
//    Matrix<T> result(vol.xSize(), vol.ySize(), 0.0);

//    aux.setSize(vol.xSize(), vol.ySize());
//    for (int i = 0; i < vol.zSize(); i++) {
//        getMatrix(aux, i);
//        result += aux;
//    }

//    return result;
//}

//template <class T> Matrix<T> getMatrix(Volume<T>& vol, const int zO)
//{
//    Matrix<T> mat(vol.xSize(), vol.ySize());

//    for (int i = 0; i < vol.xSize()*vol.ySize(); i++)
//    {
//        mat.data()[i] = vol.data()[vol.xSize()*vol.ySize()*zO + i];
//    }
//}

#endif	/* _VOLUME_DENNIS_H */

