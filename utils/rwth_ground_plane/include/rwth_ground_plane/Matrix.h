/*
 * File:   Matrix.h
 * Author: mitzel
 *
 * Created on May 14, 2009, 10:03 AM
 */

#ifndef _DENNIS_MATRIX_H
#define	_DENNIS_MATRIX_H
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <string.h>
#include <cassert>
#include "Vector.h"


using namespace std;
//+++++++++++++++++++++++++++++++ Definition +++++++++++++++++++++++++++++++++
template <class T>
class Matrix
{
public:

    //*****************************************************
    // Constructors
    //*****************************************************

    // standard constructor
    Matrix();
    // Create a matrix with specific size
    Matrix(const int x_size, const int y_size);
    // Create a matrix and fill it with another matrix
    Matrix(const Matrix<T>& source);
    // Create a mtrix with specific size and fill it with a specific value.
    Matrix(const int x_size, const int y_size, const T& fill_value);

    Matrix(const int xSizeO, const int ySizeO, const T* data);
    // Create a matrix and fill it with a vector of vector (convert vector of vector to matrix)
    Matrix(Vector<Vector<T> > &vector_of_vector);
    // Create a matrix and fill it with a specific part of another matrix
    Matrix(const Matrix<T> &source, int start_column, int end_column, int start_row, int end_row);

    ~Matrix();

    //******************************************************
    // Operators
    //******************************************************
    inline T& operator()(const int xO, const int yO);
    inline T operator()(const int xO, const int yO) const;

    inline Matrix<T>& operator+=(const Matrix<T>& MatO);
    inline Matrix<T>& operator-=(const Matrix<T>& MatO);
    inline Matrix<T>& operator*=(const Matrix<T>& MatO);

    inline Matrix<T>& operator+=(const T& scalar);
    inline Matrix<T>& operator*=(const T& scalar);

    inline Matrix<T>& operator*=(const Vector<T>& vecO);

    inline Matrix<T>& operator=(const T& aValue);
    inline Matrix<T>& operator=(const Matrix<T>& CopyO);

    void insert(Matrix<T>& mat, int start_x, int start_y);

    //******************************************************
    // Size
    //******************************************************
    inline int x_size() const ;
    inline int y_size() const ;
    inline int total_size() const;

    void set_size(int x_size,  int y_size);
    void set_size(int x_size, int y_size, const T& fill_value);

    //******************************************************
    // Read/Write from File
    //******************************************************

    // this methods expects that in the first row the y and x
    // size of the matrix ist set
    void ReadFromTXT(string file_name);
    void ReadFromTXT(string filename, int x_size, int y_size);
    void ReadFromPGM(const char* filename);
    void WriteToPGM(const char *filename);
    void ReadFromBinaryFile(const char *filename);

    // if flag is 1 then the matrix dim will be added to the file
    // in the row (y, x)
    void WriteToTXT(const char* filename, const int flag_size=0) const;

    //*******************************************************
    // Inverse
    //*******************************************************

    bool inv();

    //*******************************************************
    // Show
    //*******************************************************
    void Show() const;

    //*******************************************************
    // Transpose
    //*******************************************************
    void Transpose();

    //*******************************************************
    // Up/DownSample the matrix
    //*******************************************************
    void UpSample(int new_x_size, int new_y_size);
    void DownSample(int new_x_size, int new_y_size);

    //*******************************************************
    // Sum along axes
    //*******************************************************

    void sumAlongAxisX(Vector<T>& resultO);


    void getColumn(const int colNum, Vector<T>& target);
    Vector<T> getColumn(const int colNum);

    void getRow(const int rowNumber, Vector<T>& target);
    Vector<T> getRow(const int rowNumber);

    //*******************************************************
    // Swap / Append along y axis
    //*******************************************************
    void swap();

    void cutPart(int startC, int endC, int startR, int endR, Matrix<T>& target);

    void insertRow(Vector<T>& src, const int rowNumber);

    void transferMatToVecVec(Vector<Vector<T> >& res);

    //*******************************************************
    // Set all the matrix entries to a constant value
    //*******************************************************
    void fill(const T& fillvalue);


    //*******************************************************//
    // Get the pointer to the data                           //
    //*******************************************************//
    T* data() const;

    void transferVecVecToMat(Vector<Vector<T> >& objO);


protected:

    int x_size_;
    int y_size_;
    T *data_;

};

// Returns a matrix where all negative elements are turned positive
template <class T> Matrix<T> abs(const Matrix<T>& aMatrix);
// Returns the tranposed matrix
template <class T> Matrix<T> Transpose(const Matrix<T>& aMatrix);
// Eye
template <class T> Matrix<T> Eye(int value);
// matrix sum
template <class T> Matrix<T> operator+( Matrix<T>& aM1,  Matrix<T>& aM2);
// matrix difference
template <class T> Matrix<T> operator-( Matrix<T> const & aM1,  Matrix<T> const & aM2);
// matrix product
template <class T> Matrix<T> operator*( Matrix<T> const & aM1,  Matrix<T> const & aM2);
// Multiplication with a vector
template <class T> Vector<T> operator*( Matrix<T> const & aMatrix,  Vector<T> const & aVector);
// Multiplikation with a scalar
template <class T> Matrix<T> operator*( Matrix<T>& aMatrix,  T aValue);
template <class T> Matrix<T> operator*( T aValue,  Matrix<T>& aMatrix);

template <class T> T maxOfMatrix(const Matrix<T>& aMatrix, int& xPos, int& yPos);


//*****************************************************************************************************************************************************//
//-----------------------------------------------------------------------------------------------------------------------------------------------------//
//                                                                                                                                                     //
//                                                          IMPLEMENTATION                                                                             //
//                                                                                                                                                     //
//-----------------------------------------------------------------------------------------------------------------------------------------------------//
//*****************************************************************************************************************************************************//

//*****************************************************************
// Constructors
//*****************************************************************

// standard constructor
template <class T> Matrix<T>::Matrix()
{
    x_size_ = 0;
    y_size_ = 0;
    data_ = 0;
}

template <class T> Matrix<T>::Matrix(const int x_size, const int y_size)
{
    x_size_ = x_size;
    y_size_ = y_size;
    data_ = new T[x_size*y_size];
}

// copy constructor
template <class T> Matrix<T>::Matrix(const Matrix<T>& source)
{
    x_size_ = source.x_size_;
    y_size_ = source.y_size_;
    int total_size = x_size_*y_size_;
    data_ = new T[total_size];

    memcpy(data_, source.data_, sizeof(T)*total_size);
}

template <class T> Matrix<T>::Matrix(const int x_size, const int y_size, const T &fill_value)
{
    x_size_ = x_size;
    y_size_ = y_size;
    int total_size = x_size*y_size;
    data_ = new T[total_size];

    std::fill(data_, data_+total_size, fill_value);

}

// filling constructor
template <class T> Matrix<T>::Matrix(const int xSizeO, const int ySizeO, const T *data)
{
    x_size_ = xSizeO;
    y_size_ = ySizeO;
    data_ = new T[xSizeO*ySizeO];

    int whSize = x_size_*y_size_;
    memcpy(data_, data, sizeof(T)*whSize);
}

template <class T> Matrix<T>::Matrix(Vector<Vector<T> >& vector_of_vector)
{
    y_size_ = vector_of_vector.getSize();

    if(y_size_ > 0)
    {
        int total_size = y_size_;

        x_size_ = vector_of_vector(0).getSize();

        for(int i = 0; i < y_size_; i++)
        {
            total_size +=vector_of_vector(i).getSize();
        }

        T* new_data = new T[total_size];

        for(int i = 0; i < y_size_; i++)
        {
            for(int j = 0; j < x_size_; j++)
            {
                new_data[i*x_size_ + j] = vector_of_vector(i)(j);
            }
        }
        data_ = new_data;
    }
    else
    {
        x_size_ = 0;
        data_ = 0;
    }
}

template <class T> Matrix<T>::Matrix(const Matrix<T>& source, int start_column, int end_column, int start_row, int end_row)
{
    assert (start_column <= end_column && start_row <= end_row && start_column  >= 0 && end_column >= 0 && start_row >= 0 && end_row >= 0 && end_column < source.x_size() && end_row < source.y_size());

    x_size_ = end_column-start_column + 1;
    y_size_ = end_row-start_row + 1;

    data_ = new T[x_size_*y_size_];

    for (int i = start_column; i < end_column+1; i++)
    {
        for(int j = start_row; j < end_row+1; j++)
        {
            this->operator ()(i-start_column, j-start_row) = source(i, j);
        }
    }
}

template <class T> Matrix<T>::~Matrix()
{
    if (data_ != 0)
        delete [] data_;
    data_ = 0;
}

//*****************************************************************
// Operators
//*****************************************************************

// operator () - get item
template <class T> inline T& Matrix<T>::operator()(int x, int y)
{
    assert(x < x_size_ && y < y_size_ && x >= 0 && y >= 0);

    return data_[x_size_*y+x];
}

// operator () - get item
template <class T> inline T Matrix<T>::operator()(int x, int y) const
{
    assert(x < x_size_ && y < y_size_ && x >= 0 && y >= 0);

    return data_[x_size_*y+x];
}

// operator = scaler
template <class T> Matrix<T>& Matrix<T>::operator=(const T& fill_value)
{
    int total_size = x_size_*y_size_;

    std::fill(data_, data_ + total_size, fill_value);
    return *this;
}

// operator = matrix
template <class T> inline Matrix<T>& Matrix<T>::operator=(const Matrix<T>& copy)
{
    if (this != &copy)
    {
        delete[] data_;

        x_size_ = copy.x_size_;
        y_size_ = copy.y_size_;
        int total_size = x_size_*y_size_;
        data_ = new T[total_size];

        memcpy(data_, copy.data_, sizeof (T) * total_size);
    }
    return *this;
}

// operator + matrix
template <class T> inline Matrix<T>& Matrix<T>::operator+=(const Matrix<T>& mat)
{
    assert ((x_size_ == mat.x_size_) && (y_size_ == mat.y_size_));

    for (int i = 0; i < x_size_*y_size_; i++)
        data_[i] += mat.data_[i];
    return *this;
}

// operator - matrix
template <class T> inline Matrix<T>& Matrix<T>::operator-=(const Matrix<T>& mat)
{
    assert ((x_size_ == mat.x_size_) && (y_size_ == mat.y_size_));

    for (int i = 0; i < x_size_*y_size_; i++)
        data_[i] -= mat.data_[i];
    return *this;
}

// operator * matrix
template <class T>inline  Matrix<T>& Matrix<T>::operator*=(const Matrix<T>& mat)
{
    assert ((x_size_ == mat.y_size_));
    assert (this != &mat);
    T* copy = new T[x_size_*y_size_];
    for (int i = 0; i < x_size_*y_size_; i++) copy[i] = data_[i];
    int copyXSize = x_size_;
    x_size_ = mat.x_size_;

    delete [] data_;

    data_ = new T[y_size_ * mat.x_size_];
    for (int i = 0; i < y_size_; i++){
        for(int j = 0; j < mat.x_size_; j++){
            data_[mat.x_size_*i + j] = 0;
            for (int l = 0; l < copyXSize; l++){
                data_[mat.x_size_*i + j] += copy[copyXSize*i + l]*mat(j,l);
            }
        }
    }
    delete [] copy;
    return *this;
}

// operator + scalar
template <class T> inline Matrix<T>& Matrix<T>::operator+=(const T& scalar)
{

    for (int i = 0; i < x_size_*y_size_; i++)
        data_[i] += scalar;
    return *this;
}

// operator * scalar
template <class T> inline Matrix<T>& Matrix<T>::operator*=(const T& scalar)
{
    int max = x_size_*y_size_;
    for (int i = 0; i < max; i++)
        data_[i] *= scalar;
    return *this;
}

// Operator * vector
template <class T> inline Matrix<T>& Matrix<T>::operator*=(const Vector<T>& vec)
{
    assert (x_size_ == vec.getSize());

    T* copy = new T[x_size_*y_size_];
    for (int i = 0; i < x_size_*y_size_; i++) copy[i] = data_[i];
    delete [] data_;
    data_ = new T[y_size_];
    x_size_ = 1;
    for (int i = 0; i < y_size_; i++){
        data_[i] = 0;
        for (int l = 0; l < vec.getSize(); l++){
            data_[i] += copy[vec.getSize()*i + l]*vec(l);
        }
    }
    delete [] copy;
    return *this;
}

//*****************************************************************
// Size / Data
//*****************************************************************

template <class T> inline int Matrix<T>::y_size() const
{
    return y_size_;
}

template <class T> inline int Matrix<T>::x_size() const
{
    return x_size_;
}

template <class T> inline int Matrix<T>::total_size() const
{
    return x_size_ * y_size_;
}

template <class T> T* Matrix<T>::data() const
{
    return data_;
}


template <class T> void Matrix<T>::set_size(int x_size, int y_size) {
    if (data_ != 0) delete[] data_;
    data_ = new T[x_size*y_size];
    x_size_ = x_size;
    y_size_ = y_size;
}

template <class T> void Matrix<T>::set_size(int x_size, int y_size, const T &fill_value) {
    if (data_ != 0) delete[] data_;
    data_ = new T[x_size*y_size];
    x_size_ = x_size;
    y_size_ = y_size;

    std::fill(data_, data_+x_size_*y_size_, fill_value);
}

template <class T> void Matrix<T>::insert(Matrix<T>& mat, int start_x, int start_y)
{
    assert (x_size_ >= mat.x_size()+start_x && y_size_ >= mat.y_size()+start_y);
    for(int i = start_x; i < start_x+mat.x_size(); i++)
    {
        for(int j = start_y; j < start_y+mat.y_size(); j++)
        {
            this->operator ()(i,j) = mat(i-start_x, j-start_y);
        }
    }
}

//******************************************************
// Read/Write from File
//******************************************************

template <class T> void Matrix<T>::ReadFromTXT(string filename)
{
    const char *p = filename.c_str();
    ifstream aStream(p);
    // read in Size
    aStream >> y_size_ >> x_size_;

    delete [] data_;
    int total_size = x_size_*y_size_;
    data_ = new T[total_size];
    for (int i = 0; i < total_size ; i++)
    {
        aStream >> data_[i];
    }
}

template <class T> void Matrix<T>::ReadFromTXT(string filename, int x_size, int y_size)
{
    const char *p;
    p=filename.c_str();
    ifstream aStream(p);

    x_size_ = x_size;
    y_size_ = y_size;
    delete [] data_;
    int total_size = x_size_*y_size_;
    data_ = new T[total_size];
    for (int i = 0; i < total_size ; i++)
    {
        aStream >> data_[i];
    }
}

template <class T> void Matrix<T>::WriteToTXT(const char* filename, const int flag_size) const
{
    ofstream aStream(filename);
    if (flag_size==1)
    {
        aStream << y_size_ << " " << x_size_ << endl;
    }
    else if(flag_size==2)
    {
        aStream << y_size_ << " " << endl;
    }

    for (int i = 0; i < x_size_*y_size_; i++) {
        if ((i+1)%x_size_ != 0)
            aStream << data_[i] << " ";
        else
            aStream << data_[i] << endl;
    }
    aStream.close();
}

template <class T> void Matrix<T>::WriteToPGM(const char* filename)
{
    FILE *stream;
    stream = fopen(filename,"wb");
    // write header
    char line[60];
    sprintf(line,"P5\n%d %d\n255\n",x_size_,y_size_);
    fwrite(line,strlen(line),1,stream);
    // write data
    for (int i = 0; i < x_size_*y_size_; i++) {
        char dummy = (char)data_[i];
        fwrite(&dummy,1,1,stream);
    }
    fclose(stream);
}

template <class T> void Matrix<T>::ReadFromPGM(const char* filename)
{
    FILE *stream;
    stream = fopen(filename,"rb");
    if (stream == 0) std::cerr << "File not found: " << filename << std::endl;
    int dummy;
    // Find beginning of file (P5)
    while (getc(stream) != 'P');
    if (getc(stream) != '5') cerr << "No PGM Image" << endl;
    while (getc(stream) != '\n');
    // Remove comments and empty lines
    dummy = getc(stream);
    while (dummy == '#') {
        while (getc(stream) != '\n');
        dummy = getc(stream);
    }
    while (dummy == '\n')
        dummy = getc(stream);
    // Read image size
    x_size_ = dummy-48;
    while ((dummy = getc(stream)) >= 48 && dummy < 58)
        x_size_ = 10*x_size_+dummy-48;
    while ((dummy = getc(stream)) < 48 || dummy >= 58);
    y_size_ = dummy-48;
    while ((dummy = getc(stream)) >= 48 && dummy < 58)
        y_size_ = 10*y_size_+dummy-48;
    if (dummy != '\n') while (getc(stream) != '\n');
    while (getc(stream) != '\n');
    // Adjust size of data structure
    delete[] data_;
    data_ = new T[x_size_*y_size_];
    // Read image data
    for (int i = 0; i < x_size_*y_size_; i++)
        data_[i] = getc(stream);
    fclose(stream);
}

//***************************************************************
// Compute Inverse of a matrix
//***************************************************************
// Credits T. Brox

template <class T> bool Matrix<T>::inv() {

    assert(y_size_ == x_size_);
    int* p = new int[x_size_];
    T* hv = new T[x_size_];
    Matrix<T>& I(*this);
    int n = y_size_;
    for (int j = 0; j < n; j++)
        p[j] = j;
    for (int j = 0; j < n; j++) {
        T max = fabs(I(j, j));
        int r = j;
        for (int i = j + 1; i < n; i++)
            if (fabs(I(j, i)) > max) {
                max = fabs(I(j, i));
                r = i;
            }

        // Matrix singular
        if (max <= 0)
        {
            return false;
        }

        if (r > j) {
            for (int k = 0; k < n; k++) {
                T hr = I(k, j);
                I(k, j) = I(k, r);
                I(k, r) = hr;
            }
            int hi = p[j];
            p[j] = p[r];
            p[r] = hi;
        }
        T hr = 1 / I(j, j);
        for (int i = 0; i < n; i++)
            I(j, i) *= hr;
        I(j, j) = hr;
        hr *= -1;
        for (int k = 0; k < n; k++)
            if (k != j) {
                for (int i = 0; i < n; i++)
                    if (i != j) I(k, i) -= I(j, i) * I(k, j);
                I(k, j) *= hr;
            }
    }
    for (int i = 0; i < n; i++) {
        for (int k = 0; k < n; k++)
            hv[p[k]] = I(k, i);
        for (int k = 0; k < n; k++)
            I(k, i) = hv[k];
    }

    delete [] p;
    delete [] hv;
    return true;
}

//***************************************************************
// Show
//***************************************************************

template <class T> void Matrix<T>::Show() const
{
    for (int i = 0; i < y_size_; i++){
        printf("Row: %02d | ", i);
        for(int j = 0; j < x_size_; j++){
            printf("%f ", data_[i*x_size_ + j]);
        }
        printf("\n");
    }
    printf("\n");
}

//*******************************************************
// Further non class methods
//*******************************************************

template <class T> Matrix<T> Eye(int value)
{
    Matrix<T> eye(value, value, 0.0);

    for(int i = 0; i < value; i++)
    {
        eye(i,i) = 1;
    }

    return eye;
}

template <class T> Matrix<T> abs(const Matrix<T>& src)
{
    Matrix<T> result(src.x_size(),src.y_size());
    int wholeSize = src.size();
    for (register int i = 0; i < wholeSize; i++) {
        if (src.data()[i] < 0) result.data()[i] = -src.data()[i];
        else result.data()[i] = src.data()[i];
    }
    return result;
}

template <class T> Matrix<T> Transpose(const Matrix<T>& src)
{
    Matrix<T> result(src.y_size(),src.x_size());
    for (int y = 0; y < src.y_size(); y++)
        for (int x = 0; x < src.x_size(); x++)
            result(y,x) = src(x,y);
    return result;
}

template <class T> void Matrix<T>::Transpose()
{
    Matrix<T> copy(y_size_, x_size_);
    for (int i = 0; i < y_size_; i++)
        for (int j = 0; j < x_size_; j++)
            copy(i,j) = data_[i*x_size_ + j];

    memcpy(data_, copy.data_, sizeof(T)*x_size_*y_size_);
    y_size_ = copy.y_size();
    x_size_ = copy.x_size();
}

template <class T> Matrix<T> operator+(Matrix<T>& m1, Matrix<T>& m2)
{
    assert(m1.x_size() == m2.x_size() && m1.y_size() == m2.y_size());

    Matrix<T> result(m1.x_size(),m1.y_size());
    int whSize = m1.x_size()*m1.y_size();
    for (int i = 0; i < whSize; i++)
        result.data()[i] = m1.data()[i] + m2.data()[i];
    return result;
}

template <class T> Matrix<T> operator-(Matrix<T> const & m1, Matrix<T> const & m2)
{
    assert(m1.x_size() == m2.x_size() && m1.y_size() == m2.y_size());
    Matrix<T> result(m1.x_size(),m1.y_size());
    int whSize = m1.x_size()*m1.y_size();
    for (int i = 0; i < whSize; i++)
        result.data()[i] = m1.data()[i] - m2.data()[i];
    return result;
}

template <class T> Matrix<T> operator*(Matrix<T> const & m1, Matrix<T> const & m2)
{
    Matrix<T> result = m1;
    result *= m2;
    return result;
}

template <class T> Vector<T> operator*(Matrix<T> const & mat, Vector<T> const & vec)
{
    assert(mat.x_size() == vec.getSize());

    Vector<T> result(mat.y_size(),0.0);
    for (int y = 0; y < mat.y_size(); y++)
        for (int x = 0; x < mat.x_size(); x++)
            result(y) += mat(x,y)*vec(x);
    return result;
}

template <class T> Matrix<T> operator*(Matrix<T>& mat, T val)
{
    Matrix<T> result(mat.x_size(),mat.y_size());
    int whSize = mat.x_size()*mat.y_size();
    for (int i = 0; i < whSize; i++)
        result.data()[i] = mat.data()[i]*val;
    return result;
}

template <class T> inline Matrix<T> operator*(T val, Matrix<T>& mat)
{
    return mat*val;
}

template <class T> void Matrix<T>::UpSample(int new_x_size, int new_y_size)
{
    // Upsample in x-direction
    int aIntermedSize = new_x_size*y_size_;
    T* aIntermedData = new T[aIntermedSize];
    if (new_x_size > x_size_) {
        for (int i = 0; i < aIntermedSize; i++)
            aIntermedData[i] = 0.0;
        T factor = ((double)new_x_size)/x_size_;
        for (int y = 0; y < y_size_; y++) {
            int aFineOffset = y*new_x_size;
            int aCoarseOffset = y*x_size_;
            int i = aCoarseOffset;
            int j = aFineOffset;
            int aLastI = aCoarseOffset+x_size_;
            int aLastJ = aFineOffset+new_x_size;
            T rest = factor;
            T part = 1.0;
            do {
                if (rest > 1.0) {
                    aIntermedData[j] += part*data_[i];
                    rest -= part;
                    part = 1.0;
                    j++;
                    if (rest <= 0.0) {
                        rest = factor;
                        i++;
                    }
                }
                else {
                    aIntermedData[j] += rest*data_[i];
                    part = 1.0-rest;
                    rest = factor;
                    i++;
                }
            }
            while (i < aLastI && j < aLastJ);
        }
    }
    else {
        T* aTemp = aIntermedData;
        aIntermedData = data_;
        data_ = aTemp;
    }
    // Upsample in y-direction
    delete[] data_;
    int aDataSize = new_x_size*new_y_size;
    data_ = new T[aDataSize];
    if (new_y_size > y_size_) {
        for (int i = 0; i < aDataSize; i++)
            data_[i] = 0.0;
        double factor = ((double)new_y_size)/y_size_;
        for (int x = 0; x < new_x_size; x++) {
            int i = x;
            int j = x;
            int aLastI = y_size_*new_x_size;
            int aLastJ = new_y_size*new_x_size;
            double rest = factor;
            double part = 1.0;
            do {
                if (rest > 1.0) {
                    data_[j] += part*aIntermedData[i];
                    rest -= part;
                    part = 1.0;
                    j += new_x_size;
                    if (rest <= 0.0) {
                        rest = factor;
                        i += new_x_size;
                    }
                }
                else {
                    data_[j] += rest*aIntermedData[i];
                    part = 1.0-rest;
                    rest = factor;
                    i += new_x_size;
                }
            }
            while (i < aLastI && j < aLastJ);
        }
    }
    else {
        T* aTemp = data_;
        data_ = aIntermedData;
        aIntermedData = aTemp;
    }
    // Adapt size of matrix
    x_size_ = new_x_size;
    y_size_ = new_y_size;
    delete[] aIntermedData;
}

template <class T> void Matrix<T>::DownSample(int new_x_size, int new_y_size)
{
    // Downsample in x-direction
    int aIntermedSize = new_x_size*y_size_;
    T* aIntermedData = new T[aIntermedSize];
    if (new_x_size < x_size_) {
        for (int i = 0; i < aIntermedSize; i++)
            aIntermedData[i] = 0.0;
        T factor = ((double)x_size_)/new_x_size;
        for (int y = 0; y < y_size_; y++) {
            int aFineOffset = y*x_size_;
            int aCoarseOffset = y*new_x_size;
            int i = aFineOffset;
            int j = aCoarseOffset;
            int aLastI = aFineOffset+x_size_;
            int aLastJ = aCoarseOffset+new_x_size;
            T rest = factor;
            T part = 1.0;
            do {
                if (rest > 1.0) {
                    aIntermedData[j] += part*data_[i];
                    rest -= part;
                    part = 1.0;
                    i++;
                    if (rest <= 0.0) {
                        rest = factor;
                        j++;
                    }
                }
                else {
                    aIntermedData[j] += rest*data_[i];
                    part = 1.0-rest;
                    rest = factor;
                    j++;
                }
            }
            while (i < aLastI && j < aLastJ);
        }
    }
    else {
        T* aTemp = aIntermedData;
        aIntermedData = data_;
        data_ = aTemp;
    }
    // Downsample in y-direction
    delete[] data_;
    int aDataSize = new_x_size*new_y_size;
    data_ = new T[aDataSize];
    if (new_y_size < y_size_) {
        for (int i = 0; i < aDataSize; i++)
            data_[i] = 0.0;
        double factor = ((double)y_size_)/new_y_size;
        for (int x = 0; x < new_x_size; x++) {
            int i = x;
            int j = x;
            int aLastI = y_size_*new_x_size+x;
            int aLastJ = new_y_size*new_x_size+x;
            double rest = factor;
            double part = 1.0;
            do {
                if (rest > 1.0) {
                    data_[j] += part*aIntermedData[i];
                    rest -= part;
                    part = 1.0;
                    i += new_x_size;
                    if (rest <= 0.0) {
                        rest = factor;
                        j += new_x_size;
                    }
                }
                else {
                    data_[j] += rest*aIntermedData[i];
                    part = 1.0-rest;
                    rest = factor;
                    j += new_x_size;
                }
            }
            while (i < aLastI && j < aLastJ);
        }
    }
    else {
        T* aTemp = data_;
        data_ = aIntermedData;
        aIntermedData = aTemp;
    }
    // Normalize
    double aNormalization = ((double)aDataSize)/(x_size_*y_size_);
    for (int i = 0; i < aDataSize; i++)
        data_[i] *= aNormalization;
    // Adapt size of matrix
    x_size_ = new_x_size;
    y_size_ = new_y_size;
    delete[] aIntermedData;
}


//***************************************************************
// Sum along axes
//***************************************************************

template <class T> void Matrix<T>::sumAlongAxisX(Vector<T>& resultO) {

    resultO.setSize(x_size_, 0.0);

    for (int i = 0; i < x_size_; i++) {
        for (int j = 0; j < y_size_; j++) {
            resultO(i) += operator ()(i, j);
        }
    }
}


template <class T> void Matrix<T>::ReadFromBinaryFile(const char* filename)
{
    fstream stream(filename, ios::in |  std::ios::binary);
    stream.read((char*) this->data(), x_size_*y_size_*sizeof(T));
    stream.close();
}

template <class T> void Matrix<T>::getColumn(const int colNumber, Vector<T>& target)
{
    assert(colNumber < x_size_ && colNumber >= 0);

    target.setSize(y_size_);
    for(int i = 0; i < y_size_; i++)
    {
        target(i) = operator ()(colNumber, i);
    }
}

template <class T> Vector<T> Matrix<T>::getColumn(const int colNumber)
{
    assert(colNumber < x_size_ && colNumber >= 0);

    Vector<T> target(y_size_);

    for(int i = 0; i < y_size_; i++)
    {
        target(i) = operator ()(colNumber, i);
    }
    return target;
}

template <class T> void Matrix<T>::getRow(const int rowNumber, Vector<T>& target)
{
    assert(rowNumber < y_size_ && rowNumber >= 0);

    target.setSize(x_size_);
    for(int i = 0; i < x_size_; i++)
    {
        target(i) = operator ()(i, rowNumber);
    }
}

template <class T> Vector<T> Matrix<T>::getRow(const int rowNumber)
{
    assert(rowNumber < y_size_ && rowNumber >= 0);

    Vector<T> target(x_size_);
    for(int i = 0; i < x_size_; i++)
    {
        target(i) = operator ()(i, rowNumber);
    }

    return target;
}

template <class T> void Matrix<T>::swap()
{
    T* newData = new T[x_size_*y_size_];

    for(int i = 0; i < y_size_; i++)
    {
        for(int j = 0; j < x_size_; j++)
        {
            newData[i*x_size_ + j] = data_[((y_size_-1)-i)*x_size_ + j];
        }
    }

    delete [] data_;
    data_ = newData;
}

template <class T> void Matrix<T>::cutPart(int startC, int endC, int startR, int endR, Matrix<T>& target)
{
    assert (startC <= endC && startR <= endR && startC  >= 0 && endC >= 0 && startR >= 0 && endR >= 0 && endC < x_size_ && endR < y_size_);

    int x = endC-startC + 1;
    int y = endR-startR + 1;

    target.set_size(x, y);

    for (int i = startC; i < endC+1; i++)
    {
        for(int j = startR; j < endR+1; j++)
        {
            target(i-startC, j-startR) = data_[j*x_size_ + i];
        }
    }
}

template <class T> void Matrix<T>::insertRow(Vector<T>& src, const int rowNumber)
{
    assert(rowNumber < y_size_ && rowNumber >= 0 && src.getSize() == x_size_);

    for(int i = 0; i < x_size_; i++)
    {
        data_[rowNumber*x_size_ + i] = src[i];
    }
}

template <class T> void  Matrix<T>::transferMatToVecVec(Vector<Vector<T> >& res)
{
    Vector<T> help;
    res.clearContent();
    for(int i = 0; i < y_size_; i++)
    {
        help.clearContent();
        for(int j = 0; j < x_size_; j++)
        {
            help.pushBack(this->operator()(j,i));
        }
        res.pushBack(help);
    }
}

//*******************************************************
// Set all the matrix entries to a constant value
//*******************************************************
template <class T> void Matrix<T>::fill(const T &fillvalue)
{
    std::fill(data_, data_+(x_size_*y_size_), fillvalue);
}

template <class T> T maxOfMatrix(const Matrix<T>& aMatrix, int& xPos, int& yPos)
{

    T max = aMatrix(0,0);
    xPos = 0;
    yPos = 0;
    // start with max = first element

    for(int i = 0; i<aMatrix.x_size(); i++)
    {
        for(int j = 0; j < aMatrix.y_size();j++)
        {
            if(aMatrix(i,j) > max)
            {
                max = aMatrix(i,j);
                xPos = i;
                yPos = j;
            }
        }
    }
    return max;                // return highest value in array
}

template <class T> void Matrix<T>::transferVecVecToMat(Vector<Vector<T> >& objO)
{
    int whSize = objO.getSize();
    y_size_ = objO.getSize();

    if(y_size_ == 0) return;
    x_size_ = objO(0).getSize();


    for(int i = 0; i < objO.getSize(); i++)
    {
        whSize +=objO(i).getSize();
    }

    T* newData = new T[whSize];

    for(int i = 0; i < y_size_; i++)
    {
        for(int j = 0; j < x_size_; j++)
        {
            newData[i*x_size_ + j] = objO(i)(j);
        }
    }
    delete [] data_;
    data_ = newData;
}



#endif	/* _DENNIS_MATRIX_H */

