//
// Created by m007 on 26/06/2020.
//

#ifndef UNTITLED_MATRIX_H
#define UNTITLED_MATRIX_H


#ifndef AA_MATRIX_H
#define AA_MATRIX_H
#include <iostream>
#include "Vector.h"
using namespace std;


template<class T>
class Matrix{
protected:
    int rows;
    int columns;
    static int objectCount;
    T **n;

public:
    Matrix();
    Matrix(int rows,int columns);
    ~Matrix();

    void PrintResult();
    void Reconfigurate(int rows,int columns);
    T Determinant();
    Matrix<T> Cofactor();
    Matrix<T> Inverse();
    Matrix<T> PseudoInverse();
    Matrix<T> Transpose();

    Matrix& operator=(const Matrix &other );
    Matrix& operator=(const T param[][2]);
    Matrix& operator=(const T param[][3]);
    Matrix& operator=(const T param[][4]);
    Matrix operator+(const Matrix &other);
    Matrix operator*(const Matrix& other);
    Vector<T> operator*(Vector<T>& other);
    T& operator()(int row, int col);


};

#endif //AA_MATRIX_H

#endif //UNTITLED_MATRIX_H
