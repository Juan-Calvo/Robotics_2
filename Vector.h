//
// Created by m007 on 26/06/2020.
//

#ifndef UNTITLED_VECTOR_H
#define UNTITLED_VECTOR_H
#include <iostream>
#include <cstdio>
using namespace std;

template<class T>
class Vector {
    private:
        int dimension;
        static int objectCount;
        T *n;
    public:
        Vector();
        Vector(int dimension);
        ~Vector();
    int getDimension();
    void PrintResult();
    Vector<T> crossProduct(Vector<T> const& VectorB);


    Vector& operator=(const Vector &other );

    Vector& operator=(T param[]);

    Vector operator+(Vector<T> other);
    Vector operator-(Vector<T>& other);
    T operator*(Vector<T>& other);
    Vector operator*(T factor);
    T& operator()(int dim);
};

Vector<float> zeros(int num);


Vector<float> crossProduct(Vector<float> VectorA,Vector<float> VectorB);
#endif //UNTITLED_VECTOR_H
