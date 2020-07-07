//
// Created by m007 on 27/06/2020.
//

#ifndef UNTITLED_LINK_H
#define UNTITLED_LINK_H
#include "Vector.h"
#include "Matrix.h"

class Link {
public:
    enum Links_Type {REVOLUTION, LINEAR};
    Link();
    float ReturnLenght();

//private:
    friend class Robot;
    Matrix<float> Inertia_tensor;
    Vector<float> Cent_Gravity;
    enum Links_Type Type;
    float theta;
    float alpha;
    float d;
    float a;
    Matrix<float> A0;

    float Lenght;
    float Mass;
    float Tau;

    Vector<float> w;
    Vector<float> wd;
    Vector<float> vd;
    Vector<float> acc;
    Vector<float> f;
    Vector<float> M;


};


#endif //UNTITLED_LINK_H
