//
// Created by m007 on 29/06/2020.
//

#ifndef UNTITLED_TRANSFORMATIONMATRIX_H
#define UNTITLED_TRANSFORMATIONMATRIX_H
#include "Matrix.h"
#include "cmath"

class TransformationMatrix : public Matrix<float> {
private:
    void Clone(Matrix<float> Mat);
    Matrix<float> RotXInner(float thetaX);
    Matrix<float> RotYInner(float thetaY);
    Matrix<float> RotZInner(float thetaZ);

public:
    TransformationMatrix();
    Matrix<float>& RotX(float thetaX);
    Matrix<float>& RotY(float thetaY);
    Matrix<float>& RotZ(float thetaZ);

    //Euler angles
    Matrix<float>& EulerXZX(float thetaX1,float thetaZ2,float thetaX3);
    Matrix<float>& EulerXYX(float thetaX1,float thetaY2,float thetaX3);
    Matrix<float>& EulerYXY(float thetaY1,float thetaX2,float thetaY3);
    Matrix<float>& EulerYZY(float thetaY1,float thetaZ2,float thetaY3);
    Matrix<float>& EulerZYZ(float thetaZ1,float thetaY2,float thetaZ3);
    Matrix<float>& EulerZXZ(float thetaZ1,float thetaX2,float thetaZ3);


    //Tait–Bryan angles
    Matrix<float>& EulerXZY(float thetaX1,float thetaZ2,float thetaY3);
    Matrix<float>& EulerXYZ(float thetaX1,float thetaY2,float thetaZ3);
    Matrix<float>& EulerYXZ(float thetaY1,float thetaX2,float thetaZ3);
    Matrix<float>& EulerYZX(float thetaY1,float thetaZ2,float thetaX3);
    Matrix<float>& EulerZYX(float thetaZ1,float thetaY2,float thetaX3);
    Matrix<float>& EulerZXY(float thetaZ1,float thetaX2,float thetaY3);

    Matrix<float>& Translation(float dx, float dy, float dz);

};


#endif //UNTITLED_TRANSFORMATIONMATRIX_H
