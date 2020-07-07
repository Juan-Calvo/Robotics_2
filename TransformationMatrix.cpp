//
// Created by m007 on 29/06/2020.
//

#include "TransformationMatrix.h"
void TransformationMatrix::Clone(Matrix<float> Mat){
    for (int i = 0; i <this->rows ; i++){
        for (int j = 0; j < this->columns; j++){
            this->n[i][j] = Mat(i,j);
        }
    }
}
TransformationMatrix::TransformationMatrix():Matrix<float>(4,4){}

Matrix<float>& TransformationMatrix::RotX(float thetaX){
    float STheta = sinf(thetaX);
    float CTheta = cosf( thetaX);
    for(int x=0;x<4;x++){
        for(int y=0; y<4; y++){
            if ((x == 0 && y == 0) || (x == 3  && y == 3) ) {
                this->n[x][y]=1;
            } else if (x == 1 && y == 1) {
                this->n[x][y]=CTheta;
            } else if (x == 2 && y == 1) {
                this->n[x][y]=STheta;
            } else if (x == 1 && y == 2) {
                this->n[x][y]=-STheta;
            } else if (x == 2 && y == 2) {
                this->n[x][y]=CTheta;
            }else{
                this->n[x][y]=0;
            }
            if( (this->n[x][y]>-0.000001F)&&(this->n[x][y]<0.001F) ){
                this->n[x][y]=0;
            }
        }
    }
    return *this;
}
Matrix<float>& TransformationMatrix::RotY(float thetaY){
    float STheta = sinf(thetaY);
    float CTheta = cosf( thetaY);
    for(int x=0;x<4;x++){
        for(int y=0; y<4; y++){
            if ((x == 1 && y == 1) || (x == 3  && y == 3) ) {
                this->n[x][y]=1;
            } else if (x == 0 && y == 0) {
                this->n[x][y]=CTheta;
            } else if (x == 0 && y == 2) {
                this->n[x][y]=STheta;
            } else if (x == 2 && y == 0) {
                this->n[x][y]=-STheta;
            } else if (x == 2 && y == 2) {
                this->n[x][y]=CTheta;
            }else{
                this->n[x][y]=0;
            }
            if( (this->n[x][y]>-0.000001F)&&(this->n[x][y]<0.001F) ){
                this->n [x][y]=0;
            }
        }
    }
    return *this;
}
Matrix<float>& TransformationMatrix::RotZ(float thetaZ){
    float STheta = sinf(thetaZ);
    float CTheta = cosf( thetaZ);
    for(int x=0;x<4;x++){
        for(int y=0; y<4; y++){
            if ((x == 2 && y == 2) || (x == 3  && y == 3) ) {
                this->n[x][y]=1;
            } else if (x == 0 && y == 0) {
                this->n[x][y]=CTheta;
            } else if (x == 0 && y == 1) {
                this->n[x][y]=-STheta;
            } else if (x == 1 && y == 0) {
                this->n[x][y]=STheta;
            } else if (x == 1 && y == 1) {
                this->n[x][y]=CTheta;
            }else{
                this->n[x][y]=0;
            }
            if( (this->n[x][y]>-0.000001F)&&(this->n[x][y]<0.001F) ){
                this->n [x][y]=0;
            }
        }
    }
    return *this;
}


Matrix<float> TransformationMatrix::RotXInner(float thetaX){
    Matrix<float> Mat(4,4);
    float STheta = sinf(thetaX);
    float CTheta = cosf( thetaX);
    for(int x=0;x<4;x++){
        for(int y=0; y<4; y++){
            if ((x == 0 && y == 0) || (x == 3  && y == 3) ) {
                Mat(x,y)=1;
            } else if (x == 1 && y == 1) {
                Mat(x,y)=CTheta;
            } else if (x == 2 && y == 1) {
                Mat(x,y)=STheta;
            } else if (x == 1 && y == 2) {
                Mat(x,y)=-STheta;
            } else if (x == 2 && y == 2) {
                Mat(x,y)=CTheta;
            }else{
                Mat(x,y)=0;
            }
            if( (Mat(x,y)>-0.000001F)&&(Mat(x,y)<0.001F) ){
                Mat(x,y)=0;
            }
        }
    }
    return Mat;
}
Matrix<float> TransformationMatrix::RotYInner(float thetaY){
    Matrix<float> Mat(4,4);
    float STheta = sinf(thetaY);
    float CTheta = cosf( thetaY);
    for(int x=0;x<4;x++){
        for(int y=0; y<4; y++){
            if ((x == 1 && y == 1) || (x == 3  && y == 3) ) {
                Mat(x,y)=1;
            } else if (x == 0 && y == 0) {
                Mat(x,y)=CTheta;
            } else if (x == 0 && y == 2) {
                Mat(x,y)=STheta;
            } else if (x == 2 && y == 0) {
                Mat(x,y)=-STheta;
            } else if (x == 2 && y == 2) {
                Mat(x,y)=CTheta;
            }else{
                Mat(x,y)=0;
            }
            if( (Mat(x,y)>-0.000001F)&&(Mat(x,y)<0.001F) ){
                this->n [x][y]=0;
            }
        }
    }
    return Mat;
}
Matrix<float> TransformationMatrix::RotZInner(float thetaZ){
    Matrix<float> Mat(4,4);
    float STheta = sinf(thetaZ);
    float CTheta = cosf( thetaZ);
    for(int x=0;x<4;x++){
        for(int y=0; y<4; y++){
            if ((x == 2 && y == 2) || (x == 3  && y == 3) ) {
                Mat(x,y)=1;
            } else if (x == 0 && y == 0) {
                Mat(x,y)=CTheta;
            } else if (x == 0 && y == 1) {
                Mat(x,y)=-STheta;
            } else if (x == 1 && y == 0) {
                Mat(x,y)=STheta;
            } else if (x == 1 && y == 1) {
                Mat(x,y)=CTheta;
            }else{
                Mat(x,y)=0;
            }
            if( (Mat(x,y)>-0.000001F)&&(Mat(x,y)<0.001F) ){
                Mat(x,y)=0;
            }
        }
    }
    return Mat;
}

Matrix<float>& TransformationMatrix::EulerXZX(float thetaX1,float thetaZ2,float thetaX3){
    Clone(RotXInner(thetaX1)*RotZInner(thetaZ2)*RotXInner(thetaX3));
    return *this;
}
Matrix<float>& TransformationMatrix::EulerXYX(float thetaX1,float thetaY2,float thetaX3){
    Clone(RotZInner(thetaX1)*RotXInner(thetaY2)*RotZInner(thetaX3));
    return *this;
}
Matrix<float>& TransformationMatrix::EulerYXY(float thetaY1,float thetaX2,float thetaY3){
    Clone (RotZInner(thetaY1)*RotXInner(thetaX2)*RotZInner(thetaY3));
    return *this;
}
Matrix<float>& TransformationMatrix::EulerYZY(float thetaY1,float thetaZ2,float thetaY3){
    Clone (RotZInner(thetaY1)*RotXInner(thetaZ2)*RotZInner(thetaY3));
    return *this;
}
Matrix<float>& TransformationMatrix::EulerZYZ(float thetaZ1,float thetaY2,float thetaZ3){
    Clone (RotZInner(thetaZ1)*RotXInner(thetaY2)*RotZInner(thetaZ3));
    return *this;
}
Matrix<float>& TransformationMatrix::EulerZXZ(float thetaZ1,float thetaX2,float thetaZ3){
    Clone (RotZInner(thetaZ1)*RotXInner(thetaX2)*RotZInner(thetaZ3));
    return *this;
}

Matrix<float>& TransformationMatrix::EulerXZY(float thetaX1,float thetaZ2,float thetaY3){
    Clone (RotZInner(thetaX1)*RotXInner(thetaZ2)*RotZInner(thetaY3));
    return *this;
}
Matrix<float>& TransformationMatrix::EulerXYZ(float thetaX1,float thetaY2,float thetaZ3){
    Clone (RotZInner(thetaX1)*RotXInner(thetaY2)*RotZInner(thetaZ3));
    return *this;
}
Matrix<float>& TransformationMatrix::EulerYXZ(float thetaY1,float thetaX2,float thetaZ3){
    Clone (RotZInner(thetaY1)*RotXInner(thetaX2)*RotZInner(thetaZ3));
    return *this;
}
Matrix<float>& TransformationMatrix::EulerYZX(float thetaY1,float thetaZ2,float thetaX3){
    Clone (RotZInner(thetaY1)*RotXInner(thetaZ2)*RotZInner(thetaX3));
}
Matrix<float>& TransformationMatrix::EulerZYX(float thetaZ1,float thetaY2,float thetaX3){
    Clone (RotZInner(thetaZ1)*RotXInner(thetaY2)*RotZInner(thetaX3));
    return *this;
}
Matrix<float>& TransformationMatrix::EulerZXY(float thetaZ1,float thetaX2,float thetaY3){
    Clone (RotZInner(thetaZ1)*RotXInner(thetaX2)*RotZInner(thetaY3));
    return *this;
}


Matrix<float>& TransformationMatrix::Translation(float dx, float dy, float dz){
    for(int x=0;x<4;x++){
        for(int y=0; y<4; y++){
            if ( (x == 0 && y == 0) || (x == 1 && y == 1) || (x == 2 && y == 2)  || (x == 3  && y == 3) ) {
                this->n[x][y]=1;
            } else if (x == 0 && y == 3) {
                this->n[x][y]=dx;
            } else if (x == 1 && y == 3) {
                this->n[x][y]=dy;
            } else if (x == 2 && y == 3) {
                this->n[x][y]=dz;
            }else{
                this->n[x][y]=0;
            }
        }
    }
    return *this;
}


