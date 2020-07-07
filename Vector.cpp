//
// Created by m007 on 26/06/2020.
//

#include "Vector.h"
template<class T>
Vector<T>::Vector() {
    objectCount++;
    this->dimension = 3;
    n = new T [this->dimension];
    for (int i = 0; i <this->dimension ; i++){
        n[i] = 0;
    }
}

template<class T>
Vector<T>::Vector(int dimension) {
    objectCount++;
    this->dimension = dimension;
    n = new T [this->dimension];
    for (int i = 0; i <this->dimension ; i++){
        n[i] = 0;
    }
}

template<class T>
Vector<T>::~Vector(){
    delete [] n;
}

template<class T>
int Vector<T>::getDimension(){
    return dimension;
}

template<class T>
void Vector<T>::PrintResult(){
    for (int x = 0; x<this->dimension; x++) {
        printf(" %f ", n[x]);
    }
    printf("\n");
}


template<class T>
Vector<T> Vector<T>::crossProduct(Vector<T> const& VectorB){
    Vector<T> result(3);
    if(this->dimension == 3) {
        result(0) = (n[1] * VectorB.n[2] - n[2] * VectorB.n[1]);
        result(1) = -(n[0] * VectorB.n[2] - n[2] * VectorB.n[0]);
        result(2) = (n[0] * VectorB.n[1] - n[1] * VectorB.n[0]);
        for (int x = 0; x < 3; x++) {
            if (( result(x) > -0.000001) && (result(x)) < 0.000001) {
                result(x) = 0;
            }
        }
    }
    return result;
}




template<class T>
Vector<T>&  Vector<T>::operator=(const Vector &other ){
    if(this->dimension==other.dimension){
        for(int i=0;i<this->dimension;i++){
            this->n[i] = other.n[i];
        }
        return (*this);
    }else{
        printf("ERROR: Vector A = Vector B. B has more dimensions than A");
    }
}


template<class T>
Vector<T>& Vector<T>::operator=(T param[]){
    for(int i=0;i<this->dimension;i++){
        this->n[i]=param[i];
    }
    return (*this);

}
template<class T>
Vector<T>  Vector<T>::operator+(Vector<T> other){
    Vector<T> result(this->dimension);
    if(this->dimension == other.dimension) {
        for (int i = 0; i < this->dimension; i++) {
            result.n[i] = other.n[i] + this->n[i];
        }

        return result;
    }else{
        printf("ERROR: Vector A = Vector B. B has more dimensions than Am");
    }
}

template<class T>
Vector<T>  Vector<T>::operator-(Vector<T>& other){
    Vector<T> result(this->dimension);
    if(this->dimension== other.dimension) {
        for (int i = 0; i < this->dimension; i++) {
            result.n[i] = other.n[i] - this->n[i];
        }
        return result;
    }else{
        printf("ERROR: Vector A = Vector B. B has more dimensions than A");
    }
}

template<class T>
T Vector<T>::operator*(Vector<T>& other) {
    T ans=0;
    if(this->dimension == other.dimension) {
        for (int i = 0; i < this->dimension; i++) {
            ans = ans + (other.n[i] * this->n[i]);
        }
        return ans;
    }else{
        printf("ERROR: Vector A = Vector B. B has more dimensions than A");
    }
}

template<class T>
Vector<T>  Vector<T>::operator*(T factor){
    Vector<T> result(this->dimension);
    for (int i = 0; i < this->dimension; i++) {
        result.n[i] = factor * this->n[i];
    }
    return result;
}

template<class T>
T& Vector<T>::operator()(int dim){
    return n[dim];
}

template<class T>
int Vector<T>::objectCount = 0;


Vector<float> zeros(int num){
    Vector<float> result(num);
    return result;
}
//template<class T>
Vector<float> crossProduct(Vector<float> VectorA,Vector<float> VectorB){
    Vector<float> result(3);

    //if(VectorA->dimension == 3) {
    result(0) = (VectorA(1) * VectorB(2) - VectorA(2) * VectorB(1));
    result(1) = -(VectorA(0) * VectorB(2) - VectorA(2) * VectorB(0));
    result(2) = (VectorA(0) * VectorB(1) - VectorA(1) * VectorB(0));
    for (int x = 0; x < 3; x++) {
        if (( result(x) > -0.000001) && (result(x)) < 0.000001) {
            result(x) = 0;
        }
    }
    return result;
}

template class Vector<float>;
template class Vector<int>;