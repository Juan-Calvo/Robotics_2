#include "Matrix.h"

template<class T>
Matrix<T>::Matrix(){
        this->rows = 3;
        this->columns = 3;
        n = new T *[this->rows];
        for (int i = 0; i <this->rows ; i++){
            n[i] = new T[this->columns];
            for (int j = 0; j < this->columns; j++){
                n[i][j] = 0;
            }
        }
}

template<class T>
Matrix<T>::Matrix(int rows,int columns){
        objectCount++;
        this->rows = rows;
        this->columns = columns;
        n = new T *[this->rows];
        for (int i = 0; i <this->rows ; i++){
            n[i] = new T[this->columns];
            for (int j = 0; j < this->columns; j++){
                n[i][j] = 0;
            }
        }
}

template<class T>
Matrix<T>::~Matrix() {
    for(int i = 0; i < this->rows; ++i) {
        delete[] n[i];
    }
    //Free the array of pointers
    delete[] n;
}

template<class T>
T Matrix<T>::Determinant(){
    T det = 0;
    if(this->rows == this->columns) {
        switch (this->rows) {
            case 2: {
                det = n[0][0] * n[1][1] - n[0][1] * n[1][0];
                break;
            }
            case 3: {
                T a = n[0][0];
                T b = n[0][1];
                T c = n[0][2];
                T d = n[1][0];
                T e = n[1][1];
                T f = n[1][2];
                T g = n[2][0];
                T h = n[2][1];
                T i = n[2][2];
                det = (a * e * i + b * f * g + c * d * h);
                det = det - a * f * h;
                det = det - b * d * i;
                det = det - c * e * g;
                break;
            }
            case 4: {
                Matrix<T> *temp[4];
                for (int i = 0; i < 4; i++)
                    temp[i] = new Matrix<T>(3, 3);
                for (int k = 0; k < 4; k++) {
                    for (int i = 1; i < 4; i++) {
                        int j1 = 0;
                        for (int j = 0; j < 4; j++) {
                            if (k == j)
                                continue;
                            temp[k]->n[i - 1][j1++] = this->n[i][j];
                        }
                    }
                }
                det = this->n[0][0] * temp[0]->Determinant() - this->n[0][1] * temp[1]->Determinant() +
                      this->n[0][2] * temp[2]->Determinant()
                      - this->n[0][3] * temp[3]->Determinant();
                break;
            }
            case 5: {
                Matrix<T> *temp[5];
                for (int i = 0; i < 5; i++)
                    temp[i] = new Matrix<T>(4, 4);
                for (int k = 0; k < 5; k++) {
                    for (int i = 1; i < 5; i++) {
                        int j1 = 0;
                        for (int j = 0; j < 5; j++) {
                            if (k == j)
                                continue;
                            temp[k]->n[i - 1][j1++] = this->n[i][j];
                        }
                    }
                }
                det = this->n[0][0] * temp[0]->Determinant() - this->n[0][1] * temp[1]->Determinant()
                      + this->n[0][2] * temp[2]->Determinant() - this->n[0][3] * temp[3]->Determinant()
                      + this->n[0][4] * temp[4]->Determinant();
                break;
            }
            case 6:
            case 7:
            case 8:
            case 9:
            case 10:
            case 11:
            case 12:
            default: {
                Matrix **temp = new Matrix *[rows];
                for (int i = 0; i < rows; i++)
                    temp[i] = new Matrix(rows - 1, rows - 1);
                for (int k = 0; k < rows; k++) {
                    for (int i = 1; i < rows; i++) {
                        int j1 = 0;
                        for (int j = 0; j < rows; j++) {
                            if (k == j)
                                continue;
                            temp[k]->n[i - 1][j1++] = this->n[i][j];
                        }
                    }
                }
                for (int k = 0; k < rows; k++) {
                    if ((k % 2) == 0) {
                        det = det + (this->n[0][k] * temp[k]->Determinant());
                    } else {
                        det = det - (this->n[0][k] * temp[k]->Determinant());
                    }
                }
                for (int i = 0; i < rows; i++)
                    delete temp[i];
                delete[] temp;
               break;
            }
        }
    }
    return det;
}

template<class T>
Matrix<T> Matrix<T>::Cofactor(){
    Matrix<T> cofactor(this->rows, this->columns);

    if (this->rows != this->columns)
        return cofactor;
    if (this->rows < 2)
        return cofactor;
    else if (this->rows == 2){
        for (int i = 0; i < this->rows; i++) {
            for (int j = 0; j < this->columns; j++){
                cofactor(i, j) =this->n[((this->rows-1)-i)][((this->columns-1)-j)];
                if((i+j)%2 != 0) {
                    cofactor(i, j) = -1*cofactor(i, j);
                }
            }
        }
        return cofactor;
    }
    else if (this->rows >= 3){
        int DIM = this->rows;
        Matrix<T> ***temp = new Matrix<T>**[DIM];
        for (int i = 0; i < DIM; i++)
            temp[i] = new Matrix<T>*[DIM];
        for (int i = 0; i < DIM; i++)
            for (int j = 0; j < DIM; j++)
                temp[i][j] = new Matrix<T>(DIM - 1, DIM - 1);
        for (int k1 = 0; k1 < DIM; k1++){
            for (int k2 = 0; k2 < DIM; k2++){
                int i1 = 0;
                for (int i = 0; i < DIM; i++){
                    int j1 = 0;
                    for (int j = 0; j < DIM; j++){
                        if (k1 == i || k2 == j)
                            continue;
                        temp[k1][k2]->n[i1][j1++]
                                = this->n[i][j];
                    }
                    if (k1 != i)
                        i1++;
                }
            }
        }
        bool flagPositive = true;
        for (int k1 = 0; k1 < DIM; k1++){
            flagPositive = ((k1 % 2) == 0);
            for (int k2 = 0; k2 < DIM; k2++){
                if (flagPositive == true){
                    cofactor.n[k1][k2]
                            = temp[k1][k2]->Determinant();
                    flagPositive = false;
                }
                else{
                    cofactor.n[k1][k2]
                            = -temp[k1][k2]->Determinant();
                    flagPositive = true;
                }
            }
        }
        for (int i = 0; i < DIM; i++)
            for (int j = 0; j < DIM; j++)
                delete temp[i][j];
        for (int i = 0; i < DIM; i++)
            delete[] temp[i];
        delete[] temp;
    }
    return cofactor;
}
template<class T>
Matrix<T> Matrix<T>::Inverse(){
    Matrix<T> cofactor(this->rows, this->columns);
    Matrix<T> inv(this->rows, this->columns);
    if (this->rows != this->columns)
        return inv;

    T det = this->Determinant();
    cofactor = this->Cofactor();
    if(det != 0) {
        for (int i = 0; i < this->rows; i++) {
            for (int j = 0; j < this->columns; j++) {
                inv.n[j][i] = cofactor.n[i][j] / det;
            }
        }
    }
    return inv;
}

template<class T>
Matrix<T>  Matrix<T>::PseudoInverse(){
    Matrix<T> trans(this->columns, this->rows);
    Matrix<T> A(this->rows, this->columns);
    A = *this;
    trans = Transpose();
    Matrix<T> result = Matrix<T>(trans.rows, trans.columns);
    if((trans*A).Determinant()!=0){
        result = ((trans * A).Inverse())*trans;
    }else if((A*trans).Determinant()!=0){
        result = trans *((A*trans).Inverse());
    }else{
        printf("Error Matrix PseudoInverse\n");
    }
    return result;

}

template<class T>
Matrix<T> Matrix<T>::Transpose(){
    Matrix<T> trans(this->columns, this->rows);
        for (int i = 0; i < this->columns; i++) {
            for (int j = 0; j < this->rows; j++) {
                trans.n[i][j] = this->n[j][i];
            }
        }

    return trans;
}
template<class T>
void Matrix<T>::PrintResult(){
    for (int x = 0; x<this->rows; x++) {
        for (int y = 0; y < this->columns; y++) {
                printf(" %f ", n[x][y]);
        }
        printf("\n");
    }
    printf("\n");
}
template<class T>
void  Matrix<T>::Reconfigurate(int rows,int columns){
    Matrix<T>::~Matrix();
    this->rows = rows;
    this->columns = columns;
    n = new T *[this->rows];
    for (int i = 0; i <this->rows ; i++){
        n[i] = new T[this->columns];
        for (int j = 0; j < this->columns; j++){
            n[i][j] = 0;
        }
    }
}
template<class T>
Matrix<T>&  Matrix<T>::operator=(const Matrix &other ){
        if(this->columns<=other.columns && this->rows<=other.rows){
            for(int i=0;i<this->rows;i++){
                for(int j=0;j<this->columns;j++){
                    this->n[i][j]=other.n[i][j];
                    }
                }
            return (*this);
        }else{
            printf("ERROR: Matrix A = Matrix B. B has more dimensions than A \n");
        }
}

template<class T>
Matrix<T>& Matrix<T>::operator=(const T param[][2]){
    for(int i=0;i<this->rows;i++){
        for(int j=0;j<this->columns;j++){
            this->n[i][j]=param[i][j];
        }
    }
    return (*this);
}

template<class T>
Matrix<T>& Matrix<T>::operator=(const T param[][3]){
    for(int i=0;i<this->rows;i++){
        for(int j=0;j<this->columns;j++){
            this->n[i][j]=param[i][j];
        }
    }
    return (*this);
}

template<class T>
Matrix<T>& Matrix<T>::operator=(const T param[][4]){
    for(int i=0;i<this->rows;i++){
        for(int j=0;j<this->columns;j++){
            this->n[i][j]=param[i][j];
        }
    }
    return (*this);
}


template<class T>
Matrix<T>  Matrix<T>::operator+(const Matrix &other){
    Matrix<T> result = Matrix<T>(rows, other.columns);
    if (this->rows == other.columns && this->columns == other.rows) {
        for (int i = 0; i < this->rows; i++) {
            for (int j = 0; j < this->columns; j++) {
                result.n[i][j] = other.n[i][j] + this->n[i][j];
            }
        }
    }
        return result;
    }

template<class T>
Matrix<T>  Matrix<T>::operator*(const Matrix& other) {
        Matrix<T> result = Matrix<T>(this->rows, other.columns);
        if (this->columns == other.rows) {
            for (int i = 0; i < this->rows; i++) {
                for (int j = 0; j < this->columns; j++) {
                    for (int k = 0; k < this->columns; k++) {
                        result.n[i][j] += this->n[i][k]*other.n[k][j];
                    }
                }
            }
        }
        return result;
    }
template<class T>
Vector<T> Matrix<T>::operator*(Vector<T>& other){
    Vector<T> result = Vector<T>(other.getDimension());
    for (int x=0;x<other.getDimension();x++){
        result(x)= (this->n[x][0]*other(0)+this->n[x][1]*other(1)+this->n[x][2]*other(2));
        if( (result (x)>-0.01)&&(result (x))<0.01){
            result (x) = 0;
        }
    }
    return result;

}
template<class T>
T& Matrix<T>::operator()(int row, int col) {
        return n[row][col];
}

template<class T>
int Matrix<T>::objectCount = 0;

template class Matrix<float>;
template class Matrix<int>;
template class Vector<float>;
template class Vector<int>;