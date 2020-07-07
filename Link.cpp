//
// Created by m007 on 27/06/2020.
//

#include "Link.h"
Link::Link() : A0(4,4){

    theta = 0;
    alpha = 0;
    d = 0;
    a = 0;


    Lenght = 0;
    Mass = 0;
    Tau = 0;
}
float Link::ReturnLenght(){
    return Lenght;
}