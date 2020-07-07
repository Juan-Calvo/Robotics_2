//
// Created by m007 on 28/06/2020.
//

#ifndef UNTITLED_MOTOR_H
#define UNTITLED_MOTOR_H
#include "Vector.h"

class Motor {
    float Degrees_Kinematics;
private:
    friend class Robot;
    Vector<float> pcg;
    float Contrains_v;
    float Contrains_a;
    float Contrains_j;
   // Sensor* _Sensor;
   // Control* _Control;
public:
    Motor();
};


#endif //UNTITLED_MOTOR_H
