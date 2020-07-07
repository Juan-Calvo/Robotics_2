//
// Created by m007 on 26/06/2020.
//

#ifndef UNTITLED_ROBOT_H
#define UNTITLED_ROBOT_H
#include "Vector.h"
#include "Matrix.h"

#include "Link.h"
#include "Motor.h"
#include "End_Effector.h"
#include <cmath>

class Robot {

public:
    //enum Links_Type {REVOLUTION, LINEAR};
    //enum Trajectory_Types {TRAPEZOIDAL, SPLIN5, SPLIN7,SCURVE};
    Robot(float _ToolMatrix[4][4],float _BaseMatrix[4][4],const float DH_Table[], const float lenght[],float dyn_mat [][10],const float MotorParam[],const float ControlGain[],int RobotDOF,const Link::Links_Type link [], const float MotorContrains[]);
    Robot(Matrix<float>& _ToolMatrix,Matrix<float>& _BaseMatrix,const float DH_Table[], const float lenght[],float dyn_mat [][10],const float MotorParam[],const float ControlGain[],int RobotDOF,const Link::Links_Type link [], const float MotorContrains[]);
    ~Robot();

    Matrix <float> GetRotation(int num);
    Matrix <float> GetRotation(Matrix<float> mat);
    Vector <float> GetPosition(Matrix<float> vect);
    Link GetLink(int num);


    void ForwardKinematics(const float thetaOffset[]);
    friend void InverseKinematics(Robot ro,End_Effector* EFF_q0,float q[3], int conf);
    void Jacobian(float q[]);
    void EulerNewton(const float q[],const float qd[],const float qdd[]);
    void GravityTorque(float q[]);

private:
    Matrix <float> ToolMatrix;
    Matrix <float> BaseMatrix;
    //Trajectory _Trajectory;
    int _RobotDOF;
    Link *_Linker;
    Motor *_Motor;
};


#endif //UNTITLED_ROBOT_H
