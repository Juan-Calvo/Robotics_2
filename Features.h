#ifndef UNTITLED_FEATURES_H
#define UNTITLED_FEATURES_H
    #define DOF 4
    int RobotDOF = 4;
//Links Link[5] ={REVOLUTION,REVOLUTION,REVOLUTION,REVOLUTION};//REVOLUTION
float Constraints[DOF*3] = {
        // Velocity     Acceleration      Jerk
            1,             2,              3,
            1,             2,              3,
            1,             2,              3,
            1,             2,              3
};
float dyn_mat [DOF][10]={
     //      Ix         Iy         Iz              Ixy        Ixz       Iyz   mx               my        mz     mass
        {0.0056F,    0.0150F,   0.0150F,         1,        3,          2,   -0.8200F,         0,          0,           2},
        {0.0028F,    0.0056F,   0.0056F,         0,         0,         0,   -0.1950F,         0,          0,           1},
        {0,          0,         0.0003F,         0,         0,         0,          0,         0,     0.0013F,    0.1000F},
        {1,         2,         3,               50,        60,         70,          80,         90,    -0.0019F,    0.0500F}
};
float  ToolMatrix[4][4]={
        {1,     2,      3,      10},
        {4,     5,      6,      11},
        {7,     8,      9,      12},
        {0,     0,      0,      1}
};
float  BaseMatrix[4][4]={
        {1,     2,      3,      10},
        {4,     5,      6,      11},
        {7,     8,      9,      12},
        {0,     0,      0,      1}
};

const float DH_Table[DOF*4]={
        /*	    theta		alpha		d			   a */
        0,          1.5708F,      0,           0,
        0,               0,       0,         0.3000F,
        0,               0,       0,         0.3260F,
        0.1000F,    1.5708F,      0,          0
};

const float MotorParam[DOF*5]={
        /*	KA		KM		Scale		Offset		MaxTorque*/
        0,		0,		0,			0,			0,
        0,		0,		0,			0,			0,
        0,		0,		0,			0,			0,
        0,		0,		0,			0,			0
};

const float ControlGain[DOF*3]={
        /*	KP			KD			KI*/
/*1*/		0,			0,			0,
/*2*/		0,			0,			0,
/*3*/		0,			0,			0,
/*4*/		0,			0,			0
};


const float LinkLenght[DOF]={2,2,2,2};


#endif //UNTITLED_FEATURES_H