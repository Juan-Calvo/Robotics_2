//
// Created by m007 on 26/06/2020.
//

#include "Robot.h"
Robot::Robot(float _ToolMatrix[4][4],float _BaseMatrix[4][4],const float DH_Table[], const float lenght[],float dyn_mat [][10],const float MotorParam[],const float ControlGain[],int RobotDOF,const Link::Links_Type link [], const float MotorContrains[]):ToolMatrix(4,4),BaseMatrix(4,4){
    _RobotDOF = RobotDOF;
    ToolMatrix = _ToolMatrix;
    BaseMatrix = _BaseMatrix;

    _Linker = new Link[_RobotDOF];
    _Motor = new Motor[_RobotDOF];

    for(int i=0;i<_RobotDOF; i++){
/*
        _Robot->_Motor[i]._Control= new Control(ControlGain[i*3], ControlGain[i*3+1], ControlGain[i*3+2],
                                                MotorParam[i*5],MotorParam[i*5+1] ,20, MotorParam[i*5+2], MotorParam[i*5+3],MotorParam[i*5+4]);
        _Robot->_Motor[i]._Sensor= new Sensor(PinSensorA[i], PinSensorB[i],10.5F, 1,20, 30);*/

        _Motor->Contrains_v = MotorContrains[(i*3)];
        _Motor->Contrains_a = MotorContrains[(i*3)+1];
        _Motor->Contrains_j = MotorContrains[(i*3)+2];

         _Linker[i].Type = link[i];
        _Linker[i].Lenght = lenght[i];
         _Linker[i].theta = DH_Table[((i*4)+0)];
         _Linker[i].alpha = DH_Table[((i*4)+1)];
         _Linker[i].d = DH_Table[((i*4)+2)];
         _Linker[i].a = DH_Table[((i*4)+3)];

         _Linker[i].Inertia_tensor(0,0) = dyn_mat[i][1]+dyn_mat[i][2];
         _Linker[i].Inertia_tensor(0,1) = dyn_mat[i][3];
         _Linker[i].Inertia_tensor(0,2) = dyn_mat[i][4];

         _Linker[i].Inertia_tensor(1,0) = dyn_mat[i][3];
         _Linker[i].Inertia_tensor(1,1) = dyn_mat[i][0]+dyn_mat[i][2];
         _Linker[i].Inertia_tensor(1,2) = dyn_mat[i][5];

         _Linker[i].Inertia_tensor(2,0) = dyn_mat[i][4];
         _Linker[i].Inertia_tensor(2,1) = dyn_mat[i][5];
         _Linker[i].Inertia_tensor(2,2) = dyn_mat[i][1]+dyn_mat[i][0];

         _Linker[i].Cent_Gravity(0)= dyn_mat[i][6];
         _Linker[i].Cent_Gravity(1) = dyn_mat[i][7];
         _Linker[i].Cent_Gravity(2) = dyn_mat[i][8];

         _Linker[i].Mass = dyn_mat[i][9];

        for(int x=0; x<4; x++) {
            for (int y = 0; y < 4; y++) {
                 _Linker[i].A0(x,y)=0;
            }
        }
    }
}
Robot::Robot(Matrix<float>& _ToolMatrix,Matrix<float>& _BaseMatrix,const float DH_Table[], const float lenght[],float dyn_mat [][10],const float MotorParam[],const float ControlGain[],int RobotDOF,const Link::Links_Type link [], const float MotorContrains[]):ToolMatrix(4,4),BaseMatrix(4,4){
    _RobotDOF = RobotDOF;
    ToolMatrix = _ToolMatrix;
    BaseMatrix = _BaseMatrix;

    _Linker = new Link[_RobotDOF];
    _Motor = new Motor[_RobotDOF];

    for(int i=0;i<_RobotDOF; i++){
/*
        _Robot->_Motor[i]._Control= new Control(ControlGain[i*3], ControlGain[i*3+1], ControlGain[i*3+2],
                                                MotorParam[i*5],MotorParam[i*5+1] ,20, MotorParam[i*5+2], MotorParam[i*5+3],MotorParam[i*5+4]);
        _Robot->_Motor[i]._Sensor= new Sensor(PinSensorA[i], PinSensorB[i],10.5F, 1,20, 30);*/

        _Motor->Contrains_v = MotorContrains[(i*3)];
        _Motor->Contrains_a = MotorContrains[(i*3)+1];
        _Motor->Contrains_j = MotorContrains[(i*3)+2];

        _Linker[i].Type = link[i];
        _Linker[i].Lenght = lenght[i];
        _Linker[i].theta = DH_Table[((i*4)+0)];
        _Linker[i].alpha = DH_Table[((i*4)+1)];
        _Linker[i].d = DH_Table[((i*4)+2)];
        _Linker[i].a = DH_Table[((i*4)+3)];

        _Linker[i].Inertia_tensor(0,0) = dyn_mat[i][1]+dyn_mat[i][2];
        _Linker[i].Inertia_tensor(0,1) = dyn_mat[i][3];
        _Linker[i].Inertia_tensor(0,2) = dyn_mat[i][4];

        _Linker[i].Inertia_tensor(1,0) = dyn_mat[i][3];
        _Linker[i].Inertia_tensor(1,1) = dyn_mat[i][0]+dyn_mat[i][2];
        _Linker[i].Inertia_tensor(1,2) = dyn_mat[i][5];

        _Linker[i].Inertia_tensor(2,0) = dyn_mat[i][4];
        _Linker[i].Inertia_tensor(2,1) = dyn_mat[i][5];
        _Linker[i].Inertia_tensor(2,2) = dyn_mat[i][1]+dyn_mat[i][0];

        _Linker[i].Cent_Gravity(0)= dyn_mat[i][6];
        _Linker[i].Cent_Gravity(1) = dyn_mat[i][7];
        _Linker[i].Cent_Gravity(2) = dyn_mat[i][8];

        _Linker[i].Mass = dyn_mat[i][9];

        for(int x=0; x<4; x++) {
            for (int y = 0; y < 4; y++) {
                _Linker[i].A0(x,y)=0;
            }
        }
    }
}
Robot::~Robot(){
    delete[] _Linker;
    delete[] _Motor;
}

Matrix <float> Robot::GetRotation(int num){
    Matrix<float> result(3,3);
    result = _Linker[num].A0;
    return result;
}
Matrix <float> Robot::GetRotation(Matrix<float> mat){
    Matrix<float> result(3,3);
    result = mat;
    return result;
}
Vector <float>  Robot::GetPosition(Matrix<float> mat){
    Vector<float> result(3);
    for(int i=0; i<3;i++) {
        result(i) = mat(i,3);
    }
    return result;
}

Link Robot::GetLink(int num){
    return _Linker[num];
}


void Robot::ForwardKinematics(const float thetaOffset[]){
    float STheta,CTheta,SAlpha,CAlpha,theta,d;
    for (int i=0; i<_RobotDOF;i++) {
        if ( _Linker[i].Type == Link::REVOLUTION) {
            theta = thetaOffset[i];
            d =  _Linker[i].d;
        }else{
            d =  thetaOffset[i];
            theta =  _Linker[i].theta;
        }
        STheta = sinf(theta);
        CTheta = cosf(theta);
        SAlpha = sinf(  _Linker[i].alpha);
        CAlpha = cosf(  _Linker[i].alpha);
        if( (CAlpha>-0.000001F)&&(CAlpha<0.000001F)){
            CAlpha = 0;
        }
        if( (SAlpha>-0.000001F)&&(SAlpha<0.000001F)){
            SAlpha = 0;
        }
        if( (STheta>-0.000001F)&&(STheta<0.000001F)){
            STheta = 0;
        }
        if( (CTheta>-0.000001F)&&(CTheta<0.000001F)){
            CTheta = 0;
        }
        for (int x = 0; x < 4; x++) {
            for (int y = 0; y < 4; y++) {
                if (x == 0 && y == 0) {
                     _Linker[i] .A0 (x,y)  = CTheta;
                } else if (x == 0 && y == 1) {
                     _Linker[i] .A0  (x,y)  = -1 * CAlpha * STheta;
                } else if (x == 0 && y == 2) {
                     _Linker[i] .A0  (x,y)  = SAlpha * STheta;
                } else if (x == 0 && y == 3) {
                     _Linker[i] .A0  (x,y)  =  _Linker[i] .a * CTheta;
                } else if (x == 1 && y == 0) {
                     _Linker[i] .A0  (x,y)  = STheta;
                } else if (x == 1 && y == 1) {
                     _Linker[i] .A0  (x,y)  = CAlpha * CTheta;
                } else if (x == 1 && y == 2) {
                     _Linker[i] .A0  (x,y)  = -1 * SAlpha * CTheta;
                } else if (x == 1 && y == 3) {
                     _Linker[i] .A0  (x,y)  =  _Linker[i] .a * STheta;
                } else if (x == 2 && y == 1) {
                     _Linker[i] .A0  (x,y)  = SAlpha;
                } else if (x == 2 && y == 2) {
                     _Linker[i] .A0  (x,y)  = CAlpha;
                } else if (x == 2 && y == 3) {
                     _Linker[i] .A0  (x,y)  = d;
                } else if (x == 3 && y == 3) {
                     _Linker[i] .A0  (x,y)  = 1;
                }
                else{
                     _Linker[i] .A0  (x,y) =0;
                }
                if( ( _Linker[i] .A0  (x,y) >-0.000001F)&&( _Linker[i] .A0  (x,y) )<0.000001F){
                     _Linker[i] .A0  (x,y) =0;
                }

            }
        }
    }
}
void Robot::Jacobian(float q[]){
    ForwardKinematics(q);
    Matrix<float> A0(4,4);
    A0 = BaseMatrix;
    for(int i=0; i<_RobotDOF; i++){
        A0 = A0 *_Linker[i].A0;
    }
    struct Vector<float> p0_dof;
    p0_dof(0) = A0(0,3);
    p0_dof(1) = A0(1,3);
    p0_dof(2) = A0(2,3);

    Vector<float> p[_RobotDOF];
    Vector<float>  pA0;
    pA0(0) = BaseMatrix(0,3);
    pA0(1) = BaseMatrix(1,3);
    pA0(2) = BaseMatrix(2,3);

    p[0] = p0_dof - pA0;

    Vector<float> z[_RobotDOF];
    Vector<float> z0;
    z0(0) = BaseMatrix(0,2);
    z0(1) = BaseMatrix(1,2);
    z0(2) = BaseMatrix(2,2);

    z[0]=z0;


    Matrix<float> A01;
    A01 = BaseMatrix * _Linker[0].A0;
    Vector<float> Jv[_RobotDOF-1];
    Vector<float> Jw[_RobotDOF-1];

    if(_Linker[0].Type == Link::REVOLUTION){
        Jv[0] = z[0].crossProduct(p[0]);
        Jw[0] = z[0];
    }else{
        Jv[0] = z[0];
        Jw[0] = zeros(3);
    }
    for(int i=1;i<_RobotDOF;i++){
        pA0(0) = A01(0,3);
        pA0(1) = A01(1,3);
        pA0(2) = A01(2,3);

        z0(0) = A01(0,2);
        z0(1) = A01(1,2);
        z0(2) = A01(2,2);

        p[i] = p0_dof - pA0;
        z[i] = z0;

        if(_Linker[i].Type == Link::REVOLUTION){
            Jv[i] = z[i].crossProduct(p[i]);
            Jw[i] = z[i];
        }else{
            Jv[i] = z[i];
            Jw[i] = zeros(3);
        }

        A01 = A01 * _Linker[i].A0;
    }
}

void Robot::EulerNewton(const float q[],const float qd[],const float qdd[]){
    Matrix<float> Rtot(3,3);

    //Get inverse rotation
    ForwardKinematics(q);
    Matrix<float> R3_post_inv = GetRotation(0).Inverse();
    Rtot = GetRotation(0);

    //Vector join Si-1 and Si
    Vector<float> R_pi(3);

    R_pi(0) = _Linker[0].a;
    if (( R_pi(0) > -0.000001) && (R_pi(0)) < 0.000001) {
        R_pi(0) = 0;
    }

    R_pi(1) = _Linker[0].d*sinf(_Linker[0].alpha);
    if (( R_pi(1) > -0.000001) && (R_pi(1)) < 0.000001) {
        R_pi(1) = 0;
    }

    R_pi(2) = _Linker[0].d*cosf(_Linker[0].alpha);
    if (( R_pi(2) > -0.000001) && (R_pi(2)) < 0.000001) {
        R_pi(2) = 0;
    }

    //Center of gravity
    Vector<float> s;
    if(_Linker[0].Mass !=0 ) {
        s(0) = _Linker[0].Cent_Gravity(0) / _Linker[0].Mass;
        s(1) = _Linker[0].Cent_Gravity(1) / _Linker[0].Mass;
        s(2) = _Linker[0].Cent_Gravity(2) / _Linker[0].Mass;
    }else{
        s = zeros(3);
    }

    //
    float Z0[3] = {0,0,1};
    Vector<float> z0;
    z0 = Z0;

    float g0[3] = {0,9.81F,0};
    Vector<float> g;
    g = g0;

    Vector<float> w,wd,vd,a,z03qd,z03qdd;

    if (_Linker[0].Type == Link::REVOLUTION){
        z03qd = z0 * qd[0];
        z03qdd = z0 * qdd[0];
       w  =  R3_post_inv * z03qd;

       wd  =  R3_post_inv * z03qdd;

        vd = (wd.crossProduct(R_pi)) + w.crossProduct(w.crossProduct(R_pi)) + (R3_post_inv * g);
    }else{
        z03qdd = (z0 * qdd[0]) + g;
        w = zeros(3);
        wd = zeros(3);
        vd = R3_post_inv * z03qdd;
    }
    a = (wd.crossProduct(s)) + (w.crossProduct(w.crossProduct(s))) + (vd);

    _Linker[0].w =  w;
    _Linker[0].wd = wd;
    _Linker[0].vd = vd;
    _Linker[0].acc = a;

    Vector<float> aux;
    Vector<float> f_cdm;
    Vector<float> M_cdm;
    Vector<float> f;
    Vector<float> M;

    Matrix<float> R3_post;
    Matrix<float> R3_inv;
    Matrix<float> Rtot_inv;
    Matrix<float> R3_inv_ant;
    Vector<float> R_pi_post;
    Vector<float> A;

    float Tau;
    for (int i=1; i<_RobotDOF; i++){
        //Get inverse rotation
        Rtot = Rtot * GetRotation(i);

        R3_post_inv = GetRotation(i).Inverse();

        //Vector join Si-1 and Si
        R_pi(0) = _Linker[i].a;
        R_pi(1) = _Linker[i].d*sinf(_Linker[i].alpha);
        R_pi(2) = _Linker[i].d*cosf(_Linker[i].alpha);

        //Center of gravity
        s(0) = _Linker[i].Cent_Gravity(0)/ _Linker[i].Mass;
        s(1) = _Linker[i].Cent_Gravity(1)/ _Linker[i].Mass;
        s(2) = _Linker[i].Cent_Gravity(2)/ _Linker[i].Mass;

        z03qd = z0 * qd[i];
        z03qdd = z0 * qdd[i];

        if (_Linker[i].Type == Link::REVOLUTION){

            aux = _Linker[i-1].w + z03qd;
            w  = R3_post_inv * aux;
            aux = _Linker[i-1].wd + z03qdd;

            wd = R3_post_inv * aux + (_Linker[i-1].w.crossProduct(z03qd));
            vd = (wd.crossProduct(R_pi)) + (w.crossProduct(w.crossProduct(R_pi))) + (R3_post_inv * _Linker[i-1].vd);

        }else{

            w = R3_post_inv * _Linker[i-1].w;
            wd = R3_post_inv * _Linker[i-1].wd;
            aux = z03qdd + _Linker[i-1].vd;
            vd = R3_post_inv * aux + (w.crossProduct(R_pi)) + ((w * 2.0F).crossProduct(R3_post_inv*z0*qd[i]))
                 + (w.crossProduct(w.crossProduct(R_pi)));
        }

        a = (wd.crossProduct(s)) + (w.crossProduct(w.crossProduct(s))) + vd;
        _Linker[i].w = w;
        _Linker[i].wd = wd;
        _Linker[i].vd = vd;
        _Linker[i].acc = a;

    }

    //Vector join Si-1 and Si
    R_pi(0) = _Linker[_RobotDOF-1].a;
    R_pi(1) = _Linker[_RobotDOF-1].d*sinf(_Linker[_RobotDOF-1].alpha);
    R_pi(2) = _Linker[_RobotDOF-1].d*cosf(_Linker[_RobotDOF-1].alpha);

    //Center of gravity
    s(0) = _Linker[_RobotDOF-1].Cent_Gravity(0)/ _Linker[_RobotDOF-1].Mass;
    s(1) = _Linker[_RobotDOF-1].Cent_Gravity(1)/ _Linker[_RobotDOF-1].Mass;
    s(2) = _Linker[_RobotDOF-1].Cent_Gravity(2)/ _Linker[_RobotDOF-1].Mass;



    // Fuerza ejercida sobre i en el cdm
    f_cdm = _Linker[_RobotDOF-1].acc * _Linker[_RobotDOF-1].Mass;
    M_cdm = (_Linker[_RobotDOF-1].Inertia_tensor * _Linker[_RobotDOF-1].wd)  +
            (_Linker[_RobotDOF-1].w.crossProduct(_Linker[_RobotDOF-1].Inertia_tensor * _Linker[_RobotDOF-1].w));


    f = f_cdm * -1;

    M = (R_pi + s).crossProduct(f_cdm) + M_cdm;



    if (_Linker[_RobotDOF-1].Type == Link::REVOLUTION){
        Tau = (R3_post_inv.Transpose() * M) * z0;
    }else{
        Tau = (R3_post_inv.Transpose() * f) * z0;
    }
    _Linker[_RobotDOF-1].Tau = Tau;
    _Linker[_RobotDOF-1].f = f;
    _Linker[_RobotDOF-1].M = M;

    for (int i=_RobotDOF-2; i>=0; i--){
        R3_post = GetRotation(i+1);
        R3_inv = GetRotation(i+1).Inverse();
        Rtot = Rtot * R3_inv;
        Rtot_inv = Rtot.Inverse();

        R3_inv_ant = GetRotation(i).Inverse();

        //Vector join Si-1 and Si
        R_pi(0) = _Linker[i].a;
        R_pi(1) = _Linker[i].d*sinf(_Linker[i].alpha);
        R_pi(2) = _Linker[i].d*cosf(_Linker[i].alpha);
        R_pi_post = R3_inv * R_pi;

        //Center of gravity
        s(0) = _Linker[i].Cent_Gravity(0)/ _Linker[i].Mass;
        s(1) = _Linker[i].Cent_Gravity(1)/ _Linker[i].Mass;
        s(2) = _Linker[i].Cent_Gravity(2)/ _Linker[i].Mass;


        f_cdm = _Linker[i].acc * _Linker[i].Mass;
        M_cdm = (_Linker[i].Inertia_tensor * _Linker[i].wd)  +
                (_Linker[i].w.crossProduct(_Linker[i].Inertia_tensor * _Linker[i].w));

        f = (R3_post * _Linker[i+1].f) + f_cdm;
        A   =   (_Linker[i+1].M + R_pi_post.crossProduct(_Linker[i+1].f));
        M = (R3_post * A) + ((R_pi + s).crossProduct(f_cdm) + M_cdm);

        if (_Linker[i].Type == Link::REVOLUTION){
            Tau = (R3_inv_ant.Transpose() * M) * z0;
        }else {
            Tau = (R3_inv_ant.Transpose() * f) * z0;
        }

        _Linker[i].Tau=Tau;
        _Linker[i].f = f;
        _Linker[i].M = M;

    }

}

void Robot::GravityTorque(float q[]){
    float grav[3] = {0, 9.81F, 0};
    Vector<float> g;
    g = grav;
    ForwardKinematics(q);
    Matrix<float> A0(4,4);
    A0 = _Linker[0].A0;

    Vector<float> pcg(4);

    Vector<float> position[_RobotDOF];
    position[0] = zeros(3);
    position[1] = GetPosition(A0);

    Vector<float> z0 [_RobotDOF];
    z0[0](0) = 0;
    z0[0](1) = 0;
    z0[0](2) = 1;

    Vector<float> m(4);
    for (int i = 0; i <_RobotDOF; i++) {
        if (i > 0) {
            z0[i](0) = A0(0,2);
            z0[i](1) = A0(1,2);
            z0[i](2) = A0(2,2);

            A0 = A0 * _Linker[i].A0;
            if (i<_RobotDOF-1) {
                position[i + 1] = GetPosition(A0);
            }
        }
        m(0) = _Linker[i].Cent_Gravity(0) / _Linker[i].Mass;
        m(1) = _Linker[i].Cent_Gravity(1) / _Linker[i].Mass;
        m(2) = _Linker[i].Cent_Gravity(2) / _Linker[i].Mass;
        m(3) = 1;


        pcg = A0 * m;
        _Motor[i].pcg(0) = pcg(0);
        _Motor[i].pcg(1) = pcg(1);
        _Motor[i].pcg(2) = pcg(2);
    }
    Vector<float> M;
    Vector<float> mg;
    float Tau;
    for (int i =0; i < _RobotDOF; i++) {
        mg = g * _Linker[i].Mass;
        M = mg.crossProduct((_Motor[i].pcg - position[i]));
        for (int j = 0; j < _RobotDOF; j++) {
            if(j>i) {
                mg = g * _Linker[j].Mass;
                M = M + (mg.crossProduct(_Motor[j].pcg)-position[i]);
            }
        }
        Tau = M * z0[i];
        _Linker[i].Tau = Tau;
        M = zeros(3);
    }

}