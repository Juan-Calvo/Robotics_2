#include "main.h"

 void InverseKinematics(Robot ro,End_Effector* EFF_q0,float q[3], int conf=0){
    /*USER SPECIFICATIONS*/
    float Xf = EFF_q0->x;
    float Yf = EFF_q0->y;
    float fi = EFF_q0->pitch;
    float theta = EFF_q0->roll;

    float L1 = ro._Linker[0].ReturnLenght();
    float L2 = ro._Linker[1].ReturnLenght();
    float L3 = ro._Linker[2].ReturnLenght();
    float L4 = ro._Linker[3].ReturnLenght();

    //Wrist points
    float Cfi = cosf(fi);
    float Sfi = sinf(fi);
    if ((Cfi > -0.000001F) && (Cfi < 0.000001F)) {
        Cfi = 0;
    }
    if ((Sfi > -0.000001F) && (Sfi < 0.000001F)) {
        Sfi = 0;
    }
    float Xm = Xf - L4 * Cfi;
    float Ym = Yf - L4 * Sfi;

    //Calc articular coordinates
    //q4
    q[3] = theta;

    //q2
    float C2 = ((powf(Xm, 2.0F)) + (powf(Ym, 2.0F)) - (powf(L1, 2.0F)) - (powf(L2, 2.0F))) / (2 * L1 * L2);
    float S2 = sqrtf(1 - powf(C2, 2.0F));
    float q21 = atan2f(S2, C2);
    float q22 = atan2f(-S2, C2);
    q[1] = q21;

    //q1
    float alpha = atan2f((L2 * sinf(q[1])), (L1 + L2 * cosf(q[1])));
    float beta_1 = atan2f(Ym, Xm);
    float beta_2 = atan2f(Ym, -Xm);

    float q1;
    if (conf == 0) {
        //Elbow up
        q1 = beta_1 - alpha;
    } else {
        //Elbow down
        q1 = beta_2 - alpha;
    }
    q[0]= q1;
    //q3
    q[2] = fi-q[0]-q[1];
}
int main() {
    float q[3]={1,2,3};
    const Link::Links_Type Linker[5] ={Link::REVOLUTION,Link::REVOLUTION,Link::REVOLUTION,Link::REVOLUTION};
    Robot p560(ToolMatrix,BaseMatrix,DH_Table,LinkLenght,dyn_mat,MotorParam,ControlGain,RobotDOF,Linker,Constraints);
    p560.EulerNewton(q,q,q);
    return 0;
}