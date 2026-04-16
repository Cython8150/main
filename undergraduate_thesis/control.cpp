#include "control.h"

void ur5_control(RemoteAPIObject::sim& sim, vector<int> ur5_joints, vector<double> positions) {
    for (int i = 0; i < 6; i++) {
        sim.setJointTargetPosition(ur5_joints[i], positions[i]);
        //cout << "Moving joint " << i+1 << " to " << positions[i] << " rad" << endl;
    }
}

void rg2_control(RemoteAPIObject::sim& sim, vector<int> rg2_joints, double force, double velocity) {
    sim.setJointTargetForce(rg2_joints[0], force); //马达动力单位为N
    sim.setJointTargetVelocity(rg2_joints[0], velocity);   //速度单位为m/s
}