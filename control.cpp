#include "control.h"

void ur5_control(RemoteAPIObject::sim& sim, vector<int> ur5_joints, vector<double> positions) {
    for (int i = 0; i < 6; i++) {
        sim.setJointTargetPosition(ur5_joints[i], positions[i]);
        //cout << "Moving joint " << i+1 << " to " << positions[i] << " rad" << endl;
    }
}

void ur5_moveToJointTargetSmooth(RemoteAPIObject::sim& sim, vector<int> ur5_joints, const vector<double>& q_target,
                                 int segments, int steps_per_segment) {
    if (ur5_joints.size() != 6 || q_target.size() != 6 || segments < 2 || steps_per_segment < 1)
        return;
    vector<double> q0(6);
    for (int j = 0; j < 6; j++)
        q0[j] = sim.getJointPosition(ur5_joints[j]);
    for (int s = 1; s <= segments; s++) {
        const double a = static_cast<double>(s) / static_cast<double>(segments);
        vector<double> q(6);
        for (int j = 0; j < 6; j++)
            q[j] = q0[j] + a * (q_target[j] - q0[j]);
        ur5_control(sim, ur5_joints, q);
        for (int k = 0; k < steps_per_segment; k++)
            sim.step();
    }
}

void rg2_control(RemoteAPIObject::sim& sim, vector<int> rg2_joints, double force, double velocity) {
    sim.setJointTargetForce(rg2_joints[0], force); //马达动力单位为N
    sim.setJointTargetVelocity(rg2_joints[0], velocity);   //速度单位为m/s
}