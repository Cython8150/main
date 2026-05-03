#ifndef CONTROL_H
#define CONTROL_H

#include "RemoteAPIClient.h"
#include "vector"

using namespace std;

void ur5_control(RemoteAPIObject::sim& sim, vector<int> ur5_joints, vector<double> position);

/**
 * 从当前关节角线性插值到 q_target（与一次 ur5_control 终点相同），每段后仿真步进若干次，
 * 带载回零时比瞬间下目标更稳，减轻 Bullet 下侧摆/倾倒。
 */
void ur5_moveToJointTargetSmooth(RemoteAPIObject::sim& sim, vector<int> ur5_joints, const vector<double>& q_target,
                                 int segments, int steps_per_segment);

void rg2_control(RemoteAPIObject::sim& sim, vector<int> rg2_joints, double force, double velocity);

#endif