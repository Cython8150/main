#ifndef CONTROL_H
#define CONTROL_H

#include "RemoteAPIClient.h"
#include "vector"

using namespace std;

void ur5_control(RemoteAPIObject::sim& sim, vector<int> ur5_joints, vector<double> position);

void rg2_control(RemoteAPIObject::sim& sim, vector<int> rg2_joints, double force, double velocity);

#endif