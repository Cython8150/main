#ifndef FIND_HANDLE_H
#define FIND_HANDLE_H

#include "RemoteAPIClient.h"
#include "vector"
#include "iostream"

using namespace std;

vector<int> findUR5JointHandle(RemoteAPIObject::sim& sim);

vector<int> findRG2JointHandle(RemoteAPIObject::sim& sim);

#endif