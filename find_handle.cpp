#include "find_handle.h"
#include "iostream"

vector<int> findUR5JointHandle(RemoteAPIObject::sim& sim) {
    vector<int> ur5_joints(6);

    string jointName1 = "/UR5/joint", jointName2 = "/UR5/link/joint", jointName3 = "/UR5/link/joint/link/joint", jointName4 = "/UR5/link/joint/link/joint/link/joint",
           jointName5 = "/UR5/link/joint/link/joint/link/joint/link/joint",
           jointName6 = "/UR5/link/joint/link/joint/link/joint/link/joint/link/joint";

    vector<string> jointNames = {jointName1, jointName2, jointName3, jointName4, jointName5, jointName6};

    for (int i = 0; i < 6; i++) {
        ur5_joints[i] = sim.getObject(jointNames[i]);
    }

    return ur5_joints;
}

vector<int> findRG2JointHandle(RemoteAPIObject::sim& sim) {
    vector<int> rg2_joints(2);  //rg2机械爪的打开关节和关闭关节
    string gripper_joint_open_close = "/UR5/RG2/openCloseJoint", gripper_joint_center = "/UR5/RG2/centerJoint";
    vector<string> gripper_joints = {gripper_joint_open_close, gripper_joint_center};
    for(int i = 0; i < 2; i++) {
        rg2_joints[i] = sim.getObject(gripper_joints[i]);
    }

    return rg2_joints;
}