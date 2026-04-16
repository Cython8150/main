#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include "RemoteAPIClient.h"
#include "find_handle.h"

using namespace cv;
using namespace std;

void compareEndEffectorPose();

// 使用经典DH参数
struct DHParams {
    double a;      // 连杆长度
    double alpha;  // 连杆扭角（度）
    double d;      // 连杆偏距
    double theta0; // 初始关节角度偏移（度）
};

// UR5 经典DH参数（从CoppeliaSim提取，单位：米）
vector<DHParams> ur5_dh_params = {
    {0.0000,   90.0,   0.089159,   0.0},   // 关节1: alpha=90, d=0.089
    {-0.4250,   0.0,   0.0000,     0.0},   // 关节2: a=-0.425
    {-0.3922,   0.0,   0.0000,     0.0},   // 关节3: a=-0.392
    {0.0000,   90.0,   0.10915,    0.0},   // 关节4
    {0.0000,  -90.0,   0.09465,    0.0},   // 关节5
    {0.0000,    0.0,   0.08230,    0.0}    // 关节6
};

// 弧度与角度转换
double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / M_PI; }

// 经典DH变换矩阵
Mat dh_transform(double theta, double d, double a, double alpha) {
    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);
    
    Mat T = (Mat_<double>(4,4) <<
        ct,      -st*ca,   st*sa,    a*ct,
        st,      ct*ca,    -ct*sa,   a*st,
        0,       sa,       ca,       d,
        0,       0,        0,        1);
    
    return T;
}

// 正运动学
Mat forwardKinematics(const vector<double>& joint_angles) {
    Mat T = Mat::eye(4, 4, CV_64F);
    
    for (int i = 0; i < 6; i++) {
        //double theta = deg2rad(ur5_dh_params[i].theta0) + joint_angles[i];
        double theta = joint_angles[i];
        double d = ur5_dh_params[i].d;
        double a = ur5_dh_params[i].a;
        double alpha = deg2rad(ur5_dh_params[i].alpha);
        
        Mat Ti = dh_transform(theta, d, a, alpha);
        T = T * Ti;
    }
    
    return T;
}

// 主函数示例
int main(int argc, char* argv[]) {
    cout << "开始对比DH计算与CoppeliaSim的末端位姿..." << endl;
    compareEndEffectorPose();
    cout << "对比完成。" << endl;
    
    return 0;
}

// 新增功能：对比DH计算的末端位姿与CoppeliaSim实际位姿
void compareEndEffectorPose() {
    // 创建客户端
    RemoteAPIClient client;
    auto sim = client.getObject().sim();

    // 开始仿真
    sim.startSimulation();
    sim.setStepping(true);

    // 等待仿真稳定
    for (int i = 0; i < 10; i++) {
        sim.step();
    }

    // 1. 获取CoppeliaSim中实际的关节角度
    vector<int> ur5_joints = findUR5JointHandle(sim);
    vector<double> actual_joints;
    for(int i = 1; i <= 6; i++) {
        double jointPos = sim.getJointPosition(ur5_joints[i-1]);
        actual_joints.push_back(jointPos);  // 关节角度（弧度）
        cout << "关节" << i << "角度: " << rad2deg(jointPos) << " 度" << endl;
    }

    // 2. 使用实际关节角度进行DH正运动学计算
    Mat T_fk = forwardKinematics(actual_joints);

    // 提取位置和姿态
    double fk_x = T_fk.at<double>(0, 3);
    double fk_y = T_fk.at<double>(1, 3);
    double fk_z = T_fk.at<double>(2, 3);

    // 提取旋转矩阵转换为欧拉角（简化为ZYX）
    Mat R_fk = T_fk(Rect(0, 0, 3, 3));
    Vec3d euler_fk;
    // 简化的欧拉角计算（实际应使用更精确的方法）
    euler_fk[0] = atan2(R_fk.at<double>(2, 1), R_fk.at<double>(2, 2));  // roll
    euler_fk[1] = atan2(-R_fk.at<double>(2, 0), sqrt(R_fk.at<double>(2, 1)*R_fk.at<double>(2, 1) + R_fk.at<double>(2, 2)*R_fk.at<double>(2, 2)));  // pitch
    euler_fk[2] = atan2(R_fk.at<double>(1, 0), R_fk.at<double>(0, 0));  // yaw

    cout << "\n=== DH计算的末端位姿 (使用实际关节角度) ===" << endl;
    cout << "位置: (" << fk_x << ", " << fk_y << ", " << fk_z << ") 米" << endl;
    cout << "姿态 (欧拉角ZYX): (" << rad2deg(euler_fk[0]) << ", " << rad2deg(euler_fk[1]) << ", " << rad2deg(euler_fk[2]) << ") 度" << endl;

    // 2. 使用sim.getObjectPose()获取CoppeliaSim中的实际末端位姿
    // 假设末端对象名为 "/UR5/attachPoint" 或类似
    //int endEffectorHandle = sim.getObject("/UR5/RG2/attachPoint");
    int endEffectorHandle = sim.getObject("/UR5/link/joint/link/joint/link/joint/link/joint/link/joint/link");
    int baseHandle = sim.getObject("/UR5");  // UR5底座对象
    vector<double> pos_sim = sim.getObjectPose(endEffectorHandle, baseHandle);

    cout << "\n=== CoppeliaSim实际末端位姿 (相对于UR5底座) ===" << endl;
    cout << "位置: (" << pos_sim[0] << ", " << pos_sim[1] << ", " << pos_sim[2] << ") 米" << endl;
    cout << "姿态 (欧拉角ZYX): (" << rad2deg(pos_sim[3]) << ", " << rad2deg(pos_sim[4]) << ", " << rad2deg(pos_sim[5]) << ") 度" << endl;

    // 3. 计算差异
    double pos_diff_x = fk_x - pos_sim[0];
    double pos_diff_y = fk_y - pos_sim[1];
    double pos_diff_z = fk_z - pos_sim[2];
    double pos_error = sqrt(pos_diff_x*pos_diff_x + pos_diff_y*pos_diff_y + pos_diff_z*pos_diff_z);

    double ori_diff_x = euler_fk[0] - pos_sim[3];
    double ori_diff_y = euler_fk[1] - pos_sim[4];
    double ori_diff_z = euler_fk[2] - pos_sim[5];

    cout << "\n=== 对比结果 ===" << endl;
    cout << "位置误差: " << pos_error * 1000 << " 毫米" << endl;
    cout << "姿态误差: (" << rad2deg(ori_diff_x) << ", " << rad2deg(ori_diff_y) << ", " << rad2deg(ori_diff_z) << ") 度" << endl;

    if (pos_error < 0.001) {  // 1mm
        cout << "✓ 位置匹配良好" << endl;
    } else {
        cout << "⚠ 位置存在误差" << endl;
    }

    // 停止仿真
    sim.stopSimulation();
}