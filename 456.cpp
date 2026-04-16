#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <fstream>
#include "RemoteAPIClient.h"

using namespace cv;
using namespace std;

// 加载手眼标定结果
bool loadHandEyeCalibration(Mat& T_cam2gripper) {
    FileStorage fs("./calibration_results/hand_eye_result.xml", FileStorage::READ);
    if (!fs.isOpened()) {
        cerr << "无法打开手眼标定文件" << endl;
        return false;
    }
    
    fs["T_cam2gripper"] >> T_cam2gripper;
    
    if (T_cam2gripper.empty()) {
        cerr << "手眼标定矩阵为空" << endl;
        fs.release();
        return false;
    }
    
    cout << "手眼标定矩阵 T_cam2gripper (相机→末端):" << endl;
    cout << T_cam2gripper << endl;
    
    fs.release();
    return true;
}

// 位姿（位置+四元数）转4x4变换矩阵
Mat poseToMatrix(const vector<double>& pose) {
    if (pose.size() < 7) {
        cerr << "位姿数据长度不足" << endl;
        return Mat::eye(4, 4, CV_64F);
    }
    
    // pose格式: [x, y, z, qx, qy, qz, qw]
    double x = pose[0], y = pose[1], z = pose[2];
    double qx = pose[3], qy = pose[4], qz = pose[5], qw = pose[6];
    
    // 四元数转旋转矩阵
    double qx2 = qx*qx, qy2 = qy*qy, qz2 = qz*qz;
    double qw2 = qw*qw;
    
    double R00 = qw2 + qx2 - qy2 - qz2;
    double R01 = 2*(qx*qy - qw*qz);
    double R02 = 2*(qx*qz + qw*qy);
    double R10 = 2*(qx*qy + qw*qz);
    double R11 = qw2 - qx2 + qy2 - qz2;
    double R12 = 2*(qy*qz - qw*qx);
    double R20 = 2*(qx*qz - qw*qy);
    double R21 = 2*(qy*qz + qw*qx);
    double R22 = qw2 - qx2 - qy2 + qz2;
    
    Mat T = Mat::eye(4, 4, CV_64F);
    T.at<double>(0,0) = R00; T.at<double>(0,1) = R01; T.at<double>(0,2) = R02; T.at<double>(0,3) = x;
    T.at<double>(1,0) = R10; T.at<double>(1,1) = R11; T.at<double>(1,2) = R12; T.at<double>(1,3) = y;
    T.at<double>(2,0) = R20; T.at<double>(2,1) = R21; T.at<double>(2,2) = R22; T.at<double>(2,3) = z;
    
    return T;
}

// 4x4变换矩阵转位姿
vector<double> matrixToPose(const Mat& T) {
    vector<double> pose(7, 0.0);
    
    if (T.empty() || T.rows != 4 || T.cols != 4) {
        cerr << "变换矩阵无效" << endl;
        return pose;
    }
    
    // 位置
    pose[0] = T.at<double>(0, 3);
    pose[1] = T.at<double>(1, 3);
    pose[2] = T.at<double>(2, 3);
    
    // 旋转矩阵
    double R00 = T.at<double>(0,0), R01 = T.at<double>(0,1), R02 = T.at<double>(0,2);
    double R10 = T.at<double>(1,0), R11 = T.at<double>(1,1), R12 = T.at<double>(1,2);
    double R20 = T.at<double>(2,0), R21 = T.at<double>(2,1), R22 = T.at<double>(2,2);
    
    // 旋转矩阵转四元数
    double trace = R00 + R11 + R22;
    
    if (trace > 0) {
        double S = sqrt(trace + 1.0) * 2.0;
        pose[6] = 0.25 * S;  // qw
        pose[3] = (R21 - R12) / S;  // qx
        pose[4] = (R02 - R20) / S;  // qy
        pose[5] = (R10 - R01) / S;  // qz
    } else if ((R00 > R11) && (R00 > R22)) {
        double S = sqrt(1.0 + R00 - R11 - R22) * 2.0;
        pose[6] = (R21 - R12) / S;  // qw
        pose[3] = 0.25 * S;  // qx
        pose[4] = (R01 + R10) / S;  // qy
        pose[5] = (R02 + R20) / S;  // qz
    } else if (R11 > R22) {
        double S = sqrt(1.0 + R11 - R00 - R22) * 2.0;
        pose[6] = (R02 - R20) / S;  // qw
        pose[3] = (R01 + R10) / S;  // qx
        pose[4] = 0.25 * S;  // qy
        pose[5] = (R12 + R21) / S;  // qz
    } else {
        double S = sqrt(1.0 + R22 - R00 - R11) * 2.0;
        pose[6] = (R10 - R01) / S;  // qw
        pose[3] = (R02 + R20) / S;  // qx
        pose[4] = (R12 + R21) / S;  // qy
        pose[5] = 0.25 * S;  // qz
    }
    
    return pose;
}

// 计算两个位姿之间的误差
void computePoseError(const vector<double>& pose1, const vector<double>& pose2, 
                      double& pos_error, double& angle_error) {
    // 位置误差
    double dx = pose1[0] - pose2[0];
    double dy = pose1[1] - pose2[1];
    double dz = pose1[2] - pose2[2];
    pos_error = sqrt(dx*dx + dy*dy + dz*dz);
    
    // 四元数误差
    double qx1 = pose1[3], qy1 = pose1[4], qz1 = pose1[5], qw1 = pose1[6];
    double qx2 = pose2[3], qy2 = pose2[4], qz2 = pose2[5], qw2 = pose2[6];
    
    // 四元数点积
    double dot = qx1*qx2 + qy1*qy2 + qz1*qz2 + qw1*qw2;
    dot = min(max(dot, -1.0), 1.0);
    
    // 角度误差（弧度转度）
    angle_error = 2 * acos(fabs(dot)) * 180.0 / CV_PI;
}

// 验证手眼标定
void verifyHandEyeCalibration(RemoteAPIObject::sim& sim) {
    cout << "\n========================================" << endl;
    cout << "       手眼标定验证程序" << endl;
    cout << "========================================" << endl;
    
    // 1. 加载手眼标定结果
    Mat T_cam2gripper;
    if (!loadHandEyeCalibration(T_cam2gripper)) {
        cerr << "手眼标定验证失败：无法加载标定结果" << endl;
        return;
    }
    
    // 2. 获取句柄
    cout << "\n1. 获取对象句柄:" << endl;
    
    int visionSensorHandle = sim.getObject("/UR5/RG2/visionSensor");
    int tipHandle = sim.getObject("/UR5/RG2/attachPoint");  // 机械臂末端
    // int tipHandle = sim.getObject("/UR5");  // 机械臂末端
    int baseHandle = sim.getObject("/UR5/link1_visible");     // 机械臂基座
    
    // 检查句柄有效性
    if (visionSensorHandle == -1) {
        cerr << "  错误: 无法获取视觉传感器句柄" << endl;
        return;
    }
    if (tipHandle == -1) {
        cerr << "  错误: 无法获取机械臂末端句柄" << endl;
        return;
    }
    if (baseHandle == -1) {
        cerr << "  错误: 无法获取机械臂基座句柄" << endl;
        return;
    }
    
    cout << "  视觉传感器: " << visionSensorHandle << endl;
    cout << "  机械臂末端: " << tipHandle << endl;
    cout << "  机械臂基座: " << baseHandle << endl;
    
    // 3. 获取仿真时间，确保仿真已运行
    for (int i = 0; i < 10; i++) {
        sim.step();
    }
    
    // 4. 获取当前位姿
    cout << "\n2. 获取当前位姿:" << endl;
    
    // 获取相机相对于基座的位姿
    vector<double> camera_pose_base = sim.getObjectPose(visionSensorHandle, baseHandle);
    Mat T_camera_base = poseToMatrix(camera_pose_base);
    cout << "  相机在基座坐标系中的位姿:" << endl;
    cout << "    位置: [" << camera_pose_base[0] << ", " 
         << camera_pose_base[1] << ", " << camera_pose_base[2] << "]" << endl;
    cout << "    四元数: [" << camera_pose_base[3] << ", " 
         << camera_pose_base[4] << ", " << camera_pose_base[5] << ", " 
         << camera_pose_base[6] << "]" << endl;
    
    // 获取机械臂末端相对于基座的实际位姿
    vector<double> tip_pose_base_actual = sim.getObjectPose(tipHandle, baseHandle);
    Mat T_tip_base_actual = poseToMatrix(tip_pose_base_actual);
    cout << "\n  机械臂末端实际位姿:" << endl;
    cout << "    位置: [" << tip_pose_base_actual[0] << ", " 
         << tip_pose_base_actual[1] << ", " << tip_pose_base_actual[2] << "]" << endl;
    cout << "    四元数: [" << tip_pose_base_actual[3] << ", " 
         << tip_pose_base_actual[4] << ", " << tip_pose_base_actual[5] << ", " 
         << tip_pose_base_actual[6] << "]" << endl;
    
    // 5. 通过手眼标定计算机械臂末端位姿
    cout << "\n3. 通过手眼标定计算末端位姿:" << endl;
    
    // 方法1: 使用你的原始公式
    Mat T_gripper2cam = T_cam2gripper.inv();  // 末端→相机的变换
    Mat T_tip_base_calculated_1 = T_camera_base * T_gripper2cam;
    vector<double> tip_pose_calculated_1 = matrixToPose(T_tip_base_calculated_1);
    
    cout << "  方法1 (T_camera_base * inv(T_cam2gripper)):" << endl;
    cout << "    位置: [" << tip_pose_calculated_1[0] << ", " 
         << tip_pose_calculated_1[1] << ", " << tip_pose_calculated_1[2] << "]" << endl;
    
    // 方法2: 验证另一种关系 T_tip_base = T_gripper_cam * T_camera_base (如果标定的是末端到相机)
    // 注意：这取决于你标定时矩阵的定义方向
    Mat T_tip_base_calculated_2 = T_cam2gripper * T_camera_base;
    vector<double> tip_pose_calculated_2 = matrixToPose(T_tip_base_calculated_2);
    
    cout << "\n  方法2 (T_cam2gripper * T_camera_base):" << endl;
    cout << "    位置: [" << tip_pose_calculated_2[0] << ", " 
         << tip_pose_calculated_2[1] << ", " << tip_pose_calculated_2[2] << "]" << endl;
    
    // 6. 对比结果
    cout << "\n4. 误差分析:" << endl;
    
    // 方法1的误差
    double pos_error_1, angle_error_1;
    computePoseError(tip_pose_calculated_1, tip_pose_base_actual, pos_error_1, angle_error_1);
    cout << "  方法1误差:" << endl;
    cout << "    位置误差: " << pos_error_1 * 1000 << " mm" << endl;
    cout << "    角度误差: " << angle_error_1 << " 度" << endl;
    
    // 方法2的误差
    double pos_error_2, angle_error_2;
    computePoseError(tip_pose_calculated_2, tip_pose_base_actual, pos_error_2, angle_error_2);
    cout << "\n  方法2误差:" << endl;
    cout << "    位置误差: " << pos_error_2 * 1000 << " mm" << endl;
    cout << "    角度误差: " << angle_error_2 << " 度" << endl;
    
    // 7. 评估
    cout << "\n5. 评估:" << endl;
    
    // 选择误差较小的方法
    if (pos_error_1 < pos_error_2) {
        cout << "  推荐使用方法1: T_tip_base = T_camera_base * inv(T_cam2gripper)" << endl;
        if (pos_error_1 * 1000 < 5.0 && angle_error_1 < 2.0) {
            cout << "  ✓ 手眼标定精度良好 (位置<5mm, 角度<2°)" << endl;
        } else if (pos_error_1 * 1000 < 10.0 && angle_error_1 < 5.0) {
            cout << "  ⚠ 手眼标定精度一般 (位置<10mm, 角度<5°)" << endl;
        } else {
            cout << "  ✗ 手眼标定精度较差，建议重新标定" << endl;
        }
    } else {
        cout << "  推荐使用方法2: T_tip_base = T_cam2gripper * T_camera_base" << endl;
        if (pos_error_2 * 1000 < 5.0 && angle_error_2 < 2.0) {
            cout << "  ✓ 手眼标定精度良好 (位置<5mm, 角度<2°)" << endl;
        } else if (pos_error_2 * 1000 < 10.0 && angle_error_2 < 5.0) {
            cout << "  ⚠ 手眼标定精度一般 (位置<10mm, 角度<5°)" << endl;
        } else {
            cout << "  ✗ 手眼标定精度较差，建议重新标定" << endl;
        }
    }
    
    // 8. 建议
    cout << "\n6. 建议:" << endl;
    if (pos_error_1 * 1000 > 10.0 && pos_error_2 * 1000 > 10.0) {
        cout << "  - 检查手眼标定矩阵是否正确保存" << endl;
        cout << "  - 确认视觉传感器安装位置与标定时一致" << endl;
        cout << "  - 考虑重新进行手眼标定" << endl;
    }
    
    cout << "\n========================================" << endl;
}

int main() {
    try {
        // 创建远程API客户端
        cout << "连接到 CoppeliaSim..." << endl;
        RemoteAPIClient client;
        auto sim = client.getObject().sim();
        
        cout << "连接成功!" << endl;
        
        // 验证手眼标定结果
        verifyHandEyeCalibration(sim);
        
    } catch (const exception& e) {
        cerr << "错误: " << e.what() << endl;
        cerr << "请确保 CoppeliaSim 正在运行且 zmqRemoteAPI 已启用" << endl;
        return -1;
    }
    
    return 0;
}