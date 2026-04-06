#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <iostream>
#include <limits>
#include <algorithm>

using namespace cv;
using namespace std;

// 使用您已有的DH参数
struct DHParams {
    double d;      // 连杆偏距
    double theta0; // 初始关节角度偏移（度）
    double a;      // 连杆长度
    double alpha;  // 连杆扭角（度）
};

// UR5 DH参数（单位：米）
vector<DHParams> ur5_dh_params = {
    {0.0661,  90.0,   0.0000,  -90.0},   // 关节1
    {0.0000,  0.0,    0.4251,  0.0},     // 关节2
    {0.0000,  0.0,    0.3922,  0.0},     // 关节3
    {0.0397, -90.0,   0.0000,  -90.0},   // 关节4
    {0.0492,  90.0,   0.0000,  -90.0},   // 关节5
    {0.0000,  90.0,   0.0001,  0.0}      // 关节6
};

// 弧度与角度转换
double deg2rad(double deg) { return deg * M_PI / 180.0; }
double rad2deg(double rad) { return rad * 180.0 / M_PI; }

// DH变换矩阵
Mat dh_transform(double theta, double d, double a, double alpha) {
    double ct = cos(theta);
    double st = sin(theta);
    double ca = cos(alpha);
    double sa = sin(alpha);
    
    Mat T = (Mat_<double>(4,4) <<
        ct,     -st*ca,     st*sa,      a*ct,
        st,     ct*ca,      -ct*sa,     a*st,
        0,      sa,         ca,         d,
        0,      0,          0,          1);
    
    return T;
}

// 正运动学（与您已有的代码保持一致）
Mat forwardKinematics(const vector<double>& joint_angles) {
    Mat T = Mat::eye(4, 4, CV_64F);
    
    for (int i = 0; i < 6; i++) {
        double theta = deg2rad(ur5_dh_params[i].theta0) + joint_angles[i];
        double d = ur5_dh_params[i].d;
        double a = ur5_dh_params[i].a;
        double alpha = deg2rad(ur5_dh_params[i].alpha);
        
        Mat Ti = dh_transform(theta, d, a, alpha);
        T = T * Ti;
    }
    
    return T;
}

// 完整的UR5逆运动学函数（使用与正运动学一致的DH参数）
vector<vector<double>> ur5InverseKinematics(
    double px, double py, double pz,  // 目标位置
    double roll, double pitch, double yaw,  // 目标姿态（欧拉角，弧度）
    const vector<double>& current_joints = vector<double>()  // 可选：当前关节角度，用于选择最优解
) {
    vector<vector<double>> all_solutions;
    
    // 使用与正运动学一致的DH参数
    double d1 = ur5_dh_params[0].d;  // 0.0661
    double a2 = ur5_dh_params[1].a;  // 0.4251
    double a3 = ur5_dh_params[2].a;  // 0.3922
    double d4 = ur5_dh_params[3].d;  // 0.0397
    double d5 = ur5_dh_params[4].d;  // 0.0492
    double d6 = ur5_dh_params[5].d;  // 0.0001
    
    // 从欧拉角计算旋转矩阵
    double cr = cos(roll);
    double sr = sin(roll);
    double cp = cos(pitch);
    double sp = sin(pitch);
    double cy = cos(yaw);
    double sy = sin(yaw);
    
    // 目标旋转矩阵
    double R11 = cy*cp;
    double R12 = cy*sp*sr - sy*cr;
    double R13 = cy*sp*cr + sy*sr;
    double R21 = sy*cp;
    double R22 = sy*sp*sr + cy*cr;
    double R23 = sy*sp*cr - cy*sr;
    double R31 = -sp;
    double R32 = cp*sr;
    double R33 = cp*cr;
    
    // 腕部中心位置
    double wx = px - d6 * R13;
    double wy = py - d6 * R23;
    double wz = pz - d6 * R33;
    
    // 计算theta1
    vector<double> theta1_options;
    double r = sqrt(wx*wx + wy*wy);
    if (r > 0.001) {  // 避免除零
        theta1_options.push_back(atan2(wy, wx));
        theta1_options.push_back(atan2(-wy, -wx));
    } else {
        // 奇异位置
        theta1_options.push_back(0.0);
    }
    
    for (double theta1 : theta1_options) {
        // 计算theta2和theta3
        double A = 2*a2*sqrt(wx*wx + wy*wy - d1*d1);
        double B = 2*a2*wz;
        double C = (wx*wx + wy*wy - d1*d1 + wz*wz + a2*a2 - a3*a3 - d4*d4) / (2*a2);
        
        double D = 2*a3*d4;
        double E = 2*a3*a2;
        double F = (wx*wx + wy*wy - d1*d1 + wz*wz - a2*a2 - a3*a3 - d4*d4) / 2;
        
        // 计算可能的theta3
        vector<double> theta3_options;
        double cos_theta3 = (F - E*C) / (D*D + E*E);
        double sin_theta3_1 = sqrt(1 - cos_theta3*cos_theta3);
        double sin_theta3_2 = -sqrt(1 - cos_theta3*cos_theta3);
        
        theta3_options.push_back(atan2(sin_theta3_1, cos_theta3));
        theta3_options.push_back(atan2(sin_theta3_2, cos_theta3));
        
        for (double theta3 : theta3_options) {
            // 计算theta2
            double k1 = a2 + a3*cos(theta3) - d4*sin(theta3);
            double k2 = d4*cos(theta3) + a3*sin(theta3);
            
            double theta2 = atan2(wz, sqrt(wx*wx + wy*wy - d1*d1)) - atan2(k2, k1);
            
            // 计算前三个关节的变换矩阵
            double c1 = cos(theta1);
            double s1 = sin(theta1);
            double c2 = cos(theta2);
            double s2 = sin(theta2);
            double c3 = cos(theta3);
            double s3 = sin(theta3);
            
            // 计算R_0_3
            double R11_03 = c1*c2*c3 - c1*s2*s3;
            double R12_03 = -s1;
            double R13_03 = c1*c2*s3 + c1*s2*c3;
            
            double R21_03 = s1*c2*c3 - s1*s2*s3;
            double R22_03 = c1;
            double R23_03 = s1*c2*s3 + s1*s2*c3;
            
            double R31_03 = -s2*c3 - c2*s3;
            double R32_03 = 0;
            double R33_03 = -s2*s3 + c2*c3;
            
            // 计算R_3_6 = R_0_3^T * R_desired
            double R11_36 = R11_03*R11 + R21_03*R21 + R31_03*R31;
            double R12_36 = R11_03*R12 + R21_03*R22 + R31_03*R32;
            double R13_36 = R11_03*R13 + R21_03*R23 + R31_03*R33;
            
            double R21_36 = R12_03*R11 + R22_03*R21 + R32_03*R31;
            double R22_36 = R12_03*R12 + R22_03*R22 + R32_03*R32;
            double R23_36 = R12_03*R13 + R22_03*R23 + R32_03*R33;
            
            double R31_36 = R13_03*R11 + R23_03*R21 + R33_03*R31;
            double R32_36 = R13_03*R12 + R23_03*R22 + R33_03*R32;
            double R33_36 = R13_03*R13 + R23_03*R23 + R33_03*R33;
            
            // 计算theta4, theta5, theta6
            vector<double> theta5_options;
            double theta5_1 = atan2(sqrt(R13_36*R13_36 + R23_36*R23_36), R33_36);
            double theta5_2 = atan2(-sqrt(R13_36*R13_36 + R23_36*R23_36), R33_36);
            
            theta5_options.push_back(theta5_1);
            theta5_options.push_back(theta5_2);
            
            for (double theta5 : theta5_options) {
                if (abs(sin(theta5)) > 1e-6) {  // 避免奇异
                    // 计算theta4
                    double theta4 = atan2(R23_36/sin(theta5), R13_36/sin(theta5));
                    
                    // 计算theta6
                    double theta6 = atan2(R32_36/sin(theta5), -R31_36/sin(theta5));
                    
                    // 创建解
                    vector<double> solution(6);
                    solution[0] = theta1;
                    solution[1] = theta2;
                    solution[2] = theta3;
                    solution[3] = theta4;
                    solution[4] = theta5;
                    solution[5] = theta6;
                    
                    all_solutions.push_back(solution);
                } else {
                    // 奇异位置，theta5=0或pi
                    // theta4和theta6耦合，可以选择任意值
                    double theta4 = 0.0;  // 选择默认值
                    double theta6 = atan2(R21_36, R11_36) - theta4;
                    
                    vector<double> solution(6);
                    solution[0] = theta1;
                    solution[1] = theta2;
                    solution[2] = theta3;
                    solution[3] = theta4;
                    solution[4] = theta5;
                    solution[5] = theta6;
                    
                    all_solutions.push_back(solution);
                }
            } // end for theta5
        } // end for theta3
    } // end for theta1
    
    return all_solutions;
}

// 选择最优解（考虑关节限位和当前位置）
vector<double> selectOptimalSolution(
    const vector<vector<double>>& all_solutions,
    const vector<double>& current_joints = vector<double>()
) {
    if (all_solutions.empty()) {
        cerr << "错误: 没有有效的逆运动学解" << endl;
        return vector<double>(6, 0.0);
    }
    
    // UR5的实际关节限位（弧度）
    vector<pair<double, double>> joint_limits = {
        {-3.1416, 3.1416},  // 关节1: ±180°
        {-3.1416, 3.1416},  // 关节2: ±180°
        {-3.1416, 3.1416},  // 关节3: ±180°
        {-3.1416, 3.1416},  // 关节4: ±180°
        {-3.1416, 3.1416},  // 关节5: ±180°
        {-3.1416, 3.1416}   // 关节6: ±180°
    };
    
    vector<vector<double>> valid_solutions;
    
    // 筛选有效解
    for (const auto& sol : all_solutions) {
        bool valid = true;
        
        // 检查关节限位
        for (int i = 0; i < 6; i++) {
            double angle = sol[i];
            // 将角度归一化到[-π, π]
            while (angle > M_PI) angle -= 2*M_PI;
            while (angle < -M_PI) angle += 2*M_PI;
            
            if (angle < joint_limits[i].first || angle > joint_limits[i].second) {
                valid = false;
                break;
            }
        }
        
        if (valid) {
            valid_solutions.push_back(sol);
        }
    }
    
    if (valid_solutions.empty()) {
        cerr << "警告: 没有满足关节限位的解，返回第一个解" << endl;
        return all_solutions[0];
    }
    
    // 如果提供了当前关节角度，选择最近的解
    if (!current_joints.empty() && current_joints.size() == 6) {
        double min_distance = numeric_limits<double>::max();
        int best_index = 0;
        
        for (size_t i = 0; i < valid_solutions.size(); i++) {
            double distance = 0.0;
            for (int j = 0; j < 6; j++) {
                double diff = valid_solutions[i][j] - current_joints[j];
                // 考虑角度循环
                while (diff > M_PI) diff -= 2*M_PI;
                while (diff < -M_PI) diff += 2*M_PI;
                distance += diff * diff;
            }
            
            if (distance < min_distance) {
                min_distance = distance;
                best_index = i;
            }
        }
        
        return valid_solutions[best_index];
    }
    
    // 否则返回第一个有效解
    return valid_solutions[0];
}

// 主要的抓取计算函数
vector<double> calculateGraspJointAngles(
    double target_x, double target_y, double target_z,  // 目标物体位置（相机坐标系，米）
    const Mat& T_cam2gripper,  // 手眼标定矩阵
    double grasp_height = 0.1,  // 抓取高度（相对于物体，米）
    double approach_angle = M_PI,  // 接近角度（横滚角，弧度）
    const vector<double>& current_joints = vector<double>()  // 当前关节角度（必须提供）
) {
    cout << "\n=== 计算抓取关节角度 ===" << endl;
    cout << "目标物体位置（相机坐标系）: (" 
         << target_x << ", " << target_y << ", " << target_z << ") 米" << endl;
    
    if (current_joints.empty() || current_joints.size() != 6) {
        cerr << "错误: 必须提供当前关节角度" << endl;
        return vector<double>();
    }
    
    // 1. 计算当前末端执行器在基座坐标系中的姿态
    Mat T_base2gripper = forwardKinematics(current_joints);
    
    // 2. 坐标转换：相机坐标系 -> 末端执行器坐标系
    Mat point_camera = (Mat_<double>(4,1) << target_x, target_y, target_z, 1.0);
    Mat point_gripper = T_cam2gripper * point_camera;
    
    double gripper_x = point_gripper.at<double>(0,0);
    double gripper_y = point_gripper.at<double>(1,0);
    double gripper_z = point_gripper.at<double>(2,0);
    
    cout << "目标物体位置（末端坐标系）: (" 
         << gripper_x << ", " << gripper_y << ", " << gripper_z << ") 米" << endl;
    
    // 3. 将目标位置从末端坐标系转换到基座坐标系
    Mat point_base = T_base2gripper * point_gripper;
    
    double base_x = point_base.at<double>(0,0);
    double base_y = point_base.at<double>(1,0);
    double base_z = point_base.at<double>(2,0);
    
    cout << "目标物体位置（基座坐标系）: (" 
         << base_x << ", " << base_y << ", " << base_z << ") 米" << endl;
    
    // 4. 计算抓取位置（在物体上方一定高度）
    double grasp_x = base_x;
    double grasp_y = base_y;
    double grasp_z = base_z + grasp_height;  // 在物体上方抓取
    
    cout << "抓取位置（基座坐标系）: (" 
         << grasp_x << ", " << grasp_y << ", " << grasp_z << ") 米" << endl;
    
    // 5. 设置抓取姿态（相对于基座坐标系）
    // 修改为末端垂直向下（z轴向下），更适合抓取
    double roll = 0.0;              // 横滚角
    double pitch = M_PI;            // 俯仰角（垂直向下）
    double yaw = 0.0;              // 偏航角
    
    // 6. 计算逆运动学
    vector<vector<double>> all_solutions = ur5InverseKinematics(
        grasp_x, grasp_y, grasp_z, roll, pitch, yaw, current_joints
    );
    
    cout << "找到 " << all_solutions.size() << " 个可能的逆运动学解" << endl;
    
    if (all_solutions.empty()) {
        cerr << "错误: 没有找到逆运动学解" << endl;
        return vector<double>();
    }
    
    // 7. 选择最优解
    vector<double> best_solution = selectOptimalSolution(all_solutions, current_joints);
    
    // 8. 验证结果
    if (!best_solution.empty()) {
        Mat T_verify = forwardKinematics(best_solution);
        double error_position = sqrt(
            pow(T_verify.at<double>(0,3) - grasp_x, 2) +
            pow(T_verify.at<double>(1,3) - grasp_y, 2) +
            pow(T_verify.at<double>(2,3) - grasp_z, 2)
        );
        
        cout << "位置误差: " << error_position * 1000 << " 毫米" << endl;
        
        if (error_position < 0.001) {  // 1毫米
            cout << "✓ 逆运动学计算成功" << endl;
        } else {
            cout << "⚠ 逆运动学存在误差" << endl;
        }
    }
    
    return best_solution;
}

// 主函数示例
int main() {
    // 1. 加载手眼标定矩阵
    Mat T_cam2gripper;
    FileStorage fs("./calibration_results/hand_eye_result.xml", FileStorage::READ);
    if (fs.isOpened()) {
        fs["T_cam2gripper"] >> T_cam2gripper;
        fs.release();
        cout << "成功加载手眼标定矩阵:" << endl;
        cout << T_cam2gripper << endl;
    } else {
        cerr << "警告: 无法加载手眼标定文件，使用默认矩阵" << endl;
        T_cam2gripper = (Mat_<double>(4,4) <<
            -0.964513,0.186246,-0.187157,0.277856, 
            0.149827,-0.197595,-0.968766,0.815149, 
            -0.217410,-0.962429,0.162679,0.192469, 
            0.0,0.0,0.0,1.0);
    }
    
    // 2. 目标物体位置（从相机检测得到，单位：米）
    // 这里使用示例值，在相机坐标系中应该是相对较小的值
    double target_x = 1.0;   // 米
    double target_y = 1.0;   // 米
    double target_z = 0.8;   // 米
    
    cout << "\n目标物体位置: (" << target_x << ", " << target_y << ", " << target_z << ") 米" << endl;
    
    // 3. 设置当前关节角度（弧度）
    // 使用默认的初始位置（所有关节为0）
    vector<double> current_joints(6, 0.0);  // 所有关节角度为0弧度
    
    // 4. 计算抓取关节角度
    vector<double> joint_angles = calculateGraspJointAngles(
        target_x, target_y, target_z,
        T_cam2gripper,
        0.1,     // 抓取高度（米）
        M_PI,    // 横滚角（垂直向下）
        current_joints
    );
    
    // 5. 输出结果
    if (!joint_angles.empty()) {
        cout << "\n=== 计算结果 ===" << endl;
        cout << "关节角度（弧度）: ";
        for (double angle : joint_angles) {
            cout << angle << " ";
        }
        cout << endl;
        
        cout << "关节角度（度）: ";
        for (double angle : joint_angles) {
            cout << rad2deg(angle) << " ";
        }
        cout << endl;
        
        // 验证结果
        cout << "\n=== 验证结果 ===" << endl;
        Mat T_result = forwardKinematics(joint_angles);
        cout << "计算出的末端位置矩阵:" << endl;
        cout << T_result << endl;
        
        // 重新计算期望的抓取位置（基座坐标系）
        Mat T_base2gripper = forwardKinematics(current_joints);
        Mat point_camera = (Mat_<double>(4,1) << target_x, target_y, target_z, 1.0);
        Mat point_gripper = T_cam2gripper * point_camera;
        Mat point_base = T_base2gripper * point_gripper;
        
        double base_x = point_base.at<double>(0,0);
        double base_y = point_base.at<double>(1,0);
        double base_z = point_base.at<double>(2,0);
        
        double grasp_x = base_x;
        double grasp_y = base_y;
        double grasp_z = base_z + 0.1;
        
        cout << "期望抓取位置: (" << grasp_x << ", " << grasp_y << ", " << grasp_z << ")" << endl;
        cout << "实际末端位置: (" << T_result.at<double>(0,3) << ", " 
             << T_result.at<double>(1,3) << ", " 
             << T_result.at<double>(2,3) << ")" << endl;
    } else {
        cerr << "错误: 无法计算抓取关节角度" << endl;
    }
    
    // 6. 测试多个目标点
    cout << "\n=== 测试多个目标点 ===" << endl;
    vector<Point3f> test_points = {
        Point3f(0.05, 0.05, 0.15),   // 位置1
        Point3f(0.08, -0.03, 0.18),   // 位置2
        Point3f(0.06, 0.08, 0.22),    // 位置3
        Point3f(0.3, 0.6, 0.7)     // 位置4
    };
    
    for (size_t i = 0; i < test_points.size(); i++) {
        cout << "\n测试点 " << i+1 << ": (" 
             << test_points[i].x << ", " 
             << test_points[i].y << ", " 
             << test_points[i].z << ")" << endl;
        
        vector<double> angles = calculateGraspJointAngles(
            test_points[i].x, test_points[i].y, test_points[i].z,
            T_cam2gripper,
            0.1,  // 抓取高度
            M_PI, // 姿态
            joint_angles  // 使用上一个解作为起始点
        );
        
        if (!angles.empty()) {
            // 更新当前关节角度
            joint_angles = angles;
        }
    }
    
    cout << "\n=== 测试完成 ===" << endl;
    
    return 0;
}