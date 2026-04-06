#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <iomanip>
#include <cmath>

using namespace std;
using namespace cv;

// 将四元数转为旋转矩阵（w,x,y,z 或 x,y,z,w 都可）
Mat quaternionToRotationMatrix(double qx, double qy, double qz, double qw) {
    double norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (norm < 1e-9) norm = 1.0;
    qx /= norm; qy /= norm; qz /= norm; qw /= norm;

    Mat R = Mat::eye(3, 3, CV_64F);
    R.at<double>(0,0) = 1 - 2*qy*qy - 2*qz*qz;
    R.at<double>(0,1) = 2*qx*qy - 2*qz*qw;
    R.at<double>(0,2) = 2*qx*qz + 2*qy*qw;
    R.at<double>(1,0) = 2*qx*qy + 2*qz*qw;
    R.at<double>(1,1) = 1 - 2*qx*qx - 2*qz*qz;
    R.at<double>(1,2) = 2*qy*qz - 2*qx*qw;
    R.at<double>(2,0) = 2*qx*qz - 2*qy*qw;
    R.at<double>(2,1) = 2*qy*qz + 2*qx*qw;
    R.at<double>(2,2) = 1 - 2*qx*qx - 2*qy*qy;
    return R;
}

// 从ee_pose.txt读取末端位姿，格式：
// Image i:
//   End-effector pose: x y z qx qy qz qw
bool loadEndEffectorPosesFromFile(const string& path, map<int, vector<double>>& ee_pose_map) {
    ifstream fin(path);
    if (!fin.is_open()) {
        cerr << "警告: 无法打开末端位姿文件: " << path << endl;
        return false;
    }

    string line;
    int current_idx = -1;
    while (getline(fin, line)) {
        if (line.empty()) continue;
        if (line.rfind("Image", 0) == 0) {
            // Image N:
            size_t pos = line.find(" ");
            if (pos == string::npos) continue;
            size_t colon = line.find(":", pos);
            string numstr = (colon == string::npos) ? line.substr(pos+1) : line.substr(pos+1, colon-pos-1);
            current_idx = stoi(numstr);
        } else if (line.find("End-effector pose:") != string::npos) {
            if (current_idx < 0) continue;
            string sp = line.substr(line.find(":") + 1);
            stringstream ss(sp);
            double x,y,z,qx,qy,qz,qw;
            if (ss >> x >> y >> z >> qx >> qy >> qz >> qw) {
                ee_pose_map[current_idx] = {x,y,z,qx,qy,qz,qw};
            }
        }
    }

    fin.close();
    if (ee_pose_map.empty()) {
        cerr << "警告: 末端位姿数据为空，文件路径: " << path << endl;
        return false;
    }
    return true;
}

int main() {
    cout << "=== 手眼标定程序（仅使用ee_pose.txt）===" << endl;
    
    // 1. 加载相机内参
    cout << "\n加载相机内参..." << endl;
    FileStorage fs_camera("./calibration_results/cameraParaments.xml", FileStorage::READ);
    if (!fs_camera.isOpened()) {
        cerr << "错误: 无法加载相机标定文件" << endl;
        return 1;
    }
    
    Mat camera_matrix, dist_coeffs;
    fs_camera["intrinsic_matrix"] >> camera_matrix;
    fs_camera["distortion_matrix"] >> dist_coeffs;
    fs_camera.release();
    
    cout << "相机内参加载完成" << endl;
    cout << "相机内参矩阵:\n" << camera_matrix << endl;
    
    // 2. 设置输入路径
    cout << "\n加载图片和ee_pose.txt..." << endl;
    string image_folder = "/home/eric/undergraduate_thesis/build/image/hand_eye_new_1774432181";
    string ee_pose_file = image_folder + "/ee_pose.txt";  // 由123.cpp保存

    // 棋盘格参数
    Size boardSize(11, 8);  // 内角点数量
    float square_size = 0.015f;  // 棋盘格方格尺寸 (米)
    int num_images = 12;  // 图片数量
    
    // 准备3D世界坐标点
    vector<Point3f> world_corners;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            world_corners.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));
        }
    }
    
    // 4. 处理每张图片，计算相机位姿
    cout << "\n处理图片,计算相机位姿..." << endl;
    
    vector<Mat> R_target2cam;  // 标定板到相机的旋转
    vector<Mat> t_target2cam;  // 标定板到相机的平移
    vector<int> valid_image_indices;
    
    for (int i = 0; i < num_images; i++) {
        cout << "处理图片 " << (i+1) << "/" << num_images << "..." << endl;
        
        // 加载图片 (123.cpp 保存格式 img_000.jpg)
        char img_name[256];
        sprintf(img_name, "%s/img_%03d.jpg", image_folder.c_str(), i);
        string image_path = img_name;
        Mat image = imread(image_path);
        
        if (image.empty()) {
            cerr << "  无法加载图片: " << image_path << endl;
            continue;
        }
        
        // 转换为灰度图
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        
        // 检测棋盘格角点
        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, boardSize, corners,
            CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
        
        if (!found) {
            cerr << "  未检测到棋盘格角点" << endl;
            continue;
        }
        
        // 提高角点精度
        cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
            TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
        
        // 使用solvePnP计算相机位姿
        Mat rvec, tvec;
        bool success = solvePnP(world_corners, corners, camera_matrix, dist_coeffs, rvec, tvec);
        
        if (!success) {
            cerr << "  计算相机位姿失败" << endl;
            continue;
        }
        
        // 将旋转向量转换为旋转矩阵
        Mat R;
        Rodrigues(rvec, R);
        
        R_target2cam.push_back(R);
        t_target2cam.push_back(tvec);
        valid_image_indices.push_back(i);
        
        // 显示第一个相机位姿（用于验证）
        if (valid_image_indices.size() == 1) {
            cout << "第一个相机位姿 - 平移向量: " << tvec.t() << " 米" << endl;
            cout << "长度: " << norm(tvec) << " 米" << endl;
        }
    }
    
    cout << "成功计算相机位姿: " << R_target2cam.size() << "/" << num_images << " 张图片" << endl;
    
    if (R_target2cam.size() < 5) {
        cerr << "错误: 需要至少5个成功的位姿进行手眼标定" << endl;
        return 1;
    }
    
    // 5. 加载机械臂末端位姿（从ee_pose.txt）
    cout << "\n加载机械臂末端位姿..." << endl;

    map<int, vector<double>> ee_pose_map;
    bool has_ee_pose = loadEndEffectorPosesFromFile(ee_pose_file, ee_pose_map);
    if (!has_ee_pose) {
        cerr << "错误: 无法加载末端位姿文件: " << ee_pose_file << endl;
        return 1;
    }

    cout << "已加载末端位姿文件: " << ee_pose_file << "（共 " << ee_pose_map.size() << " 条）" << endl;

    vector<Mat> R_gripper2base;  // 末端到基座的旋转
    vector<Mat> t_gripper2base;  // 末端到基座的平移

    for (int idx : valid_image_indices) {
        if (ee_pose_map.count(idx)) {
            auto &p = ee_pose_map[idx];
            if (p.size() == 7) {
                double x = p[0], y = p[1], z = p[2];
                double qx = p[3], qy = p[4], qz = p[5], qw = p[6];

                Mat R = quaternionToRotationMatrix(qx, qy, qz, qw);
                Mat t = (Mat_<double>(3,1) << x, y, z);

                R_gripper2base.push_back(R);
                t_gripper2base.push_back(t);

                if (R_gripper2base.size() == 1) {
                    cout << "第一个末端位姿 - 平移向量: " << t.t() << " 米" << endl;
                    cout << "长度: " << norm(t) << " 米" << endl;
                }
            } else {
                cerr << "警告: 末端位姿数据格式错误，索引 " << idx << endl;
            }
        } else {
            cerr << "警告: 缺少末端位姿数据，索引 " << idx << endl;
        }
    }
    
    cout << "计算机械臂末端位姿: " << R_gripper2base.size() << " 个" << endl;
    
    // 检查数据一致性
    if (R_gripper2base.size() != R_target2cam.size()) {
        cerr << "错误: 机械臂位姿数量 (" << R_gripper2base.size() 
             << ") 与相机位姿数量 (" << R_target2cam.size() << ") 不匹配" << endl;
        return 1;
    }
    
    // 6. 执行手眼标定
    cout << "\n执行手眼标定..." << endl;
    
    Mat R_cam2gripper, t_cam2gripper;
    
    // 使用不同的算法进行计算
    vector<HandEyeCalibrationMethod> methods = {
        CALIB_HAND_EYE_TSAI, 
        CALIB_HAND_EYE_PARK, 
        CALIB_HAND_EYE_HORAUD, 
        CALIB_HAND_EYE_ANDREFF,
        CALIB_HAND_EYE_DANIILIDIS
    };
    vector<string> method_names = {"TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS"};
    
    vector<Mat> R_results;
    vector<Mat> t_results;
    vector<double> errors;
    
    for (size_t i = 0; i < methods.size(); i++) {
        try {
            Mat R_temp, t_temp;
            
            // calibrateHandEye 不返回bool，它会直接计算结果
            calibrateHandEye(R_gripper2base, t_gripper2base, 
                           R_target2cam, t_target2cam,
                           R_temp, t_temp, methods[i]);
            
            R_results.push_back(R_temp);
            t_results.push_back(t_temp);
            
            cout << "\n算法 " << method_names[i] << " 结果:" << endl;
            cout << "旋转矩阵 R:" << endl << R_temp << endl;
            cout << "平移向量 t (米): " << t_temp.t() << endl;
            
            // 检查旋转矩阵的有效性
            double det = determinant(R_temp);
            cout << "行列式 det(R) = " << det << " (应接近±1)" << endl;
            
            // 计算误差
            double error_sum = 0.0;
            int error_count = 0;
            
            // 使用这个标定结果验证一致性
            for (size_t j = 0; j < R_target2cam.size(); j++) {
                for (size_t k = j+1; k < R_target2cam.size(); k++) {
                    Mat T_gripper2base_j = Mat::eye(4, 4, CV_64F);
                    R_gripper2base[j].copyTo(T_gripper2base_j(Rect(0, 0, 3, 3)));
                    t_gripper2base[j].copyTo(T_gripper2base_j(Rect(3, 0, 1, 3)));
                    
                    Mat T_target2cam_j = Mat::eye(4, 4, CV_64F);
                    R_target2cam[j].copyTo(T_target2cam_j(Rect(0, 0, 3, 3)));
                    t_target2cam[j].copyTo(T_target2cam_j(Rect(3, 0, 1, 3)));
                    
                    Mat T_temp = Mat::eye(4,4,CV_64F);
                    R_temp.copyTo(T_temp(Rect(0,0,3,3)));
                    t_temp.copyTo(T_temp(Rect(3,0,1,3)));
                    
                    Mat T_gripper2base_k = Mat::eye(4, 4, CV_64F);
                    R_gripper2base[k].copyTo(T_gripper2base_k(Rect(0, 0, 3, 3)));
                    t_gripper2base[k].copyTo(T_gripper2base_k(Rect(3, 0, 1, 3)));
                    
                    Mat T_target2cam_k = Mat::eye(4, 4, CV_64F);
                    R_target2cam[k].copyTo(T_target2cam_k(Rect(0, 0, 3, 3)));
                    t_target2cam[k].copyTo(T_target2cam_k(Rect(3, 0, 1, 3)));
                    
                    Mat board_in_base_j = T_gripper2base_j * T_temp * T_target2cam_j;
                    Mat board_in_base_k = T_gripper2base_k * T_temp * T_target2cam_k;
                    
                    error_sum += norm(board_in_base_j(Rect(3,0,1,3)) - board_in_base_k(Rect(3,0,1,3)));
                    error_count++;
                }
            }
            
            double avg_error = (error_count > 0) ? error_sum / error_count : 0.0;
            errors.push_back(avg_error);
            cout << "平均误差: " << avg_error * 1000 << " 毫米" << endl;
            
        } catch (const exception& e) {
            cerr << "算法 " << method_names[i] << " 异常: " << e.what() << endl;
            // 添加空结果
            R_results.push_back(Mat());
            t_results.push_back(Mat());
            errors.push_back(1e6); // 大误差值
        }
    }
    
    if (R_results.empty()) {
        cerr << "错误: 所有手眼标定算法都失败了" << endl;
        return 1;
    }
    
    // 选择最佳结果（最小误差的算法）
    int best_index = 0;
    double min_error = 1e6;
    for (size_t i = 0; i < errors.size(); i++) {
        if (errors[i] < min_error) {
            min_error = errors[i];
            best_index = i;
        }
    }
    
    R_cam2gripper = R_results[0];
    t_cam2gripper = t_results[0];
    
    // 7. 创建完整的变换矩阵
    cout << "\n=== 手眼标定结果 ===" << endl;
    cout << "使用算法: " << method_names[0] << " (误差: " << min_error * 1000 << " 毫米)" << endl;
    
    Mat T_cam2gripper = Mat::eye(4, 4, CV_64F);
    R_cam2gripper.copyTo(T_cam2gripper(Rect(0, 0, 3, 3)));
    t_cam2gripper.copyTo(T_cam2gripper(Rect(3, 0, 1, 3)));
    
    cout << "相机到机械臂末端的变换矩阵 T_cam2gripper:" << endl;
    cout << T_cam2gripper << endl;
    
    // 计算逆变换
    Mat T_gripper2cam = T_cam2gripper.inv();
    cout << "\n机械臂末端到相机的变换矩阵 T_gripper2cam:" << endl;
    cout << T_gripper2cam << endl;
    
    // 8. 验证结果
    cout << "\n验证手眼标定结果..." << endl;
    
    double total_error = 0.0;
    int count = 0;
    
    // 使用所有数据点计算标定板在基座坐标系中的位置
    vector<Point3d> board_positions;
    
    for (size_t i = 0; i < R_target2cam.size(); i++) {
        // 计算标定板在基座坐标系中的位置
        Mat T_gripper2base_i = Mat::eye(4, 4, CV_64F);
        R_gripper2base[i].copyTo(T_gripper2base_i(Rect(0, 0, 3, 3)));
        t_gripper2base[i].copyTo(T_gripper2base_i(Rect(3, 0, 1, 3)));
        
        Mat T_target2cam_i = Mat::eye(4, 4, CV_64F);
        R_target2cam[i].copyTo(T_target2cam_i(Rect(0, 0, 3, 3)));
        t_target2cam[i].copyTo(T_target2cam_i(Rect(3, 0, 1, 3)));
        
        Mat board_in_base_i = T_gripper2base_i * T_cam2gripper * T_target2cam_i;
        
        // 保存标定板位置
        Point3d board_pos(
            board_in_base_i.at<double>(0, 3),
            board_in_base_i.at<double>(1, 3),
            board_in_base_i.at<double>(2, 3)
        );
        board_positions.push_back(board_pos);
        
        // 显示第一个位置
        if (i == 0) {
            cout << "标定板在基座坐标系中的位置 (第一个位姿):" << endl;
            cout << board_in_base_i << endl;
            cout << "坐标: (" << board_pos.x << ", " << board_pos.y << ", " << board_pos.z << ") 米" << endl;
        }
    }
    
    // 计算所有标定板位置的平均值和标准差
    if (!board_positions.empty()) {
        Point3d avg_pos(0, 0, 0);
        for (const auto& pos : board_positions) {
            avg_pos += pos;
        }
        avg_pos.x /= board_positions.size();
        avg_pos.y /= board_positions.size();
        avg_pos.z /= board_positions.size();
        
        Point3d std_dev(0, 0, 0);
        for (const auto& pos : board_positions) {
            std_dev.x += pow(pos.x - avg_pos.x, 2);
            std_dev.y += pow(pos.y - avg_pos.y, 2);
            std_dev.z += pow(pos.z - avg_pos.z, 2);
        }
        std_dev.x = sqrt(std_dev.x / board_positions.size());
        std_dev.y = sqrt(std_dev.y / board_positions.size());
        std_dev.z = sqrt(std_dev.z / board_positions.size());
        
        cout << "\n标定板位置统计:" << endl;
        cout << "平均值: (" << avg_pos.x << ", " << avg_pos.y << ", " << avg_pos.z << ") 米" << endl;
        cout << "标准差: (" << std_dev.x*1000 << ", " << std_dev.y*1000 << ", " << std_dev.z*1000 << ") 毫米" << endl;
        
        // 计算最大误差
        double max_error = 0;
        for (size_t i = 0; i < board_positions.size(); i++) {
            for (size_t j = i+1; j < board_positions.size(); j++) {
                double error = norm(board_positions[i] - board_positions[j]);
                total_error += error;
                count++;
                if (error > max_error) max_error = error;
            }
        }
        
        double avg_error = (count > 0) ? total_error / count : 0.0;
        cout << "\n手眼标定误差统计:" << endl;
        cout << "平均位置误差: " << avg_error * 1000 << " 毫米" << endl;
        cout << "最大位置误差: " << max_error * 1000 << " 毫米" << endl;
        
        if (avg_error < 0.005) {  // 5毫米
            cout << "✓ 手眼标定精度良好" << endl;
        } else if (avg_error < 0.01) {  // 10毫米
            cout << "⚠ 手眼标定精度一般" << endl;
        } else {
            cout << "✗ 手眼标定精度较差" << endl;
        }
    }
    
    // 9. 保存结果
    cout << "\n保存手眼标定结果..." << endl;
    
    // 创建目录
    system("mkdir -p ./calibration_results");
    
    FileStorage fs_result("./calibration_results/hand_eye_result.xml", FileStorage::WRITE);
    fs_result << "T_cam2gripper" << T_cam2gripper;
    fs_result << "R_cam2gripper" << R_cam2gripper;
    fs_result << "t_cam2gripper" << t_cam2gripper;
    fs_result << "T_gripper2cam" << T_gripper2cam;
    fs_result << "num_images" << (int)R_target2cam.size();
    fs_result << "method" << method_names[best_index];
    fs_result << "square_size" << square_size;
    fs_result << "board_width" << boardSize.width;
    fs_result << "board_height" << boardSize.height;
    fs_result << "avg_error_mm" << (total_error/count)*1000;
    fs_result.release();
    
    cout << "结果已保存到: ./calibration_results/hand_eye_result.xml" << endl;
    
    cout << "\n=== 手眼标定完成（仅使用ee_pose.txt）===" << endl;
    
    return 0;
}