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
    // CoppeliaSim使用Hamilton约定，返回[x, y, z, qx, qy, qz, qw]
    // 但poseToMatrix内部可能使用不同的顺序
    
    double norm = sqrt(qx*qx + qy*qy + qz*qz + qw*qw);
    if (norm < 1e-12) {
        return Mat::eye(3, 3, CV_64F);
    }
    
    // 归一化
    double x = qx / norm;
    double y = qy / norm;
    double z = qz / norm;
    double w = qw / norm;
    
    Mat R = Mat::eye(3, 3, CV_64F);
    
    // 测试两种可能的公式
    
    // 公式1：Hamilton (w, x, y, z)
    R.at<double>(0,0) = 1 - 2*y*y - 2*z*z;
    R.at<double>(0,1) = 2*x*y - 2*z*w;
    R.at<double>(0,2) = 2*x*z + 2*y*w;
    
    R.at<double>(1,0) = 2*x*y + 2*z*w;
    R.at<double>(1,1) = 1 - 2*x*x - 2*z*z;
    R.at<double>(1,2) = 2*y*z - 2*x*w;
    
    R.at<double>(2,0) = 2*x*z - 2*y*w;
    R.at<double>(2,1) = 2*y*z + 2*x*w;
    R.at<double>(2,2) = 1 - 2*x*x - 2*y*y;
    
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

// 计算旋转矩阵的旋转角度（度）
double rotationMatrixToAngle(const Mat& R) {
    double trace = R.at<double>(0,0) + R.at<double>(1,1) + R.at<double>(2,2);
    trace = min(max(trace, -1.0), 1.0);
    double angle = acos((trace - 1.0) / 2.0) * 180.0 / CV_PI;
    return angle;
}

// 计算重投影误差
void computeReprojectionErrors(const vector<Mat>& R_gripper2base,
                               const vector<Mat>& t_gripper2base,
                               const vector<Mat>& R_target2cam,
                               const vector<Mat>& t_target2cam,
                               const Mat& R_cam2gripper,
                               const Mat& t_cam2gripper,
                               double& avg_rotation_error_deg,
                               double& avg_translation_error_mm,
                               double& max_translation_error_mm,
                               double& std_translation_error_mm) {
    
    vector<double> translation_errors_mm;
    vector<double> rotation_errors_deg;
    
    // 计算标定板在基座坐标系中的位置
    vector<Point3d> board_positions;
    
    Mat T_cam2gripper_mat = Mat::eye(4, 4, CV_64F);
    R_cam2gripper.copyTo(T_cam2gripper_mat(Rect(0, 0, 3, 3)));
    t_cam2gripper.copyTo(T_cam2gripper_mat(Rect(3, 0, 1, 3)));
    
    for (size_t i = 0; i < R_target2cam.size(); i++) {
        // 构建变换矩阵
        Mat T_gripper2base_i = Mat::eye(4, 4, CV_64F);
        R_gripper2base[i].copyTo(T_gripper2base_i(Rect(0, 0, 3, 3)));
        t_gripper2base[i].copyTo(T_gripper2base_i(Rect(3, 0, 1, 3)));
        
        Mat T_target2cam_i = Mat::eye(4, 4, CV_64F);
        R_target2cam[i].copyTo(T_target2cam_i(Rect(0, 0, 3, 3)));
        t_target2cam[i].copyTo(T_target2cam_i(Rect(3, 0, 1, 3)));
        
        // 计算标定板在基座坐标系中的位置
        // board_in_base = T_gripper2base * T_cam2gripper * T_target2cam
        Mat board_in_base_i = T_gripper2base_i * T_cam2gripper_mat * T_target2cam_i;
        
        Point3d board_pos(
            board_in_base_i.at<double>(0, 3),
            board_in_base_i.at<double>(1, 3),
            board_in_base_i.at<double>(2, 3)
        );
        board_positions.push_back(board_pos);
    }
    
    // 计算标定板位置的平均值
    Point3d avg_board_pos(0, 0, 0);
    for (const auto& pos : board_positions) {
        avg_board_pos += pos;
    }
    avg_board_pos.x /= board_positions.size();
    avg_board_pos.y /= board_positions.size();
    avg_board_pos.z /= board_positions.size();
    
    // 计算每个位置的重投影误差
    for (size_t i = 0; i < board_positions.size(); i++) {
        // 计算标定板位置的误差
        double pos_error = norm(board_positions[i] - avg_board_pos);
        translation_errors_mm.push_back(pos_error * 1000.0);
        
        // 计算标定板姿态的误差
        Mat T_gripper2base_i = Mat::eye(4, 4, CV_64F);
        R_gripper2base[i].copyTo(T_gripper2base_i(Rect(0, 0, 3, 3)));
        t_gripper2base[i].copyTo(T_gripper2base_i(Rect(3, 0, 1, 3)));
        
        Mat T_target2cam_i = Mat::eye(4, 4, CV_64F);
        R_target2cam[i].copyTo(T_target2cam_i(Rect(0, 0, 3, 3)));
        t_target2cam[i].copyTo(T_target2cam_i(Rect(3, 0, 1, 3)));
        
        // 通过手眼标定计算的标定板在基座中的位姿
        Mat board_in_base_i_calculated = T_gripper2base_i * T_cam2gripper_mat * T_target2cam_i;
        
        // 通过平均位置计算的标定板在基座中的位姿
        Mat board_in_base_i_avg = Mat::eye(4, 4, CV_64F);
        board_in_base_i_avg.at<double>(0, 3) = avg_board_pos.x;
        board_in_base_i_avg.at<double>(1, 3) = avg_board_pos.y;
        board_in_base_i_avg.at<double>(2, 3) = avg_board_pos.z;
        
        // 从计算的位置到平均位置的反变换
        Mat T_error = board_in_base_i_calculated.inv() * board_in_base_i_avg;
        double rot_error = rotationMatrixToAngle(T_error(Rect(0, 0, 3, 3)));
        rotation_errors_deg.push_back(rot_error);
    }
    
    // 计算统计信息
    avg_translation_error_mm = 0;
    avg_rotation_error_deg = 0;
    max_translation_error_mm = 0;
    std_translation_error_mm = 0;
    
    if (!translation_errors_mm.empty()) {
        // 计算平均值
        for (double err : translation_errors_mm) {
            avg_translation_error_mm += err;
            if (err > max_translation_error_mm) {
                max_translation_error_mm = err;
            }
        }
        avg_translation_error_mm /= translation_errors_mm.size();
        
        // 计算标准差
        double variance = 0;
        for (double err : translation_errors_mm) {
            variance += pow(err - avg_translation_error_mm, 2);
        }
        variance /= translation_errors_mm.size();
        std_translation_error_mm = sqrt(variance);
    }
    
    if (!rotation_errors_deg.empty()) {
        for (double err : rotation_errors_deg) {
            avg_rotation_error_deg += err;
        }
        avg_rotation_error_deg /= rotation_errors_deg.size();
    }
}

int main() {
    cout << "=== 手眼标定程序（使用重投影误差评估精度）===" << endl;
    
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
    string image_folder = "/home/eric/undergraduate_thesis/build/image/hand_eye_new_1774424305";
    string ee_pose_file = image_folder + "/ee_pose.txt";  // 由123.cpp保存

    // 棋盘格参数
    Size boardSize(11, 8);  // 内角点数量
    float square_size = 0.015f;  // 棋盘格方格尺寸 (米)
    int num_images = 20;  // 图片数量
    
    // 准备3D世界坐标点
    vector<Point3f> world_corners;
    for (int i = 0; i < boardSize.height; i++) {
        for (int j = 0; j < boardSize.width; j++) {
            world_corners.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));
        }
    }
    
    // 3. 处理每张图片，计算相机位姿
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
    
    // 4. 加载机械臂末端位姿（从ee_pose.txt）
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
    
    // 5. 执行手眼标定
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
    vector<double> reproj_trans_errors;  // 重投影平移误差
    vector<double> reproj_rot_errors;    // 重投影旋转误差
    
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
            
            // 计算该算法的重投影误差
            double avg_rot_err, avg_trans_err, max_trans_err, std_trans_err;
            computeReprojectionErrors(R_gripper2base, t_gripper2base,
                                     R_target2cam, t_target2cam,
                                     R_temp, t_temp,
                                     avg_rot_err, avg_trans_err, max_trans_err, std_trans_err);
            
            reproj_rot_errors.push_back(avg_rot_err);
            reproj_trans_errors.push_back(avg_trans_err);
            
            cout << "重投影误差统计:" << endl;
            cout << "  平均旋转误差: " << avg_rot_err << " 度" << endl;
            cout << "  平均平移误差: " << avg_trans_err << " 毫米" << endl;
            cout << "  最大平移误差: " << max_trans_err << " 毫米" << endl;
            cout << "  平移误差标准差: " << std_trans_err << " 毫米" << endl;
            
        } catch (const exception& e) {
            cerr << "算法 " << method_names[i] << " 异常: " << e.what() << endl;
            // 添加空结果
            R_results.push_back(Mat());
            t_results.push_back(Mat());
            reproj_rot_errors.push_back(1e6);
            reproj_trans_errors.push_back(1e6);
        }
    }
    
    if (R_results.empty()) {
        cerr << "错误: 所有手眼标定算法都失败了" << endl;
        return 1;
    }
    
    // 选择最佳结果（最小重投影误差的算法）
    int best_index = 0;
    double min_reproj_error = 1e6;
    for (size_t i = 0; i < reproj_trans_errors.size(); i++) {
        // 综合误差 = 平移误差 + 旋转误差权重
        double combined_error = reproj_trans_errors[i] + reproj_rot_errors[i] * 0.1;  // 旋转误差权重0.1
        if (combined_error < min_reproj_error && !R_results[i].empty()) {
            min_reproj_error = combined_error;
            best_index = i;
        }
    }
    
    R_cam2gripper = R_results[best_index];
    t_cam2gripper = t_results[best_index];
    
    // 6. 计算最佳算法的详细重投影误差
    cout << "\n=== 手眼标定结果 ===" << endl;
    cout << "使用算法: " << method_names[best_index] << endl;
    
    double avg_rot_err, avg_trans_err, max_trans_err, std_trans_err;
    computeReprojectionErrors(R_gripper2base, t_gripper2base,
                             R_target2cam, t_target2cam,
                             R_cam2gripper, t_cam2gripper,
                             avg_rot_err, avg_trans_err, max_trans_err, std_trans_err);
    
    cout << "\n最佳算法的重投影误差统计:" << endl;
    cout << "  平均旋转误差: " << avg_rot_err << " 度" << endl;
    cout << "  平均平移误差: " << avg_trans_err << " 毫米" << endl;
    cout << "  最大平移误差: " << max_trans_err << " 毫米" << endl;
    cout << "  平移误差标准差: " << std_trans_err << " 毫米" << endl;
    
    // 创建完整的变换矩阵
    Mat T_cam2gripper = Mat::eye(4, 4, CV_64F);
    R_cam2gripper.copyTo(T_cam2gripper(Rect(0, 0, 3, 3)));
    t_cam2gripper.copyTo(T_cam2gripper(Rect(3, 0, 1, 3)));
    
    cout << "\n相机到机械臂末端的变换矩阵 T_cam2gripper:" << endl;
    cout << T_cam2gripper << endl;
    
    // 计算逆变换
    Mat T_gripper2cam = T_cam2gripper.inv();
    cout << "\n机械臂末端到相机的变换矩阵 T_gripper2cam:" << endl;
    cout << T_gripper2cam << endl;
    
    // 7. 评估标定质量
    cout << "\n=== 标定质量评估 ===" << endl;
    
    if (avg_trans_err < 5.0 && avg_rot_err < 2.0) {
        cout << "  ✓ 手眼标定精度优秀" << endl;
        cout << "  ✓ 满足高精度应用需求" << endl;
    } else if (avg_trans_err < 10.0 && avg_rot_err < 5.0) {
        cout << "  ⚠ 手眼标定精度良好" << endl;
        cout << "  ⚠ 适用于一般工业应用" << endl;
    } else if (avg_trans_err < 20.0 && avg_rot_err < 10.0) {
        cout << "  ⚠ 手眼标定精度一般" << endl;
        cout << "  ⚠ 建议优化标定" << endl;
    } else {
        cout << "  ✗ 手眼标定精度不足" << endl;
        cout << "  ✗ 需要重新标定" << endl;
    }
    
    // 8. 保存结果
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
    
    // 保存重投影误差
    fs_result << "avg_reprojection_rotation_error_deg" << avg_rot_err;
    fs_result << "avg_reprojection_translation_error_mm" << avg_trans_err;
    fs_result << "max_reprojection_translation_error_mm" << max_trans_err;
    fs_result << "std_reprojection_translation_error_mm" << std_trans_err;
    
    // 保存所有算法的误差
    for (size_t i = 0; i < methods.size(); i++) {
        string prefix = "method_" + method_names[i] + "_";
        fs_result << (prefix + "trans_error_mm") << reproj_trans_errors[i];
        fs_result << (prefix + "rot_error_deg") << reproj_rot_errors[i];
    }
    
    fs_result.release();
    
    cout << "结果已保存到: ./calibration_results/hand_eye_result.xml" << endl;
    
    // 9. 输出建议
    cout << "\n=== 建议 ===" << endl;
    if (avg_trans_err > 10.0) {
        cout << "1. 检查标定板尺寸是否正确（当前: " << square_size * 1000 << " mm）" << endl;
        cout << "2. 检查标定图片质量，确保棋盘格清晰可见" << endl;
        cout << "3. 增加标定图片数量，建议至少15-20张" << endl;
        cout << "4. 确保机械臂在标定时稳定，无振动" << endl;
        cout << "5. 检查相机内参标定是否准确" << endl;
    } else {
        cout << "手眼标定成功！可以使用标定结果进行视觉引导操作。" << endl;
    }
    
    cout << "\n=== 手眼标定完成 ===" << endl;
    
    return 0;
}