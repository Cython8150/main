#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <filesystem>
#include <sstream>

using namespace std;
using namespace cv;
namespace fs = std::filesystem;

// 与 123.cpp / eye_in_hand_calibration.cpp 一致：CoppeliaSim 末端位姿 [x y z qx qy qz qw]
Mat quaternionToRotationMatrix(double qx, double qy, double qz, double qw) {
    double norm = sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
    if (norm < 1e-12)
        return Mat::eye(3, 3, CV_64F);
    double x = qx / norm, y = qy / norm, z = qz / norm, w = qw / norm;
    Mat R = Mat::eye(3, 3, CV_64F);
    R.at<double>(0, 0) = 1 - 2 * y * y - 2 * z * z;
    R.at<double>(0, 1) = 2 * x * y - 2 * z * w;
    R.at<double>(0, 2) = 2 * x * z + 2 * y * w;
    R.at<double>(1, 0) = 2 * x * y + 2 * z * w;
    R.at<double>(1, 1) = 1 - 2 * x * x - 2 * z * z;
    R.at<double>(1, 2) = 2 * y * z - 2 * x * w;
    R.at<double>(2, 0) = 2 * x * z - 2 * y * w;
    R.at<double>(2, 1) = 2 * y * z + 2 * x * w;
    R.at<double>(2, 2) = 1 - 2 * x * x - 2 * y * y;
    return R;
}

bool loadEndEffectorPosesFromFile(const string& path, map<int, vector<double>>& ee_pose_map) {
    ifstream fin(path);
    if (!fin.is_open()) {
        cerr << "无法打开末端位姿文件: " << path << endl;
        return false;
    }
    string line;
    int current_idx = -1;
    while (getline(fin, line)) {
        if (line.empty())
            continue;
        if (line.rfind("Image", 0) == 0) {
            size_t pos = line.find(' ');
            if (pos == string::npos)
                continue;
            size_t colon = line.find(':', pos);
            string numstr = (colon == string::npos) ? line.substr(pos + 1) : line.substr(pos + 1, colon - pos - 1);
            try {
                current_idx = stoi(numstr);
            } catch (...) {
                current_idx = -1;
            }
        } else if (line.find("End-effector pose:") != string::npos) {
            if (current_idx < 0)
                continue;
            string sp = line.substr(line.find(':') + 1);
            stringstream ss(sp);
            double x, y, z, qx, qy, qz, qw;
            if (ss >> x >> y >> z >> qx >> qy >> qz >> qw)
                ee_pose_map[current_idx] = {x, y, z, qx, qy, qz, qw};
        }
    }
    return !ee_pose_map.empty();
}

static Mat RtToT(const Mat& R, const Mat& t) {
    Mat T = Mat::eye(4, 4, CV_64F);
    R.copyTo(T(Rect(0, 0, 3, 3)));
    t.copyTo(T(Rect(3, 0, 1, 3)));
    return T;
}

static double rotationMatrixToAngleDeg(const Mat& R) {
    // trace(R) in [-1,3] for SO(3); cos(theta)=(trace-1)/2 must be clamped, not trace itself
    double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
    double c = 0.5 * (trace - 1.0);
    c = min(max(c, -1.0), 1.0);
    return acos(c) * 180.0 / CV_PI;
}

// 眼在手外：OpenCV 解得的 X 为 ^{b}T_c。自检链 ^{g}T_b * ^{b}T_c * ^{c}T_t 应恒定。
// 第一组参数须为 calibrateHandEye 所用的 Hg：对眼在手外，官方测试传入的是 inv(^{b}T_g)=^{g}T_b
//（见 opencv modules/calib3d/test/test_calibration_hand_eye.cpp 中 simulateDataEyeToHand），
// 与头文件里参数名 R_gripper2base 的字面含义相反，易踩坑。
void computeEyeToHandConsistency(const vector<Mat>& R_gripper_pose_for_handeye,
                                 const vector<Mat>& t_gripper_pose_for_handeye,
                                 const vector<Mat>& R_target2cam,
                                 const vector<Mat>& t_target2cam,
                                 const Mat& R_base_from_cam,
                                 const Mat& t_base_from_cam,
                                 double& avg_trans_mm,
                                 double& avg_rot_deg) {
    Mat T_bc = RtToT(R_base_from_cam, t_base_from_cam);
    vector<Point3d> origins;
    vector<double> rot_errs;
    Mat T_ref = Mat::eye(4, 4, CV_64F);
    bool have_ref = false;

    for (size_t i = 0; i < R_gripper_pose_for_handeye.size(); i++) {
        Mat T_gb = RtToT(R_gripper_pose_for_handeye[i], t_gripper_pose_for_handeye[i]);
        Mat T_tc = RtToT(R_target2cam[i], t_target2cam[i]);
        Mat T_gt = T_gb * T_bc * T_tc;
        origins.push_back(Point3d(T_gt.at<double>(0, 3), T_gt.at<double>(1, 3), T_gt.at<double>(2, 3)));
        if (!have_ref) {
            T_ref = T_gt.clone();
            have_ref = true;
        } else {
            Mat Terr = T_ref.inv() * T_gt;
            rot_errs.push_back(rotationMatrixToAngleDeg(Terr(Rect(0, 0, 3, 3))));
        }
    }
    Point3d mean(0, 0, 0);
    for (const auto& o : origins)
        mean += o;
    mean.x /= origins.size();
    mean.y /= origins.size();
    mean.z /= origins.size();
    avg_trans_mm = 0;
    for (const auto& o : origins)
        avg_trans_mm += norm(Point3d(o.x - mean.x, o.y - mean.y, o.z - mean.z));
    avg_trans_mm = (avg_trans_mm / origins.size()) * 1000.0;

    avg_rot_deg = 0;
    for (double e : rot_errs)
        avg_rot_deg += e;
    if (!rot_errs.empty())
        avg_rot_deg /= rot_errs.size();
}

int main(int argc, char** argv) {
    cout << "=== 眼在手外（相机固定、标定板在末端）手眼标定 ===" << endl;
    cout << "数据格式与 123.cpp 一致：img_XXX.jpg + ee_pose.txt（末端相对基座 [x y z qx qy qz qw]）\n"
         << endl;

    string image_folder = "./image/hand_eye_new_1775897010";
    string camera_xml = "./calibration_results/cameraParaments.xml";
    if (argc >= 2)
        image_folder = argv[1];
    if (argc >= 3)
        camera_xml = argv[2];

    Size boardSize(11, 8);
    float square_size = 0.015f;

    cout << "图像目录: " << image_folder << endl;
    cout << "相机内参: " << camera_xml << endl;

    FileStorage fs_camera(camera_xml, FileStorage::READ);
    if (!fs_camera.isOpened()) {
        cerr << "错误: 无法打开相机标定文件（请先用 camera_calibration 生成或在命令行传入路径）" << endl;
        return 1;
    }
    Mat camera_matrix, dist_coeffs;
    fs_camera["intrinsic_matrix"] >> camera_matrix;
    fs_camera["distortion_matrix"] >> dist_coeffs;
    fs_camera.release();

    string ee_pose_file = image_folder + "/ee_pose.txt";
    map<int, vector<double>> ee_pose_map;
    if (!loadEndEffectorPosesFromFile(ee_pose_file, ee_pose_map)) {
        cerr << "错误: 无法读取 " << ee_pose_file << endl;
        return 1;
    }

    vector<Point3f> world_corners;
    for (int i = 0; i < boardSize.height; i++)
        for (int j = 0; j < boardSize.width; j++)
            world_corners.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

    vector<Mat> R_target2cam, t_target2cam;
    vector<Mat> R_b_g, t_b_g;
    vector<int> valid_indices;

    int max_idx = 0;
    for (const auto& kv : ee_pose_map)
        max_idx = max(max_idx, kv.first);

    cout << "\n检测棋盘格并匹配 ee_pose（索引 0.." << max_idx << "）..." << endl;
    for (int idx = 0; idx <= max_idx; idx++) {
        if (!ee_pose_map.count(idx))
            continue;
        ostringstream oss;
        oss << image_folder << "/img_" << setfill('0') << setw(3) << idx << ".jpg";
        string image_path = oss.str();
        if (!fs::exists(image_path)) {
            cerr << "  跳过索引 " << idx << "：无图像 " << image_path << endl;
            continue;
        }
        Mat image = imread(image_path);
        if (image.empty()) {
            cerr << "  跳过索引 " << idx << "：读图失败 " << image_path << endl;
            continue;
        }
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);
        vector<Point2f> corners;
        bool found = findChessboardCorners(gray, boardSize, corners,
                                           CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
        if (!found) {
            cerr << "  索引 " << idx << "：未检测到棋盘格" << endl;
            continue;
        }
        cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1),
                     TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
        Mat rvec, tvec;
        if (!solvePnP(world_corners, corners, camera_matrix, dist_coeffs, rvec, tvec)) {
            cerr << "  索引 " << idx << "：solvePnP 失败" << endl;
            continue;
        }
        Mat R;
        Rodrigues(rvec, R);
        const auto& p = ee_pose_map[idx];
        Mat Rg = quaternionToRotationMatrix(p[3], p[4], p[5], p[6]);
        Mat tg = (Mat_<double>(3, 1) << p[0], p[1], p[2]);

        R_target2cam.push_back(R);
        t_target2cam.push_back(tvec);
        R_b_g.push_back(Rg);
        t_b_g.push_back(tg);
        valid_indices.push_back(idx);
    }

    if (R_target2cam.size() < 3) {
        cerr << "错误: 有效位姿少于 3 组（OpenCV 建议明显多于 3）。请增加采集角度与覆盖范围。" << endl;
        return 1;
    }

    cout << "有效样本数: " << R_target2cam.size() << endl;

    // ee_pose 为 ^{b}T_g（Coppelia getObjectPose(ee,base)）。眼在手外时 calibrateHandEye 第一参须传 inv(^{b}T_g)=^{g}T_b，
    // 与 OpenCV 自带测试 simulateDataEyeToHand 一致；若直接传 ^{b}T_g，解出的 ^{b}T_c 会完全错误（自检离散极大）。
    vector<Mat> R_g_b, t_g_b;
    for (size_t i = 0; i < R_b_g.size(); i++) {
        Mat Rinv = R_b_g[i].t();
        Mat tinv = -Rinv * t_b_g[i];
        R_g_b.push_back(Rinv);
        t_g_b.push_back(tinv);
    }

    vector<HandEyeCalibrationMethod> methods = {
        CALIB_HAND_EYE_TSAI,      CALIB_HAND_EYE_PARK, CALIB_HAND_EYE_HORAUD,
        CALIB_HAND_EYE_ANDREFF,   CALIB_HAND_EYE_DANIILIDIS};
    vector<string> method_names = {"TSAI", "PARK", "HORAUD", "ANDREFF", "DANIILIDIS"};

    cout << "\n执行 calibrateHandEye（眼在手外：第一参为 inv(ee_pose 的 ^{b}T_g)，与 OpenCV 测试代码一致）..." << endl;

    int best_i = 0;
    double best_score = 1e300;
    Mat R_best, t_best;

    for (size_t mi = 0; mi < methods.size(); mi++) {
        Mat R_bc, t_bc;
        try {
            calibrateHandEye(R_g_b, t_g_b, R_target2cam, t_target2cam, R_bc, t_bc, methods[mi]);
        } catch (const exception& e) {
            cerr << "  " << method_names[mi] << " 异常: " << e.what() << endl;
            continue;
        }
        double det = determinant(R_bc);
        if (abs(abs(det) - 1.0) > 0.05) {
            cerr << "  " << method_names[mi] << "：det(R) 异常 " << det << "，跳过" << endl;
            continue;
        }
        double avg_mm, avg_deg;
        computeEyeToHandConsistency(R_g_b, t_g_b, R_target2cam, t_target2cam, R_bc, t_bc, avg_mm, avg_deg);
        double score = avg_mm + 0.1 * avg_deg;
        cout << "  " << method_names[mi] << "：标定板在末端系下平移离散 " << fixed << setprecision(3) << avg_mm
             << " mm，相对首帧旋转差均值 " << setprecision(3) << avg_deg << " deg" << endl;
        if (score < best_score) {
            best_score = score;
            best_i = (int)mi;
            R_best = R_bc.clone();
            t_best = t_bc.clone();
        }
    }

    if (R_best.empty()) {
        cerr << "错误: 所有算法均未得到有效结果。" << endl;
        return 1;
    }

    Mat T_base_from_cam = RtToT(R_best, t_best);
    Mat T_cam_from_base = T_base_from_cam.inv();

    double avg_mm, avg_deg;
    computeEyeToHandConsistency(R_g_b, t_g_b, R_target2cam, t_target2cam, R_best, t_best, avg_mm, avg_deg);

    cout << "\n=== 结果（最佳: " << method_names[best_i] << "）===" << endl;
    cout << "^{b}T_c（点在相机系 -> 基座系），OpenCV 输出变量名为 R_cam2gripper/t_cam2gripper（眼在手外实为 cam->base）：" << endl;
    cout << "T_base_from_cam =\n"
         << T_base_from_cam << endl;
    cout << "\nT_cam_from_base = inv(T_base_from_cam) =\n"
         << T_cam_from_base << endl;
    cout << "\n一致性（标定板相对末端应刚性不变）：平均平移离散 " << avg_mm << " mm，平均旋转差 " << avg_deg << " deg" << endl;

    fs::create_directories("calibration_results");
    string out_xml = "calibration_results/hand_eye_eye_to_hand_result.xml";
    FileStorage fs(out_xml, FileStorage::WRITE);
    fs << "configuration" << "eye_to_hand";
    fs << "method" << method_names[best_i];
    fs << "T_base_from_cam" << T_base_from_cam;
    fs << "R_base_from_cam" << R_best;
    fs << "t_base_from_cam" << t_best;
    fs << "T_cam_from_base" << T_cam_from_base;
    fs << "num_poses" << (int)R_target2cam.size();
    {
        ostringstream idx_csv;
        for (size_t k = 0; k < valid_indices.size(); k++) {
            if (k)
                idx_csv << ',';
            idx_csv << valid_indices[k];
        }
        fs << "valid_image_indices_csv" << idx_csv.str();
    }
    fs << "avg_board_jitter_translation_mm" << avg_mm;
    fs << "avg_board_jitter_rotation_deg" << avg_deg;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size_m" << square_size;
    fs.release();
    cout << "\n已写入: " << out_xml << endl;
    cout << "\n用法: " << argv[0] << " <采集目录> [cameraParaments.xml]" << endl;

    return 0;
}