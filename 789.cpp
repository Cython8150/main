#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <iomanip>
#include "RemoteAPIClient.h"

using namespace cv;
using namespace std;

static const char* kHandEyeXml = "./calibration_results/hand_eye_eye_to_hand_result.xml";

static Mat quaternionToRotationMatrix(double qx, double qy, double qz, double qw) {
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

static Mat poseVecToT(const vector<double>& p) {
    Mat T = Mat::eye(4, 4, CV_64F);
    if (p.size() < 7)
        return T;
    Mat R = quaternionToRotationMatrix(p[3], p[4], p[5], p[6]);
    R.copyTo(T(Rect(0, 0, 3, 3)));
    T.at<double>(0, 3) = p[0];
    T.at<double>(1, 3) = p[1];
    T.at<double>(2, 3) = p[2];
    return T;
}

static double rotationAngleDegFromR(const Mat& R) {
    double trace = R.at<double>(0, 0) + R.at<double>(1, 1) + R.at<double>(2, 2);
    double c = 0.5 * (trace - 1.0);
    c = min(max(c, -1.0), 1.0);
    return acos(c) * 180.0 / CV_PI;
}

static Mat loadT_base_from_cam(const string& xml_path) {
    FileStorage fs(xml_path, FileStorage::READ);
    if (!fs.isOpened())
        return Mat();
    Mat T;
    fs["T_base_from_cam"] >> T;
    fs.release();
    if (T.empty() || T.rows != 4 || T.cols != 4)
        return Mat();
    T.convertTo(T, CV_64F);
    return T;
}

static vector<double> matrixToPose7(const Mat& T) {
    vector<double> pose(7, 0.0);
    if (T.empty() || T.rows != 4 || T.cols != 4)
        return pose;
    pose[0] = T.at<double>(0, 3);
    pose[1] = T.at<double>(1, 3);
    pose[2] = T.at<double>(2, 3);
    double R00 = T.at<double>(0, 0), R01 = T.at<double>(0, 1), R02 = T.at<double>(0, 2);
    double R10 = T.at<double>(1, 0), R11 = T.at<double>(1, 1), R12 = T.at<double>(1, 2);
    double R20 = T.at<double>(2, 0), R21 = T.at<double>(2, 1), R22 = T.at<double>(2, 2);
    double trace = R00 + R11 + R22;
    if (trace > 0) {
        double S = sqrt(trace + 1.0) * 2.0;
        pose[6] = 0.25 * S;
        pose[3] = (R21 - R12) / S;
        pose[4] = (R02 - R20) / S;
        pose[5] = (R10 - R01) / S;
    } else if ((R00 > R11) && (R00 > R22)) {
        double S = sqrt(1.0 + R00 - R11 - R22) * 2.0;
        pose[6] = (R21 - R12) / S;
        pose[3] = 0.25 * S;
        pose[4] = (R01 + R10) / S;
        pose[5] = (R02 + R20) / S;
    } else if (R11 > R22) {
        double S = sqrt(1.0 + R11 - R00 - R22) * 2.0;
        pose[6] = (R02 - R20) / S;
        pose[3] = (R01 + R10) / S;
        pose[4] = 0.25 * S;
        pose[5] = (R12 + R21) / S;
    } else {
        double S = sqrt(1.0 + R22 - R00 - R11) * 2.0;
        pose[6] = (R10 - R01) / S;
        pose[3] = (R02 + R20) / S;
        pose[4] = (R12 + R21) / S;
        pose[5] = 0.25 * S;
    }
    return pose;
}

vector<double> cal(const string& xml_path = "") {
    string path = xml_path.empty() ? string(kHandEyeXml) : xml_path;
    Mat T = loadT_base_from_cam(path);
    if (T.empty()) {
        cerr << "cal(): 无法加载 T_base_from_cam: " << path << endl;
        return {};
    }
    return matrixToPose7(T);
}

static void printPoseBlock(const char* title, const vector<double>& p) {
    cout << fixed << setprecision(6);
    cout << title << "\n";
    cout << "  xyz: " << p[0] << " " << p[1] << " " << p[2] << " m\n";
    cout << "  quat(qx qy qz qw): " << p[3] << " " << p[4] << " " << p[5] << " " << p[6] << "\n";
}

int main(int argc, char** argv) {
    string xml = (argc >= 2) ? argv[1] : string(kHandEyeXml);

    try {
        RemoteAPIClient client;
        auto sim = client.getObject().sim();
        sim.startSimulation();
        sim.setStepping(true);

        int visionSensorHandle = sim.getObject("/visionSensor");
        int baseHandle = sim.getObject("/UR5");
        if (visionSensorHandle == -1 || baseHandle == -1) {
            cerr << "句柄无效: vision=" << visionSensorHandle << " base=" << baseHandle << endl;
            return 1;
        }
        for (int i = 0; i < 10; i++)
            sim.step();

        vector<double> pose_sim = sim.getObjectPose(visionSensorHandle, baseHandle);
        Mat T_sim = poseVecToT(pose_sim);
        Mat T_file = loadT_base_from_cam(xml);
        if (T_file.empty()) {
            cerr << "无法加载手眼 XML: " << xml << endl;
            return 1;
        }

        vector<double> pose_handeye = cal(xml);
        if (pose_handeye.size() < 7) {
            cerr << "手眼位姿无效" << endl;
            return 1;
        }

        Mat Rf, Rs;
        T_file(Rect(0, 0, 3, 3)).copyTo(Rf);
        T_sim(Rect(0, 0, 3, 3)).copyTo(Rs);

        double dx = pose_sim[0] - pose_handeye[0];
        double dy = pose_sim[1] - pose_handeye[1];
        double dz = pose_sim[2] - pose_handeye[2];
        double pos_mm = sqrt(dx * dx + dy * dy + dz * dz) * 1000.0;

        Mat R_conv = Rf.t() * Rs;
        double rot_direct_deg = rotationAngleDegFromR(R_conv);

        Mat Rs_aligned = Rs * R_conv.t();
        double rot_aligned_deg = rotationAngleDegFromR(Rf.t() * Rs_aligned);
        double r_frob = norm(Rs_aligned - Rf, NORM_L2);

        printPoseBlock("【1 仿真】相机相对基座 getObjectPose", pose_sim);
        printPoseBlock("【2 手眼标定】T_base_from_cam -> cal()（OpenCV 约定）", pose_handeye);

        cout << "【3 误差】仿真【1】与手眼计算【2】\n";
        cout << "  平移: ||xyz_sim - xyz_handeye|| = " << pos_mm << " mm\n";
        cout << "  旋转(直接比 R): angle(R_handeye^T * R_sim) = " << rot_direct_deg
             << " deg（两软件相机轴约定不同时常较大）\n";
        cout << "  旋转(扣固定轴约定 R_conv=R_handeye^T*R_sim 后): " << rot_aligned_deg
             << " deg, ||R_sim*R_conv^T - R_handeye||_F = " << r_frob << "\n";

        sim.stopSimulation();
        return 0;
    } catch (const exception& e) {
        cerr << "异常: " << e.what() << endl;
        return 1;
    }
}
