#include "find_handle.h"
#include "control.h"
#include "visual_module.h"
#include "jacobian_cartesian_control.h"
#include <algorithm>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>
#include <random>

using namespace std;

namespace {
/** 检测点一般为物块中心，末端在接触前先在上方一点再下降 */
constexpr double kApproachOffsetZ = 0.12;
/** 闭合夹爪后抬起高度 (m) */
constexpr double kLiftDeltaZ = 0.10;
/** 若视觉点与指尖接触点有系统偏差，在此微调 (m) */
constexpr double kGraspOffsetX = 0.0;
constexpr double kGraspOffsetY = 0.0;
constexpr double kGraspOffsetZ = 0.0;
/**
 * 相对「接近阶段完成后的实际末端 z」最多再下降多少 (m)，防止视觉 z 偏深把目标点到桌子底下。
 * 略大于指尖到物块顶面的几何距离即可，例如 0.04–0.07；过小可能够不着物体。
 */
constexpr double kMaxDescendFromHover = 0.064;
/**
 * 基座系下末端 z 的安全下限 (m)；若工作台在 z=0 附近且 +z 向上，可改为 0.0 或 0.01。
 * 设为极小负数表示不启用（仅用语义清晰的哨兵值）。
 */
constexpr double kMinTipZBase = -1.0e6;

/**
 * 回「家」关节角 (rad)。六关节全 0 在多数 CoppeliaSim UR5 里不是竖直稳定位，大臂往往外伸，
 * 重心偏一侧；抓取后负载增大，Bullet 下易出现明显下垂或往一侧「倒」。
 * 请改成你在场景里摆好的稳定姿态（可在仿真里读当前关节角再填这里）。
 */
constexpr double kHomeJointRad[6] = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0};

/** Cuboid 世界系位置：x、y 在区间内随机，z 固定（与 CoppeliaSim 世界系一致） */
constexpr double kCuboidWorldXMin = 0.841;
constexpr double kCuboidWorldXMax = 1.508;
constexpr double kCuboidWorldYMin = -0.341;
constexpr double kCuboidWorldYMax = -0.008;
constexpr double kCuboidWorldZ = 0.725;

bool validDetection(const cv::Point3f& p) {
    return !(p.x == -1.f && p.y == -1.f && p.z == -1.f) && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}
} // namespace

int main() {
    RemoteAPIClient client;

    auto sim = client.getObject().sim();

    sim.startSimulation();

    sim.setStepping(true);

    vector<int> ur5_joints(6);
    ur5_joints = findUR5JointHandle(sim);

    vector<int> rg2_joints(2);
    rg2_joints = findRG2JointHandle(sim);

    RemoteAPIClient client_vision_rgb;
    RemoteAPIClient client_vision_depth;
    auto sim_vision_rgb = client_vision_rgb.getObject().sim();
    auto sim_vision_depth = client_vision_depth.getObject().sim();
    int visionSensorHandle = sim_vision_rgb.getObject("/visionSensor");
    int baseHandle = sim.getObject("/UR5");
    int tipHandle = sim.getObject("/UR5/RG2/attachPoint");
    int objectHandle = sim.getObject("/Cuboid");
    if (objectHandle >= 0) {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist_x(kCuboidWorldXMin, kCuboidWorldXMax);
        std::uniform_real_distribution<double> dist_y(kCuboidWorldYMin, kCuboidWorldYMax);
        const double wx = dist_x(gen);
        const double wy = dist_y(gen);
        constexpr int64_t kWorld = -1; // sim.handle_world
        std::vector<double> cuboid_pose = sim.getObjectPose(objectHandle, kWorld);
        if (cuboid_pose.size() >= 7) {
            cuboid_pose[0] = wx;
            cuboid_pose[1] = wy;
            cuboid_pose[2] = kCuboidWorldZ;
            sim.setObjectPose(objectHandle, cuboid_pose, kWorld);
        } else {
            sim.setObjectPose(objectHandle, {wx, wy, kCuboidWorldZ, 0.0, 0.0, 0.0, 1.0}, kWorld);
        }
        cout << "Cuboid world pose set: x=" << wx << " y=" << wy << " z=" << kCuboidWorldZ << endl;
        for (int i = 0; i < 30; i++)
            sim.step();
    } else {
        cerr << "警告: 未找到 /Cuboid，跳过位置设置\n";
    }

    start_vision_thread_rgb(sim_vision_rgb, visionSensorHandle);
    start_vision_thread_depth(sim_vision_depth, visionSensorHandle);

    rg2_control(sim, rg2_joints, 20, 0.05);

    for (int i = 0; i < 50; i++) {
        sim.step();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    cv::Mat image_detect = get_share_rgb_picture();
    cv::Point3f center_point = detectObject3D(image_detect);
    cout << "center point (cam-aligned) " << center_point << endl;
    center_point = point_cam2base(center_point);
    cout << "center point in base (m) " << center_point << endl;
    cv::imwrite("./1.jpg", image_detect);

    if (!validDetection(center_point)) {
        cerr << "检测或手眼变换失败，无法运动到目标\n";
        stop_vision_thread_rgb();
        stop_vision_thread_depth();
        sim.stopSimulation();
        sim_vision_rgb.stopSimulation();
        sim_vision_depth.stopSimulation();
        return 1;
    }

    const double gx = center_point.x + kGraspOffsetX;
    const double gy = center_point.y + kGraspOffsetY;
    const double gz = center_point.z + kGraspOffsetZ;

    JacobianPbvsParams jac_params;
    jac_params.position_tol_m = 0.008;
    jac_params.max_joint_step_rad = 0.2;
    jac_params.damping_lambda = 0.045;

    cout << "PBVS: 接近点 (基座系 m) " << gx << " " << gy << " " << (gz + kApproachOffsetZ) << endl;
    if (!moveTipPositionPbvsDls(sim, ur5_joints, tipHandle, baseHandle, gx, gy, gz + kApproachOffsetZ, jac_params)) {
        cerr << "雅可比迭代：接近点失败\n";
    }

    vector<double> tipAfterApproach = sim.getObjectPose(tipHandle, baseHandle);
    double z_hover = (tipAfterApproach.size() >= 3) ? tipAfterApproach[2] : (gz + kApproachOffsetZ);
    /* 抓取 z：信视觉但不得低于「悬停高度 − kMaxDescendFromHover」，避免深度误差压穿桌面 */
    double z_grasp = std::max(gz, z_hover - kMaxDescendFromHover);
    if (kMinTipZBase > -1.0e5)
        z_grasp = std::max(z_grasp, kMinTipZBase);

    cout << "PBVS: 抓取点 视觉z=" << gz << " 悬停末端z=" << z_hover << " 限幅后z=" << z_grasp << endl;
    if (!moveTipPositionPbvsDls(sim, ur5_joints, tipHandle, baseHandle, gx, gy, z_grasp, jac_params)) {
        cerr << "雅可比迭代：抓取点失败\n";
    }

    rg2_control(sim, rg2_joints, 20, -0.05);
    sim.setObjectParent(objectHandle, tipHandle, true);

    for (int i = 0; i < 40; i++) {
        sim.step();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    rg2_control(sim, rg2_joints, 0, 0.0);
    // vector<double> tipPose = sim.getObjectPose(tipHandle, baseHandle);
    // if (tipPose.size() >= 3) {
    //     cout << "PBVS: 抬起 delta_z=" << kLiftDeltaZ << endl;
    //     moveTipPositionPbvsDls(sim, ur5_joints, tipHandle, baseHandle, tipPose[0], tipPose[1], tipPose[2] + kLiftDeltaZ,
    //                            jac_params);
    // }

    int shapeHandle = sim.getObject("/Cuboid");
    vector<double> objectPoseBase = sim.getObjectPose(shapeHandle, baseHandle);
    cout << "Cuboid in base (sim): ";
    for (int i = 0; i < 3 && i < static_cast<int>(objectPoseBase.size()); i++)
        cout << objectPoseBase[i] << " ";
    cout << endl;

    image_detect = get_share_rgb_picture();
    center_point = detectObject3D(image_detect);
    cout << "after grasp, center point (cam) " << center_point << endl;
    center_point = point_cam2base(center_point);
    cout << "after grasp, center in base " << center_point << endl;
    cv::imwrite("./2.jpg", image_detect);

    /* 先张开夹爪可明显减小腕部负载力矩，回位更稳；若演示需一直夹住物体可注释本行 */
    // rg2_control(sim, rg2_joints, 20, 0.05);
    std::vector<double> home_positions(kHomeJointRad, kHomeJointRad + 6);
    ur5_control(sim, ur5_joints, home_positions);

    for (int i = 0; i < 50; i++) {
        sim.step();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    printf("Simulation completed!\n");
    stop_vision_thread_rgb();
    stop_vision_thread_depth();
    sim.stopSimulation();
    sim_vision_rgb.stopSimulation();
    sim_vision_depth.stopSimulation();

    return 0;
}
