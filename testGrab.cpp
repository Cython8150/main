#include "find_handle.h"
#include "control.h"
#include "visual_module.h"
#include "jacobian_cartesian_control.h"
#include <algorithm>
#include <iostream>
#include <stdexcept>
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
// constexpr double kHomeJointRad[6] = {0.0, -1.57, 1.57, 0.0, 0.0, 0.0};
constexpr double kHomeJointRad[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

/** Cuboid 世界系位置：x、y 在区间内随机，z 固定（与 CoppeliaSim 世界系一致） */
constexpr double kCuboidWorldXMin = 0.941;  // 0.841
constexpr double kCuboidWorldXMax = 1.408;  // 1.508
constexpr double kCuboidWorldYMin = -0.441; // -0.341
constexpr double kCuboidWorldYMax = -0.108;  // -0.008
constexpr double kCuboidWorldZ = 0.725;

/** 与 target_cube_color 一致：红->plane[0]，绿->plane[1]，蓝->plane[2]；路径按场景树修改 */
constexpr const char* kPlanePathRed = "/Plane[0]";
constexpr const char* kPlanePathGreen = "/Plane[1]";
constexpr const char* kPlanePathBlue = "/Plane[2]";
/** 释放点相对 plane 位姿原点在基座系 +z 方向抬高 (m)，避免压在板面上 */
constexpr double kReleaseAbovePlaneZ = 0.3;
/** 再略抬高 z 目标，红/蓝 Plane 处纯竖直分量更难压到毫米级，与 position_tol 一起缓解「徘徊」 */
constexpr double kReleaseZExtraSlack = 0.012;
/** 先到达「最终释放高度」之上多少米，再竖直下降，减轻远伸时 PBVS 卡在奇异附近 */
constexpr double kReleaseApproachExtraZ = 0.14;
/** 释放专用：PBVS 的 ||e|| 阈值 (m)，略松可避免在部分构型下 z 差几毫米永远不判收敛 */
constexpr double kReleasePositionTolM = 0.026;
constexpr double kCubeEdgeM = 0.05;
/** 解除父子并开动力学后立方体质量 (kg)，便于 Bullet 下落 */
constexpr double kReleasedCubeMassKg = 0.08;
/** 松爪后专门步进仿真，让物体下落 */
constexpr int kFallSimSteps = 50;

/** 回 home：插值段数与每段步数越大越稳、越慢（终点仍为 kHomeJointRad） */
constexpr int kHomeBlendSegments = 24;
constexpr int kHomeStepsPerSegment = 6;

const char* planePathForCubeColor(int cube_color) {
    if (cube_color == kTargetCubeRed)
        return kPlanePathRed;
    if (cube_color == kTargetCubeGreen)
        return kPlanePathGreen;
    return kPlanePathBlue;
}

bool validDetection(const cv::Point3f& p) {
    return !(p.x == -1.f && p.y == -1.f && p.z == -1.f) && std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

/** Coppelia setObjectColor：纯 (255,0,0) / (0,255,0) / (0,0,255)，与视觉 RGB 阈值一致 */
std::vector<double> coppeliaRgbFromCubeColor(int cube_color) {
    if (cube_color == kTargetCubeRed)
        return {1.0, 0.0, 0.0};
    if (cube_color == kTargetCubeGreen)
        return {0.0, 1.0, 0.0};
    return {0.0, 0.0, 1.0};
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

    constexpr int64_t kWorld = -1;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist_x(kCuboidWorldXMin, kCuboidWorldXMax);
    std::uniform_real_distribution<double> dist_y(kCuboidWorldYMin, kCuboidWorldYMax);
    std::uniform_int_distribution<int> dist_color(0, 2);

    start_vision_thread_rgb(sim_vision_rgb, visionSensorHandle);
    start_vision_thread_depth(sim_vision_depth, visionSensorHandle);

    JacobianPbvsParams jac_params;
    jac_params.position_tol_m = 0.008;
    jac_params.max_joint_step_rad = 0.2;
    jac_params.damping_lambda = 0.045;

    const std::vector<double> home_positions(kHomeJointRad, kHomeJointRad + 6);

    for (int cycle = 0;; ++cycle) {
        int64_t old_cuboid = -1;
        try {
            old_cuboid = sim.getObject("/Cuboid");
        } catch (const std::runtime_error&) {
            /* 场景里没有该路径时 Coppelia 远程 API 会抛错，不是返回 -1 */
        }
        if (old_cuboid >= 0)
            sim.removeObject(old_cuboid);

        const int target_cube_color = dist_color(gen);
        const double wx = dist_x(gen);
        const double wy = dist_y(gen);

        int64_t objectHandle =
            sim.createPrimitiveShape(sim.primitiveshape_cuboid, {kCubeEdgeM, kCubeEdgeM, kCubeEdgeM}, 0);
        if (objectHandle < 0) {
            cerr << "createPrimitiveShape 失败\n";
            stop_vision_thread_rgb();
            stop_vision_thread_depth();
            sim.stopSimulation();
            sim_vision_rgb.stopSimulation();
            sim_vision_depth.stopSimulation();
            return 1;
        }
        sim.setObjectParent(objectHandle, kWorld, true);
        sim.setObjectPose(objectHandle, {wx, wy, kCuboidWorldZ, 0.0, 0.0, 0.0, 1.0}, kWorld);
        const std::vector<double> rgb01 = coppeliaRgbFromCubeColor(target_cube_color);
        sim.setObjectColor(objectHandle, 0, sim.colorcomponent_diffuse, rgb01);
        sim.setObjectColor(objectHandle, 0, sim.colorcomponent_ambient, rgb01);
        sim.setObjectInt32Param(objectHandle, sim.shapeintparam_static, 1);
        sim.setObjectInt32Param(objectHandle, sim.shapeintparam_respondable, 1);

        cout << "===== cycle " << cycle << " ===== 目标正方体: color=" << target_cube_color
             << " (0红1绿2蓝) pos_world " << wx << " " << wy << " " << kCuboidWorldZ << endl;
        for (int i = 0; i < 30; i++)
            sim.step();

        rg2_control(sim, rg2_joints, 20, 0.05);

        for (int i = 0; i < 50; i++) {
            sim.step();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }

        cv::Mat image_detect = get_share_rgb_picture();
        cv::Point3f center_point = detectObject3D(image_detect, target_cube_color);
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

        rg2_control(sim, rg2_joints, 20, 0.05);
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
        // 

        vector<double> objectPoseBase = sim.getObjectPose(objectHandle, baseHandle);
        cout << "Cuboid in base (sim): ";
        for (int i = 0; i < 3 && i < static_cast<int>(objectPoseBase.size()); i++)
            cout << objectPoseBase[i] << " ";
        cout << endl;

        image_detect = get_share_rgb_picture();
        center_point = detectObject3D(image_detect, target_cube_color);
        cout << "after grasp, center point (cam) " << center_point << endl;
        center_point = point_cam2base(center_point);
        cout << "after grasp, center in base " << center_point << endl;
        cv::imwrite("./2.jpg", image_detect);

        /* 先张开夹爪可明显减小腕部负载力矩，回位更稳；若演示需一直夹住物体可注释本行 */
        rg2_control(sim, rg2_joints, 20, 0.05);
        ur5_moveToJointTargetSmooth(sim, ur5_joints, home_positions, kHomeBlendSegments, kHomeStepsPerSegment);
        for (int i = 0; i < 60; i++) {
            sim.step();
            std::this_thread::sleep_for(std::chrono::milliseconds(12));
        }

        int64_t planeHandle = -1;
        try {
            planeHandle = sim.getObject(planePathForCubeColor(target_cube_color));
        } catch (const std::runtime_error& e) {
            cerr << "未找到释放用平面 " << planePathForCubeColor(target_cube_color) << " : " << e.what() << endl;
        }
        if (planeHandle >= 0) {
            vector<double> planePoseBase = sim.getObjectPose(planeHandle, baseHandle);
            if (planePoseBase.size() >= 3) {
                const double rx = planePoseBase[0];
                const double ry = planePoseBase[1];
                const double rz = planePoseBase[2];
                const double z_release = rz + kReleaseAbovePlaneZ + kReleaseZExtraSlack;
                const double z_approach = z_release + kReleaseApproachExtraZ;

                JacobianPbvsParams jac_release = jac_params;
                jac_release.position_tol_m = kReleasePositionTolM;
                jac_release.max_iterations = 280;
                jac_release.damping_lambda = 0.075;
                jac_release.max_joint_step_rad = 0.16;
                jac_release.settle_tol_rad = 0.022;
                jac_release.fd_settle_tol_rad = 0.03;
                jac_release.err_near_m = 0.04;
                jac_release.gain_far_max = 2.0;

                cout << "PBVS: 释放 plane 中心上方(基座系) xy=(" << rx << "," << ry << ") 先 z=" << z_approach << " 再 z="
                     << z_release << " (plane.z=" << rz << " tol_m=" << kReleasePositionTolM << ")\n";

                bool ok1 = moveTipPositionPbvsDls(sim, ur5_joints, tipHandle, baseHandle, rx, ry, z_approach, jac_release);
                if (!ok1)
                    cerr << "雅可比：释放接近点(高)失败\n";
                for (int i = 0; i < 25; i++) {
                    sim.step();
                    std::this_thread::sleep_for(std::chrono::milliseconds(8));
                }

                bool ok2 = ok1 && moveTipPositionPbvsDls(sim, ur5_joints, tipHandle, baseHandle, rx, ry, z_release, jac_release);
                if (!ok2)
                    cerr << "雅可比：释放最终点失败\n";

                if (ok2) {
                    for (int i = 0; i < 35; i++) {
                        sim.step();
                        std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    }
                    /* 1) 脱离 attachPoint（保持世界位姿） 2) 打开 Body dynamic 3) 设质量 4) 再松爪，让步进下落 */
                    sim.setObjectParent(objectHandle, kWorld, true);
                    sim.setObjectInt32Param(objectHandle, sim.shapeintparam_static, 0);
                    sim.setShapeMass(objectHandle, kReleasedCubeMassKg);
                    sim.resetDynamicObject(objectHandle);

                    rg2_control(sim, rg2_joints, 20, 0.05);
                    cout << "目标已脱离指尖并设为动力学体，仿真步进下落 " << kFallSimSteps << " 步\n";
                    for (int i = 0; i < kFallSimSteps; i++) {
                        sim.step();
                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                    }
                }
            } else
                cerr << "getObjectPose(plane, base) 维数不足\n";
        }

        /* 本轮放置结束后：回 home、删除该 cuboid，下一轮再按原流程创建 */
        rg2_control(sim, rg2_joints, 20, 0.05);
        ur5_moveToJointTargetSmooth(sim, ur5_joints, home_positions, kHomeBlendSegments, kHomeStepsPerSegment);
        for (int i = 0; i < 60; i++) {
            sim.step();
            std::this_thread::sleep_for(std::chrono::milliseconds(12));
        }
        sim.removeObject(objectHandle);
        for (int i = 0; i < 25; i++)
            sim.step();
    }

    printf("Simulation completed!\n");
    stop_vision_thread_rgb();
    stop_vision_thread_depth();
    sim.stopSimulation();
    sim_vision_rgb.stopSimulation();
    sim_vision_depth.stopSimulation();

    return 0;
}
