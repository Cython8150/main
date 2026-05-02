#pragma once

#include "RemoteAPIClient.h"
#include <vector>

/** 末端在基座系下只跟踪位置 (PBVS)，雅可比由仿真里数值差分得到，不依赖本地 DH。 */
struct JacobianPbvsParams {
    double fd_delta_rad = 7e-4;
    double damping_lambda = 0.038;
    /** 每步关节增量上限 (rad)，误差大时可配合 gain_far 放大等效步长 */
    double max_joint_step_rad = 0.22;
    double position_tol_m = 0.007;
    int max_iterations = 140;
    /** 主迭代下发 Δq 后等待关节到位的最大步数 */
    int settle_max_steps = 42;
    /** 差分雅可比时每次扰动后的最大步数（通常可远小于 settle_max_steps） */
    int fd_settle_max_steps = 14;
    /** 主迭代关节到位判据 (rad) */
    double settle_tol_rad = 0.012;
    /** 差分雅可比用更松判据以显著加速（略增噪声，由 DLS 阻尼吸收） */
    double fd_settle_tol_rad = 0.02;
    /** 当位置误差 > err_near_m 时，对 Δq 乘以 min(gain_far_max, err/err_ref_m) 以远距离更快逼近 */
    double err_ref_m = 0.055;
    double err_near_m = 0.022;
    double gain_far_max = 2.4;
};

/**
 * 阻尼最小二乘 (DLS) 迭代：e = p_des - p_tip，Δq = J^T (J J^T + λ² I)^{-1} e。
 * 全程 sim.setJointTargetPosition + sim.step；差分雅可比用 fd_settle_* 加速。
 */
bool moveTipPositionPbvsDls(RemoteAPIObject::sim& sim, const std::vector<int>& ur5_joints, int tip_handle, int base_handle,
                            double target_x, double target_y, double target_z, const JacobianPbvsParams& params = {});
