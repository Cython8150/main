#include "jacobian_cartesian_control.h"
#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

void setArmTargets(RemoteAPIObject::sim& sim, const std::vector<int>& ur5_joints, const std::vector<double>& q) {
    for (int j = 0; j < 6; j++)
        sim.setJointTargetPosition(ur5_joints[j], q[j]);
}

void settleArmToTargets(RemoteAPIObject::sim& sim, const std::vector<int>& ur5_joints, const std::vector<double>& q, int max_steps,
                        double tol_rad) {
    for (int s = 0; s < max_steps; s++) {
        sim.step();
        bool ok = true;
        for (int j = 0; j < 6; j++) {
            if (std::fabs(sim.getJointPosition(ur5_joints[j]) - q[j]) > tol_rad) {
                ok = false;
                break;
            }
        }
        if (ok)
            return;
    }
}

void tipPosition(RemoteAPIObject::sim& sim, int tip_handle, int base_handle, double out[3]) {
    std::vector<double> pose = sim.getObjectPose(tip_handle, base_handle);
    if (pose.size() < 3) {
        out[0] = out[1] = out[2] = 0.0;
        return;
    }
    out[0] = pose[0];
    out[1] = pose[1];
    out[2] = pose[2];
}

bool invert3(const double m[9], double inv[9]) {
    const double a = m[0], b = m[1], c = m[2];
    const double d = m[3], e = m[4], f = m[5];
    const double g = m[6], h = m[7], i = m[8];
    const double A = e * i - f * h;
    const double B = -(d * i - f * g);
    const double C = d * h - e * g;
    const double D = -(b * i - c * h);
    const double E = a * i - c * g;
    const double F = -(a * h - b * g);
    const double G = b * f - c * e;
    const double H = -(a * f - c * d);
    const double I = a * e - b * d;
    const double det = a * A + b * B + c * C;
    if (std::fabs(det) < 1e-18)
        return false;
    const double invdet = 1.0 / det;
    inv[0] = A * invdet;
    inv[1] = D * invdet;
    inv[2] = G * invdet;
    inv[3] = B * invdet;
    inv[4] = E * invdet;
    inv[5] = H * invdet;
    inv[6] = C * invdet;
    inv[7] = F * invdet;
    inv[8] = I * invdet;
    return true;
}

bool dlsStep(const double J[18], const double e[3], double lambda, double dq[6]) {
    double JJT[9] = {0};
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            double s = 0.0;
            for (int k = 0; k < 6; k++)
                s += J[r * 6 + k] * J[c * 6 + k];
            JJT[r * 3 + c] = s;
        }
    }
    const double lam2 = lambda * lambda;
    JJT[0] += lam2;
    JJT[4] += lam2;
    JJT[8] += lam2;

    double invJJT[9];
    if (!invert3(JJT, invJJT))
        return false;

    double w[3] = {0};
    for (int r = 0; r < 3; r++) {
        double s = 0.0;
        for (int c = 0; c < 3; c++)
            s += invJJT[r * 3 + c] * e[c];
        w[r] = s;
    }

    for (int j = 0; j < 6; j++) {
        double s = 0.0;
        for (int i = 0; i < 3; i++)
            s += J[i * 6 + j] * w[i];
        dq[j] = s;
    }
    return true;
}

double frobeniusNormJ(const double J[18]) {
    double s = 0.0;
    for (int k = 0; k < 18; k++)
        s += J[k] * J[k];
    return std::sqrt(s);
}

void buildJacobianPositionFd(RemoteAPIObject::sim& sim, const std::vector<int>& ur5_joints, int tip_handle, int base_handle,
                             const std::vector<double>& q, double delta_rad, int fd_settle_max_steps, double fd_tol_rad, double J[18]) {
    double p0[3];
    setArmTargets(sim, ur5_joints, q);
    settleArmToTargets(sim, ur5_joints, q, fd_settle_max_steps, fd_tol_rad);
    tipPosition(sim, tip_handle, base_handle, p0);

    for (int col = 0; col < 6; col++) {
        std::vector<double> q_eps = q;
        q_eps[col] += delta_rad;
        setArmTargets(sim, ur5_joints, q_eps);
        settleArmToTargets(sim, ur5_joints, q_eps, fd_settle_max_steps, fd_tol_rad);
        double p1[3];
        tipPosition(sim, tip_handle, base_handle, p1);
        for (int row = 0; row < 3; row++)
            J[row * 6 + col] = (p1[row] - p0[row]) / delta_rad;
    }

    setArmTargets(sim, ur5_joints, q);
    settleArmToTargets(sim, ur5_joints, q, fd_settle_max_steps, fd_tol_rad);
}

void clipVectorL2(std::vector<double>& v, double max_norm) {
    double n2 = 0.0;
    for (double x : v)
        n2 += x * x;
    const double n = std::sqrt(n2);
    if (n > max_norm && n > 1e-12) {
        const double s = max_norm / n;
        for (double& x : v)
            x *= s;
    }
}

} // namespace

bool moveTipPositionPbvsDls(RemoteAPIObject::sim& sim, const std::vector<int>& ur5_joints, int tip_handle, int base_handle,
                            double target_x, double target_y, double target_z, const JacobianPbvsParams& params) {
    if (ur5_joints.size() != 6 || tip_handle < 0 || base_handle < 0) {
        std::cerr << "moveTipPositionPbvsDls: 无效句柄或关节数量\n";
        return false;
    }

    JacobianPbvsParams p = params;
    std::vector<double> q(6);
    for (int j = 0; j < 6; j++)
        q[j] = sim.getJointPosition(ur5_joints[j]);
    setArmTargets(sim, ur5_joints, q);
    settleArmToTargets(sim, ur5_joints, q, p.settle_max_steps, p.settle_tol_rad);

    double J[18];

    for (int iter = 0; iter < p.max_iterations; iter++) {
        for (int j = 0; j < 6; j++)
            q[j] = sim.getJointPosition(ur5_joints[j]);

        double p_cur[3];
        tipPosition(sim, tip_handle, base_handle, p_cur);
        const double ex = target_x - p_cur[0];
        const double ey = target_y - p_cur[1];
        const double ez = target_z - p_cur[2];
        const double err = std::sqrt(ex * ex + ey * ey + ez * ez);

        if (err < p.position_tol_m) {
            setArmTargets(sim, ur5_joints, q);
            settleArmToTargets(sim, ur5_joints, q, p.settle_max_steps, p.settle_tol_rad);
            std::cout << "moveTipPositionPbvsDls: 收敛 iter=" << iter << " err_m=" << err << "\n";
            return true;
        }

        buildJacobianPositionFd(sim, ur5_joints, tip_handle, base_handle, q, p.fd_delta_rad, p.fd_settle_max_steps, p.fd_settle_tol_rad, J);

        const double jnorm = frobeniusNormJ(J);
        if (jnorm < 1e-8) {
            std::cerr << "moveTipPositionPbvsDls: 雅可比范数过小 (" << jnorm << ")，差分无效\n";
        }

        double e[3] = {ex, ey, ez};
        double dq[6];
        if (!dlsStep(J, e, p.damping_lambda, dq)) {
            for (int j = 0; j < 6; j++) {
                double s = 0.0;
                for (int i = 0; i < 3; i++)
                    s += J[i * 6 + j] * e[i];
                dq[j] = 0.1 * s;
            }
        }

        std::vector<double> dqv(dq, dq + 6);
        /* 远距离加大步长，接近目标恢复为 1 减轻“徘徊” */
        double gain = 1.0;
        if (err > p.err_near_m)
            gain = std::min(p.gain_far_max, err / std::max(p.err_ref_m, 1e-6));
        for (double& x : dqv)
            x *= gain;

        clipVectorL2(dqv, p.max_joint_step_rad);
        for (int j = 0; j < 6; j++)
            q[j] += dqv[j];

        setArmTargets(sim, ur5_joints, q);
        settleArmToTargets(sim, ur5_joints, q, p.settle_max_steps, p.settle_tol_rad);
    }

    std::cerr << "moveTipPositionPbvsDls: 未在 max_iterations 内达到阈值\n";
    return false;
}
