#include "Pampc.hpp"

#include <acado_common.h>
#include <cstring>

ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

namespace pampc
{
namespace
{
constexpr double gravity = 9.8066;
constexpr double sample_time = 0.05;
const Eigen::Quaterniond flip(0., 1., 0., 0.);

using State = Eigen::Matrix<real_t, ACADO_NX, 1>;
using Cost = Eigen::Matrix<real_t, ACADO_NYN, 1>;

Cost referenceCost(const Motion& motion, const Eigen::Quaterniond& attitude, double thrust)
{
    const Eigen::Quaterniond rotation = flip * attitude * flip.conjugate();
    Cost cost;
    cost << (flip * motion.position).cast<real_t>(),
        rotation.w(), rotation.x(), rotation.y(), rotation.z(),
        (flip * motion.velocity).cast<real_t>(), thrust, 0, 0, 0, 0, 0, 0;
    return cost;
}
}

void MpcSolver::reset()
{
    initialized_ = false;
    thrust_state_ = gravity;
}

std::optional<BodyRateCommand> MpcSolver::solve(
    const Motion& start, const Eigen::Quaterniond& attitude,
    const ApproachReference& reference, const View& view, double control_step)
{
    const auto rotation = attitude.normalized();
    const Eigen::Affine3d world_from_camera =
        Eigen::Translation3d(start.position) * rotation * view.body_from_camera;
    const Vector feature_camera = world_from_camera.inverse() * reference.target.position;
    if (!feature_camera.allFinite() || feature_camera.z() <= 0.02) {
        reset();
        return std::nullopt;
    }
    const double distance = feature_camera.norm();
    const Vector direction = feature_camera / distance;
    const Eigen::Quaterniond feature = Eigen::Quaterniond(
        1. + direction.z(), -direction.y(), direction.x(), 0.).normalized();
    const Eigen::Quaterniond camera_rotation(flip.toRotationMatrix() * view.body_from_camera.linear());
    const Vector camera_translation = flip * Vector(view.body_from_camera.translation());
    const Vector center = flip * reference.target.position;
    const Eigen::Quaterniond yaw(Eigen::AngleAxisd(view.yaw, Vector::UnitZ()));

    auto& vars = acadoVariables;
    if (!initialized_) {
        std::memset(&vars, 0, sizeof(vars));
        acado_initializeSolver();
    }

    State initial;
    initial.head<11>() = referenceCost(start, rotation, thrust_state_).head<11>();
    if (initial.segment<4>(3).dot(referenceCost(start, yaw, gravity).segment<4>(3)) < 0.) {
        initial.segment<4>(3) *= -1.;
    }
    initial.tail<7>() << feature.w(), feature.x(), feature.y(), feature.z(), distance, 0, 0;
    Eigen::Map<State>(vars.x0) = initial;

    // The landing reference supplies position and velocity directly. The
    // upstream gate-progress polynomial remains neutral in the cost.
    const std::array<real_t, ACADO_NY> weights{
        200, 200, 500, 50, 50, 50, 50, 10, 10, 10, 1, 30, 30, 0, 0, 0, 0,
        1, 1, 1, 1, 10000, 50
    };
    for (int step = 0; step <= ACADO_N; ++step) {
        const double time = step * sample_time;
        const Motion desired = reference.sample(time);
        const double thrust = (Vector(0., 0., gravity) + flip * desired.acceleration).norm();
        const Cost cost = referenceCost(desired, yaw, thrust);
        Eigen::Map<Eigen::Matrix<real_t, ACADO_NOD, 1>> online(vars.od + step * ACADO_NOD);
        online.setZero();
        online.segment<3>(0) = camera_translation.cast<real_t>();
        online.segment<4>(3) << camera_rotation.w(), camera_rotation.x(),
            camera_rotation.y(), camera_rotation.z();
        online.segment<3>(7) = center.cast<real_t>();
        online[10] = 1.;
        online.segment<3>(14) = center.cast<real_t>();
        online[17] = 1.;
        online[33] = distance;
        online[34] = 1.;
        online[35] = 1.;
        if (step == ACADO_N) {
            Eigen::Map<Cost>(vars.yN) = cost;
            for (int i = 0; i < ACADO_NYN; ++i) {
                vars.WN[i * (ACADO_NYN + 1)] = weights[i];
            }
            break;
        }
        Eigen::Map<Eigen::Matrix<real_t, ACADO_NY, 1>> stage(vars.y + step * ACADO_NY);
        stage << cost, 0, 0, 0, 0, 0, 0;
        for (int i = 0; i < ACADO_NY; ++i) {
            vars.W[step * ACADO_NY * ACADO_NY + i * (ACADO_NY + 1)] = weights[i];
        }
    }
    if (!initialized_) {
        acado_initializeNodesByForwardSimulation();
    }
    if (const int status = acado_preparationStep(); status != 0) {
        reset();
        return std::nullopt;
    }
    if (const int status = acado_feedbackStep(); status != 0) {
        reset();
        return std::nullopt;
    }
    const Eigen::Map<const State> first(vars.x + ACADO_NX);
    const Eigen::Map<const Eigen::Matrix<real_t, ACADO_NU, 1>> input(vars.u);
    if (!first.allFinite() || !input.allFinite()) {
        reset();
        return std::nullopt;
    }
    // FSC optimizes thrust rate. PX4 accepts collective thrust itself.
    thrust_state_ = std::clamp(thrust_state_ + std::clamp(control_step, 0., sample_time) * input[0], 2., 20.);
    initialized_ = true;
    return BodyRateCommand{
        thrust_state_,
        flip * Vector(input.segment<3>(1).cast<double>()),
        flip * Vector(first.segment<3>(7).cast<double>()),
    };
}
}
