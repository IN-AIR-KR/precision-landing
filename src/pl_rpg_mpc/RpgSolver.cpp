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
constexpr double sample_time = 0.1;
const Eigen::Quaterniond flip(0., 1., 0., 0.);

using State = Eigen::Matrix<real_t, ACADO_NX, 1>;

State solverState(const Motion& motion, const Eigen::Quaterniond& attitude)
{
    const Eigen::Quaterniond rotation = flip * attitude * flip.conjugate();
    State state;
    state << (flip * motion.position).cast<real_t>(),
        rotation.w(), rotation.x(), rotation.y(), rotation.z(),
        (flip * motion.velocity).cast<real_t>();
    return state;
}
}

void MpcSolver::reset()
{
    initialized_ = false;
}

std::optional<BodyRateCommand> MpcSolver::solve(
    const Motion& start, const Eigen::Quaterniond& attitude_ned_frd,
    const ApproachReference& reference, const View& view, double)
{
    if (!attitude_ned_frd.coeffs().allFinite() || attitude_ned_frd.norm() < 1e-6) {
        return std::nullopt;
    }
    if (!initialized_) {
        std::memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
        std::memset(&acadoVariables, 0, sizeof(acadoVariables));
        acado_initializeSolver();
    }
    auto& variables = acadoVariables;
    const Eigen::Quaterniond yaw(Eigen::AngleAxisd(view.yaw, Vector::UnitZ()));
    const Eigen::Quaterniond camera_rotation(flip.toRotationMatrix() * view.body_from_camera.linear());
    const Eigen::Matrix<real_t, 3, 1> camera_position =
        (flip * Vector(view.body_from_camera.translation())).cast<real_t>();
    State initial = solverState(start, attitude_ned_frd.normalized());
    const State heading = solverState(start, yaw);
    if (initial.segment<4>(3).dot(heading.segment<4>(3)) < 0) {
        initial.segment<4>(3) *= -1;
    }
    Eigen::Map<State>(variables.x0) = initial;
    const std::array<real_t, ACADO_NY> weights{200, 200, 500, 50, 50, 50, 50,
        10, 10, 10, 30, 30, 1, 1, 1, 1};
    for (int k = 0; k <= ACADO_N; ++k) {
        const double time = k * sample_time;
        const Motion desired = reference.sample(time);
        const State desired_state = solverState(desired, yaw);
        const Vector feature = reference.target.position + time * reference.target.velocity;
        Eigen::Map<Eigen::Matrix<real_t, ACADO_NOD, 1>> online(variables.od + k * ACADO_NOD);
        online << (flip * feature).cast<real_t>(), camera_position,
            camera_rotation.w(), camera_rotation.x(), camera_rotation.y(), camera_rotation.z();
        if (!initialized_) {
            Eigen::Map<State>(variables.x + k * ACADO_NX) = initial;
        }

        if (k == ACADO_N) {
            Eigen::Map<Eigen::Matrix<real_t, ACADO_NYN, 1>> terminal(variables.yN);
            terminal << desired_state, 0, 0;
            for (int i = 0; i < ACADO_NYN; ++i) {
                variables.WN[i * (ACADO_NYN + 1)] = weights[i];
            }
            break;
        }
        const real_t thrust = (Vector(0., 0., gravity) + flip * desired.acceleration).norm();
        Eigen::Map<Eigen::Matrix<real_t, ACADO_NY, 1>> stage(variables.y + k * ACADO_NY);
        stage << desired_state, 0, 0, thrust, 0, 0, 0;
        for (int i = 0; i < ACADO_NY; ++i) {
            variables.W[k * ACADO_NY * ACADO_NY + i * (ACADO_NY + 1)] = weights[i];
        }
        Eigen::Map<Eigen::Matrix<real_t, ACADO_NU, 1>>(variables.lbValues + k * ACADO_NU) << 2, -3, -3, -1;
        Eigen::Map<Eigen::Matrix<real_t, ACADO_NU, 1>>(variables.ubValues + k * ACADO_NU) << 20, 3, 3, 1;
        if (!initialized_) {
            Eigen::Map<Eigen::Matrix<real_t, ACADO_NU, 1>>(variables.u + k * ACADO_NU) << gravity, 0, 0, 0;
        }
    }
    if (!initialized_) {
        acado_initializeNodesByForwardSimulation();
    }
    if (acado_preparationStep() != 0 || acado_feedbackStep() != 0) {
        reset();
        return std::nullopt;
    }

    const Eigen::Map<const State> first(variables.x + ACADO_NX);
    const Eigen::Map<const Eigen::Matrix<real_t, ACADO_NU, 1>> input(variables.u);
    if (!first.allFinite() || !input.allFinite()) {
        reset();
        return std::nullopt;
    }
    initialized_ = true;
    return BodyRateCommand{
        input[0],
        flip.conjugate() * Vector(input.tail<3>().cast<double>()),
        flip.conjugate() * Vector(first.tail<3>().cast<double>()),
    };
}
}
