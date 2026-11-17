#pragma once

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <optional>
#include <cmath>

namespace pampc
{
using Vector = Eigen::Vector3d;

struct Motion {
    Vector position = Vector::Zero();
    Vector velocity = Vector::Zero();
    Vector acceleration = Vector::Zero();
};

struct ApproachReference {
    Motion target;
    double start_z;
    double goal_z;
    double vertical_speed;

    Motion sample(double time) const
    {
        Motion desired{target.position + time * target.velocity, target.velocity};
        desired.position.z() = start_z;
        desired.velocity.z() = 0.;
        if (vertical_speed <= 0.) {
            return desired;
        }

        constexpr double braking_time = 0.6;
        const double error = goal_z - start_z;
        const double distance = std::abs(error);
        const double direction = std::copysign(1., error);
        const double cruise_time = std::max(0., distance / vertical_speed - braking_time);
        const double remaining = time < cruise_time ? distance - vertical_speed * time :
            std::min(distance, vertical_speed * braking_time) *
                std::exp(-(time - cruise_time) / braking_time);
        desired.position.z() = goal_z - direction * remaining;
        desired.velocity.z() = direction * std::min(vertical_speed, remaining / braking_time);
        desired.acceleration.z() = time < cruise_time ? 0. : -desired.velocity.z() / braking_time;
        return desired;
    }
};

struct MinimumJerk {
    double duration;
    std::array<Vector, 6> coefficients;

    MinimumJerk(const Motion& start, const Motion& end, double time)
        : duration(time)
    {
        auto& c = coefficients;
        c[0] = start.position;
        c[1] = time * start.velocity;
        c[2] = 0.5 * time * time * start.acceleration;
        const Vector p = end.position - c[0] - c[1] - c[2];
        const Vector v = time * end.velocity - c[1] - 2 * c[2];
        const Vector a = time * time * end.acceleration - 2 * c[2];
        c[3] = 10 * p - 4 * v + 0.5 * a;
        c[4] = -15 * p + 7 * v - a;
        c[5] = 6 * p - 3 * v + 0.5 * a;
    }

    bool feasible(double horizontal_speed, double vertical_speed, double acceleration) const
    {
        // Derivative control points bound the complete polynomial via the
        // convex-hull property of the Bernstein basis.
        const auto& c = coefficients;
        const std::array<Vector, 5> velocity{
            c[1] / duration,
            (c[1] + 0.5 * c[2]) / duration,
            (c[1] + c[2] + 0.5 * c[3]) / duration,
            (c[1] + 1.5 * c[2] + 1.5 * c[3] + c[4]) / duration,
            (c[1] + 2 * c[2] + 3 * c[3] + 4 * c[4] + 5 * c[5]) / duration
        };
        for (std::size_t i = 0; i < velocity.size(); ++i) {
            if (!velocity[i].allFinite() || velocity[i].head<2>().norm() > horizontal_speed ||
                std::abs(velocity[i].z()) > vertical_speed || velocity[i].z() < -1e-9 ||
                (i > 0 && 4 * (velocity[i] - velocity[i - 1]).norm() > duration * acceleration)) {
                return false;
            }
        }
        return true;
    }

    Motion sample(double time) const
    {
        const double s = std::clamp(time / duration, 0., 1.);
        const auto& c = coefficients;
        return {
            c[0] + s * (c[1] + s * (c[2] + s * (c[3] + s * (c[4] + s * c[5])))),
            (c[1] + s * (2 * c[2] + s * (3 * c[3] + s * (4 * c[4] + s * 5 * c[5])))) / duration,
            (2 * c[2] + s * (6 * c[3] + s * (12 * c[4] + s * 20 * c[5]))) / (duration * duration)
        };
    }
};

struct View {
    Eigen::Affine3d body_from_camera;
    Eigen::Quaterniond marker_orientation;
    Eigen::Vector2d focal_length;
    Eigen::Vector2d principal_point;
    Eigen::Vector2d image_size;
    double marker_size;
    double yaw;
    double margin = 0.08;

    double markerMargin(const Vector& vehicle_position, const Eigen::Quaterniond& attitude,
                        const Vector& marker) const
    {
        const Eigen::Affine3d camera_from_world =
            (Eigen::Translation3d(vehicle_position) * attitude * body_from_camera).inverse();
        double clearance = 0.5;
        for (const double x : {-0.5, 0.5}) {
            for (const double y : {-0.5, 0.5}) {
                const Vector corner = camera_from_world *
                    (marker + marker_orientation * Vector(x * marker_size, y * marker_size, 0.));
                if (!corner.allFinite() || corner.z() <= 0.02) {
                    return -1.;
                }
                const Eigen::Vector2d pixel = principal_point +
                    focal_length.cwiseProduct(corner.head<2>() / corner.z());
                const Eigen::Vector2d fraction = pixel.cwiseQuotient(image_size);
                clearance = std::min({clearance, fraction.minCoeff(), 1. - fraction.maxCoeff()});
            }
        }
        return clearance;
    }

    bool containsMarker(const Vector& vehicle_position, const Eigen::Quaterniond& attitude,
                        const Vector& marker) const
    {
        return markerMargin(vehicle_position, attitude, marker) > margin;
    }
};

struct Limits {
    double horizontal_speed;
    double vertical_speed;
    double acceleration;
};

// PX4 applies the inverse of its configured motor thrust curve to its
// normalized thrust command. Convert through motor signal so measured and
// configured curves may differ.
struct ThrustCalibration {
    double hover_command = 0.745;
    double px4_factor = 0.;
    double measured_factor = 1.;

    double command(double thrust_over_gravity) const
    {
        const auto thrust = [](double signal, double factor) {
            return signal * ((1. - factor) + factor * signal);
        };
        const auto signal = [](double normalized_thrust, double factor) {
            if (normalized_thrust <= 0.) { return 0.; }
            const double linear = 1. - factor;
            return 2. * normalized_thrust /
                (linear + std::sqrt(linear * linear + 4. * factor * normalized_thrust));
        };
        const double hover_signal = signal(hover_command, px4_factor);
        const double desired = std::clamp(thrust(hover_signal, measured_factor) * thrust_over_gravity, 0., 1.);
        return thrust(signal(desired, measured_factor), px4_factor);
    }
};

struct BodyRateCommand {
    double specific_thrust;
    Vector rates_frd;
    Vector predicted_velocity_ned;
};

// Uses the generated RPG ACADO solver. One controller owns its global workspace.
class RpgSolver
{
public:
    static constexpr double sample_time = 0.1;
    void reset();
    std::optional<BodyRateCommand> solve(const Motion& start, const Eigen::Quaterniond& attitude_ned_frd,
                                         const ApproachReference& reference, const View& view);

private:
    bool initialized_ = false;
};
}
