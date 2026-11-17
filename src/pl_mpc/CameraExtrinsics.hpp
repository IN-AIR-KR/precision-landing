#pragma once
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

// T_body_optical: camera optical coordinates -> vehicle body FRD.
// Translation is the camera origin in body FRD meters; quaternion order is w,x,y,z.
inline Eigen::Affine3d cameraExtrinsics(const std::vector<double>& position,
                                      const std::vector<double>& quaternion)
{
    const auto finite = [](double value) { return std::isfinite(value); };
    if (position.size() != 3 || quaternion.size() != 4 ||
        !std::all_of(position.begin(), position.end(), finite) ||
        !std::all_of(quaternion.begin(), quaternion.end(), finite)) {
        throw std::invalid_argument("camera_position_body must contain 3 finite values and camera_orientation_body 4 finite wxyz values");
    }
    Eigen::Quaterniond rotation(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    if (!std::isfinite(rotation.norm()) || rotation.norm() < 1e-9) {
        throw std::invalid_argument("camera_orientation_body must be a nonzero finite quaternion");
    }
    rotation.normalize();
    return Eigen::Translation3d(position[0], position[1], position[2]) * rotation;
}
