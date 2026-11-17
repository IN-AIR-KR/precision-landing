#include "../CameraExtrinsics.hpp"
#include <gtest/gtest.h>
#include <limits>

TEST(CameraExtrinsics, PreservesOriginalDownwardCamera)
{
    const auto transform = cameraExtrinsics({0., 0., -0.1}, {0.7071067811865476, 0., 0., 0.7071067811865476});
    // Optical x (right) -> body y, optical y (down) -> body -x.
    EXPECT_TRUE((transform * Eigen::Vector3d(1., 2., 3.)).isApprox(Eigen::Vector3d(-2., 1., 2.9), 1e-12));
    // Rotate and translate the vehicle as well; extrinsics must be applied in body coordinates.
    const Eigen::Affine3d vehicle = Eigen::Translation3d(10., 20., 30.) *
        Eigen::AngleAxisd(std::acos(-1.) / 2., Eigen::Vector3d::UnitZ());
    EXPECT_TRUE((vehicle * transform * Eigen::Vector3d(1., 2., 3.)).isApprox(Eigen::Vector3d(9., 18., 32.9), 1e-12));
}

TEST(CameraExtrinsics, SupportsDifferentMountAndNormalizesQuaternion)
{
    const auto transform = cameraExtrinsics({0.2, -0.3, 0.4}, {0., 2., 0., 0.});
    EXPECT_TRUE((transform * Eigen::Vector3d(1., 2., 3.)).isApprox(Eigen::Vector3d(1.2, -2.3, -2.6), 1e-12));
}

TEST(CameraExtrinsics, RejectsMalformedOrNonfiniteParameters)
{
    EXPECT_THROW(cameraExtrinsics({0., 0.}, {1., 0., 0., 0.}), std::invalid_argument);
    EXPECT_THROW(cameraExtrinsics({0., 0., 0.}, {1., 0., 0.}), std::invalid_argument);
    EXPECT_THROW(cameraExtrinsics({0., 0., 0.}, {0., 0., 0., 0.}), std::invalid_argument);
    EXPECT_THROW(cameraExtrinsics({std::numeric_limits<double>::quiet_NaN(), 0., 0.}, {1., 0., 0., 0.}), std::invalid_argument);
    EXPECT_THROW(cameraExtrinsics({0., 0., 0.}, {1., std::numeric_limits<double>::infinity(), 0., 0.}), std::invalid_argument);
}
