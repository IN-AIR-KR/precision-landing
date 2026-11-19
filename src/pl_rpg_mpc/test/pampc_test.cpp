#include "../Pampc.hpp"

#include <gtest/gtest.h>

using namespace pampc;

TEST(Pampc, MinimumJerkMatchesMovingBoundaryConditions)
{
    const Motion start{{0., 0., -0.55}, {0.1, 0., 0.2}};
    const Motion end{{0.4, 0., 0.}, {0.3, 0., 0.25}};
    const MinimumJerk trajectory(start, end, 2.);
    EXPECT_TRUE(trajectory.sample(0.).position.isApprox(start.position));
    EXPECT_TRUE(trajectory.sample(0.).velocity.isApprox(start.velocity));
    EXPECT_TRUE(trajectory.sample(2.).position.isApprox(end.position));
    EXPECT_TRUE(trajectory.sample(2.).velocity.isApprox(end.velocity));
    EXPECT_LT(trajectory.sample(2.).acceleration.norm(), 1e-9);
    EXPECT_FALSE(trajectory.feasible(0.1, 0.6, 1.5));
    EXPECT_TRUE(trajectory.feasible(2., 0.7, 1.5));
}

TEST(Pampc, MpcSolverTracksReferenceWithPerceptionEnabled)
{
    const View view{Eigen::Affine3d::Identity(), Eigen::Quaterniond::Identity(),
                    {300., 300.}, {320., 240.}, {640., 480.}, 0.5, 0.};
    const Motion vehicle{{0., 0., -1.}};
    const auto attitude = Eigen::Quaterniond::Identity();
    EXPECT_TRUE(view.containsMarker(vehicle.position, attitude, Vector::Zero()));
    EXPECT_FALSE(view.containsMarker(vehicle.position, attitude, Vector(1., 0., 0.)));
    EXPECT_FALSE(view.containsMarker(vehicle.position, attitude, Vector(0., 0., -2.)));
    const Motion target{{0.5, 0., 0.}};
    MpcSolver solver;
    const auto command = solver.solve(vehicle, Eigen::Quaterniond::Identity(),
                                      ApproachReference{target, -1., -1., 0.5}, view);
    ASSERT_TRUE(command);
    EXPECT_GT(command->predicted_velocity_ned.x(), 0.);
    EXPECT_GE(command->specific_thrust, 2.);
    EXPECT_LE(command->specific_thrust, 20.);
    EXPECT_TRUE(command->rates_frd.allFinite());
}

TEST(Pampc, NominalTerminalSegmentFitsDescentLimits)
{
    const Motion start{{0., 0., -1.1}, {0.3, 0., 0.}};
    const double duration = 1.3 * 1.1 / 0.25;
    const Motion end{{0.3 * duration, 0., 0.}, {0.3, 0., 0.25}};
    EXPECT_TRUE(MinimumJerk(start, end, duration).feasible(2., 0.6, 1.5));
}


TEST(Pampc, ApproachReferenceLimitsDescentAndBrakes)
{
    const ApproachReference reference{Motion{}, -2.5, -1., 0.5};
    EXPECT_DOUBLE_EQ(reference.sample(1.).position.z(), -2.);
    EXPECT_DOUBLE_EQ(reference.sample(1.).velocity.z(), 0.5);
    EXPECT_LT(reference.sample(3.).velocity.z(), 0.5);
    EXPECT_LT(reference.sample(3.).position.z(), -1.);
    const ApproachReference hold{Motion{}, -2.5, -1., 0.};
    EXPECT_DOUBLE_EQ(hold.sample(1.).position.z(), -2.5);
    EXPECT_DOUBLE_EQ(hold.sample(1.).velocity.z(), 0.);
}

TEST(Pampc, ThrustCalibrationPreservesHoverAndCompensatesMotorCurve)
{
    const ThrustCalibration sitl{0.745, 0., 1.};
    EXPECT_NEAR(sitl.command(1.), 0.745, 1e-12);
    EXPECT_NEAR(sitl.command(0.25), 0.3725, 1e-12);
    EXPECT_DOUBLE_EQ(sitl.command(0.), 0.);
    EXPECT_DOUBLE_EQ(sitl.command(10.), 1.);

    const ThrustCalibration matched{0.4, 0.6, 0.6};
    EXPECT_NEAR(matched.command(1.5), 0.6, 1e-12);
}
