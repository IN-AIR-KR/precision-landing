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

TEST(Pampc, RpgSolverTracksReferenceWithPerceptionEnabled)
{
    const View view{Eigen::Affine3d::Identity(), Eigen::Quaterniond::Identity(),
                    {300., 300.}, {320., 240.}, {640., 480.}, 0.5, 0.};
    const Motion vehicle{{0., 0., -1.}};
    const auto attitude = Eigen::Quaterniond::Identity();
    EXPECT_TRUE(view.containsMarker(vehicle.position, attitude, Vector::Zero()));
    EXPECT_FALSE(view.containsMarker(vehicle.position, attitude, Vector(1., 0., 0.)));
    EXPECT_FALSE(view.containsMarker(vehicle.position, attitude, Vector(0., 0., -2.)));
    const Motion target{{0.5, 0., 0.}};
    const Motion end{{0.5, 0., -1.}};
    RpgSolver solver;
    const auto command = solver.solve(vehicle, Eigen::Quaterniond::Identity(),
                                      MinimumJerk(vehicle, end, 2.), target, view,
                                      Limits{2., 0.6, 1.5, 0.}, 0.05);
    ASSERT_TRUE(command);
    EXPECT_GT(command->x(), 0.);
    EXPECT_LE(command->z(), 0.);
    EXPECT_LE(command->head<2>().norm(), 2.);
}

TEST(Pampc, NominalTerminalSegmentFitsDescentLimits)
{
    const Motion start{{0., 0., -1.1}, {0.3, 0., 0.}};
    const double duration = 1.3 * 1.1 / 0.25;
    const Motion end{{0.3 * duration, 0., 0.}, {0.3, 0., 0.25}};
    EXPECT_TRUE(MinimumJerk(start, end, duration).feasible(2., 0.6, 1.5));
}
