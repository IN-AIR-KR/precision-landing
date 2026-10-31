#include "PrecisionLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

static const std::string kModeName = "PrecisionLandCustom";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

PrecisionLand::PrecisionLand(rclcpp::Node& node)
	: ModeBase(node, kModeName)
	, _node(node)
{

	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	_target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",
			   rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::targetPoseCallback, this, std::placeholders::_1));

	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
				     rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::vehicleLandDetectedCallback, this, std::placeholders::_1));

	_target_pose_world_pub = _node.create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose_world", rclcpp::QoS(1).best_effort());

	loadParameters();

	modeRequirements().manual_control = false;
}

void PrecisionLand::loadParameters()
{
	_node.declare_parameter<float>("descent_vel", 1.0);
	_node.declare_parameter<float>("vel_p_gain", 1.5);
	_node.declare_parameter<float>("vel_i_gain", 0.0);
	_node.declare_parameter<float>("max_velocity", 3.0);
	_node.declare_parameter<float>("target_timeout", 3.0);
	_node.declare_parameter<float>("delta_position", 0.25);
	_node.declare_parameter<float>("delta_velocity", 0.25);
	_node.declare_parameter<float>("target_accel_noise", 1.0);
	_node.declare_parameter<float>("target_pos_noise", 0.1);

	_node.get_parameter("descent_vel", _param_descent_vel);
	_node.get_parameter("vel_p_gain", _param_vel_p_gain);
	_node.get_parameter("vel_i_gain", _param_vel_i_gain);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("target_timeout", _param_target_timeout);
	_node.get_parameter("delta_position", _param_delta_position);
	_node.get_parameter("delta_velocity", _param_delta_velocity);
	_node.get_parameter("target_accel_noise", _param_target_accel_noise);
	_node.get_parameter("target_pos_noise", _param_target_pos_noise);

	RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
	RCLCPP_INFO(_node.get_logger(), "vel_i_gain: %f", _param_vel_i_gain);
}

void PrecisionLand::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
}

void PrecisionLand::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	if (_search_started) {
		auto tag = ArucoTag {
			.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
			.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			.timestamp = _node.now(),
		};

		// Save tag position/orientation in NED world frame
		_tag = getTagWorld(tag);

		// Correct the target Kalman filter with this new position measurement. The filter keeps
		// predicting every control tick regardless (see predictTarget(), called from
		// updateSetpoint()), so the estimate keeps advancing smoothly through brief gaps where
		// the tag isn't visible, instead of freezing until the next detection arrives.
		correctTarget(_tag.position);

		// Publish world-frame tag pose for visualization: filtered position, raw orientation
		// (orientation isn't part of the filter state).
		geometry_msgs::msg::PoseStamped world_pose_msg;
		world_pose_msg.header.stamp = _node.now();
		world_pose_msg.header.frame_id = "map";
		world_pose_msg.pose.position.x = _target_position.x();
		world_pose_msg.pose.position.y = _target_position.y();
		world_pose_msg.pose.position.z = _target_position.z();
		world_pose_msg.pose.orientation.w = _tag.orientation.w();
		world_pose_msg.pose.orientation.x = _tag.orientation.x();
		world_pose_msg.pose.orientation.y = _tag.orientation.y();
		world_pose_msg.pose.orientation.z = _tag.orientation.z();
		_target_pose_world_pub->publish(world_pose_msg);
	}

}

PrecisionLand::ArucoTag PrecisionLand::getTagWorld(const ArucoTag& tag)
{
	// Convert from optical to NED
	// Optical: X right, Y down, Z away from lens
	// NED: X forward, Y right, Z away from viewer
	Eigen::Matrix3d R;
	R << 0, -1, 0,
	1, 0, 0,
	0, 0, 1;
	Eigen::Quaterniond quat_NED(R);

	auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());

	Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
	Eigen::Affine3d camera_transform = Eigen::Translation3d(0, 0, -0.1) * quat_NED;
	Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;
	Eigen::Affine3d tag_world_transform = drone_transform * camera_transform * tag_transform;

	ArucoTag world_tag = {
		.position = tag_world_transform.translation(),
		.orientation = Eigen::Quaterniond(tag_world_transform.rotation()),
		.timestamp = tag.timestamp,
	};

	return world_tag;
}

void PrecisionLand::predictTarget(float dt_s)
{
	if (!_target_initialized || dt_s <= 0.f) {
		return;
	}

	const double dt = static_cast<double>(dt_s);

	// Constant-velocity motion model: x = F x, with F = [[I, dt*I], [0, I]]
	Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
	F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;

	Eigen::Matrix<double, 6, 1> x;
	x << _target_position, _target_velocity;
	x = F * x;
	_target_position = x.head<3>();
	_target_velocity = x.tail<3>();

	// Process noise (discrete white-noise-acceleration model): grows P every tick to reflect
	// that the constant-velocity assumption is only approximate (target may turn/accelerate).
	// target_accel_noise is the assumed std-dev of that unmodeled acceleration [m/s^2].
	const double sigma_a_sq = _param_target_accel_noise * _param_target_accel_noise;
	Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
	Q.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * (0.25 * dt * dt * dt * dt);
	Q.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * (0.5 * dt * dt * dt);
	Q.block<3, 3>(3, 0) = Eigen::Matrix3d::Identity() * (0.5 * dt * dt * dt);
	Q.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * (dt * dt);
	Q *= sigma_a_sq;

	_target_P = F * _target_P * F.transpose() + Q;
}

void PrecisionLand::correctTarget(const Eigen::Vector3d& measured_position_world)
{
	if (!_target_initialized) {
		// First detection: initialize state directly from the measurement. Velocity is unknown
		// yet (zero), so give it a much larger initial uncertainty than the position.
		_target_position = measured_position_world;
		_target_velocity = Eigen::Vector3d::Zero();
		_target_P = Eigen::Matrix<double, 6, 6>::Identity();
		_target_P.block<3, 3>(0, 0) *= _param_target_pos_noise * _param_target_pos_noise;
		_target_P.block<3, 3>(3, 3) *= 4.0;
		_target_initialized = true;
		return;
	}

	// Measurement model: z = H x + v, H = [I 0] (we only measure position, not velocity)
	Eigen::Matrix<double, 3, 6> H = Eigen::Matrix<double, 3, 6>::Zero();
	H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();

	Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * (_param_target_pos_noise * _param_target_pos_noise);

	Eigen::Vector3d y = measured_position_world - _target_position;             // innovation
	Eigen::Matrix3d S = H * _target_P * H.transpose() + R;                      // innovation covariance
	Eigen::Matrix<double, 6, 3> K = _target_P * H.transpose() * S.inverse();    // Kalman gain

	Eigen::Matrix<double, 6, 1> x;
	x << _target_position, _target_velocity;
	x += K * y;
	_target_position = x.head<3>();
	_target_velocity = x.tail<3>();

	_target_P = (Eigen::Matrix<double, 6, 6>::Identity() - K * H) * _target_P;
}

void PrecisionLand::onActivate()
{
	generateSearchWaypoints();
	_search_started = true;
	switchToState(State::Search);
}

void PrecisionLand::onDeactivate()
{
	// No-op
}

void PrecisionLand::updateSetpoint(float dt_s)
{
	// Advance the target estimate by one control tick regardless of whether a new detection
	// arrived this tick. This is what lets the estimate keep moving smoothly (rather than
	// freezing) during brief gaps where the tag isn't visible.
	predictTarget(dt_s);

	bool target_lost = checkTargetTimeout();

	if (target_lost && !_target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target lost: State %s", stateName(_state).c_str());

	} else if (!target_lost && _target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target acquired");
	}

	_target_lost_prev = target_lost;

	// State machine
	switch (_state) {
	case State::Idle: {
		// No-op -- just spin
		break;
	}

	case State::Search: {

		if (_target_initialized) {
			_approach_altitude = _vehicle_local_position->positionNed().z();
			switchToState(State::Approach);
			break;
		}

		auto waypoint_position = _search_waypoints[_search_waypoint_index];

		_trajectory_setpoint->updatePosition(waypoint_position);

		if (positionReached(waypoint_position)) {
			_search_waypoint_index++;

			// If we have searched all waypoints, start over
			if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
				_search_waypoint_index = 0;
			}
		}

		break;
	}

	case State::Approach: {

		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}

		// Approach using the same feedforward-compensated horizontal velocity controller as
		// Descend, holding altitude fixed at _approach_altitude. A plain position setpoint here
		// structurally lags a moving target (PX4's internal position controller has no
		// feedforward for a moving setpoint), so delta_pos would never shrink below the
		// Approach->Descend threshold once the target moves fast enough.
		Eigen::Vector2f vel = calculateVelocitySetpointXY();
		px4_ros2::TrajectorySetpoint setpoint;
		setpoint.withHorizontalVelocity(vel).withPositionZ(_approach_altitude);
		_trajectory_setpoint->update(setpoint);

		auto target_position = Eigen::Vector3f(_target_position.x(), _target_position.y(), _approach_altitude);

		if (positionReached(target_position)) {
			switchToState(State::Descend);
		}

		break;
	}

	case State::Descend: {

		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}

		// Descend using velocity setpoints and P velocity controller for XY
		Eigen::Vector2f vel = calculateVelocitySetpointXY();
		_trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel), std::nullopt,
					     px4_ros2::quaternionToYaw(_tag.orientation));

		if (_land_detected) {
			switchToState(State::Finished);
		}

		break;
	}

	case State::Finished: {
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

Eigen::Vector2f PrecisionLand::calculateVelocitySetpointXY()
{
	float p_gain = _param_vel_p_gain;
	float i_gain = _param_vel_i_gain;

	// P component
	float delta_pos_x = _vehicle_local_position->positionNed().x() - _target_position.x();
	float delta_pos_y = _vehicle_local_position->positionNed().y() - _target_position.y();

	// I component
	_vel_x_integral += delta_pos_x;
	_vel_y_integral += delta_pos_y;
	float max_integral = _param_max_velocity;
	_vel_x_integral = std::clamp(_vel_x_integral, -1.f * max_integral, max_integral);
	_vel_y_integral = std::clamp(_vel_y_integral, -1.f * max_integral, max_integral);

	float Xp = delta_pos_x * p_gain;
	float Xi = _vel_x_integral * i_gain;
	float Yp = delta_pos_y * p_gain;
	float Yi = _vel_y_integral * i_gain;

	// Sum P and I gains, plus a feedforward term for the target's own velocity so the
	// drone doesn't lag behind a moving target (pure PI tracking has steady-state error ~ v_target / p_gain)
	float vx = -1.f * (Xp + Xi) + static_cast<float>(_target_velocity.x());
	float vy = -1.f * (Yp + Yi) + static_cast<float>(_target_velocity.y());

	// 0.1m/s min vel and 3m/s max vel
	vx = std::clamp(vx, -1.f * _param_max_velocity, _param_max_velocity);
	vy = std::clamp(vy, -1.f * _param_max_velocity, _param_max_velocity);

	return Eigen::Vector2f(vx, vy);
}

bool PrecisionLand::checkTargetTimeout()
{
	if (!_tag.valid()) {
		return true;
	}

	if (_node.now().seconds() - _tag.timestamp.seconds() > _param_target_timeout) {
		return true;
	}

	return false;
}

void PrecisionLand::generateSearchWaypoints()
{
	// Generate spiral search waypoints
	// The search waypoints are generated in the NED frame
	// Parameters for the search pattern
	double start_x = 0.0;
	double start_y = 0.0;
	double current_z = _vehicle_local_position->positionNed().z();
	auto min_z = -1.0;

	double max_radius = 2.0;
	double layer_spacing = 0.5;
	int points_per_layer = 16;
	std::vector<Eigen::Vector3f> waypoints;

	// Generate waypoints
	// Calculate the number of layers needed
	int num_layers = (static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1 ? 1 : (static_cast<int>((
				 min_z - current_z) / layer_spacing) / 2);

	// Generate waypoints
	for (int layer = 0; layer < num_layers; ++layer) {
		std::vector<Eigen::Vector3f> layer_waypoints;

		// Spiral out to max radius
		double radius = 0.0;

		for (int point = 0; point < points_per_layer + 1; ++point) {
			double angle = 2.0 * M_PI * point / points_per_layer;
			double x = start_x + radius * cos(angle);
			double y = start_y + radius * sin(angle);
			double z = current_z;

			layer_waypoints.push_back(Eigen::Vector3f(x, y, z));
			radius += max_radius / points_per_layer;
		}

		// Push the spiral out waypoints to the main waypoints vector
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Decrease the altitude for the inward spiral
		current_z += layer_spacing;

		// Reverse the layer waypoints for spiral in
		std::reverse(layer_waypoints.begin(), layer_waypoints.end());

		// Adjust the z-coordinate for the inward spiral
		for (auto& waypoint : layer_waypoints) {
			waypoint.z() = current_z;
		}

		// Push the reversed waypoints to the main waypoints vector
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Decrease the altitude for the next outward spiral
		current_z += layer_spacing;
	}

	_search_waypoints = waypoints;
}

bool PrecisionLand::positionReached(const Eigen::Vector3f& target) const
{
	auto position = _vehicle_local_position->positionNed();
	auto velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	// Compare against velocity relative to the target instead of absolute vehicle velocity:
	// for a moving target (e.g. Approach tracking the tag), the vehicle must keep moving at the
	// target's speed, so absolute velocity would never settle near zero. _target_velocity is zero
	// while no tag has been detected yet, so this is unchanged for the static Search waypoints.
	const auto relative_velocity = velocity - _target_velocity.cast<float>();
	return (delta_pos.norm() < _param_delta_position) && (relative_velocity.norm() < _param_delta_velocity);
}

std::string PrecisionLand::stateName(State state)
{
	switch (state) {
	case State::Idle:
		return "Idle";

	case State::Search:
		return "Search";

	case State::Approach:
		return "Approach";

	case State::Descend:
		return "Descend";

	case State::Finished:
		return "Finished";

	default:
		return "Unknown";
	}
}

void PrecisionLand::switchToState(State state)
{
	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<PrecisionLand>>(kModeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}
