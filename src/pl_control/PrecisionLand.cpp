#include "PrecisionLand.hpp"
#include "CameraExtrinsics.hpp"
#include <px4_ros2/utils/message_version.hpp>

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>

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
	setupStartService();
    _monitor_pub = _node.create_publisher<std_msgs::msg::String>("/pl/control/status", rclcpp::QoS(1).best_effort());
    _monitor_timer = _node.create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (!_monitor_pub->get_subscription_count()) { return; }
        std_msgs::msg::String message;
        message.data = "{\"state\":\"" + stateName(_state) + "\",\"active\":" +
            (isActive() ? "true" : "false") + ",\"target_lost\":" +
            (checkTargetTimeout() ? "true" : "false") + ",\"mode_id\":" + std::to_string(id()) + ",\"kf_target_frd_m\":";
        if (isActive() && _used_target_frd) {
            const auto& position = *_used_target_frd;
            message.data += "[" + std::to_string(position.x()) + "," +
                std::to_string(position.y()) + "," + std::to_string(position.z()) + "]";
        } else {
            message.data += "null";
        }
        message.data += "}";
        _monitor_pub->publish(message);
    });
}

// Deferred service responses keep the single ROS executor free to receive
// PX4 status/acknowledgements and run the landing controller while we wait.
void PrecisionLand::setupStartService()
{
    using Status = px4_msgs::msg::VehicleStatus;
    using Command = px4_msgs::msg::VehicleCommand;
    using Ack = px4_msgs::msg::VehicleCommandAck;
    _command_pub = _node.create_publisher<Command>(
        "/fmu/in/vehicle_command" + px4_ros2::getMessageNameVersion<Command>(), 10);
    _status_sub = _node.create_subscription<Status>(
        "/fmu/out/vehicle_status" + px4_ros2::getMessageNameVersion<Status>(), rclcpp::QoS(1).best_effort(),
        [this](Status::SharedPtr msg) { _status = *msg; _status_received = SteadyClock::now(); });
    _ack_sub = _node.create_subscription<Ack>(
        "/fmu/out/vehicle_command_ack" + px4_ros2::getMessageNameVersion<Ack>(), rclcpp::QoS(10).best_effort(),
        [this](Ack::SharedPtr msg) {
            if (!_start_requests.empty() && msg->command == Command::VEHICLE_CMD_SET_NAV_STATE &&
                msg->target_system == 255 && msg->target_component == 191 &&
                msg->result != Ack::VEHICLE_CMD_RESULT_ACCEPTED && msg->result != Ack::VEHICLE_CMD_RESULT_IN_PROGRESS) {
                respondStart(false, "PX4 rejected mode request: result=" + std::to_string(msg->result));
            }
        });
    _start_service = _node.create_service<Trigger>("/pl/start_landing",
        [this](std::shared_ptr<rmw_request_id_t> header, std::shared_ptr<Trigger::Request>) {
            auto reply = [this, &header](bool success, const std::string& message) {
                Trigger::Response response; response.success = success; response.message = message + "; mode=" + std::to_string(id());
                _start_service->send_response(*header, response);
            };
            const auto now = SteadyClock::now();
            if (id() == kModeIDInvalid) { reply(false, "Mode is not registered"); return; }
            if (now - _status_received > std::chrono::seconds(2) || now - _land_received > std::chrono::seconds(2) ||
                !_vehicle_local_position->positionXYValid() || !_vehicle_local_position->positionZValid()) {
                reply(false, "Missing or stale vehicle state"); return;
            }
            if (_status.arming_state != Status::ARMING_STATE_ARMED || _land_detected) {
                reply(false, "Vehicle must already be armed and airborne"); return;
            }
            if (_status.nav_state == id() && isActive()) { reply(true, "Already active"); return; }
            _start_requests.push_back(header);
            if (_start_requests.size() > 1) { return; }
            Command command{};
            command.command = Command::VEHICLE_CMD_SET_NAV_STATE;
            command.param1 = id();
            command.target_system = _status.system_id;
            command.target_component = _status.component_id;
            command.source_system = 255; command.source_component = 191; command.from_external = true;
            _start_deadline = now + std::chrono::seconds(5);
            _command_pub->publish(command);
        });
    _start_timer = _node.create_wall_timer(std::chrono::milliseconds(50), [this]() {
        if (_start_requests.empty()) { return; }
        const auto now = SteadyClock::now();
        if (now - _status_received < std::chrono::seconds(2) && _status.nav_state == id() && isActive()) {
            respondStart(true, "Activated: mode=" + std::to_string(id()));
        } else if (now >= _start_deadline) {
            respondStart(false, "Timed out waiting for PX4 mode activation");
        }
    });
}

void PrecisionLand::respondStart(bool success, const std::string& message)
{
    Trigger::Response response; response.success = success; response.message = message;
    for (const auto& header : _start_requests) { _start_service->send_response(*header, response); }
    _start_requests.clear();
}

void PrecisionLand::loadParameters()
{
	// Defaults reproduce the original downward-camera transform exactly in meaning.
	const auto camera_position = _node.declare_parameter<std::vector<double>>(
		"camera_position_body", {0.0, 0.0, -0.1});
	const auto camera_orientation = _node.declare_parameter<std::vector<double>>(
		"camera_orientation_body", {0.7071067811865476, 0.0, 0.0, 0.7071067811865476});
	_camera_transform = cameraExtrinsics(camera_position, camera_orientation);

	_node.declare_parameter<float>("descent_vel", 1.0);
	_node.declare_parameter<float>("vel_p_gain", 1.5);
	_node.declare_parameter<float>("vel_i_gain", 0.0);
	_node.declare_parameter<float>("vel_d_gain", 0.0);
	_node.declare_parameter<float>("max_velocity", 3.0);
	_node.declare_parameter<float>("target_timeout", 3.0);
	_node.declare_parameter<float>("delta_position", 0.25);
	_node.declare_parameter<float>("delta_velocity", 0.25);
	_node.declare_parameter<float>("target_accel_noise", 1.0);
	_node.declare_parameter<float>("target_pos_noise", 0.1);

	_node.get_parameter("descent_vel", _param_descent_vel);
	_node.get_parameter("vel_p_gain", _param_vel_p_gain);
	_node.get_parameter("vel_i_gain", _param_vel_i_gain);
	_node.get_parameter("vel_d_gain", _param_vel_d_gain);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("target_timeout", _param_target_timeout);
	_node.get_parameter("delta_position", _param_delta_position);
	_node.get_parameter("delta_velocity", _param_delta_velocity);
	_node.get_parameter("target_accel_noise", _param_target_accel_noise);
	_node.get_parameter("target_pos_noise", _param_target_pos_noise);

	RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
	RCLCPP_INFO(_node.get_logger(), "vel_i_gain: %f, vel_d_gain: %f", _param_vel_i_gain, _param_vel_d_gain);
}

void PrecisionLand::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
	_land_received = SteadyClock::now();
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
	auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());

	Eigen::Affine3d drone_transform = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
	Eigen::Affine3d tag_transform = Eigen::Translation3d(tag.position) * tag.orientation;
	Eigen::Affine3d tag_world_transform = drone_transform * _camera_transform * tag_transform;

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
	// Wait for a new target observation on every activation.
	_tag = ArucoTag{};
	_target_position.setConstant(std::numeric_limits<double>::quiet_NaN());
	_target_velocity.setZero();
	_target_P.setZero();
	_target_initialized = false;
	_used_target_frd.reset();
	_target_lost_prev = true;
	_xy_integral_velocity.setZero();
	_search_waypoint_index = 0;
	generateSearchWaypoints();
	_search_started = true;
	switchToState(State::SEARCH);
}

void PrecisionLand::onDeactivate()
{
	_used_target_frd.reset();
}

void PrecisionLand::updateSetpoint(float dt_s)
{
	_used_target_frd.reset();
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
	case State::IDLE: {
		// No-op -- just spin
		break;
	}

	case State::SEARCH: {

		if (_target_initialized && !target_lost) {
			_approach_altitude = _vehicle_local_position->positionNed().z();
			switchToState(State::ALIGN);
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

	case State::ALIGN: {

		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::IDLE);
			return;
		}

		// In ALIGN, use the same feedforward-compensated horizontal velocity controller as
		// APPROACH, holding altitude fixed at _approach_altitude. A plain position setpoint here
		// structurally lags a moving target (PX4's internal position controller has no
		// feedforward for a moving setpoint), so delta_pos would never shrink below the
		// ALIGN->APPROACH threshold once the target moves fast enough.
		Eigen::Vector2f vel = calculateVelocitySetpointXY(dt_s);
		px4_ros2::TrajectorySetpoint setpoint;
		setpoint.withHorizontalVelocity(vel).withPositionZ(_approach_altitude);
		_trajectory_setpoint->update(setpoint);

		auto target_position = Eigen::Vector3f(_target_position.x(), _target_position.y(), _approach_altitude);

		if (positionReached(target_position)) {
			switchToState(State::APPROACH);
		}

		break;
	}

	case State::APPROACH: {

		if (target_lost) {
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::IDLE);
			return;
		}

		// Descend at the configured rate while tracking the target with the XY PID.
		Eigen::Vector2f vel = calculateVelocitySetpointXY(dt_s);
		_trajectory_setpoint->update(Eigen::Vector3f(vel.x(), vel.y(), _param_descent_vel), std::nullopt,
					     px4_ros2::quaternionToYaw(_tag.orientation));

		if (_land_detected) {
			switchToState(State::FINISHED);
		}

		break;
	}

	case State::FINISHED: {
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

Eigen::Vector2f PrecisionLand::calculateVelocitySetpointXY(float dt_s)
{
	const Eigen::Vector3d relative_ned =
		_target_position - _vehicle_local_position->positionNed().cast<double>();
	const Eigen::Vector2f position_error = relative_ned.head<2>().cast<float>();
	// Record the exact KF sample used by the XY controller in the same frame as the HUD.
	const Eigen::Quaterniond attitude = _vehicle_attitude->attitude().cast<double>();
	if (relative_ned.allFinite() && attitude.coeffs().allFinite() && attitude.squaredNorm() > 1e-12) {
		_used_target_frd = attitude.normalized().conjugate() * relative_ned;
	}
	const Eigen::Vector2f target_velocity = _target_velocity.head<2>().cast<float>();
	const Eigen::Vector2f p_term = _param_vel_p_gain * position_error;

	// Relative velocity is the derivative of relative position error.
	Eigen::Vector2f d_term = Eigen::Vector2f::Zero();
	if (_param_vel_d_gain != 0.0f) {
		const Eigen::Vector2f vehicle_velocity = _vehicle_local_position->velocityNed().head<2>();
		const Eigen::Vector2f velocity_error = target_velocity - vehicle_velocity;
		if (velocity_error.allFinite()) {
			d_term = _param_vel_d_gain * velocity_error;
		}
	}

	const Eigen::Vector2f pd_with_feedforward = target_velocity + p_term + d_term;
	const float limit = _param_max_velocity;
	if (std::isfinite(dt_s) && dt_s > 0.0f && _param_vel_i_gain != 0.0f) {
		const Eigen::Vector2f integral_step = _param_vel_i_gain * position_error * dt_s;
		for (int axis = 0; axis < 2; ++axis) {
			const float candidate = std::clamp(
				_xy_integral_velocity(axis) + integral_step(axis), -limit, limit);
			const float candidate_command = pd_with_feedforward(axis) + candidate;
			const bool winds_up = (candidate_command > limit && integral_step(axis) > 0.0f)
				|| (candidate_command < -limit && integral_step(axis) < 0.0f);
			if (!winds_up) {
				_xy_integral_velocity(axis) = candidate;
			}
		}
	}

	const Eigen::Vector2f i_term = _xy_integral_velocity;
	const Eigen::Vector2f command = pd_with_feedforward + i_term;
	return command.cwiseMax(-limit).cwiseMin(limit);
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
	// for a moving target (e.g. ALIGN tracking the tag), the vehicle must keep moving at the
	// target's speed, so absolute velocity would never settle near zero. _target_velocity is zero
	// while no tag has been detected yet, so this is unchanged for the static SEARCH waypoints.
	const auto relative_velocity = velocity - _target_velocity.cast<float>();
	return (delta_pos.norm() < _param_delta_position) && (relative_velocity.norm() < _param_delta_velocity);
}

std::string PrecisionLand::stateName(State state)
{
	switch (state) {
	case State::IDLE:
		return "IDLE";

	case State::SEARCH:
		return "SEARCH";

	case State::ALIGN:
		return "ALIGN";

	case State::APPROACH:
		return "APPROACH";

	case State::FINISHED:
		return "FINISHED";

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
