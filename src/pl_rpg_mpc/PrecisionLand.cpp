#include "PrecisionLand.hpp"
#include "CameraExtrinsics.hpp"
#include <px4_ros2/utils/message_version.hpp>

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdexcept>
#include <sstream>

static const std::string kModeName = "PrecisionLandCustom";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

PrecisionLand::PrecisionLand(rclcpp::Node& node)
	: ModeBase(node, kModeName)
	, _node(node)
{

	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
	_rates_setpoint = std::make_shared<px4_ros2::RatesSetpointType>(*this);

	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	_target_observation_sub = _node.create_subscription<pl_msgs::msg::TargetObservation>("/pl/target_observation",
			   rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::targetObservationCallback, this, std::placeholders::_1));

	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
				     rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::vehicleLandDetectedCallback, this, std::placeholders::_1));

	_target_pose_world_pub = _node.create_publisher<geometry_msgs::msg::PoseStamped>("/pl/target_pose_world", rclcpp::QoS(1).best_effort());

	loadParameters();
	_camera_info_sub = _node.create_subscription<sensor_msgs::msg::CameraInfo>(
		"/pl/camera/camera_info", rclcpp::QoS(1).best_effort(),
		[this](const sensor_msgs::msg::CameraInfo& info) {
			if (info.width == 0 || info.height == 0 ||
			    !std::all_of(info.k.begin(), info.k.end(), [](double v) { return std::isfinite(v); }) ||
			    info.k[0] <= 0 || info.k[4] <= 0 ||
			    info.k[2] <= 0.08 * info.width || info.k[2] >= 0.92 * info.width ||
			    info.k[5] <= 0.08 * info.height || info.k[5] >= 0.92 * info.height) {
				return;
			}
			_view = pampc::View{_camera_transform, Eigen::Quaterniond::Identity(),
				{info.k[0], info.k[4]}, {info.k[2], info.k[5]},
				{static_cast<double>(info.width), static_cast<double>(info.height)}, _marker_size, 0.};
		});

	modeRequirements().manual_control = false;
	setupStartService();
    _monitor_pub = _node.create_publisher<std_msgs::msg::String>("/pl/control/status", rclcpp::QoS(1).best_effort());
    _monitor_timer = _node.create_wall_timer(std::chrono::milliseconds(100), [this]() {
        if (!_monitor_pub->get_subscription_count()) { return; }
        std_msgs::msg::String message;
        const auto vector_json = [](const std::optional<Eigen::Vector3d>& value) {
            if (!value || !value->allFinite()) { return std::string("null"); }
            return "[" + std::to_string(value->x()) + "," + std::to_string(value->y()) + "," +
                std::to_string(value->z()) + "]";
        };
        std::ostringstream status;
        status << std::boolalpha << "{\"state\":\"" << stateName(_state)
               << "\",\"active\":" << isActive()
               << ",\"target_lost\":" << checkTargetTimeout()
               << ",\"mode_id\":" << static_cast<int>(id())
               << ",\"kf_target_frd_m\":" << vector_json(_used_target_frd)
               << ",\"final_tracking_error_ned_m\":" << vector_json(_final_error)
               << ",\"contact_wait_s\":" << _contact_wait
               << ",\"failure_reason\":\"" << _failure_reason << "\"}";
        message.data = status.str();
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
            if (_status.nav_state == id() && isActive()) {
                if (!_failure_reason.empty()) {
                    reply(false, "Previous landing failed; exit this mode before retrying: " + _failure_reason);
                } else {
                    reply(true, "Already active");
                }
                return;
            }
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
	_node.declare_parameter<float>("max_velocity", 3.0);
	_node.declare_parameter<float>("target_timeout", 3.0);
	_node.declare_parameter<float>("delta_position", 0.25);
	_node.declare_parameter<float>("delta_velocity", 0.25);
	_node.declare_parameter<float>("target_accel_noise", 1.0);
	_node.declare_parameter<float>("target_pos_noise", 0.1);

	_node.get_parameter("descent_vel", _param_descent_vel);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("target_timeout", _param_target_timeout);
	_node.get_parameter("delta_position", _param_delta_position);
	_node.get_parameter("delta_velocity", _param_delta_velocity);
	_node.get_parameter("target_accel_noise", _param_target_accel_noise);
	_node.get_parameter("target_pos_noise", _param_target_pos_noise);

	_stationary_target = _node.declare_parameter<bool>("stationary_target", false);
	_approach_descent_speed = _node.declare_parameter<double>("approach_descent_speed", 0.5);
	if (!std::isfinite(_approach_descent_speed) || _approach_descent_speed <= 0.) {
		throw std::invalid_argument("PAMPC approach descent speed must be positive and finite");
	}
	_thrust_calibration.hover_command = _node.declare_parameter<double>("hover_thrust", 0.745);
	_thrust_calibration.px4_factor = _node.declare_parameter<double>("px4_thrust_model_factor", 0.);
	_thrust_calibration.measured_factor = _node.declare_parameter<double>("measured_motor_thrust_model_factor", 1.);
	_land_detector_z_velocity_max = _node.declare_parameter<double>("land_detector_z_velocity_max", 0.25);
	const auto valid_factor = [](double value) { return value >= 0. && value <= 1.; };
	if (!(_thrust_calibration.hover_command > 0. && _thrust_calibration.hover_command < 1.) ||
	    !valid_factor(_thrust_calibration.px4_factor) || !valid_factor(_thrust_calibration.measured_factor) ||
	    !std::isfinite(_land_detector_z_velocity_max) || _land_detector_z_velocity_max <= 0.) {
		throw std::invalid_argument("Invalid PAMPC thrust calibration or PX4 land detector threshold");
	}
	_marker_size = _node.declare_parameter<double>("marker_size", 1.0);
	if (!std::isfinite(_marker_size) || _marker_size <= 0 ||
	    !std::isfinite(_param_max_velocity) || _param_max_velocity <= 0 ||
	    !std::isfinite(_param_descent_vel) || _param_descent_vel <= 0) {
		throw std::invalid_argument("PAMPC requires positive finite marker size and velocity limits");
	}

	RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
}

void PrecisionLand::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
	_land_received = SteadyClock::now();
}

void PrecisionLand::targetObservationCallback(const pl_msgs::msg::TargetObservation::SharedPtr msg)
{
	if (!msg->detected) {
		return;
	}
	if (_tracking_enabled && _state != State::LANDING && _state != State::FINISHED) {
		auto tag = ArucoTag {
			.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
			.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			.timestamp = _node.now(),
		};

		if (!tag.position.allFinite() || !tag.orientation.coeffs().allFinite() ||
		    tag.orientation.norm() < 1e-6 || tag.position.z() <= 0.) {
			return;
		}
		tag.orientation.normalize();
		// Save tag position/orientation in NED world frame
		const auto world_tag = getTagWorld(tag);
		if (!world_tag.position.allFinite() || !world_tag.orientation.coeffs().allFinite()) {
			return;
		}
		_tag = world_tag;

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
	_tag.timestamp = rclcpp::Time(0, 0, _node.get_clock()->get_clock_type());
	_target_initialized = false;
	_target_velocity.setZero();
	_target_lost_prev = true;
	_handoff_time = 0.;
	_contact_wait = 0.;
	_final_error.reset();
	_used_target_frd.reset();
	_failure_reason.clear();
	_pampc_solver.reset();
	_final_trajectory.reset();
	_tracking_enabled = true;
	_hold_position.reset();
	switchToState(State::IDLE);
}

void PrecisionLand::onDeactivate()
{
	_tracking_enabled = false;
	_final_trajectory.reset();
	switchToState(State::IDLE);
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
	case State::IDLE: {
		if (!_hold_position) {
			const auto position = _vehicle_local_position->positionNed();
			if (!_vehicle_local_position->positionXYValid() ||
			    !_vehicle_local_position->positionZValid() || !position.allFinite()) {
				break;
			}
			_hold_position = position;
		}
		_trajectory_setpoint->updatePosition(*_hold_position);
		if (_tracking_enabled && _target_initialized && !target_lost && _view) {
			switchToState(State::APPROACH);
		}
		break;
	}

	case State::APPROACH:
	case State::LANDING:
		updatePampc(dt_s);
		break;

	case State::FINISHED: {
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

void PrecisionLand::failLanding(const char* reason)
{
	_failure_reason = reason;
	_hold_position.reset();
	RCLCPP_WARN(_node.get_logger(), "PAMPC landing stopped: %s", reason);
	_tracking_enabled = false;
	_final_trajectory.reset();
	ModeBase::completed(px4_ros2::Result::ModeFailureOther);
	switchToState(State::IDLE);
}

void PrecisionLand::updatePampc(float dt_s)
{
	using namespace pampc;
	const Motion vehicle{_vehicle_local_position->positionNed().cast<double>(),
	                     _vehicle_local_position->velocityNed().cast<double>()};
	if (!_vehicle_local_position->positionXYValid() || !_vehicle_local_position->positionZValid() ||
	    !vehicle.position.allFinite() || !vehicle.velocity.allFinite()) {
		failLanding("invalid local state");
		return;
	}

	if (_state == State::LANDING) {
		if (_land_detected && SteadyClock::now() - _land_received < std::chrono::seconds(2)) {
			switchToState(State::FINISHED);
			return;
		}
		_final_elapsed += std::max(0.f, dt_s);
		const auto& trajectory = *_final_trajectory;
		auto sample = trajectory.sample(_final_elapsed);
		// Keep descending while PX4's land detector confirms contact. Bound the
		// vertical position extension so tracking error remains meaningful on ground.
		constexpr double contact_timeout_s = 3.;
		constexpr double contact_extension_m = 0.20;
		const double overrun = std::max(0., _final_elapsed - trajectory.duration);
		sample.position.head<2>() += overrun * sample.velocity.head<2>();
		sample.position.z() += std::min(overrun * sample.velocity.z(), contact_extension_m);
		if (overrun > 0.) {
			// PX4 requires a descent request of at least 1.1 * LNDMC_Z_VEL_MAX.
			sample.velocity.z() = std::max(sample.velocity.z(), 1.2 * _land_detector_z_velocity_max);
		}
		_contact_wait = overrun;
		_final_error = vehicle.position - sample.position;
		if (overrun > contact_timeout_s) {
			failLanding("final contact timeout");
			return;
		}
		if (_final_error->head<2>().norm() > 0.35 || std::abs(_final_error->z()) > 0.35) {
			failLanding("final trajectory tracking error");
			return;
		}
		px4_ros2::TrajectorySetpoint setpoint;
		setpoint.withPosition(sample.position.cast<float>()).withVelocity(sample.velocity.cast<float>())
		        .withYaw(_landing_yaw);
		_trajectory_setpoint->update(setpoint);
		return;
	}

	if (checkTargetTimeout() || !_target_position.allFinite() || !_target_velocity.allFinite()) {
		failLanding("target estimate unavailable");
		return;
	}
	const double age = (_node.now() - _tag.timestamp).seconds();
	if (age < 0. || age > 0.35) {
		_handoff_time = 0.;
		px4_ros2::TrajectorySetpoint hold;
		hold.withHorizontalVelocity(Eigen::Vector2f::Zero())
		    .withPositionZ(static_cast<float>(vehicle.position.z()));
		_trajectory_setpoint->update(hold);
		return;
	}

	const Vector target_velocity = _stationary_target ? Vector::Zero() :
		Vector(_target_velocity.x(), _target_velocity.y(), 0.);
	const Motion target{_target_position, target_velocity};
	const double horizontal_error = (vehicle.position - target.position).head<2>().norm();
	const double relative_speed = (vehicle.velocity - target.velocity).head<2>().norm();
	const double clearance = target.position.z() - vehicle.position.z();
	const double vertical_speed = std::min(0.6, static_cast<double>(_param_descent_vel));
	Limits limits{std::min(2., static_cast<double>(_param_max_velocity)), vertical_speed, 1.5};
	auto view = *_view;
	view.marker_orientation = _tag.orientation.normalized();
	const Eigen::Quaterniond attitude = _vehicle_attitude->attitude().cast<double>();
	view.yaw = px4_ros2::quaternionToYaw(_vehicle_attitude->attitude());
	if (!attitude.coeffs().allFinite() || attitude.norm() < 1e-6 ||
	    !std::isfinite(view.yaw) || clearance <= 0.) {
		failLanding("invalid camera heading or target clearance");
		return;
	}
	constexpr double approach_clearance_m = 1.;
	constexpr double handoff_hold_s = 0.35;
	const bool ready = horizontal_error <= _param_delta_position &&
		std::abs(clearance - approach_clearance_m) <= _param_delta_position &&
		relative_speed <= _param_delta_velocity &&
		std::abs(vehicle.velocity.z()) <= _param_delta_velocity;
	_handoff_time = ready ? _handoff_time + std::max(0.f, dt_s) : 0.;
	if (_handoff_time >= handoff_hold_s) {
		const double touchdown_speed = std::min(0.25, 0.5 * limits.vertical_speed);
		const double duration = std::max(1., 1.3 * clearance / touchdown_speed);
		Motion end{target.position + duration * target.velocity, target.velocity};
		end.velocity.z() = touchdown_speed;
		const MinimumJerk trajectory(vehicle, end, duration);
		if (view.containsMarker(vehicle.position, attitude.normalized(), _tag.position) &&
		    trajectory.feasible(limits.horizontal_speed, limits.vertical_speed, limits.acceleration)) {
			_final_trajectory = trajectory;
			_final_elapsed = 0.;
			_landing_yaw = static_cast<float>(view.yaw);
			RCLCPP_INFO(_node.get_logger(), "Landing handoff: clearance=%.3f m, XY error=%.3f m, duration=%.3f s",
				clearance, horizontal_error, duration);
			switchToState(State::LANDING);
			return;
		}
	}

	const double goal_z = target.position.z() - approach_clearance_m;
	constexpr double preview_s = 0.4;
	constexpr double margin_ramp = 0.08;
	const auto orientation = attitude.normalized();
	const ApproachReference full_descent{target, vehicle.position.z(), goal_z, _approach_descent_speed};
	Vector preview_position = vehicle.position + preview_s * vehicle.velocity;
	preview_position.z() = full_descent.sample(preview_s).position.z();
	const double corner_margin = std::min(
		view.markerMargin(vehicle.position, orientation, target.position),
		view.markerMargin(preview_position, orientation, target.position + preview_s * target.velocity));
	// Gate downward motion on all four corners. Allow climbing back to the
	// approach height independently of the descent gate.
	const double descent_scale = goal_z > vehicle.position.z() ?
		std::clamp((corner_margin - view.margin) / margin_ramp, 0., 1.) : 1.;
	const ApproachReference reference{target, vehicle.position.z(), goal_z,
	                                  _approach_descent_speed * descent_scale};
	const auto command = _pampc_solver.solve(vehicle, attitude, reference, view, dt_s);
	if (!command) {
		failLanding("MPC solver failed");
		return;
	}
	RCLCPP_DEBUG_THROTTLE(_node.get_logger(), *_node.get_clock(), 500,
		"APPROACH: clearance=%.3f xy_error=%.3f relative_speed=%.3f vz=%.3f handoff=%.3f margin=%.3f vz_ref=%.3f",
		clearance, horizontal_error, relative_speed, vehicle.velocity.z(), _handoff_time,
		corner_margin, reference.sample(0.).velocity.z());
	_used_target_frd = attitude.normalized().conjugate() * (target.position - vehicle.position);
	// PX4 expects normalized collective thrust along negative FRD z.
	constexpr double gravity = 9.8066;
	const float thrust = static_cast<float>(_thrust_calibration.command(command->specific_thrust / gravity));
	_rates_setpoint->update(command->rates_frd.cast<float>(), Eigen::Vector3f(0.f, 0.f, -thrust));
}

bool PrecisionLand::checkTargetTimeout()
{
	if (!_tracking_enabled || !_tag.valid()) {
		return true;
	}

	if (_node.now().seconds() - _tag.timestamp.seconds() > _param_target_timeout) {
		return true;
	}

	return false;
}

std::string PrecisionLand::stateName(State state)
{
	switch (state) {
	case State::IDLE:
		return "IDLE";

	case State::APPROACH:
		return "APPROACH";

	case State::LANDING:
		return "LANDING";

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
