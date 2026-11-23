#pragma once
#include "Pampc.hpp"
#include <optional>
#include <sensor_msgs/msg/camera_info.hpp>
#include <chrono>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <pl_msgs/msg/target_observation.hpp>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <vector>

class PrecisionLand : public px4_ros2::ModeBase
{
public:
	explicit PrecisionLand(rclcpp::Node& node);

	void targetObservationCallback(const pl_msgs::msg::TargetObservation::SharedPtr msg);
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);

	// See ModeBasep
	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	struct ArucoTag {
		// Initialize position with NaN values directly in the struct
		Eigen::Vector3d position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
		Eigen::Quaterniond orientation;
		rclcpp::Time timestamp;

		bool valid() { return timestamp.nanoseconds() > 0; };
	};

	void loadParameters();
	using Trigger = std_srvs::srv::Trigger;
	using SteadyClock = std::chrono::steady_clock;
	void setupStartService();
	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _monitor_pub;
	rclcpp::TimerBase::SharedPtr _monitor_timer;
	void respondStart(bool success, const std::string& message);
	rclcpp::Service<Trigger>::SharedPtr _start_service;
	rclcpp::TimerBase::SharedPtr _start_timer;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _command_pub;
	rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr _ack_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _status_sub;
	px4_msgs::msg::VehicleStatus _status{};
	SteadyClock::time_point _status_received{}, _land_received{}, _start_deadline{};
	std::vector<std::shared_ptr<rmw_request_id_t>> _start_requests;


	ArucoTag getTagWorld(const ArucoTag& tag);

	// Target Kalman filter: constant-velocity model of the (possibly moving) landing target in the
	// world/NED frame. predictTarget() runs every control tick regardless of detections, so the
	// estimate keeps advancing smoothly through brief gaps where the tag isn't visible.
	// correctTarget() runs whenever a new /pl/target_observation detection arrives.
	void predictTarget(float dt_s);
	void correctTarget(const Eigen::Vector3d& measured_position_world);

	void updatePampc(float dt_s);
	void failLanding(const char* reason);
	rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;
	std::optional<pampc::View> _view;
	pampc::MpcSolver _pampc_solver;
	std::optional<pampc::MinimumJerk> _final_trajectory;
	double _marker_size = 1.0;
	pampc::ThrustCalibration _thrust_calibration;
	double _land_detector_z_velocity_max = 0.25;
	double _approach_descent_speed = 0.5;
	bool _stationary_target = false;
	double _handoff_time = 0.;
	double _final_elapsed = 0.;
	double _contact_wait = 0.;
	std::optional<Eigen::Vector3d> _final_error;
	std::optional<Eigen::Vector3d> _used_target_frd;
	std::string _failure_reason;
	float _landing_yaw = 0.f;
	bool checkTargetTimeout();

	enum class State {
		IDLE,
		APPROACH, 	// Stay over landing target while descending
		LANDING,
		FINISHED
	};

	void switchToState(State state);
	std::string stateName(State state);

	// ros2
	rclcpp::Node& _node;
	rclcpp::Subscription<pl_msgs::msg::TargetObservation>::SharedPtr _target_observation_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_world_pub;

	// px4_ros2_cpp
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;
	std::shared_ptr<px4_ros2::RatesSetpointType> _rates_setpoint;

	// Data
	State _state = State::IDLE;
	bool _tracking_enabled = false;

	ArucoTag _tag;
	std::optional<Eigen::Vector3f> _hold_position;
	Eigen::Affine3d _camera_transform = Eigen::Affine3d::Identity();

	// Target Kalman filter state (world/NED frame): position + velocity, and their covariance.
	// State vector order is [px, py, pz, vx, vy, vz].
	Eigen::Vector3d _target_position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
	Eigen::Vector3d _target_velocity = Eigen::Vector3d::Zero();
	Eigen::Matrix<double, 6, 6> _target_P = Eigen::Matrix<double, 6, 6>::Zero();
	bool _target_initialized = false;

	// Land detection
	bool _land_detected = false;
	bool _target_lost_prev = true;

	// Parameters
	float _param_descent_vel = {};
	float _param_max_velocity = {};
	float _param_target_timeout = {};
	float _param_delta_position = {};
	float _param_delta_velocity = {};
	float _param_target_accel_noise = {};
	float _param_target_pos_noise = {};

};
