// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_impedance_example_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <franka/robot_state.h>

namespace franka_example_controllers {

bool JointImpedanceExampleController::init(hardware_interface::RobotHW* robot_hw,
                                           ros::NodeHandle& node_handle) {
	std::string arm_id;
	if (!node_handle.getParam("arm_id", arm_id)) {
		ROS_ERROR("JointImpedanceExampleController: Could not read parameter arm_id");
		return false;
	}
	if (!node_handle.getParam("radius", radius_)) {
		ROS_INFO_STREAM(
			"JointImpedanceExampleController: No parameter radius, defaulting to: " << radius_);
	}
	if (std::fabs(radius_) < 0.005) {
		ROS_INFO_STREAM("JointImpedanceExampleController: Set radius to small, defaulting to: " << 0.1);
		radius_ = 0.1;
	}

	if (!node_handle.getParam("vel_max", vel_max_)) {
		ROS_INFO_STREAM(
			"JointImpedanceExampleController: No parameter vel_max, defaulting to: " << vel_max_);
	}
	if (!node_handle.getParam("acceleration_time", acceleration_time_)) {
		ROS_INFO_STREAM(
			"JointImpedanceExampleController: No parameter acceleration_time, defaulting to: "
			<< acceleration_time_);
	}

	std::vector<std::string> joint_names;
	if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
		ROS_ERROR(
			"JointImpedanceExampleController: Invalid or no joint_names parameters provided, aborting "
			"controller init!");
		return false;
	}

	if (!node_handle.getParam("k_gains", k_gains_) || k_gains_.size() != 7) {
		ROS_ERROR(
			"JointImpedanceExampleController:  Invalid or no k_gain parameters provided, aborting "
			"controller init!");
		return false;
	}

	if (!node_handle.getParam("d_gains", d_gains_) || d_gains_.size() != 7) {
		ROS_ERROR(
			"JointImpedanceExampleController:  Invalid or no d_gain parameters provided, aborting "
			"controller init!");
		return false;
	}

	double publish_rate(30.0);
	if (!node_handle.getParam("publish_rate", publish_rate)) {
		ROS_INFO_STREAM("JointImpedanceExampleController: publish_rate not found. Defaulting to "
						<< publish_rate);
	}
	rate_trigger_ = franka_hw::TriggerRate(publish_rate);

	if (!node_handle.getParam("coriolis_factor", coriolis_factor_)) {
		ROS_INFO_STREAM("JointImpedanceExampleController: coriolis_factor not found. Defaulting to "
						<< coriolis_factor_);
	}

	auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
	if (model_interface == nullptr) {
		ROS_ERROR_STREAM(
			"JointImpedanceExampleController: Error getting model interface from hardware");
		return false;
	}
	try {
		model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
			model_interface->getHandle(arm_id + "_model"));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM(
			"JointImpedanceExampleController: Exception getting model handle from interface: "
			<< ex.what());
		return false;
	}

	auto* cartesian_pose_interface = robot_hw->get<franka_hw::FrankaPoseCartesianInterface>();
	if (cartesian_pose_interface == nullptr) {
		ROS_ERROR_STREAM(
			"JointImpedanceExampleController: Error getting cartesian pose interface from hardware");
		return false;
	}
	try {
		cartesian_pose_handle_ = std::make_unique<franka_hw::FrankaCartesianPoseHandle>(
			cartesian_pose_interface->getHandle(arm_id + "_robot"));
	} catch (hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM(
			"JointImpedanceExampleController: Exception getting cartesian pose handle from interface: "
			<< ex.what());
		return false;
	}

	auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
	if (effort_joint_interface == nullptr) {
		ROS_ERROR_STREAM(
			"JointImpedanceExampleController: Error getting effort joint interface from hardware");
		return false;
	}
	for (size_t i = 0; i < 7; ++i) {
		try {
		joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
		} catch (const hardware_interface::HardwareInterfaceException& ex) {
		ROS_ERROR_STREAM(
			"JointImpedanceExampleController: Exception getting joint handles: " << ex.what());
		return false;
		}
	}
	torques_publisher_.init(node_handle, "torque_comparison", 1);

	std::fill(dq_filtered_.begin(), dq_filtered_.end(), 0);
	trajectory_subscriber_ = node_handle.subscribe("/trajectory", 10, &JointImpedanceExampleController::trajectoryCallback, this);

	received_trajectory_ = false;

	return true;
}
void JointImpedanceExampleController::trajectoryCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
	ROS_INFO_STREAM("Received trajectory");
	std::vector<std::vector<double>> trajectory;
	size_t num_points = msg->data.size() / 7;
	for (size_t i = 0; i < num_points; ++i) {
		std::vector<double> point(msg->data.begin() + i * 7, msg->data.begin() + (i + 1) * 7);
		trajectory.push_back(point);
	}
	buffer_.writeFromNonRT(trajectory);
	trajectory_ = trajectory;  // 保存轨迹
	// trajectory_duration_ = (num_points -1)* time_step_;  // 假设每个点的时间间隔是 time_step_
	if (num_points > 0) {
		trajectory_duration_ = (num_points - 1) * time_step_;
	}
	else {
		trajectory_duration_ = 0.0; // 或其他默认值
	}

	received_trajectory_ = true;
	elapsed_time_ = ros::Duration(0.0);
	ROS_INFO_STREAM("Received trajectory with " << trajectory.size() << " points.");
}

void JointImpedanceExampleController::starting(const ros::Time& /*time*/) {
	initial_pose_ = cartesian_pose_handle_->getRobotState().O_T_EE_d;
	elapsed_time_ = ros::Duration(0.0);
	received_trajectory_ = false;
}
//


void JointImpedanceExampleController::update(const ros::Time& /*time*/, const ros::Duration& period){
	if (!received_trajectory_) {
		// 如果没有接收到轨迹，保持初始位置
		for (size_t i = 0; i < 7; ++i) {
		joint_handles_[i].setCommand(0.0); // 设置力矩为0
		//   ROS_INFO_STREAM("Not received trajectory, maintaining initial position.");
		}
		return;
	}

	elapsed_time_ += period;
	ROS_INFO_STREAM(elapsed_time_);

	double t_, slerp_t_, dslerp_t_;

	t_ = std::min(std::max(elapsed_time_.toSec()  / trajectory_duration_, 0.0), 1.0);
	slerp_t_ = 10 * std::pow(t_, 3) - 15 * std::pow(t_, 4) + 6 * std::pow(t_, 5);
	dslerp_t_ = 30 * std::pow(t_, 2) - 60 * std::pow(t_, 3) + 30 * std::pow(t_, 4);

	size_t n = trajectory_.size() - 1;
	size_t segment = std::min(static_cast<size_t>(slerp_t_ * n), n - 1);
	double local_t = (slerp_t_ * n) - segment;

	std::vector<double> start_jp(7, 0.0);
	std::vector<double> next_jp(7, 0.0);
	std::vector<double> joint_pos(7, 0.0);
	std::vector<double> joint_vel(7, 0.0);

	for (size_t i = 0; i < 7; ++i) {
		double p0 = trajectory_[segment][i];
		double p1 = trajectory_[segment + 1][i];
		joint_pos[i] = p0 + (p1 - p0) * local_t;
		joint_vel[i] = (p1 - p0) / time_step_ * dslerp_t_;
		
		start_jp[i] = p0;
		next_jp[i] = p1;
		
	}

	franka::RobotState robot_state = cartesian_pose_handle_->getRobotState();
	std::array<double, 7> coriolis = model_handle_->getCoriolis();
	std::array<double, 7> gravity = model_handle_->getGravity();

	double alpha = 0.99;
	for (size_t i = 0; i < 7; i++) {
		dq_filtered_[i] = (1 - alpha) * dq_filtered_[i] + alpha * robot_state.dq[i];
	}

	ROS_INFO_STREAM("-----slerp_t:"<<slerp_t_<<" t_:"<<t_);
	std::array<double, 7> tau_d_calculated;
	for (size_t i = 0; i < 7; ++i) {
		tau_d_calculated[i] = coriolis_factor_ * coriolis[i] +
						k_gains_[i] * (joint_pos[i] - robot_state.q[i]) +
						d_gains_[i] * (joint_vel[i] - dq_filtered_[i]);
	
		ROS_INFO_STREAM("tau_d_calculated:"<<tau_d_calculated[i]<<" i:"<<i);
	}
	// 限制力矩变化率
	std::array<double, 7> tau_d_saturated = saturateTorqueRate(tau_d_calculated, robot_state.tau_J_d);

	for (size_t i = 0; i < 7; ++i) {
		joint_handles_[i].setCommand(tau_d_saturated[i]);
		
		ROS_INFO_STREAM("tau_d_saturated:"<<tau_d_saturated[i]<<" i:"<<i);
	}

	if (rate_trigger_() && torques_publisher_.trylock()) {
		std::array<double, 7> tau_j = robot_state.tau_J;
		std::array<double, 7> tau_error;
		double error_rms(0.0);
		for (size_t i = 0; i < 7; ++i) {
			tau_error[i] = last_tau_d_[i] - tau_j[i];
			error_rms += std::sqrt(std::pow(tau_error[i], 2.0)) / 7.0;
		}
		torques_publisher_.msg_.root_mean_square_error = error_rms;
		for (size_t i = 0; i < 7; ++i) {
			torques_publisher_.msg_.tau_commanded[i] = last_tau_d_[i];
			torques_publisher_.msg_.tau_error[i] = tau_error[i];
			torques_publisher_.msg_.tau_measured[i] = tau_j[i];
		}
		torques_publisher_.unlockAndPublish();
	}

	for (size_t i = 0; i < 7; ++i) {
		last_tau_d_[i] = tau_d_saturated[i] + gravity[i];
	}
}



std::array<double, 7> JointImpedanceExampleController::saturateTorqueRate(
    const std::array<double, 7>& tau_d_calculated,
    const std::array<double, 7>& tau_J_d) {  // NOLINT (readability-identifier-naming)
	std::array<double, 7> tau_d_saturated{};
	for (size_t i = 0; i < 7; i++) {
		double difference = tau_d_calculated[i] - tau_J_d[i];
		tau_d_saturated[i] = tau_J_d[i] + std::max(std::min(difference, kDeltaTauMax), -kDeltaTauMax);
	}
	return tau_d_saturated;
}
//0804
std::vector<double> JointImpedanceExampleController::interpolateTrajectory(const std::vector<std::vector<double>>& trajectory, double t) {
	size_t idx = static_cast<size_t>(t * (trajectory.size() - 1));
	size_t next_idx = std::min(idx + 1, trajectory.size() - 1);
	double local_t = t * (trajectory.size() - 1) - idx;
	std::vector<double> interpolated(7);
	for (size_t i = 0; i < 7; ++i) {
		interpolated[i] = trajectory[idx][i] + local_t * (trajectory[next_idx][i] - trajectory[idx][i]);
	}
	return interpolated;
}
//0804
std::vector<double> JointImpedanceExampleController::interpolateVelocity(const std::vector<std::vector<double>>& trajectory, double dt) {
	size_t idx = static_cast<size_t>(dt * (trajectory.size() - 1));
	size_t next_idx = std::min(idx + 1, trajectory.size() - 1);
	std::vector<double> velocity(7);
	for (size_t i = 0; i < 7; ++i) {
		velocity[i] = (trajectory[next_idx][i] - trajectory[idx][i]) / (trajectory_duration_ / (trajectory.size() - 1));
	}
	return velocity;
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointImpedanceExampleController,
                       controller_interface::ControllerBase)
