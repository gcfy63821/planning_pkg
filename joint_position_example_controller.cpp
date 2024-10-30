// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*
edited 0804
can work

*/
#include <franka_example_controllers/joint_position_example_controller.h>
#include <cmath>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_buffer.h>

namespace franka_example_controllers {

bool JointPositionExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                          ros::NodeHandle& node_handle) {
  position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
  if (position_joint_interface_ == nullptr) {
    ROS_ERROR(
        "JointPositionExampleController: Error getting position joint interface from hardware!");
    return false;
  }
  std::vector<std::string> joint_names;
  if (!node_handle.getParam("joint_names", joint_names)) {
    ROS_ERROR("JointPositionExampleController: Could not parse joint names");
  }
  if (joint_names.size() != 7) {
    ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                     << joint_names.size() << " instead of 7 names!");
    return false;
  }
  position_joint_handles_.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    try {
      position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
    } catch (const hardware_interface::HardwareInterfaceException& e) {
      ROS_ERROR_STREAM(
          "JointPositionExampleController: Exception getting joint handles: " << e.what());
      return false;
    }
  }
  // //it is set
  // std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

  // for (size_t i = 0; i < q_start.size(); i++) {
  //   if (std::abs(position_joint_handles_[i].getPosition() - q_start[i]) > 0.1) {
  //     ROS_ERROR_STREAM(
  //         "JointPositionExampleController: Robot is not in the expected starting position for "
  //         "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
  //         "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
  //     return false;
  //   }
  // }
  //BEFORE 0804
  // trajectory_subscriber_ = node_handle.subscribe("/interpolated_trajectory", 10, &JointPositionExampleController::trajectoryCallback, this);

  trajectory_subscriber_ = node_handle.subscribe("/trajectory", 10, &JointPositionExampleController::trajectoryCallback, this);

  received_trajectory_ = false;
  //initiate head
  // current_trajectory_point_ = 0;

  return true;
}

void JointPositionExampleController::trajectoryCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
  ROS_INFO_STREAM("Received trajectory");
  std::vector<std::vector<double>> trajectory;
  size_t num_points = msg->data.size() / 7;
  for (size_t i = 0; i < num_points; ++i) {
    std::vector<double> point(msg->data.begin() + i * 7, msg->data.begin() + (i + 1) * 7);
    trajectory.push_back(point);
  }
  buffer_.writeFromNonRT(trajectory);
  trajectory_ = trajectory;  // 保存轨迹
  trajectory_duration_ = (num_points -1)* time_step_;  // 假设每个点的时间间隔是 time_step_
  received_trajectory_ = true;
  elapsed_time_ = ros::Duration(0.0);
  ROS_INFO_STREAM("Received trajectory with " << trajectory.size() << " points.");
}

void JointPositionExampleController::starting(const ros::Time& /* time */) {
  for (size_t i = 0; i < 7; ++i) {
    
    initial_pose_[i] = position_joint_handles_[i].getPosition();
    ROS_INFO_STREAM("Initial pose: " <<i << " value: "<< initial_pose_[i]);
  }

  elapsed_time_ = ros::Duration(0.0);
}

// 插值函数，生成最小抖动轨迹
std::vector<double> JointPositionExampleController::interpolateTrajectory(const std::vector<std::vector<double>>& trajectory, double t) {
  size_t n = trajectory.size() - 1;
  size_t segment = std::min(static_cast<size_t>(t * n), n - 1);
  double local_t = (t * n) - segment;

  std::vector<double> interpolated_point(7, 0.0);
  for (size_t i = 0; i < 7; ++i) {
    double p0 = trajectory[segment][i];
    double p1 = trajectory[segment + 1][i];
    interpolated_point[i] = p0 + (p1 - p0) * local_t;
  }
  return interpolated_point;
}

void JointPositionExampleController::update(const ros::Time& /*time*/, const ros::Duration& period) {
  if (!received_trajectory_) {
    // 如果没有接收到轨迹，保持初始位置
    //just for simulation
    for (size_t i = 0; i < 7; ++i) {
      position_joint_handles_[i].setCommand(initial_pose_[i]);
      ROS_INFO_STREAM("not received");
    }
    return;
  }

  elapsed_time_ += period;
  // ROS_INFO_STREAM("elapsed_time: " << elapsed_time_.toSec());

  // 计算当前时间点
  double t_ = elapsed_time_.toSec() / trajectory_duration_;
  if (t_ > 1.0) {
    t_ = 0.0;
    return;
    // received_trajectory_ = false;  // 轨迹完成
  }

  double slerp_t_ = 10 * std::pow(t_, 3) - 15 * std::pow(t_, 4) + 6 * std::pow(t_, 5);
  double dslerp_t_ = 30 * std::pow(t_, 2) - 60 * std::pow(t_, 3) + 30 * std::pow(t_, 4);

  // 生成最小抖动轨迹
  std::vector<double> next_point = interpolateTrajectory(trajectory_, slerp_t_);

  // 执行轨迹
  // ROS_INFO_STREAM("-----slerp_t:"<<slerp_t_<<" t_:"<<t_);
  ROS_INFO_STREAM("t: "<<t_<< " sletp_t: "<< slerp_t_<<" joints: " <<next_point[0]<<" "<<next_point[1]<<" "<<next_point[2]<<" "<<next_point[3]<<" "<<next_point[4]<<" "<<next_point[5]<<" "<<next_point[6]<<" ");
  for (size_t i = 0; i < 7; ++i) {
    position_joint_handles_[i].setCommand(next_point[i]);
    // ROS_INFO_STREAM("joint"<< i <<"i "<<next_point[i]);
  }
}

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)
