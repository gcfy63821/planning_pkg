#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotTrajectory.h>
#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/robot_state.h>
#include <franka/control_types.h>

// 转换 MoveIt! 路径为 Franka 机械臂可以执行的格式
std::vector<franka::JointPositions> convertToFrankaTrajectory(const moveit_msgs::RobotTrajectory& trajectory) {
    std::vector<franka::JointPositions> franka_trajectory;
    for (const auto& point : trajectory.joint_trajectory.points) {
        franka::JointPositions joint_positions;
        std::copy(point.positions.begin(), point.positions.end(), joint_positions.q.begin());
        franka_trajectory.push_back(joint_positions);
    }
    return franka_trajectory;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "moveit_franka_control");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化 MoveIt! 接口
    moveit::planning_interface::MoveGroupInterface move_group("panda_arm");
    move_group.setPlanningTime(10.0);

    // 设置目标位置
    geometry_msgs::Pose target_pose;
    target_pose.orientation.w = 1.0;
    target_pose.position.x = 0.3;
    target_pose.position.y = 0.0;
    target_pose.position.z = 0.5;
    move_group.setPoseTarget(target_pose);

    // 进行路径规划
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    if (success) {
        // 打印规划信息
        ROS_INFO("Planning successful.");

        // 将 MoveIt! 路径转换为 Franka 机械臂可以执行的格式
        std::vector<franka::JointPositions> franka_trajectory = convertToFrankaTrajectory(my_plan.trajectory_);

        try {
            // 初始化 Franka 机械臂控制接口
            franka::Robot robot("172.16.0.2");

            // 控制机械臂执行规划路径
            for (const auto& joint_positions : franka_trajectory) {
                robot.control([&joint_positions](const franka::RobotState&, franka::Duration) -> franka::JointPositions {
                    return joint_positions;
                });
            }

            ROS_INFO("Motion executed.");
        } catch (franka::Exception const& e) {
            ROS_ERROR("Franka exception: %s", e.what());
        }
    } else {
        ROS_ERROR("Planning failed.");
    }

    ros::shutdown();
    return 0;
}
