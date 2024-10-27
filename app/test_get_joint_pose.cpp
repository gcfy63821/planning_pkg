#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "joint_state_reader");
    ros::NodeHandle nh;

    // 创建机器人模型加载器
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

    // 获取机器人模型
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    // 创建机器人状态
    robot_state::RobotStatePtr robot_state(new robot_state::RobotState(robot_model));

    // 获取关节空间的关节名称
    const std::vector<std::string>& joint_names = robot_model->getJointModelGroup("panda_manipulator")->getVariableNames();

    // 查询当前机械臂的关节状态
    std::vector<double> joint_values;
    robot_state->copyJointGroupPositions("panda_manipulator", joint_values);

    // 打印关节空间坐标
    ROS_INFO("Current Joint Positions:");
    for (std::size_t i = 0; i < joint_names.size(); ++i) {
        ROS_INFO("%s: %f", joint_names[i].c_str(), joint_values[i]);
    }

    return 0;
}
