#include <ros/ros.h>
#include <franka_example_controllers/crq_joint_position_controller.h>

// 检查控制器是否执行完成
bool isControllerFinished(const franka_example_controllers::JointPositionExampleController& controller, const ros::Duration& threshold_time) {
    return controller.getElapsedTime() >= threshold_time;
}

// 等待控制器执行完成
int waitForController(franka_example_controllers::JointPositionExampleController& controller, const ros::Duration& finish_threshold, const ros::Duration& timeout, const ros::Duration& wait_interval) {
    ros::Time start_time = ros::Time::now();
    while (!isControllerFinished(controller, finish_threshold)) {
        ros::Duration elapsed_time = ros::Time::now() - start_time;
        if (elapsed_time > timeout) {
            ROS_ERROR("Timeout while waiting for controller to finish");
            return -1; // 返回错误代码表示超时
        }
        ros::Duration(wait_interval).sleep(); // 等待一段时间
    }
    return 0; // 返回 0 表示成功
}

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "joint_position_controller_node");
    ros::NodeHandle nh;

    // 创建一个包含机器人关节名称的字符串数组
    std::vector<std::string> joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", 
                                             "panda_joint4", "panda_joint5", "panda_joint6", 
                                             "panda_joint7"};

    // 将机器人关节名称参数添加到 NodeHandle 中
    nh.setParam("joint_names", joint_names);

    franka_example_controllers::JointPositionExampleController joint_position_controller;

    hardware_interface::RobotHW robot_hardware;

    if (!joint_position_controller.init(&robot_hardware, nh)) {
        ROS_ERROR("Failed to initialize the controller");
        return -1;
    }

    //假设的目标位置。
    std::vector<std::array<double, 7>> target_positions_list = {
        {1.0, 0.5, 0.0, -2.0, 0.0, 2.5, 0.0}, 
        {0.0, 0.2, 0.0, -1.0, 0.0, 1.2, 0.0}, 
        {-1.0, 0.5, 0.0, -1.2, 0.0, 1.6, 0.0} 
    };

    // 设置完成时间的阈值
    ros::Duration finish_threshold(2.0); // 设置为2秒
    // 设置超时时间
    ros::Duration timeout(10.0); // 设置为10秒
    // 设置等待间隔
    ros::Duration wait_interval(0.1); // 设置为0.1秒；


    // for (const auto& target_positions : target_positions_list) {
        
    //     joint_position_controller.update(ros::Time::now(),wait_interval);
    //     // if (!joint_position_controller.setTargetPositions(target_positions)) {
    //     //     ROS_ERROR("Failed to set target positions");
    //     //     return -1;
    //     // }

    //     // 等待控制器执行完成
    //     if (waitForController(joint_position_controller, finish_threshold, timeout, wait_interval) != 0) {
    //         // 如果等待超时或出现其他错误，返回错误代码
    //         return -1;
    //     }
    // }


    // 主循环
    ros::Rate rate(1000); // 控制器更新频率，单位为Hz
    while (ros::ok()) {
        // 更新控制器
        joint_position_controller.update(ros::Time::now(), ros::Duration(1.0 / 1000)); // 更新频率的倒数

        // 在主循环中执行其他任务（如果有需要的话）

        // 等待直到下一个循环
        rate.sleep();
    }


    return 0;
}
