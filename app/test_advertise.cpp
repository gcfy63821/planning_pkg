/*
0710
测试在moveit中发布规划的轨迹到话题display_planned_path
*/
#include "arm_planning/common_include.h"
#include "arm_planning/planning_interface.h"
#include "arm_planning/planning_request.h"
#include "arm_planning/planning_response.h"
#include <moveit_msgs/DisplayTrajectory.h>

using namespace arm_planning;

// Function to add a single obstacle
void addObstacle()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "panda_link0";
        // The id of the object is used to identify it.
        collision_object.id = "box1";
        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.2;
        primitive.dimensions[primitive.BOX_Y] = 0.2;
        primitive.dimensions[primitive.BOX_Z] = 0.7;
        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.3;
        box_pose.position.y = 0.3;
        box_pose.position.z = 0.4;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }
    {
        moveit_msgs::CollisionObject collision_object;
        collision_object.header.frame_id = "panda_link0";
        // The id of the object is used to identify it.
        collision_object.id = "box2";
        // Define a box to add to the world.
        shape_msgs::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.1;
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.6;
        // Define a pose for the box (specified relative to frame_id)
        geometry_msgs::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = -0.3;
        box_pose.position.y = -0.3;
        box_pose.position.z = 0.4;
        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        collision_objects.push_back(collision_object);
    }
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Duration(0.5).sleep();
    planning_scene_interface.addCollisionObjects(collision_objects);
    ros::Duration(1.0).sleep(); // required: leave some time for publishing the message to Move Group
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    addObstacle();

    PlanningInterface::Ptr pi = std::make_shared<PlanningInterface>(nh, "robot_description");

    PlanningRequest req;
    PlanningResponse res;
    req.group_name_ = "panda_manipulator";

    //initial joint position
    // req.start_joint_pos_ = std::vector<double>{0.0, 0.0, 0.0, -1.57, 0.0, 1.61, 0.86};
    req.start_joint_pos_ = moveit::planning_interface::MoveGroup::getCurrentJointValues();
    req.goal_type_ = "tcp_pose";
    geometry_msgs::Pose goal_pose;
    goal_pose.position.x = -0.25337;
    goal_pose.position.y = 0.414;
    goal_pose.position.z = -0.110;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 1.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 0.0;
    req.goal_pose_ = goal_pose;

    req.b_visualize_start_goal_ = true;
    req.b_visualize_planning_process_ = false;
    req.b_visualize_res_path_ = true;

    std::string algorithm_name = "JTRRT";
    const int num_test = 3;
    double time_cost = 0.0;
    int num_success = 0;
    int iter = 0;

    // 创建发布器
    ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);


    for (size_t i = 0; ros::ok() && i < num_test; i++) {
        std::cout << "test " << i << ": " << std::endl;
        pi->solve(algorithm_name, req, res);
        if (res.success_) {
            num_success++;
            time_cost += res.total_time_cost_;
            iter += res.total_iter_;

            // 将 path_ 转换为 RobotTrajectory
            std::vector<std::string> joint_names = {"panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6", "panda_joint7" }; // 替换为你的机械臂关节名称
            moveit_msgs::RobotTrajectory robot_trajectory = convertToRobotTrajectory(res.path_, joint_names);

            // 发布规划路径
            moveit_msgs::DisplayTrajectory display_trajectory;
            display_trajectory.trajectory_start = req.start_joint_pose_  // 需要设置初始状态
            display_trajectory.trajectory.push_back(robot_trajectory);
            display_publisher.publish(display_trajectory);
        }
    }

    std::cout << "Results for JTRRT: success rate: " << double(num_success) / double(num_test) << std::endl;
    std::cout << "Results for JTRRT: average time cost: " << time_cost / double(num_success) << std::endl;
    std::cout << "Results for JTRRT: average iteration: " << iter / double(num_success) << std::endl;

    ros::Duration(0.2).sleep();
    return 0;
}

