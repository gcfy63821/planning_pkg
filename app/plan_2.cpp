/*0
125
plan2
从现有的tf树查询坐标。
已经可以实现从默认位置找到脖子。但是初始位姿没能引入。
多余的人需要遮住。

Point in NEck coordinates transformed to robot frame: 0.501, 0.182, 0.923

*/
#include "arm_planning/common_include.h"
#include "arm_planning/planning_interface.h"
#include "arm_planning/planning_request.h"
#include "arm_planning/planning_response.h"
#include <moveit/move_group_interface/move_group_interface.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/bind.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace arm_planning;

double record[6];

void bodyTrackingCallback(const visualization_msgs::MarkerArray::ConstPtr& msg, tf2_ros::Buffer& tfBuffer)
{
    tf2_ros::TransformBroadcaster br;

    for (const auto& marker : msg->markers)
    {
        
        int id = marker.id;
        int modified_id = id % 100;  

        if (modified_id == 3)
        {


            geometry_msgs::PointStamped point;
            point.header.stamp = ros::Time(0);
            point.header.frame_id = "Neck0";


            try
            {
                geometry_msgs::TransformStamped robotTransformStamped;

                robotTransformStamped = tfBuffer.lookupTransform("panda_link0", "Neck0", ros::Time(0));

                geometry_msgs::PointStamped robotTransformedPoint;
                tf2::doTransform(point, robotTransformedPoint, robotTransformStamped);
                // // 发布到话题
                // robotTransformedPointPub.publish(robotTransformedPoint);
                record[0]= robotTransformedPoint.point.x;
                record[1]= robotTransformedPoint.point.y;
                record[2]= robotTransformedPoint.point.z;
                
            }
            catch (tf2::TransformException& ex)
            {
                ROS_WARN("Failed to transform joint %d: %s", modified_id, ex.what());
            }
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster br;

    ros::Subscriber sub = nh.subscribe<visualization_msgs::MarkerArray>("/k4a/body_tracking_data", 1, boost::bind(bodyTrackingCallback, _1, boost::ref(tfBuffer)));


    ros::AsyncSpinner spinner(4);
    spinner.start();

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    

    PlanningInterface::Ptr pi = std::make_shared<PlanningInterface>(nh, "robot_description");

    PlanningRequest req;
    PlanningResponse res;
    req.group_name_ = "panda_manipulator";

    // moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");

    // visual_tools.publishRobotState(planning_scene->getCurrentStateNonConst(), rviz_visual_tools::GREEN);
    // planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    // planning_scene->getCurrentStateNonConst();

    //std::vector< double > moveit::planning_interface::MoveGroup::getCurrentJointValues();	
    
    //initial joint position
    req.start_joint_pos_ = std::vector<double>{0.0, 0.0, 0.0, -1.57, 0.0, 1.61, 0.86};// ��ʼ�ؽ�λ��
    // req.start_joint_pos_ = moveit::planning_interface::MoveGroup::getCurrentJointValues();//test?
    req.goal_type_ = "tcp_pose";
    geometry_msgs::Pose goal_pose;
    // goal_pose.position.x = -0.25337;
    // goal_pose.position.y = 0.414;
    // goal_pose.position.z = -0.110;
    // goal_pose.orientation.x = 0.0;
    // goal_pose.orientation.y = 1.0;
    // goal_pose.orientation.z = 0.0;
    // goal_pose.orientation.w = 0.0;
    // req.goal_pose_ = goal_pose;
    goal_pose.orientation.x = -0.6696;
    goal_pose.orientation.y = 0.2453;
    goal_pose.orientation.z = -0.6517;
    goal_pose.orientation.w = 0.2583;

    req.b_visualize_start_goal_ = true;// �Ƿ���ӻ���ʼ��Ŀ��λ��
    req.b_visualize_planning_process_ = false;// �Ƿ���ӻ��滮����
    req.b_visualize_res_path_ = true;// �Ƿ���ӻ��滮���·��

    std::string algorithm_name = "JTRRT";
    const int num_test = 3;
    double time_cost = 0.0;
    int num_success = 0;
    int iter = 0;

    for (size_t i = 0; ros::ok() && i < num_test; i++) {
        std::cout << "test " << i << ": " << std::endl;

        goal_pose.position.x = record[0];
        goal_pose.position.y = record[1];
        goal_pose.position.z = record[2];
        ROS_INFO("Point in NEck coordinates transformed to robot frame: %.3f, %.3f, %.3f",
            record[0], record[1], record[2]);
        req.goal_pose_ = goal_pose;
        pi->solve(algorithm_name, req, res);
        if (res.success_) {
            num_success++;
            time_cost += res.total_time_cost_;
            iter += res.total_iter_;
        }
    }

    std::cout << "Results for JTRRT: success rate: " << double(num_success) / double(num_test) << std::endl;
    std::cout << "Results for JTRRT: average time cost: " << time_cost / double(num_success) << std::endl;
    std::cout << "Results for JTRRT: average iteration: " << iter / double(num_success) << std::endl;

    ros::Duration(0.2).sleep();
    return 0;
}

