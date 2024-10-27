#include "arm_planning/common_include.h"
#include "arm_planning/planning_interface.h"
#include "arm_planning/planning_request.h"
#include "arm_planning/planning_response.h"

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/bind.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace arm_planning;

double record[4];


void tf_to_robot(const geometry_msgs::PointStamped &pointInJoint3, tf2_ros::Buffer& tfBuffer) {
    std::string robot_frame = "robot_frame";  // 机器人坐标系的名称，请根据实际情况修改
    geometry_msgs::TransformStamped robotTransformStamped;
    tf2_ros::TransformBroadcaster br;

    // 在机器人坐标系中广播坐标变换
    robotTransformStamped.header.stamp = ros::Time::now();
    robotTransformStamped.header.frame_id = "joint_3";  // 修改为 modified_id=3 的关节坐标系
    robotTransformStamped.child_frame_id = robot_frame;

    // Convert geometry_msgs::Point to geometry_msgs::Vector3
    robotTransformStamped.transform.translation.x = 0.2;
    robotTransformStamped.transform.translation.y = 0.0;
    robotTransformStamped.transform.translation.z = 0.0;//编的

    robotTransformStamped.transform.rotation = tf2::toMsg(tf2::Quaternion::getIdentity());


    br.sendTransform(robotTransformStamped);


    robotTransformStamped = tfBuffer.lookupTransform(robot_frame, "joint_3", ros::Time(0));

    geometry_msgs::PointStamped robotTransformedPoint;
    tf2::doTransform(pointInJoint3, robotTransformedPoint, robotTransformStamped);
    // // 发布到话题
    // robotTransformedPointPub.publish(robotTransformedPoint);
    record[0]= robotTransformedPoint.point.x;
    record[1]= robotTransformedPoint.point.y;
    record[2]= robotTransformedPoint.point.z;
}
void bodyTrackingCallback(const visualization_msgs::MarkerArray::ConstPtr& msg, tf2_ros::Buffer& tfBuffer)
{
    tf2_ros::TransformBroadcaster br;

    for (const auto& marker : msg->markers)
    {
        
        int id = marker.id;
        int modified_id = id % 100;  

        if (modified_id <= 3)
        {
            // ROS_INFO("start!");
            std::string parent_frame;
            std::string child_frame = "joint_" + std::to_string(modified_id);

            if(modified_id == 0)
                parent_frame = "camera_frame";
            else
                parent_frame = "joint_" + std::to_string(modified_id - 1);

            //initializing transform
            geometry_msgs::Pose pose = marker.pose;
            geometry_msgs::TransformStamped transform;
            
            transform.header.frame_id = parent_frame;
            transform.child_frame_id = child_frame;
            transform.header.stamp = ros::Time::now();

            transform.transform.translation.x = pose.position.x;
            transform.transform.translation.y = pose.position.y;
            transform.transform.translation.z = pose.position.z;

            transform.transform.rotation = pose.orientation;


            br.sendTransform(transform);

            geometry_msgs::PointStamped point;
            point.header.stamp = ros::Time(0);
            point.header.frame_id = child_frame;

            try
            {
                // 使用 lookupTransform 查找坐标变换
                geometry_msgs::TransformStamped transformStamped;
                //transformStamped = tfBuffer.lookupTransform(parent_frame, child_frame, ros::Time(0));
                transformStamped = tfBuffer.lookupTransform("camera_frame", child_frame, ros::Time(0));

                // 使用坐标变换得到变换后的点
                geometry_msgs::PointStamped transformedPoint;
                tf2::doTransform(point, transformedPoint, transformStamped);

                ROS_INFO("Joint %d transformed coordinates in %s: %.3f, %.3f, %.3f", modified_id, parent_frame.c_str(),
                         transformedPoint.point.x, transformedPoint.point.y, transformedPoint.point.z);

                if(modified_id == 3) {

                    tf_to_robot(transformedPoint,tfBuffer);
                }
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


    PlanningRequest req;
    PlanningResponse res;

    req.group_name_ = "panda_manipulator";

    //initial joint position
    req.start_joint_pos_ = std::vector<double>{0.0, 0.0, 0.0, -1.57, 0.0, 1.61, 0.86};
    req.goal_type_ = "tcp_pose";
    geometry_msgs::Pose goal_pose;
    // goal_pose.position.x = -0.25337;
    // goal_pose.position.y = 0.414;
    // goal_pose.position.z = -0.110;
    goal_pose.orientation.x = 0.0;
    goal_pose.orientation.y = 1.0;
    goal_pose.orientation.z = 0.0;
    goal_pose.orientation.w = 0.0;
    

    req.b_visualize_start_goal_ = true;
    req.b_visualize_planning_process_ = false;
    req.b_visualize_res_path_ = true;

    std::string algorithm_name = "JTRRT";
    const int num_test = 3;
    double time_cost = 0.0;
    int num_success = 0;
    int iter = 0;

    PlanningInterface::Ptr pi = std::make_shared<PlanningInterface>(nh, "robot_description");

    for (size_t i = 0; ros::ok() && i < num_test; i++) {
        std::cout << "test " << i << ": " << std::endl;
        // Create a subscriber for robotTransformedPoint
        // ros::Subscriber robotTransformedPointSub = nh.subscribe<geometry_msgs::PointStamped>(
        // "/robot_transformed_point", 10, robotTransformedPointCallback);


        goal_pose.position.x = record[0];
        goal_pose.position.y = record[1];
        goal_pose.position.z = record[2];
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

