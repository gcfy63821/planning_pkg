/*
test_transform.cpp
date:2024.1.20
author:rqc


*/
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/bind.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>


void tf_to_robot(const geometry_msgs::PointStamped &pointInJoint3, tf2_ros::Buffer& tfBuffer) {
    std::string robot_frame = "base_frame";  // 机器人坐标系的名称，请根据实际情况修改
    geometry_msgs::TransformStamped robotTransformStamped;
    tf2_ros::TransformBroadcaster br;

    // 在机器人坐标系中广播坐标变换
    robotTransformStamped.header.stamp = ros::Time::now();
    robotTransformStamped.header.frame_id = "joint_3";  // 修改为 modified_id=3 的关节坐标系
    robotTransformStamped.child_frame_id = robot_frame;

    // Convert geometry_msgs::Point to geometry_msgs::Vector3
    robotTransformStamped.transform.translation.x = 2.0;//编的
    robotTransformStamped.transform.translation.y = 2.0;
    robotTransformStamped.transform.translation.z = 2.0;

    robotTransformStamped.transform.rotation = tf2::toMsg(tf2::Quaternion::getIdentity());


    br.sendTransform(robotTransformStamped);


    //robotTransformStamped = tfBuffer.lookupTransform("joint_3", robot_frame, ros::Time(0));
    robotTransformStamped = tfBuffer.lookupTransform(robot_frame, "joint_3", ros::Time(0));

    geometry_msgs::PointStamped robotTransformedPoint;
    tf2::doTransform(pointInJoint3, robotTransformedPoint, robotTransformStamped);

    ROS_INFO("Point in joint_3 coordinates transformed to robot frame: %.3f, %.3f, %.3f",
            robotTransformedPoint.point.x, robotTransformedPoint.point.y, robotTransformedPoint.point.z);
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
            ROS_INFO("start!");
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "transform_example");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster br;

    std::string base_frame = "world_frame";

    ros::Subscriber sub = nh.subscribe<visualization_msgs::MarkerArray>("/k4a/body_tracking_data", 1, boost::bind(bodyTrackingCallback, _1, boost::ref(tfBuffer)));
    

    ros::Rate rate(10);  // 设置循环频率为10Hz

    while (ros::ok())
    {
        ros::spinOnce();  // 处理回调函数
        rate.sleep();     // 控制循环频率
    }

    return 0;
}
