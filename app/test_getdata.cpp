/*
date:2024.1.20
author:rqc

testing body_tracking data:ok
note：NECK是203,父关节为SPINE_CHEST
索引	关节名称	父关节
0	PELVIS	-
1	SPINE_NAVAL	PELVIS
2	SPINE_CHEST	SPINE_NAVAL
3	NECK	SPINE_CHEST

CMakeLists 和package.xml中需要增加 tf2 tf2_ros

*/
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <boost/bind.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>



void bodyTrackingCallback(const visualization_msgs::MarkerArray::ConstPtr& msg, tf2_ros::Buffer& tfBuffer)
{
    tf2_ros::TransformBroadcaster br;

    geometry_msgs::Pose pre_pose;
    for (const auto& marker : msg->markers)
    {
        
        int id = marker.id;
        int modified_id = id % 100;  
        
        if(modified_id == 0) {
            pre_pose.position.x = 0;
            pre_pose.position.y = 0;
            pre_pose.position.z = 0;
        }

        if (modified_id <= 3)
        {
            ROS_INFO("start!");
            std::string parent_frame;
            std::string child_frame = "joint_" + std::to_string(modified_id);

            if(modified_id == 0)
                parent_frame = "base_frame";
            else
                parent_frame = "joint_" + std::to_string(modified_id - 1);

            geometry_msgs::Pose pose = marker.pose;
            geometry_msgs::TransformStamped transform;
            transform.header.stamp = ros::Time::now();
            transform.header.frame_id = parent_frame;
            transform.child_frame_id = child_frame;

            // Convert geometry_msgs::Point to geometry_msgs::Vector3
            transform.transform.translation.x = pose.position.x - pre_pose.position.x;
            transform.transform.translation.y = pose.position.y - pre_pose.position.y;
            transform.transform.translation.z = pose.position.z - pre_pose.position.z;

            transform.transform.rotation = pose.orientation;

            br.sendTransform(transform);

            geometry_msgs::PointStamped point;
            point.header.stamp = ros::Time(0);
            point.header.frame_id = child_frame;

            try
            {
                // 使用 lookupTransform 查找坐标变换
                geometry_msgs::TransformStamped transformStamped;
                transformStamped = tfBuffer.lookupTransform(parent_frame, child_frame, ros::Time(0));

                // 使用坐标变换得到变换后的点
                geometry_msgs::PointStamped transformedPoint;
                tf2::doTransform(point, transformedPoint, transformStamped);

                ROS_INFO("Joint %d transformed coordinates in %s: %.3f, %.3f, %.3f", modified_id, parent_frame.c_str(),
                         transformedPoint.point.x, transformedPoint.point.y, transformedPoint.point.z);
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
