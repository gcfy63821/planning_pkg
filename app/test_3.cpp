/*
date:2024.1.20
author:rqc

testing body_tracking data:ok
note：NECK是203、103,父关节为SPINE_CHEST

*/
#include "ros/ros.h"
#include "visualization_msgs/MarkerArray.h"




void bodyTrackingCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    for (const auto& marker : msg->markers)
    {
        if (marker.id == 103)//neck
        {
            // Found marker with id 203, print its information
            ROS_INFO("Marker 103 Information:");
            ROS_INFO("Position: %.3f, %.3f, %.3f", marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
            ROS_INFO("Orientation: %.3f, %.3f, %.3f, %.3f", marker.pose.orientation.x, marker.pose.orientation.y, marker.pose.orientation.z, marker.pose.orientation.w);
            ROS_INFO("Scale: %.3f, %.3f, %.3f", marker.scale.x, marker.scale.y, marker.scale.z);
            ROS_INFO("Color: %.3f, %.3f, %.3f, %.3f", marker.color.r, marker.color.g, marker.color.b, marker.color.a);
            
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "body_tracking_reader");
    ros::NodeHandle nh;

    // Subscribe to the body tracking data topic
    ros::Subscriber sub = nh.subscribe("/k4a/body_tracking_data", 1, bodyTrackingCallback);

    ros::spin();
    return 0;
}
