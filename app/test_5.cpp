/*
test_get_cylindar

*/
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

geometry_msgs::Pose get_elbow_to_hand(tf2_ros::Buffer& tfBuffer) {
    geometry_msgs::TransformStamped TransformStamped;
    geometry_msgs::Pose ret;
    try {
        TransformStamped = tfBuffer.lookupTransform("Elbow_right0", "Hand_right0", ros::Time(2));
        // TransformStamped = tfBuffer.lookupTransform("Neck0", "Hand_right0", ros::Time(0));
        
        ret.orientation = TransformStamped.transform.rotation;

        TransformStamped = tfBuffer.lookupTransform("panda_link0", "Hand_right0", ros::Time(2));
        ret.position.x = TransformStamped.transform.translation.x;
        ret.position.y = TransformStamped.transform.translation.y;
        ret.position.z = TransformStamped.transform.translation.z;
    }catch (tf2::TransformException &ex) {
        ROS_WARN("Failed to transform joint %d: %s", 1, ex.what());
    }

    

    return ret;
}

// Function to add a single obstacle
void addObstacle(tf2_ros::Buffer& tfBuffer) {
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = "panda_link0";
    
    //naming the obstacle
    collision_object.id = "cylinder_1";
    
    //setting the obstacle form
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions.resize(2);
    primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.3;
    primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;
    
    //setting the position of obstacle
    geometry_msgs::Pose cylinder_pose;
    // cylinder_pose.orientation.w = 1.0;
    // cylinder_pose.position.x = 0.3;
    // cylinder_pose.position.y = 0.3;
    // cylinder_pose.position.z = 0.4;
    cylinder_pose = get_elbow_to_hand(tfBuffer);
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(cylinder_pose);
    collision_object.operation = collision_object.ADD;

    collision_objects.push_back(collision_object);
    
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    ros::Duration(0.5).sleep();
    planning_scene_interface.addCollisionObjects(collision_objects);
    ros::Duration(1.0).sleep();
}

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

    addObstacle(tfBuffer);
    
    //initial joint position
    req.start_joint_pos_ = std::vector<double>{0.0, 0.0, 0.0, -1.57, 0.0, 1.61, 0.86};// ��ʼ�ؽ�λ��
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

