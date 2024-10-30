// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_us_controllers/edit_whole_task_controller.h>

#include <cmath>
#include <memory>

#include <controller_interface/controller_base.h>
#include <franka/robot_state.h>
#include <pluginlib/class_list_macros.h>

#include <franka_us_controllers/pseudo_inversion.h>
#include <time.h>
#include <ros/ros.h>

#include "std_msgs/String.h"

#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_buffer.h>

namespace franka_us_controllers {

constexpr double PI = 3.14159265358979323846;
template<typename T>
bool is_in_vector(const std::vector<T> & vector, const T & elt)
{
  return vector.end() != std::find(vector.begin(),vector.end(),elt);
}

bool EditWholeTaskController::init(hardware_interface::RobotHW* robot_hw,
                                               ros::NodeHandle& node_handle) {
    std::vector<double> cartesian_stiffness_vector;
    std::vector<double> cartesian_damping_vector;

    scene_info.a_h = 0.0;

    // get current package path
    try{
      current_package_path_ = ros::package::getPath("franka_us_controllers");
      std::cout<<"Current package path: "<<current_package_path_<<std::endl;
    }
    catch (std::exception& ex){
      ROS_ERROR_STREAM(
          "EditWholeTaskController: cannot get franka_us_controllers path!"<< ex.what());
    }

    // get urdf file path
    if (!node_handle.getParam("urdf_file_relative_to_us_pkg_path", urdf_file_relative_to_us_pkg_path)) {
      ROS_ERROR(
          "MyAvoidCollisionController: Invalid urdf_file, "
          "aborting controller init!");
      return false;
    }
    urdf_filename = current_package_path_+"/"+urdf_file_relative_to_us_pkg_path;
    std::cout<<urdf_filename<<std::endl;
    
    // initing pinocchio
    initialize_pinocchio();

    // get a_h, a_p, p_j4
    sub_perception_info_ = node_handle.subscribe(
      "/us_scene_info", 20, &EditWholeTaskController::PerceptionInfoCallback, this,
      ros::TransportHints().reliable().tcpNoDelay());

    // get trajectory
    sub_heightmap_ = node_handle.subscribe("/us_trajectory", 10, &EditWholeTaskController::NeckTrajectoryCallback, this,
    ros::TransportHints().reliable().tcpNoDelay());  

    if (params.use_service==true){
        heightmap_client_ = node_handle.serviceClient<franka_us_controllers::heightmap>("perception_heightmap_get");
        bool success = heightmap_client_.waitForExistence(ros::Duration(5.0));
        std::cout << "Find service success? " << success << std::endl;    
    }

    // publisher to get trajectory
    get_pub_ = node_handle.advertise<std_msgs::String>("get_trajectory", 1000);

    // for lookup transform, not used
    tfListener = new tf2_ros::TransformListener(tfBuffer);

    if (!node_handle.getParam("debug_mode", debug_mode)) {
        ROS_ERROR_STREAM("EditWholeTaskController: Could not read parameter debug_mode");
        return false;
    }

    if (!node_handle.getParam("ns_torque_threshold", ns_torque_threshold)) {
        ROS_ERROR_STREAM("EditWholeTaskController: Could not read parameter ns_torque_threshold");
        return false;
    }

    if (!node_handle.getParam("probe_rotate_x", probe_rotate_x)) {
        ROS_ERROR_STREAM("EditWholeTaskController: Could not read parameter probe_rotate_x");
        return false;
    }

    if (!node_handle.getParam("p_j4_threshold", p_j4_threshold)) {
        ROS_ERROR_STREAM("EditWholeTaskController: Could not read parameter p_j4_threshold");
        return false;
    }

    std::string arm_id;
    if (!node_handle.getParam("arm_id", arm_id)) {
        ROS_ERROR_STREAM("EditWholeTaskController: Could not read parameter arm_id");
        return false;
    }
    std::vector<std::string> joint_names;
    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7) {
        ROS_ERROR(
            "EditWholeTaskController: Invalid or no joint_names parameters provided, "
            "aborting controller init!");
        return false;
    }

    std::vector<double> scanning_trajectory_offset_vector;
    if (!node_handle.getParam("scanning_trajectory_offset", scanning_trajectory_offset_vector) || scanning_trajectory_offset_vector.size() != 3) {
        ROS_ERROR(
            "EditWholeTaskController: Invalid or no scanning_trajectory_offset parameters provided, "
            "aborting controller init!");
        return false;
    }
    scanning_trajectory_offset<<scanning_trajectory_offset_vector[0],scanning_trajectory_offset_vector[1],scanning_trajectory_offset_vector[2];

    auto* model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr) {
        ROS_ERROR_STREAM(
            "EditWholeTaskController: Error getting model interface from hardware");
        return false;
    }
    try {
        model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
            model_interface->getHandle(arm_id + "_model"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "EditWholeTaskController: Exception getting model handle from interface: "
            << ex.what());
        return false;
    }

    auto* state_interface = robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr) {
        ROS_ERROR_STREAM(
            "EditWholeTaskController: Error getting state interface from hardware");
        return false;
    }
    try {
        state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
            state_interface->getHandle(arm_id + "_robot"));
    } catch (hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "EditWholeTaskController: Exception getting state handle from interface: "
            << ex.what());
        return false;
    }

    auto* effort_joint_interface = robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr) {
        ROS_ERROR_STREAM(
            "EditWholeTaskController: Error getting effort joint interface from hardware");
        return false;
    }
    for (size_t i = 0; i < 7; ++i) {
        try {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
        } catch (const hardware_interface::HardwareInterfaceException& ex) {
        ROS_ERROR_STREAM(
            "EditWholeTaskController: Exception getting joint handles: " << ex.what());
        return false;
        }
    }

    dynamic_reconfigure_compliance_param_node_ =
        ros::NodeHandle(node_handle.getNamespace() + "dynamic_reconfigure_compliance_param_node");

    dynamic_server_compliance_param_ = std::make_unique<
        dynamic_reconfigure::Server<franka_us_controllers::debug_offset_paramConfig>>(
        dynamic_reconfigure_compliance_param_node_);

    dynamic_server_compliance_param_->setCallback(
        boost::bind(&EditWholeTaskController::complianceParamCallback, this, _1, _2));

    end_effector_being_grabbed_ = false;
    probe_too_far = true;
    first_call_service = false;

    position_d_.setZero();
    orientation_d_.coeffs() << 0.0, 0.0, 0.0, 1.0;
    position_d_target_.setZero();
    orientation_d_target_.coeffs() << 0.0, 0.0, 0.0, 1.0;

    cartesian_stiffness_.setZero();
    cartesian_damping_.setZero();

    /* initialize self defined parameter */

    // trajectory generator
    omega = 2 * M_PI / 10.0;
    move_range_in_each_dim << 0.10, 0.0, -0.10;
    // rotation profile generator
    // initialize_rotation_profile();

    // rosbag data logger
    /*
    auto t = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::stringstream ss;
    ss << std::put_time(std::localtime(&t), "/home/robotics/WGK2/ws_ultrasound/src/franka_ros_ultra/franka_us_controllers/bag/%Y%m%d_%H%M%S_my_pd_plus_controller.bag");
    std::string ros_bag_path = ss.str();
    bag.open(ros_bag_path, rosbag::bagmode::Write);
    */

    // Setup publisher for the filtered and original forces.
    /*
    publish_rate_ = franka_hw::TriggerRate(30.0);
    force_torque_pub_.init(node_handle, "force_and_torque", 1, true);
    */
    // debug_pub_.init(node_handle, "controller_debug", 1, true);
    
    // ati force sensor
    ati_sub_ = node_handle.subscribe("/ft_sensor/netft_data", 1000, &EditWholeTaskController::ati_reader_callback, 
        this, ros::TransportHints().reliable().tcpNoDelay());

    force_sensor_test_pub_.init(node_handle, "yxj_test_ft_reader", 1);
    {
        std::lock_guard<realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped>> lock(
            force_sensor_test_pub_);
        force_sensor_test_pub_.msg_.wrench.force.x = 0.0;
        force_sensor_test_pub_.msg_.wrench.force.y = 0.0;
        force_sensor_test_pub_.msg_.wrench.force.z = 0.0;
        force_sensor_test_pub_.msg_.wrench.torque.x = 0.0;
        force_sensor_test_pub_.msg_.wrench.torque.y = 0.0;
        force_sensor_test_pub_.msg_.wrench.torque.z = 0.0;
    }
    af_pub_.init(node_handle, "af", 1);
    {
        std::lock_guard<realtime_tools::RealtimePublisher<std_msgs::Float64>> lock(
            af_pub_);
        af_pub_.msg_.data = 0.0;
    }
    //arm planning
    trajectory_subscriber_ = node_handle.subscribe("/trajectory", 10, &EditWholeTaskController::TrajectoryCallback, this);

    received_trajectory_ = false;

    return true;
}

void EditWholeTaskController::starting(const ros::Time& /*time*/) {
    // compute initial velocity with jacobian and set x_attractor and q_d_nullspace
    // to initial configuration
    franka::RobotState initial_state = state_handle_->getRobotState();
    // get jacobian
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);
    // convert to eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q_initial(initial_state.q.data());
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));

    // set equilibrium point to current state
    position_d_ = initial_transform.translation();
    orientation_d_ = Eigen::Quaterniond(initial_transform.linear());
    position_d_target_ = initial_transform.translation();
    orientation_d_target_ = Eigen::Quaterniond(initial_transform.linear());

    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(initial_state.dq.data());
    dq_last = dq;
    ddq_last << 0.0 ,0.0, 0.0, 0.0 ,0.0, 0.0, 0.0;
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    jacobian_last = jacobian;

    std::array<double, 49> mass = model_handle_->getMass();
    Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());

    // yxj 0118
    Eigen::Matrix<double, 3, 7> J1 = jacobian.block<3,7>(0,0);
    Eigen::Matrix<double, 3, 7> J2 = jacobian.block<3,7>(3,0);
    Eigen::Matrix<double, 1, 7> J3;
    J3<<1,0,0,0,0,0,0;
    Eigen::Matrix<double, 7, 7> J;
    J.block<3,7>(0,0) = J1;
    J.block<3,7>(3,0) = J2;
    J.block<1,7>(6,0) = J3;
    Eigen::Matrix<double, 7, 7> N1 = Eigen::MatrixXd::Identity(7, 7);
    Eigen::Matrix<double, 7, 3> J1_dyn_pinv = M.inverse() * J1.transpose() * (J1 * M.inverse() * J1.transpose()).inverse();
    Eigen::Matrix<double, 7, 7> N2 = Eigen::MatrixXd::Identity(7, 7) - J1.transpose()*J1_dyn_pinv.transpose();
    Eigen::Matrix<double, 3, 7> J2_ba = J2*N2.transpose();
    Eigen::Matrix<double, 7, 3> J2_ba_dyn_pinv=M.inverse() * J2_ba.transpose() * (J2_ba * M.inverse() * J2_ba.transpose()).inverse();
    Eigen::Matrix<double, 7, 7> N3 = N2-J2_ba.transpose()*J2_ba_dyn_pinv.transpose();
    Eigen::Matrix<double, 1, 7> J3_ba=J3*N3.transpose();
    Eigen::Matrix<double, 7, 7> J_ba;
    J_ba.block<3,7>(0,0) = J1;
    J_ba.block<3,7>(3,0) = J2_ba;
    J_ba.block<1,7>(6,0) = J3_ba;
    J_ba_last_ = J_ba;
    Eigen::Matrix<double, 7, 7> B = J_ba*J.inverse();
    B_last_ = B;

    d_d1_ = 2*d_d1_scale_*sqrt(k_d1_);
    d_d2_ = 2*d_d2_scale_*sqrt(k_d2_);
    K_=Eigen::MatrixXd::Identity(7, 7);
    D_=Eigen::MatrixXd::Identity(7, 7);
    K_.block<3,3>(0,0) = k_d1_*Eigen::MatrixXd::Identity(3, 3);
    K_.block<3,3>(3,3) = k_d2_*Eigen::MatrixXd::Identity(3, 3);
    K_(6,6) = k_d3_;
    D_.block<3,3>(0,0) = d_d1_*Eigen::MatrixXd::Identity(3, 3);
    D_.block<3,3>(3,3) = d_d2_*Eigen::MatrixXd::Identity(3, 3);
    d_d3_ = 2*d_d3_scale_*sqrt(k_d3_);
    D_(6,6) = d_d3_;

    scene_info.p_j4.x = 99;
    scene_info.p_j4.y = 99;
    scene_info.p_j4.z = 99;


    // initialize perception_info
    current_point.affine = initial_transform;
    current_point.translational_velocity = (jacobian * dq).block<3, 1>(0, 0);
    current_point.q = q_initial;
    equilibrium_point = current_point;
    ProcessStatus lookup_transform_status;
    lookup_transform_status = lookUpTransformNeck2Base();
    // initialize trajectory base point
    if (lookup_transform_status == ProcessStatus::SUCCESS) {
        perception_info.base_affine_interp = transform_neck2base;
    }
    else {
        perception_info.base_affine_interp = current_point.affine;
    }
    perception_info.heightmap.clear();
    perception_info.last_desired_translation = initial_transform.translation();
    perception_info.last_desired_velocity.setZero();

    // set nullspace equilibrium configuration to initial q
    q_d_nullspace_ = q_initial;
    initial_q_ = q_initial;

    // added x_d_dot, x_d_ddot
    x_d_dot << 0,0,0,0,0,0;
    x_d_ddot << 0,0,0,0,0,0;
    F_ext_filtered_ << 0,0,0,0,0,0;
    F_ext_direct_read_filtered_ << 0,0,0,0,0,0;
    F_ext_K_filtered_ << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    error_last_.setZero();
    error_dot_last_.setZero();
    // q_d_nullspace_fixed << 0.30806299, -0.169668, -0.19364537, -2.41503111,  1.86353862, 2.20101864, -0.80558605;

    // trajectory generator
    initial_traj_point << position_d_[0], position_d_[1], position_d_[2];
    initial_traj_ori = orientation_d_;
    // parse configuration from file
    // parseConfigurationFromFile();

    // rotation_probe2neck << 0, 1, 0, 0, 0, -1, -1, 0, 0;  // FOR PROBE 20220916 icra version
    rotation_probe2neck << 1,0,0,0,0,-1,0,1,0;  // FOR PROBE 20230907 icra version
    // rotation_probe2neck << 1, 0, 0, 0, 0, -1, 0, 1, 0;  // FOR GRIPPER
    K_d_changing_ratio_ = 2.0;
    da_h_accumulate_max_volume_ = 0.05;
    scan_task_period = 30;

    reaching_task_time = 0.0;
    current_scan_start_time = 0.0;
    elapsed_time_ = ros::Duration(0.0);
    progress_time_ = 0.0;

    goto_scan_ = false;

    l8_ati_wrench_.setZero();
}

void EditWholeTaskController::update(const ros::Time& time,
                                         const ros::Duration& period) {
    elapsed_time_ += period;

    // get state variables
    franka::RobotState robot_state = state_handle_->getRobotState();
    std::array<double, 7> coriolis_array = model_handle_->getCoriolis();
    std::array<double, 42> jacobian_array =
        model_handle_->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Map<Eigen::Matrix<double, 6, 1>> F_ext(robot_state.O_F_ext_hat_K.data());
    Eigen::Map<Eigen::Matrix<double, 6, 1>> F_ext_K(robot_state.K_F_ext_hat_K.data());
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_ext_read(robot_state.tau_ext_hat_filtered.data());


    double actual_period = 1e-3;
    if (period.toSec() > 1e-3) {
        actual_period = period.toSec();
    }

    

    Eigen::Matrix<double, 7, 1> ddq = (dq - dq_last) / actual_period;
    Eigen::Matrix<double, 6, 7> djacobian = (jacobian - jacobian_last) / actual_period;
    Eigen::Map<Eigen::Matrix<double, 7, 1>> tau_J_d(  // NOLINT (readability-identifier-naming)
        robot_state.tau_J_d.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());
    current_point.affine = transform;
    current_point.translational_velocity = (jacobian * dq).block<3, 1>(0, 0);
    current_point.q = q;

    Eigen::Matrix<double,7,1> x_dot;
    x_dot.head(6) << jacobian * dq;
    x_dot.tail(1) << dq[0];

    std::array<double, 49> mass = model_handle_->getMass();
    Eigen::Map<Eigen::Matrix<double, 7, 7>> M(mass.data());

    Eigen::Matrix<double, 3, 7> J1 = jacobian.block<3,7>(0,0);
    Eigen::Matrix<double, 3, 7> J2 = jacobian.block<3,7>(3,0);
    Eigen::Matrix<double, 1, 7> J3;
    J3<<1,0,0,0,0,0,0;
    Eigen::Matrix<double, 7, 7> J;
    J.block<3,7>(0,0) = J1;
    J.block<3,7>(3,0) = J2;
    J.block<1,7>(6,0) = J3;
    Eigen::Matrix<double, 7, 7> N1 = Eigen::MatrixXd::Identity(7, 7);
    Eigen::Matrix<double, 7, 3> J1_dyn_pinv = M.inverse() * J1.transpose() * (J1 * M.inverse() * J1.transpose()).inverse();
    Eigen::Matrix<double, 7, 7> N2 = Eigen::MatrixXd::Identity(7, 7) - J1.transpose()*J1_dyn_pinv.transpose();
    Eigen::Matrix<double, 3, 7> J2_ba = J2*N2.transpose();
    Eigen::Matrix<double, 7, 3> J2_ba_dyn_pinv=M.inverse() * J2_ba.transpose() * (J2_ba * M.inverse() * J2_ba.transpose()).inverse();
    Eigen::Matrix<double, 7, 7> N3 = N2-J2_ba.transpose()*J2_ba_dyn_pinv.transpose();
    Eigen::Matrix<double, 1, 7> J3_ba=J3*N3.transpose();
    Eigen::Matrix<double, 7, 7> J_ba;
    J_ba.block<3,7>(0,0) = J1;
    J_ba.block<3,7>(3,0) = J2_ba;
    J_ba.block<1,7>(6,0) = J3_ba;
    Eigen::Matrix<double, 7, 7> J_ba_dot = (J_ba-J_ba_last_)/actual_period;

    Eigen::Matrix<double, 7, 1> v = J_ba * dq;

    //pinocchio to get C
    Eigen::MatrixXd C(pinocchio::computeCoriolisMatrix(reduced_model2,data,q,dq));
    forwardKinematics(reduced_model2,data,q);

    Eigen::Matrix<double, 7, 7> Lambda = J_ba.inverse().transpose()*M*J_ba.inverse();
    
    Eigen::Matrix<double, 7, 7> mu = (J_ba.inverse().transpose()*C-Lambda*J_ba_dot)*J_ba.inverse();
    Eigen::Matrix<double, 7, 7> mu_ba;
    mu_ba.setZero();
    mu_ba.block<3,3>(0,0) << mu.block<3,3>(0,0);
    mu_ba.block<3,3>(3,3) << mu.block<3,3>(3,3);
    mu_ba.block<1,1>(6,6) << mu.block<1,1>(6,6);
    Eigen::Matrix<double, 7, 7> B = J_ba*J.inverse();
    Eigen::Matrix<double, 7, 7> B_dot = (B-B_last_)/actual_period;

    std::array<double, 42> jacobian_flange_array =
        model_handle_->getZeroJacobian(franka::Frame::kFlange);
    Eigen::Map<Eigen::Matrix<double, 6, 7>> jacobian_flange(jacobian_flange_array.data());

    // pseudoinverse for nullspace handling
    // kinematic pseuoinverse
    Eigen::MatrixXd jacobian_pinv;
    pseudoInverse(jacobian, jacobian_pinv);

    Eigen::MatrixXd jacobian_transpose_pinv;
    pseudoInverse(jacobian.transpose(), jacobian_transpose_pinv);

    F_ext_filtered_ = force_filter_params_ * (jacobian_transpose_pinv * tau_ext_read) + (1.0 - force_filter_params_) * F_ext_filtered_;
    // F_ext_filtered_ = force_filter_params_ * (F_ext) + (1.0 - force_filter_params_) * F_ext_filtered_;
    F_ext_direct_read_filtered_ = force_filter_params_ * F_ext + (1.0 - force_filter_params_) * F_ext_direct_read_filtered_;
    F_ext_K_filtered_ = force_filter_params_ * F_ext_K + (1.0 - force_filter_params_) * F_ext_K_filtered_;
    double f_0 = a_f_weightfunc_center;// parameters
    // double F_z = std::abs(F_ext_K_filtered_(2));// icra23 version, use f_ext in franka

    double F_z = l8_ati_wrench_[2];// icra24 version, use ati sensor
    Eigen::MatrixXd adj = Eigen::MatrixXd::Identity(6, 6);
    adj.block<3,3>(0,0)=transform.rotation();
    adj.block<3,3>(3,3)=transform.rotation();
    b_ati_wrench_ = adj*l8_ati_wrench_;
    double a_f = (1 / (1 + std::pow((f_0 / std::max(1e-5, std::abs(F_z))), 6)));
    tau_null_ = -tau_ext_read-jacobian_flange.transpose()*b_ati_wrench_;
    a_n = (1 / (1 + std::pow((ns_torque_threshold / std::max(1e-5, tau_null_.norm())), 6)));

    a_f_filtered_ = filter_params_ * a_f + (1 - filter_params_) * a_f_filtered_;
    a_f_raw_ = a_f;
    // std::cout << "af_raw: " << a_f << " F_z: " << F_z << std::endl;
    if (af_pub_.trylock()){
        af_pub_.msg_.data = a_f_filtered_;
        af_pub_.unlockAndPublish();
    }
    p_j4<<scene_info.p_j4.x,scene_info.p_j4.y,scene_info.p_j4.z;
    a_b = 1/(1+std::pow( (p_j4.norm()/p_j4_threshold),6));
    //=====================================================
    Eigen::Matrix<double, 6, 3> desired_traj;


    if ( (1 - a_h_filtered_) <0.2) {
        desired_traj = GuidingModeTrajectoryAndStiffnessGenerator(elapsed_time_.toSec(), period.toSec());
    }
    else {
        // nullspace operation
        

        ROS_INFO_STREAM_THROTTLE(1, "p_j4: "<<p_j4);
        if (tau_null_.norm()>ns_torque_threshold){
            x3_d = NSContactModeTrajectoryGenerator();
        }
        else{
            k_d3_ = filter_params_ * k_d3_target_ + (1.0 - filter_params_) * k_d3_;
            x3_d = NSAvoidingModeTrajectoryGenerator(p_j4);
        }
        
        // taskspace operation
        if (!flag_human_is_ready_){
            desired_traj = WaitingModeTrajectoryAndStiffnessGenerator(elapsed_time_.toSec(), period.toSec());
            if (!perception_info.heightmap.empty()){
                human_preparing_time_+=period.toSec();
                if (human_preparing_time_>=3){
                    ROS_WARN("5 seconds up, HUMAN IS READY!!!!");
                    flag_human_is_ready_ = true;
                }
            }
        }
        else if(!perception_info.heightmap.empty() && flag_human_is_ready_){
            // If scanning heightmap is received, it means scanning mode or recovery mode
            // now we need to decide which point in the traj is the one we are going to approach
            ProcessStatus lookup_transform_status;
            // if (!neck_frame_fixed_flag){
            //     lookup_transform_status = lookUpTransformNeck2Base();
            //     neck_frame_fixed_flag = true;
            // }
            lookup_transform_status = lookUpTransformNeck2Base();
            
            // initialize trajectory base point
            if (lookup_transform_status == ProcessStatus::SUCCESS) {
                perception_info.base_affine_interp = transform_neck2base;
            }
            else {
                perception_info.base_affine_interp = current_point.affine;
            }
            double x = 0.0, y = 0.0, z = 0.0;
            double scan_task_omega = 2 * M_PI / scan_task_period;
            double phase = 0.5 * (1 - std::cos(scan_task_omega * progress_time_));
            x = (1 - phase) * perception_info.x_min + phase * perception_info.x_max;
            for(int i = 0; i < perception_info.heightmap.size() - 1; i++) {
                if (x >= perception_info.heightmap[i](0) && x <= perception_info.heightmap[i+1](0)) {
                    double alpha_interp = (x - perception_info.heightmap[i](0)) / (perception_info.heightmap[i+1](0) - perception_info.heightmap[i](0));
                    x = alpha_interp * perception_info.heightmap[i+1](0) + (1 - alpha_interp) * perception_info.heightmap[i](0);
                    y = alpha_interp * perception_info.heightmap[i+1](1) + (1 - alpha_interp) * perception_info.heightmap[i](1);
                    z = alpha_interp * perception_info.heightmap[i+1](2) + (1 - alpha_interp) * perception_info.heightmap[i](2);
                }
            }
            
            Eigen::Vector3d desired_position_in_neck_frame = {x, y, z};

            Eigen::Matrix3d rotation_probe_bias;
            
            rotation_probe_bias << 1,                        0,                        0,
                                   0, std::cos(M_PI*probe_rotate_x/180), -std::sin(M_PI*probe_rotate_x/180),
                                   0, std::sin(M_PI*probe_rotate_x/180), std::cos(M_PI*probe_rotate_x/180);

            desired_orientation_ = Eigen::Quaterniond(perception_info.base_affine_interp.linear() * rotation_probe2neck * rotation_probe_bias);
            if (state_ == 2 || state_ == 3) {// recovery mode or scanning mode
                tf2_ros::TransformBroadcaster br;
                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "panda_link0";
                transformStamped.child_frame_id = "received_point_without_offset";

                transformStamped.transform.translation.x = (perception_info.base_affine_interp * desired_position_in_neck_frame)[0];
                transformStamped.transform.translation.y = (perception_info.base_affine_interp * desired_position_in_neck_frame)[1];
                transformStamped.transform.translation.z = (perception_info.base_affine_interp * desired_position_in_neck_frame)[2];
                transformStamped.transform.rotation.x = desired_orientation_.x();
                transformStamped.transform.rotation.y = desired_orientation_.y();
                transformStamped.transform.rotation.z = desired_orientation_.z();
                transformStamped.transform.rotation.w = desired_orientation_.w();
                br.sendTransform(transformStamped);

                desired_translation_ = perception_info.base_affine_interp * (desired_position_in_neck_frame + scanning_trajectory_offset);//transform the desired point in neck frame to base frame 
                
            }
            else {// waiting mode
                desired_translation_ = perception_info.base_affine_interp * desired_position_in_neck_frame;
            }
            

            if ((((desired_translation_-current_point.affine.translation()).norm()>0.05) && a_f_filtered_<0.5) ){// 0922video, this correspond to our method in paper
                // desired_traj = RecoveryModeTrajectoryGenerator(actual_period,desired_translation_, desired_orientation_);
                BroadcastTransform(desired_translation_,desired_orientation_,"panda_link0","planning_target");
                desired_traj = RecoveryModeTrajectoryGenerator(actual_period);
                // ROS_INFO_STREAM_THROTTLE(0.1,"Recovery mode, desired traj variation:"<<(desired_traj(0,0)-desired_translation_(0))<<" "
                // <<(desired_traj(1,0)-desired_translation_(1))<<" "<<(desired_traj(2,0)-desired_translation_(2)));
            }
            else{
                if (recovery_t_ <1.0) {
                    // desired_traj = RecoveryModeTrajectoryGenerator(actual_period,desired_translation_, desired_orientation_);
                    BroadcastTransform(desired_translation_,desired_orientation_,"panda_link0","planning_target");
                    desired_traj = RecoveryModeTrajectoryGenerator(actual_period);
                }
                else {
                    desired_traj = ScanningModeTrajectoryGenerator(elapsed_time_.toSec(), actual_period, desired_translation_, desired_orientation_);
                }
            }

        }
    }


//-----------
    position_d_ = desired_traj.block<3, 1>(0, 0);
    x_d_dot.block<3, 1>(0, 0) = desired_traj.block<3, 1>(0, 1);
    x_d_ddot.block<3, 1>(0, 0) = desired_traj.block<3, 1>(0, 2);
    // convert euler angle to quaternion
    Eigen::Vector3d eulerAngle = desired_traj.block<3, 1>(3, 0);
    Eigen::Quaterniond orientation_d_(Eigen::AngleAxisd(eulerAngle(0), Eigen::Vector3d::UnitZ()) *
                                        Eigen::AngleAxisd(eulerAngle(1), Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(eulerAngle(2), Eigen::Vector3d::UnitX()));
    Eigen::Vector3d omega_d_ = desired_traj.block<3, 1>(3, 1);
    Eigen::Vector3d beta_d_ = desired_traj.block<3, 1>(3, 2);
    x_d_dot.block<3, 1>(3, 0) = omega_d_;
    x_d_ddot.block<3, 1>(3, 0) = beta_d_;

    //=====================================================

    // ROS_INFO_STREAM_THROTTLE(1,"6===="<<orientation_d_.coeffs());
    //0912!!! for debug use
    // Eigen::Vector3d x1_d,x1_d_dot,x1_d_ddot;
    // x1_d[0] = position_d_[0];
    // x1_d[1] = position_d_[1];
    // x1_d[2] = position_d_[2]+0.1 * std::sin(M_PI / 5.0 * (elapsed_time_.toSec()));
    // Eigen::Quaterniond x2_d(initial_traj_ori);
    // double x3_d,x3_d_dot,x3_d_ddot;

    // avoid collision
    // Eigen::Vector3d p_j4;
    // p_j4<<scene_info.p_j4.x,scene_info.p_j4.y,scene_info.p_j4.z;
    // // 检查每个元素是否为NaN
    // bool containsNaN = std::isnan(p_j4[0]) || std::isnan(p_j4[1]) || std::isnan(p_j4[2]);
    // if (containsNaN) p_j4<<99,99,99;
    // // std::cout<<p_j4[0]<<p_j4[1]<<p_j4[2];

    x3_d_dot = 0;//末端位置和方向的期望速度和加速度可以从traj generator得到，但是joint0的速度加速度就置0吧
    x3_d_ddot = 0;
    Eigen::Matrix<double, 7, 1> x_d_dot_7d,x_d_ddot_7d;
    x_d_dot_7d<<x_d_dot[0],x_d_dot[1],x_d_dot[2],0,0,0,x3_d_dot;
    x_d_ddot_7d<<x_d_ddot[0],x_d_ddot[1],x_d_ddot[2],0,0,0,x3_d_ddot;

    // update stiffness and damping
    d_d1_ = 2*d_d1_scale_*sqrt(k_d1_);
    d_d2_ = 2*d_d2_scale_*sqrt(k_d2_);
    K_=Eigen::MatrixXd::Identity(7, 7);
    D_=Eigen::MatrixXd::Identity(7, 7);
    K_.block<3,3>(0,0) = k_d1_*Eigen::MatrixXd::Identity(3, 3);
    K_.block<3,3>(3,3) = k_d2_*Eigen::MatrixXd::Identity(3, 3);
    K_(6,6) = k_d3_;
    D_.block<3,3>(0,0) = d_d1_*Eigen::MatrixXd::Identity(3, 3);
    D_.block<3,3>(3,3) = d_d2_*Eigen::MatrixXd::Identity(3, 3);
    d_d3_ = 2*d_d3_scale_*sqrt(k_d3_);
    D_(6,6) = d_d3_;

    // compute error to desired pose
    // position error
    Eigen::Matrix<double, 6, 1> error;
    error.head(3) << position - position_d_;

    Eigen::Matrix<double, 7, 1> error7d;
    error7d.head(3) << position - position_d_;

    // orientation error
    if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
    error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error.tail(3) << -transform.linear() * error.tail(3);

    error7d.block<3,1>(3,0) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    error7d.block<3,1>(3,0) << -transform.linear() * error7d.block<3,1>(3,0);

    // task 3 error
    error7d.tail(1)<<q[0]-x3_d;

    // ROS_INFO_STREAM_THROTTLE(0.5,"error trans: " << error[0] << error[1] << error[2]);
    // ROS_INFO_STREAM_THROTTLE(0.5,"error rot: " << error[3] << error[4] << error[5]);

    /* compute control */
    // allocate variables
    // Eigen::VectorXd tau_task(7), tau_nullspace(7), tau_d(7);

    Eigen::Matrix<double, 6, 1> error_dot = (error-error_last_)/1e-3;

    Eigen::VectorXd tau_pd(7), tau_ff(7), tau_mu(7), tau_ext_comp(7), tau_d(7);

    // the controller for real robot
    tau_pd = J_ba.transpose()*(-D_*x_dot - K_*error7d);
    tau_mu = J_ba.transpose()*(mu-mu_ba)*v;
    //  bool containsNaN = std::isnan(p_j4[0]) || std::isnan(p_j4[1]) || std::isnan(p_j4[2]);
    // tau_mu.setZero();
    tau_ff = J_ba.transpose()*(Lambda*(B*x_d_ddot_7d+B_dot*x_d_dot_7d)+mu_ba*B*x_d_dot_7d);
    
    Eigen::Matrix<double,7,1> F_ext_xdot;
    // Eigen::Matrix<double,7,1> F_ext_xdot = J.inverse().transpose()*tau_collavoid_;
    F_ext_xdot.head(6)<<0,0,0,0,0,0;
    
    // tau_ext_comp = J_ba.transpose()*(Eigen::MatrixXd::Identity(7, 7)-B.inverse().transpose())*J.inverse().transpose()*tau_ext_read;//效果很不好
    tau_ext_comp = J_ba.transpose()*(Eigen::MatrixXd::Identity(7, 7)-B.inverse().transpose())*F_ext_xdot;
    
    // ROS_INFO_STREAM_THROTTLE(1,"error"<<error);
    // ROS_INFO_STREAM_THROTTLE(1,"tau_collavoid_"<<tau_collavoid_);
    // ROS_INFO_STREAM_THROTTLE(0.5, "tau_pd: " << tau_pd << ", tau_mu: " << tau_mu << ", tau_ff: " << tau_ff << ", tau_ext_comp: " << tau_ext_comp) ;
    // Desired torque
    tau_d << tau_pd + tau_mu + tau_ff + tau_ext_comp;
    if (debug_mode) {
        ROS_INFO_THROTTLE(0.01,"error:%f,%f,%f,%f,%f,%f,%f",error[0],error[1],error[2],error[3],error[4],error[5],error[6]);
        ROS_INFO_THROTTLE(0.01,"K_:%f,%f,%f,%f,%f,%f,%f",K_(0,0),K_(1,1),K_(2,2),K_(3,3),K_(4,4),K_(5,5),K_(6,6));
        ROS_INFO_THROTTLE(0.01,"D_:%f,%f,%f,%f,%f,%f,%f",D_(0,0),D_(1,1),D_(2,2),D_(3,3),D_(4,4),D_(5,5),D_(6,6));
        ROS_INFO_THROTTLE(0.01,"error_dot:%f,%f,%f,%f,%f,%f",error_dot[0],error_dot[1],error_dot[2],error_dot[3],error_dot[4],error_dot[5]);
        // ROS_INFO_STREAM_THROTTLE(0.01,"J_ba:"<<std::endl<<J_ba);
        // ROS_INFO_STREAM_THROTTLE(0.01,"mu_ba:"<<std::endl<<mu_ba);
        // ROS_INFO_STREAM_THROTTLE(0.01,"mu:"<<std::endl<<mu);
        ROS_INFO_THROTTLE(0.01,"tau_pd:%f,%f,%f,%f,%f,%f,%f",tau_pd[0],tau_pd[1],tau_pd[2],tau_pd[3],tau_pd[4],tau_pd[5],tau_pd[6]);
        ROS_INFO_THROTTLE(0.01,"tau_ff:%f,%f,%f,%f,%f,%f,%f",tau_ff[0],tau_ff[1],tau_ff[2],tau_ff[3],tau_ff[4],tau_ff[5],tau_ff[6]);
        ROS_INFO_THROTTLE(0.01,"position_d_:%f,%f,%f",position_d_[0],position_d_[1],position_d_[2]);
        // ROS_INFO_THROTTLE(0.01,"tau_mu:%f,%f,%f,%f,%f,%f,%f",tau_mu[0],tau_mu[1],tau_mu[2],tau_mu[3],tau_mu[4],tau_mu[5],tau_mu[6]);
        // ROS_INFO_THROTTLE(0.01,"tau_ext_comp:%f,%f,%f,%f,%f,%f,%f",tau_ext_comp[0],tau_ext_comp[1],tau_ext_comp[2],tau_ext_comp[3],tau_ext_comp[4],tau_ext_comp[5],tau_ext_comp[6]);
        ROS_INFO_THROTTLE(0.01,"state:%d",state_);
        ROS_INFO_STREAM_THROTTLE(0.01,"tau_J_d,"<<std::endl<<tau_J_d);
        ROS_INFO_STREAM_THROTTLE(0.01,"tau_d,"<<std::endl<<tau_d);
    }


    // tau_d << tau_pd + tau_mu + tau_ff;

    // ROS_INFO_STREAM_THROTTLE(1,"update K_d1   "<<K_d1<<"   update K_d2   "<<K_d2<<"    K_nullspace  "<<k_d3_);

    // norm of forces
    // ROS_INFO_STREAM_THROTTLE(0.5, "TASK_ERROR: " << tau_task_error_part.norm() << ", TASK_FORCE: " << tau_task_force_part.norm() << ", NULL_ERROR: " << tau_nullspace_error_part.norm() << ", CORIOLIS: " << coriolis.norm()) ;

    // Saturate torque rate to avoid discontinuities
    tau_d << saturateTorqueRate(tau_d, tau_J_d);

    // Eigen::Matrix<double, 7, 1> limit_torque;
    // limit_torque<<5,5,5,5,5,5,5;
    // tau_d << saturateTorqueRate(tau_d, limit_torque);
    // tau_d.setZero();
    for (size_t i = 0; i < 7; ++i) {
        joint_handles_[i].setCommand(tau_d(i));
    }
    // k_d1_ = filter_params_ * k_d1_target_ + (1.0 - filter_params_) * k_d1_;
    // k_d2_ = filter_params_ * k_d2_target_ + (1.0 - filter_params_) * k_d2_;
    // k_d3_ = filter_params_ * k_d3_target_ + (1.0 - filter_params_) * k_d3_;
    d_d1_scale_ = filter_params_ * d_d1_scale_target_ + (1.0 - filter_params_) * d_d1_scale_;
    d_d2_scale_ = filter_params_ * d_d2_scale_target_ + (1.0 - filter_params_) * d_d2_scale_;
    d_d3_scale_ = filter_params_ * d_d3_scale_target_ + (1.0 - filter_params_) * d_d3_scale_;

    // ROS_INFO_STREAM_THROTTLE(1,"======time: "<< elapsed_time_.toSec() << ", latest period: " << period.toSec()<<"=====" );

    dq_last = dq;
    ddq_last = ddq;
    jacobian_last = jacobian;
    error_last_ = error;
    J_ba_last_ = J_ba;
    B_last_ = B;

    if (yxj_counter%100==0){
        // std::cout<<"jacobian: "<<(jacobian*jacobian.transpose()).determinant()<<std::endl;
        yxj_counter=0;
        log_F_ext.push_back(F_ext);
        log_error.push_back(error);
        log_tau.push_back(tau_d);
        log_F_ext_filtered.push_back(F_ext_direct_read_filtered_);
        log_t.push_back(time.toSec());
        log_x_d.push_back(desired_traj.block<6, 1>(0, 0));
        log_position.push_back(position);
        log_orientation.push_back(orientation);
        log_a_h.push_back(a_h_filtered_);
        log_a_p.push_back(a_p_filtered_);
        log_a_f.push_back(a_f_filtered_);
        log_tau_null.push_back(tau_null_);
        log_kd1.push_back(k_d1_);
        log_kd2.push_back(k_d2_);
        log_kd3.push_back(k_d3_);
        log_x3_d.push_back(x3_d);
        log_ns_state.push_back(ns_state_);
        log_progress_time.push_back(progress_time_);
        log_state.push_back(state_);
        log_orientation_d_.push_back(orientation_d_);
        log_recovery_start_time.push_back(current_recovery_start_time);
        log_F_ext_K.push_back(F_ext_K_filtered_);
        log_ati_K.push_back(l8_ati_wrench_);
        log_p_j4.push_back(p_j4);
        log_human_induced_goal_offset_.push_back(human_induced_goal_offset_);
        log_a_b.push_back(a_b);
        log_a_n.push_back(a_n);
        log_error_3.push_back(error7d[6]);
    }
    yxj_counter++;

}

Eigen::Matrix<double, 7, 1> EditWholeTaskController::saturateTorqueRate(
        const Eigen::Matrix<double, 7, 1>& tau_d_calculated,
        const Eigen::Matrix<double, 7, 1>& tau_J_d) {  // NOLINT (readability-identifier-naming)
    Eigen::Matrix<double, 7, 1> tau_d_saturated{};
    for (size_t i = 0; i < 7; i++) {
        double difference = tau_d_calculated[i] - tau_J_d[i];
        tau_d_saturated[i] =
            tau_J_d[i] + std::max(std::min(difference, delta_tau_max_), -delta_tau_max_);
    }
    return tau_d_saturated;
}

void EditWholeTaskController::stopping(const ros::Time &time){
    ROS_INFO("Stopping Controller and Saving data......");
    std::string source_path("/home/robotics/yxj/ws_ultrasound/src/data/");

    time_t now = std::time(NULL);
    tm* tm_t = localtime(&now);
    std::stringstream ss;
    ss<<tm_t->tm_year+1900<<"-"<<tm_t->tm_mon+1<<"-"<<tm_t->tm_mday<<"-"<<tm_t->tm_hour<<"-"<<tm_t->tm_min;
    std::string command_create_folder;
    command_create_folder = "mkdir -p "+ source_path + ss.str() + "/";
    system(command_create_folder.c_str());

    std::string path(source_path + ss.str() + "/");
    std::string filename1("log_error.bin");
    std::string filename2("log_F_ext.bin");
    std::string filename3("log_t.bin");
    std::string filename4("log_tau.bin");
    std::string filename5("log_F_ext_filtered.bin");
    std::string filename6("log_x_d.bin");
    std::string filename7("log_position.bin");
    std::string filename8("log_orientation.bin");
    std::string filename9("log_a_h.bin");
    std::string filename10("log_a_p.bin");
    std::string filename11("log_a_f.bin");
    std::string filename12("log_kd1.bin");
    std::string filename13("log_kd2.bin");
    std::string filename19("log_kd3.bin");
    std::string filename14("log_progress_time.bin");
    std::string filename15("log_state.bin");
    std::string filename16("log_orientation_d_.bin");
    std::string filename17("log_recovery_start_time.bin");
    std::string filename18("log_F_ext_K.bin");
    std::string filename20("log_tau_null.bin");
    std::string filename21("log_ns_state.bin");
    std::string filename22("log_x3_d.bin");
    std::string filename23("log_p_j4.bin");
    std::string filename24("log_ati_K.bin");
    std::string filename25("log_human_induced_goal_offset_.bin");
    std::string filename26("log_a_b.bin");
    std::string filename27("log_a_n.bin");
    std::string filename28("log_error_3.bin");
    
    


    saveData(log_error,path+filename1);
    saveData(log_F_ext,path+filename2);
    saveData(log_t,path+filename3);
    saveData(log_tau,path+filename4);
    saveData(log_F_ext_filtered,path+filename5);
    saveData(log_x_d,path+filename6);
    saveData(log_position,path+filename7);
    saveData(log_orientation,path+filename8);
    saveData(log_a_h,path+filename9);
    saveData(log_a_p,path+filename10);
    saveData(log_a_f,path+filename11);
    saveData(log_kd1,path+filename12);
    saveData(log_kd2,path+filename13);
    saveData(log_kd3,path+filename19);
    saveData(log_progress_time,path+filename14);
    saveData(log_state,path+filename15);
    saveData(log_orientation_d_,path+filename16);
    saveData(log_recovery_start_time,path+filename17);
    saveData(log_F_ext_K,path+filename18);
    saveData(log_tau_null,path+filename20);
    saveData(log_ns_state,path+filename21);
    saveData(log_x3_d,path+filename22);
    saveData(log_p_j4,path+filename23);
    saveData(log_ati_K,path+filename24);
    saveData(log_human_induced_goal_offset_,path+filename25);
    saveData(log_a_b,path+filename26);
    saveData(log_a_n,path+filename27);
    saveData(log_error_3,path+filename28);
    ROS_INFO("The Data is saved......");
} 


void EditWholeTaskController::complianceParamCallback(franka_us_controllers::debug_offset_paramConfig& config,
    uint32_t /*level*/) {
    std::lock_guard<std::mutex> joint1_vel_target_lock(joint1_vel_target_mutex_);
    joint1_vel_target_ = config.joint1_vel_target;
    k_d1_target_ = config.k_d1_target;
    k_d2_target_ = config.k_d2_target;
    k_d3_target_ = config.k_d3_target;
    d_d1_scale_target_ = config.d_d1_scale_target;
    d_d2_scale_target_ = config.d_d2_scale_target;
    d_d3_scale_target_ = config.d_d3_scale_target;
    scanning_trajectory_offset[0] = config.offset_x;
    scanning_trajectory_offset[1] = config.offset_y;
    scanning_trajectory_offset[2] = config.offset_z;
    probe_rotate_x = config.probe_rotate_x;
}

void EditWholeTaskController::NeckTrajectoryCallback(const us_trajectory& msg){
    if (flag_get_traj_ == false){
        if (!perception_info.heightmap.empty()) {perception_info.heightmap.clear();}
        for (int i=0;i<msg.trajectory_size;i++) {
            Eigen::Vector3d map_point;
            map_point(0) = msg.path_points[i].x;
            map_point(1) = msg.path_points[i].y;
            map_point(2) = msg.path_points[i].z;
            perception_info.heightmap.push_back(map_point);
        }
        perception_info.x_min = perception_info.heightmap.front()(0);
        perception_info.x_max = perception_info.heightmap.back()(0);
        ROS_WARN("Successfully subscribe the new us trajectory!");
        flag_get_traj_ = true;
    }
}

double EditWholeTaskController::NSContactModeTrajectoryGenerator(){
    ns_state_ = 2;
    ROS_WARN_THROTTLE(1, "Now we are in NS contacting!!");
    // k_d3_ = k_d3_target_*(ns_torque_threshold/(std::max(ns_torque_threshold,tau_null_.norm())));
    double a_n = (1 / (1 + std::pow((ns_torque_threshold / std::max(1e-5, tau_null_.norm())), 6)));
    k_d3_ = k_d3_target_ * a_n;
    equilibrium_point = current_point;
    joint1_vel_target_ = current_point.q(0)-(initial_q_(0)+human_induced_goal_offset_);
    return 0;
}

double EditWholeTaskController::NSAvoidingModeTrajectoryGenerator(Eigen::Matrix<double, 3, 1> p_j4){
   if (p_j4.norm()<0.34){
        ns_state_ = 1;
        ROS_WARN_THROTTLE(1, "Now we are in NS avoiding!!");
        safe_flag_ = 0;
        positive_decrease_count_=0;
        negative_increase_count_=0;
        if (p_j4[1]>0){
            if (positive_increase_count_ % int(step_to_converge_/k_d3_)==0)
                {   
                    human_induced_goal_offset_+=0.1;
                    positive_increase_count_=1;
                }
            else {positive_increase_count_++;}
        }
        else{
            if (negtive_decrease_count_ % int(step_to_converge_/k_d3_)==0)
                {
                    // if (-0.8<human_induced_goal_offset_<=0){
                    //     human_induced_goal_offset_=-0.8;
                    // }
                    if (human_induced_goal_offset_>-1.2){
                        human_induced_goal_offset_-=0.2;
                    } 
                    negtive_decrease_count_=1;
                }
            else {negtive_decrease_count_++;}
        }
    }
    else{
        safe_flag_ = 1;
        positive_increase_count_=0;
        negtive_decrease_count_=0;
        if (human_induced_goal_offset_>0.001){
            if (positive_decrease_count_ % int(step_to_converge_/k_d3_)==0)
                {
                    human_induced_goal_offset_-=0.1;
                    positive_decrease_count_=1;
                }
            else {positive_decrease_count_++;}
        }
        if (human_induced_goal_offset_<-0.001){
            // ROS_INFO_THROTTLE(0.01,"negative_increase_count_:%d",negative_increase_count_);
            if (negative_increase_count_ % int(step_to_converge_/k_d3_)==0)
                {   
                    human_induced_goal_offset_+=0.1;
                    negative_increase_count_=1;
                }
            else {negative_increase_count_++;}
        }
    }
    ROS_INFO_THROTTLE(1,"human_induced_goal_offset_:%f",human_induced_goal_offset_);
    x3_d = initial_q_(0)+human_induced_goal_offset_+joint1_vel_target_;

    if (human_induced_goal_offset_<=0.1 and human_induced_goal_offset_>=-0.1) ns_state_ = 0;

    return x3_d;
}

Eigen::Matrix<double, 6, 3> EditWholeTaskController::WaitingModeTrajectoryAndStiffnessGenerator(double time, double period) {
    // waiting
    ROS_WARN_THROTTLE(1, "Now we are in waiting mode!!");
    state_ = 0;

    /* update incrementally */
    double dk_d1 = 4 * (1 / K_d_changing_ratio_) * std::pow(((1 - a_h_filtered_) - 0.5), 2) * k_d1_target_ * (1 - 2 * a_f_filtered_) * period;
    double dk_d2 = 2 * (1 / K_d_changing_ratio_) * std::pow(((1 - a_h_filtered_) - 0.5), 2) * k_d2_target_ * (1 - 2 * a_f_filtered_) * period;
    k_d1_ = std::max(0.0, std::min((k_d1_ + dk_d1 * (1 - a_h_filtered_)), k_d1_target_));  // saturate
    k_d2_ = std::max(0.5 * k_d2_target_, std::min((k_d2_ + dk_d2 * (1 - a_h_filtered_)), k_d2_target_));  // saturate

    // set return value
    Eigen::Matrix<double, 6, 3> desired_motion_profile;
    desired_motion_profile.block<3, 1>(0, 0) = equilibrium_point.affine.translation();
    desired_motion_profile.block<3, 1>(0, 1).setZero();
    desired_motion_profile.block<3, 1>(0, 2).setZero();
    desired_motion_profile.block<3, 1>(3, 0) = equilibrium_point.affine.linear().eulerAngles(2, 1, 0);
    desired_motion_profile.block<3, 1>(3, 1).setZero();
    desired_motion_profile.block<3, 1>(3, 2).setZero();
    // q_d_nullspace_ = equilibrium_point.q;
    // x3_d = equilibrium_point.q[0];

    // send tf of desired trajectory point to rviz
    Eigen::Quaterniond desired_orientation(equilibrium_point.affine.linear());
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "panda_link0";
    transformStamped.child_frame_id = "desired_point";

    transformStamped.transform.translation.x = desired_motion_profile(0,0);
    transformStamped.transform.translation.y = desired_motion_profile(1,0);
    transformStamped.transform.translation.z = desired_motion_profile(2,0);
    transformStamped.transform.rotation.x = desired_orientation.x();
    transformStamped.transform.rotation.y = desired_orientation.y();
    transformStamped.transform.rotation.z = desired_orientation.z();
    transformStamped.transform.rotation.w = desired_orientation.w();

    br.sendTransform(transformStamped);

    return desired_motion_profile;
}

Eigen::Matrix<double, 6, 3> EditWholeTaskController::GuidingModeTrajectoryAndStiffnessGenerator(double time, double period) {
    // completely grasp
    ROS_WARN_THROTTLE(1, "The probe was GRASPED by someone! Now we are in guiding mode!!");
    state_ = 1;
    end_effector_being_grabbed_ = true;
    // perception_info.heightmap.clear();

    // update equilibrium pointcurrent_recovery_start_translation
    equilibrium_point = current_point;
    
    // update stiffness
    int sign_a_h = (da_h_accumulate_ > 0) - (da_h_accumulate_< 0);
    // ROS_INFO_STREAM_THROTTLE(1, "sign_a_h   " << sign_a_h);
    if (sign_a_h > 0) {
        k_d1_ = ((1 - a_h_filtered_)*(1 - a_f_filtered_)) * k_d1_target_;
        k_d2_ = ((1 - a_h_filtered_)*(1 - a_f_filtered_)) * k_d2_target_;
        k_d3_ = ((1 - a_h_filtered_)*(1 - a_f_filtered_)) * k_d3_target_;
        k_d1_ = std::max(0.0, std::min(k_d1_, k_d1_target_));  // saturate
        k_d2_ = std::max(0.0, std::min(k_d2_, k_d2_target_));  // saturate
        k_d3_ = std::max(0.0, std::min(k_d3_, k_d3_target_));  // saturate
    }
    else {
        double dk_d1 = 4 * (1 / K_d_changing_ratio_) * std::pow((1 - a_h_filtered_) - 0.5, 2) * k_d1_target_ * 1.0 * period;
        double dk_d2 = 4 * (1 / K_d_changing_ratio_) * std::pow((1 - a_h_filtered_) - 0.5, 2) * k_d2_target_ * 1.0 * period;
        double dK_nullspace = 4 * (1 / K_d_changing_ratio_) * std::pow((1 - a_h_filtered_) - 0.5, 2) * k_d3_target_ * 1.0 * period;
        k_d1_ = std::max(0.0, std::min((k_d1_ + dk_d1 * (1 - a_h_filtered_)), k_d1_target_));  // saturate
        k_d2_ = std::max(0.0, std::min((k_d2_ + dk_d2 * (1 - a_h_filtered_)), k_d2_target_));  // saturate
        k_d3_ = std::max(0.0, std::min((k_d3_ + dK_nullspace * (1 - a_h_filtered_)), k_d3_target_));  // saturate
    }

    // set return value
    Eigen::Matrix<double, 6, 3> desired_motion_profile;
    desired_motion_profile.block<3, 1>(0, 0) = equilibrium_point.affine.translation();
    desired_motion_profile.block<3, 1>(0, 1).setZero();
    desired_motion_profile.block<3, 1>(0, 2).setZero();
    desired_motion_profile.block<3, 1>(3, 0) = equilibrium_point.affine.linear().eulerAngles(2, 1, 0);
    desired_motion_profile.block<3, 1>(3, 1).setZero();
    desired_motion_profile.block<3, 1>(3, 2).setZero();
    // q_d_nullspace_ = current_point.q;
    x3_d = equilibrium_point.q[0];

    // send tf of desired trajectory point to rviz
    Eigen::Quaterniond desired_orientation(equilibrium_point.affine.linear());
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "panda_link0";
    transformStamped.child_frame_id = "desired_point";

    transformStamped.transform.translation.x = desired_motion_profile(0,0);
    transformStamped.transform.translation.y = desired_motion_profile(1,0);
    transformStamped.transform.translation.z = desired_motion_profile(2,0);
    transformStamped.transform.rotation.x = desired_orientation.x();
    transformStamped.transform.rotation.y = desired_orientation.y();
    transformStamped.transform.rotation.z = desired_orientation.z();
    transformStamped.transform.rotation.w = desired_orientation.w();

    br.sendTransform(transformStamped);

    return desired_motion_profile;
}


void EditWholeTaskController::BroadcastTransform(const Eigen::Vector3d& translation, const Eigen::Quaterniond& orientation, const std::string& parent_frame, const std::string& child_frame) {
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = parent_frame;
    transformStamped.child_frame_id = child_frame;

    transformStamped.transform.translation.x = translation(0);
    transformStamped.transform.translation.y = translation(1);
    transformStamped.transform.translation.z = translation(2);
    transformStamped.transform.rotation.x = orientation.x();
    transformStamped.transform.rotation.y = orientation.y();
    transformStamped.transform.rotation.z = orientation.z();
    transformStamped.transform.rotation.w = orientation.w();

    br.sendTransform(transformStamped);
}

Eigen::Matrix<double, 6, 3> EditWholeTaskController::RecoveryModeTrajectoryGenerator(double period) {
    if(!received_trajectory_) {

        ROS_WARN_THROTTLE(1,"Not Receiving Trajectory!");
        return WaitingModeTrajectoryAndStiffnessGenerator(0,period);//用不上
    }
    Eigen::Matrix<double, 6, 3> desired_motion_profile;
    state_ = 2;
    ROS_WARN_THROTTLE(1,"now we are in RECOVERY mode!!");

    double dK_d1 = 4 * (1 / K_d_changing_ratio_) * std::pow(((1 - a_h_filtered_) - 0.5), 2) * k_d1_target_ * (1 - 2 * a_f_filtered_) * period;
    double dK_d2 = 2 * (1 / K_d_changing_ratio_) * std::pow(((1 - a_h_filtered_) - 0.5), 2) * k_d2_target_ * (1 - 2 * a_f_filtered_) * period;
    double dK_nullspace = 4 * (1 / K_d_changing_ratio_) * std::pow(((1 - a_h_filtered_) - 0.5), 2) * k_d3_target_ * (1 - 2 * a_f_filtered_) * period;

    k_d1_ = std::max(0.0, std::min((k_d1_ + dK_d1 * (1 - a_h_filtered_)), k_d1_target_));  // saturate
    k_d2_ = std::max(0.5 * k_d2_target_, std::min((k_d2_ + dK_d2 * (1 - a_h_filtered_)), k_d2_target_));  // saturate
    // k_d3_ = std::max(0.3 * k_d3_target_, std::min((k_d3_ + dK_nullspace) * (1 - a_h_filtered_), k_d3_target_));  // saturate

    Eigen::Vector3d current_translation(current_point.affine.translation());
    Eigen::Quaterniond current_orientation(current_point.affine.linear());

    double t_, slerp_t_, dslerp_t_;
    if (last_recovery_execution_time == 0.0 || (elapsed_time_.toSec() - last_recovery_execution_time) > 1e-2) {
        current_recovery_start_time = elapsed_time_.toSec();
        current_recovery_start_translation = current_point.affine.translation();
        current_recovery_start_orientation = current_point.affine.linear();
        current_recovery_start_q = current_point.q;
    }
    // t_ = std::min(std::max((elapsed_time_.toSec() - current_recovery_start_time) / recovery_last_time, 0.0), 1.0);
    t_ = std::min(std::max((elapsed_time_.toSec() - current_recovery_start_time) / trajectory_duration_, 0.0), 1.0);
    recovery_t_ = t_;
    slerp_t_ = 10 * std::pow(t_, 3) - 15 * std::pow(t_, 4) + 6 * std::pow(t_, 5);
    dslerp_t_ = 30 * std::pow(t_, 2) - 60 * std::pow(t_, 3) + 30 * std::pow(t_, 4);
    // Eigen::Vector3d start_translation(current_recovery_start_translation);
    // Eigen::Quaterniond start_orientation(current_recovery_start_orientation);
    // Eigen::Matrix<double, 7, 1> start_q(current_recovery_start_q);
    // Eigen::Matrix<double, 7, 1> desired_q(q_d_nullspace_fixed);

    // position and velocity
    // std::array<double, 7> position;
    // std::vector<double> velocity(7, 0.0);
    size_t n = trajectory_.size() - 1;
    size_t segment = std::min(static_cast<size_t>(slerp_t_ * n), n - 1);
    double local_t = (slerp_t_ * n) - segment;

    // ROS_INFO("segent:%ld, local_t:%.4f, ", segment, local_t);


    std::vector<double> start_jp(7, 0.0);
    std::vector<double> next_jp(7, 0.0);
    // Eigen::VectorXd position(model.nq);
    std::vector<double> position(7, 0.0);
    // Eigen::VectorXd start_jp(model.nq);
    // Eigen::VectorXd next_jp(model.nq);

    for (size_t i = 0; i < 7; ++i) {
        double p0 = trajectory_[segment][i];
        double p1 = trajectory_[segment + 1][i];
        position[i] = p0 + (p1 - p0) * local_t;
        // velocity[i] = (p1 - p0) / time_step_ * dslerp_t_;
        
        start_jp[i] = p0;
        next_jp[i] = p1;
        
    }
    // ROS_INFO_THROTTLE(0.2,"%.4f %.4f %.4f %.4f %.4f %.4f %.4f ", position[0], position[1], position[2], position[3], position[4], position[5], position[6] );

    // std::array<double, 16> pose_matrix = model_handle_->getPose(franka::Frame::kEndEffector, position);
    // std::array<double, 16> start_pose_matrix = model_handle_->getPose(franka::Frame::kEndEffector, start_jp);
    // std::array<double, 16> next_pose_matrix = model_handle_->getPose(franka::Frame::kEndEffector, next_jp);

    
     // Compute forward kinematics for current position, start, and next joint positions-------------
    // pinocchio::forwardKinematics(reduced_model2, data, position);
    // updateFramePlacements(reduced_model2,data);
    // Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
    // pose_matrix.block<3, 3>(0, 0) = data_.oMf[model.njoints - 1].rotation();
    // pose_matrix.block<3, 1>(0, 3) = data_.oMf[model.njoints - 1].translation();
    
    // pinocchio::forwardKinematics(reduced_model2, data, start_jp);
    // updateFramePlacements(reduced_model2,data);
    // Eigen::Matrix4d start_pose_matrix = Eigen::Matrix4d::Identity();
    // start_pose_matrix.block<3, 3>(0, 0) = data_.oMf[model.njoints - 1].rotation();
    // start_pose_matrix.block<3, 1>(0, 3) = data_.oMf[model.njoints - 1].translation();
    
    // pinocchio::forwardKinematics(reduced_model2, data, next_jp);
    // updateFramePlacements(reduced_model2,data);
    // Eigen::Matrix4d next_pose_matrix = Eigen::Matrix4d::Identity();
    // next_pose_matrix.block<3, 3>(0, 0) = data.oMf[model.njoints - 1].rotation();
    // next_pose_matrix.block<3, 1>(0, 3) = data.oMf[model.njoints - 1].translation();
    //---------------------------------------------------

    // Convert Eigen matrices to std::array
    // std::array<double, 16> pose_array = eigenMatrixToArray(pose_matrix);
    // std::array<double, 16> start_pose_array = eigenMatrixToArray(start_pose_matrix);
    // std::array<double, 16> next_pose_array = eigenMatrixToArray(next_pose_matrix);

    
    // // Eigen::Matrix4d pose;

    // Eigen::Matrix4d start_pose, next_pose;

    // for (size_t i = 0; i < 16; ++i) {
    // //   pose(i / 4, i % 4) = pose_matrix[i];
    //   start_pose(i / 4, i % 4) = start_pose_matrix[i];
    //   next_pose(i / 4, i % 4) = next_pose_matrix[i];
    // }

    // Eigen::Vector3d translation = pose.block<3, 1>(0, 3);
    // Eigen::Matrix3d rotation = pose.block<3, 3>(0, 0);
    // Eigen::Quaterniond orientation(rotation);

    Eigen::Matrix4d start_pose_matrix = getFkSolution(start_jp);
    Eigen::Matrix4d next_pose_matrix = getFkSolution(next_jp);

    Eigen::Vector3d start_translation = start_pose_matrix.block<3, 1>(0, 3);
    Eigen::Vector3d desired_translation = next_pose_matrix.block<3, 1>(0, 3);

    Eigen::Matrix3d start_rotation = start_pose_matrix.block<3, 3>(0, 0);
    Eigen::Matrix3d desired_rotation = next_pose_matrix.block<3, 3>(0, 0);
    Eigen::Quaterniond start_orientation(start_rotation);
    Eigen::Quaterniond desired_orientation(desired_rotation);


    // Eigen::Vector3d desired_minjerk_translation = start_translation + (desired_translation - start_translation) * slerp_t_;
    Eigen::Vector3d desired_minjerk_translation = start_translation + (desired_translation - start_translation) * local_t;

    // Eigen::Vector3d desired_minjerk_velocity = (desired_translation - start_translation) / recovery_last_time * dslerp_t_;
    Eigen::Vector3d desired_minjerk_velocity = (desired_translation - start_translation) / (trajectory_duration_ / n) * dslerp_t_;

    // Eigen::Quaterniond desired_minjerk_orientation = start_orientation.slerp(slerp_t_, desired_orientation);
    Eigen::Quaterniond desired_minjerk_orientation = start_orientation.slerp(local_t, desired_orientation);
    
    // Eigen::Matrix<double, 7, 1> desired_minjerk_nullspace_q = start_q + (desired_q - start_q) * slerp_t_;
    // Eigen::Matrix<double, 7, 1> desired_minjerk_nullspace_q = start_q + (desired_q - start_q) * local_t;

    Eigen::Matrix<double, 7, 1> desired_minjerk_nullspace_q = Eigen::Map<Eigen::Matrix<double, 7, 1>>(position.data());

    // i choose to make x3_d remain its original value. it is operated in NSAvoiding
    q_d_nullspace_ = desired_minjerk_nullspace_q;
    x3_d = desired_minjerk_nullspace_q[0];

    // ROS_INFO_STREAM_THROTTLE(0.5, "slerp_t_: " << slerp_t_);

    // send tf of desired trajectory point to rviz


    BroadcastTransform(desired_minjerk_translation, desired_minjerk_orientation, "panda_link0", "desired_point");
    BroadcastTransform(start_translation, start_orientation, "panda_link0", "start_point");
    BroadcastTransform(desired_translation, desired_orientation, "panda_link0", "next_point");
    

    // desired_motion_profile.block<3, 1>(0, 0) = current_translation;
    // desired_motion_profile.block<3, 1>(0, 1) = desired_velocity;
    desired_motion_profile.block<3, 1>(0, 0) = desired_minjerk_translation;
    desired_motion_profile.block<3, 1>(0, 1) = desired_minjerk_velocity;
    desired_motion_profile.block<3, 1>(0, 2).setZero();
    desired_motion_profile.block<3, 1>(3, 0) = desired_minjerk_orientation.toRotationMatrix().eulerAngles(2, 1, 0);
    desired_motion_profile.block<3, 1>(3, 1).setZero();
    desired_motion_profile.block<3, 1>(3, 2).setZero();

    last_recovery_execution_time = elapsed_time_.toSec();

    return desired_motion_profile;
}

Eigen::Matrix<double, 6, 3> EditWholeTaskController::ScanningModeTrajectoryGenerator(double time, double period, Eigen::Vector3d desired_translation,Eigen::Quaterniond desired_orientation) {
    Eigen::Matrix<double, 6, 3> desired_motion_profile;
    state_ = 3;
    ROS_WARN_THROTTLE(1,"now we are in SCANNING mode!!");

    double dK_d1 = 4 * (1 / K_d_changing_ratio_) * std::pow((1 - a_h_filtered_) - 0.5, 2) * k_d1_target_ * (1 - 2 * a_f_filtered_) * period;
    double dK_d2 = 2 * (1 / K_d_changing_ratio_) * std::pow((1 - a_h_filtered_) - 0.5, 2) * k_d2_target_ * (1 - 2 * a_f_filtered_) * period;
    double dK_nullspace = 4 * (1 / K_d_changing_ratio_) * std::pow((1 - a_h_filtered_) - 0.5, 2) * k_d3_target_ * (1 - 2 * a_f_filtered_) * period;

    k_d1_ = std::max(0.0, std::min((k_d1_ + dK_d1 * (1 - a_h_filtered_)), k_d1_target_));  // saturate
    k_d2_ = std::max(0.5 * k_d2_target_, std::min((k_d2_ + dK_d2 * (1 - a_h_filtered_)), k_d2_target_));  // saturate
    // k_d3_ = std::max(0.3 * k_d3_target_, std::min((k_d3_ + dK_nullspace) * (1 - a_h_filtered_), k_d3_target_));  // saturate

    Eigen::Vector3d current_translation(current_point.affine.translation());
    Eigen::Quaterniond current_orientation(current_point.affine.linear());


    desired_translation = (1 - (1 - a_h_filtered_)) * current_point.affine.translation() + ((1 - a_h_filtered_)) * desired_translation;
    desired_orientation = current_orientation.slerp((1 - a_h_filtered_), desired_orientation);

    // ROS_INFO_STREAM_THROTTLE(1,"2===="<<desired_orientation.coeffs());
    Eigen::Vector3d desired_velocity = (desired_translation - perception_info.last_desired_translation) / period;
    // std::cout<<desired_translation<<std::endl<<perception_info.last_desired_translation<<std::endl<<period;
    Eigen::Vector3d desired_acceleration = (desired_velocity - perception_info.last_desired_velocity) / period;
    perception_info.last_desired_translation = desired_translation;
    perception_info.last_desired_velocity = desired_velocity;
    desired_motion_profile.block<3, 1>(0, 0) = desired_translation;
    desired_motion_profile.block<3, 1>(0, 1) = desired_velocity;
    desired_motion_profile.block<3, 1>(0, 2) = desired_acceleration;
    desired_motion_profile.block<3, 1>(3, 0) = desired_orientation.toRotationMatrix().eulerAngles(2, 1, 0);
    desired_motion_profile.block<3, 1>(3, 1).setZero();
    desired_motion_profile.block<3, 1>(3, 2).setZero();

    // ROS_INFO_STREAM_THROTTLE(0.2,"desired_velocity   "<<desired_velocity);
    // ROS_INFO_STREAM_THROTTLE(0.2,"desired_acceleration   "<<desired_acceleration);

    // send tf of desired trajectory point to rviz
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "panda_link0";
    transformStamped.child_frame_id = "desired_point";

    // ROS_INFO_STREAM_THROTTLE(1,"3===="<<desired_orientation.coeffs());

    transformStamped.transform.translation.x = desired_translation[0];
    transformStamped.transform.translation.y = desired_translation[1];
    transformStamped.transform.translation.z = desired_translation[2];
    transformStamped.transform.rotation.x = desired_orientation.x();
    transformStamped.transform.rotation.y = desired_orientation.y();
    transformStamped.transform.rotation.z = desired_orientation.z();
    transformStamped.transform.rotation.w = desired_orientation.w();

    br.sendTransform(transformStamped);

    progress_time_+=period;

    return desired_motion_profile;
}

void EditWholeTaskController::CallForService() {
    franka_us_controllers::heightmap srv;
    if (heightmap_client_.call(srv)) {
        if (!perception_info.heightmap.empty()) perception_info.heightmap.clear();
        for (auto pose : srv.response.heightmap.poses) {
            Eigen::Vector3d map_point;
            map_point(0) = pose.position.x;
            map_point(1) = pose.position.y;
            map_point(2) = 0;
            assert(pose.position.z == 0);
            perception_info.heightmap.push_back(map_point);
        }
        perception_info.x_max = perception_info.heightmap.back()(0);
        ROS_INFO("Successfully called service from perception! Got new heightmap!");
    }
    else {
        ROS_ERROR("Failure to call service from perception!");
    }
}

void EditWholeTaskController::parseConfigurationFromFile() {
    // open the configuration file
    std::cout << "Open the configuration file!" << std::endl;
    std::ifstream file_configuration;
    file_configuration.open(config_file_path);
    std::string line;

    // read q_d_nullspace
    Eigen::Matrix<double, 7, 1> q_d_nullspace;
    getline(file_configuration, line);
    if (file_configuration.is_open()) {
        file_configuration >> q_d_nullspace[0] >> q_d_nullspace[1] >> q_d_nullspace[2] >> 
                                q_d_nullspace[3] >> q_d_nullspace[4] >> q_d_nullspace[5] >>
                                q_d_nullspace[6];
    }
    q_d_nullspace_ = q_d_nullspace;

    getline(file_configuration, line);

    // read desired orientation as rotation matrix
    Eigen::Matrix3d rotation_matrix;
    Eigen::Vector3d matrix_line1;
    Eigen::Vector3d matrix_line2;
    Eigen::Vector3d matrix_line3;
    getline(file_configuration, line);
    if (file_configuration.is_open()) {
        file_configuration >> matrix_line1[0] >> matrix_line1[1] >> matrix_line1[2] >>
                                matrix_line2[0] >> matrix_line2[1] >> matrix_line2[2] >>
                                matrix_line3[0] >> matrix_line3[1] >> matrix_line3[2];
    }
    rotation_matrix.block<1, 3>(0, 0) = matrix_line1;
    rotation_matrix.block<1, 3>(1, 0) = matrix_line2;
    rotation_matrix.block<1, 3>(2, 0) = matrix_line3;
    orientation_d_target_ = Eigen::Quaterniond(rotation_matrix);

    getline(file_configuration, line);

    // read start point of the translation trajectory
    Eigen::Vector3d x_d_start_point;
    getline(file_configuration, line);
    file_configuration >> x_d_start_point[0] >> x_d_start_point[1] >> x_d_start_point[2];
    x_d_start_point_ = x_d_start_point;

    getline(file_configuration, line);

    // read end point of the translation trajectory
    Eigen::Vector3d x_d_end_point;
    getline(file_configuration, line);
    file_configuration >> x_d_end_point[0] >> x_d_end_point[1] >> x_d_end_point[2];
    x_d_end_point_ = x_d_end_point;

    getline(file_configuration, line);

    // read the desired orientation axis
    // the end effector will change its orientation with respect to the specific axis
    Eigen::Vector3d rotation_axis_d;
    getline(file_configuration, line);
    file_configuration >> rotation_axis_d[0] >> rotation_axis_d[1] >> rotation_axis_d[2];
    rotation_axis_d_ = rotation_axis_d;

    getline(file_configuration, line);

    // task periods
    getline(file_configuration, line);
    file_configuration >> scan_task_period;

    getline(file_configuration, line);

    // k_d_changing_ratio
    getline(file_configuration, line);
    file_configuration >> K_d_changing_ratio_;

    getline(file_configuration, line);

    // da_h_accumulate_max_volume_
    getline(file_configuration, line);
    file_configuration >> da_h_accumulate_max_volume_;

    getline(file_configuration, line);

    // rotation_probe2neck
    getline(file_configuration, line);
    if (file_configuration.is_open()) {
        file_configuration >> matrix_line1[0] >> matrix_line1[1] >> matrix_line1[2] >>
                                matrix_line2[0] >> matrix_line2[1] >> matrix_line2[2] >>
                                matrix_line3[0] >> matrix_line3[1] >> matrix_line3[2];
    }
    rotation_probe2neck.block<1, 3>(0, 0) = matrix_line1;
    rotation_probe2neck.block<1, 3>(1, 0) = matrix_line2;
    rotation_probe2neck.block<1, 3>(2, 0) = matrix_line3;

    getline(file_configuration, line);

    // recovery last time
    getline(file_configuration, line);
    file_configuration >> recovery_last_time;

    getline(file_configuration, line);

    // scanning trajectory offset
    getline(file_configuration, line);
    file_configuration >> scanning_trajectory_offset[0] >> scanning_trajectory_offset[1] >> scanning_trajectory_offset[2];

    getline(file_configuration, line);

    // a_f_weightfunc_center
    getline(file_configuration, line);
    file_configuration >> a_f_weightfunc_center;

    getline(file_configuration, line);

    file_configuration.close();
}

void EditWholeTaskController::PerceptionInfoCallback(
    const us_scene_info& msg) {//calculate a_h_filtered_ and a_p_filtered_
    std::lock_guard<std::mutex> perception_info_mutex_lock(
        scene_info_mutex_);
    scene_info.header = msg.header;
    scene_info.nurse_exists = msg.nurse_exists;
    scene_info.nurse_id = msg.nurse_id;
    scene_info.patient_exists = msg.patient_exists;
    scene_info.patient_id = msg.patient_id;
    scene_info.probe_touched = msg.probe_touched;
    scene_info.d_h = msg.d_h;
    scene_info.a_h = msg.a_h;
    scene_info.d_p = msg.d_p;
    scene_info.a_p = msg.a_p;
    scene_info.p_j4 = msg.p_j4;
    double p_j4_norm = sqrt(scene_info.p_j4.x * scene_info.p_j4.x +
                       scene_info.p_j4.y * scene_info.p_j4.y +
                       scene_info.p_j4.z * scene_info.p_j4.z);
    scene_info.robot_body_touched = msg.robot_body_touched;
    double last_a_h_filtered_ = a_h_filtered_;
    double last_a_p_filtered_ = a_p_filtered_;
    // if (elapsed_time_.toSec()>1 && elapsed_time_.toSec()<4){
    //     if (msg.a_h<a_h_filtered_)
    //     scene_info.a_h = a_h_filtered_;
    // }
    a_h_filtered_ =  30 * filter_params_ * msg.a_h + (1 - 30 * filter_params_) * a_h_filtered_;
    a_p_filtered_ =  30 * filter_params_ * msg.a_p + (1 - 30 * filter_params_) * a_p_filtered_;
    da_h_accumulate_ += (a_h_filtered_ - last_a_h_filtered_);
    da_h_accumulate_ = std::max(-da_h_accumulate_max_volume_, std::min(da_h_accumulate_, da_h_accumulate_max_volume_));
    da_p_accumulate += (a_p_filtered_ - last_a_p_filtered_);
    da_p_accumulate = std::max(-da_p_accumulate_max_volume_, std::min(da_p_accumulate, da_p_accumulate_max_volume_));
    /* log for debug
    ROS_INFO("Succesfully received /us_scene_info!");
    */
    ROS_INFO_STREAM_THROTTLE(1,"a_h: "<<msg.a_h<<"   a_p: "<<msg.a_p<<"  a_f_raw: " << a_f_raw_ <<"   a_f: "<<a_f_filtered_<<"   p_j4: "<< msg.p_j4.x<<" "<<msg.p_j4.y<<" "<<msg.p_j4.z<<" norm:"<<p_j4_norm);
}

ProcessStatus EditWholeTaskController::lookUpTransformNeck2Base() {
    // get neck tf transform
    geometry_msgs::TransformStamped transformStamped;
    try{
        if (!tfBuffer.canTransform("panda_link0", "neck_scan_base",ros::Time(0))) return ProcessStatus::FAILURE;
        transformStamped = tfBuffer.lookupTransform("panda_link0","neck_scan_base",ros::Time(0));
        // get desired position and orientation
        transform_neck2base = tf2::transformToEigen(transformStamped);
        /* log for debug
        ROS_INFO("Succesfully get transform from human heck to base frame!");
        std::cout << "transform_neck2base translation: " << transform_neck2base.translation() << std::endl;
        std::cout << "transform_neck2base orientation: " << transform_neck2base.linear() << std::endl;
        */
        return ProcessStatus::SUCCESS;
    }
    catch (tf2::TransformException &ex){
        ROS_WARN("%s",ex.what());
        return ProcessStatus::FAILURE;
    }
}

void EditWholeTaskController::initialize_pinocchio() {

  // std::cout << "urdf_filename: " << urdf_filename << std::endl;
  // Load the urdf model
  pinocchio::urdf::buildModel(urdf_filename,model);
  //   std::cout << "\n\nSECOND CASE: BUILD A REDUCED MODEL FROM A LIST OF JOINT TO KEEP UNLOCKED" << std::endl;
  // The same thing, but this time with an input list of joint to keep

  std::vector<std::string> list_of_joints_to_keep_unlocked_by_name;
  list_of_joints_to_keep_unlocked_by_name.push_back("panda_joint1");
  list_of_joints_to_keep_unlocked_by_name.push_back("panda_joint2");
  list_of_joints_to_keep_unlocked_by_name.push_back("panda_joint3");
  list_of_joints_to_keep_unlocked_by_name.push_back("panda_joint4");
  list_of_joints_to_keep_unlocked_by_name.push_back("panda_joint5");
  list_of_joints_to_keep_unlocked_by_name.push_back("panda_joint6");
  list_of_joints_to_keep_unlocked_by_name.push_back("panda_joint7");

  std::vector<pinocchio::JointIndex> list_of_joints_to_keep_unlocked_by_id;
  for(std::vector<std::string>::const_iterator it = list_of_joints_to_keep_unlocked_by_name.begin();
      it != list_of_joints_to_keep_unlocked_by_name.end(); ++it)
  {
    const std::string & joint_name = *it;
    if(model.existJointName(joint_name))
      list_of_joints_to_keep_unlocked_by_id.push_back(model.getJointId(joint_name));
    else
      std::cout << "joint: " << joint_name << " does not belong to the model";
  }

  // Transform the list into a list of joints to lock
  std::vector<pinocchio::JointIndex> list_of_joints_to_lock_by_id;
  int size = model.joints.size();
  std::cout<<size<<std::endl;
  for(pinocchio::JointIndex joint_id = 1; joint_id < model.joints.size(); ++joint_id)
  {
    const std::string joint_name = model.names[joint_id];
    if(is_in_vector(list_of_joints_to_keep_unlocked_by_name,joint_name))
      continue;
    else
    {
      list_of_joints_to_lock_by_id.push_back(joint_id);
    }
  }
  Eigen::VectorXd q_rand = randomConfiguration(model);
  
  // Build the reduced model from the list of lock joints
  reduced_model2 = buildReducedModel(model,list_of_joints_to_lock_by_id,q_rand);
  
  // Print the list of joints in the second reduced model
  std::cout << "List of joints in the second reduced model:" << std::endl;
  for(pinocchio::JointIndex joint_id = 1; joint_id < reduced_model2.joints.size(); ++joint_id)
    std::cout << "\t- " << reduced_model2.names[joint_id] << std::endl;

  // Create data required by the algorithms
  data=pinocchio::Data(reduced_model2);
  data_=pinocchio::Data(model);
}

//yxj 0603
template <class T>
void EditWholeTaskController::saveData(T &Data, std::string filePath){
  //format: 4 bytes vector number + 4 bytes totalsize + data
	std::ofstream ofile(filePath.c_str(), std::ios::binary);
	if(ofile.is_open()==false){
		std::cout<<"Open file fail!"<<std::endl;
		exit(1);
	}
	int length = Data.size();
	ofile.write((char*)&length, sizeof(int)); 
	
	int totalSize = Data.size()*sizeof(Data[0]);
	ofile.write((char*)&totalSize, sizeof(int));
	
	ofile.write((char*)&Data[0], totalSize);
	
	ofile.close();
} 

void EditWholeTaskController::ati_reader_callback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  l8_ati_wrench_(0) = msg->wrench.force.x;
  l8_ati_wrench_(1) = msg->wrench.force.y;
  l8_ati_wrench_(2) = msg->wrench.force.z;
  l8_ati_wrench_(3) = msg->wrench.torque.x;
  l8_ati_wrench_(4) = msg->wrench.torque.y;
  l8_ati_wrench_(5) = msg->wrench.torque.z;

  // adj
  Eigen::MatrixXd adj = Eigen::MatrixXd::Identity(6, 6);
  Eigen::MatrixXd l8_ati_R = Eigen::MatrixXd::Identity(3, 3);
  l8_ati_R<<0,1,0,-1,0,0,0,0,1;
  adj.block<3,3>(0,0)=l8_ati_R;
  adj.block<3,3>(3,3)=l8_ati_R;

  l8_ati_wrench_ = adj*l8_ati_wrench_;

    if (force_sensor_test_pub_.trylock()) {
        force_sensor_test_pub_.msg_.header.stamp = ros::Time::now();
        force_sensor_test_pub_.msg_.wrench.force.x = l8_ati_wrench_(0);
        force_sensor_test_pub_.msg_.wrench.force.y = l8_ati_wrench_(0);
        force_sensor_test_pub_.msg_.wrench.force.z = l8_ati_wrench_(0);
        force_sensor_test_pub_.msg_.wrench.torque.x = l8_ati_wrench_(0);
        force_sensor_test_pub_.msg_.wrench.torque.y = l8_ati_wrench_(0);
        force_sensor_test_pub_.msg_.wrench.torque.z = l8_ati_wrench_(0);
        force_sensor_test_pub_.unlockAndPublish();
    }

}
//=========================0806===========================
// std::vector<double> EditWholeTaskController::interpolateTrajectory(const std::vector<std::vector<double>>& trajectory, double t) {
//   size_t n = trajectory.size() - 1;
//   size_t segment = std::min(static_cast<size_t>(t * n), n - 1);
//   double local_t = (t * n) - segment;

//   std::vector<double> interpolated_point(7, 0.0);
//   for (size_t i = 0; i < 7; ++i) {
//     double p0 = trajectory[segment][i];
//     double p1 = trajectory[segment + 1][i];
//     interpolated_point[i] = p0 + (p1 - p0) * local_t;
//   }
//   return interpolated_point;
// }
void EditWholeTaskController::TrajectoryCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    ROS_INFO_STREAM("Received trajectory");
    std::vector<std::vector<double>> trajectory;
    size_t num_points = msg->data.size() / 7;
    for (size_t i = 0; i < num_points; ++i) {
        std::vector<double> point(msg->data.begin() + i * 7, msg->data.begin() + (i + 1) * 7);
        trajectory.push_back(point);
    }
    buffer_.writeFromNonRT(trajectory);
    trajectory_ = trajectory;  // 保存轨迹
    trajectory_duration_ = num_points * time_step_;  // 假设每个点的时间间隔是 time_step_
    received_trajectory_ = true;
    elapsed_time_ = ros::Duration(0.0);
    ROS_INFO_STREAM("Received trajectory with " << trajectory.size() << " points.");
}

// 计算单个关节的齐次变换矩阵
Eigen::Matrix4d EditWholeTaskController::getTfMat(int i, const std::vector<std::vector<double>>& dh) {
    double a = dh[i][0];
    double d = dh[i][1];
    double alpha = dh[i][2];
    double theta = dh[i][3];
    double q = theta;

    Eigen::Matrix4d T;
    T << std::cos(q), -std::sin(q), 0, a,
         std::sin(q) * std::cos(alpha), std::cos(q) * std::cos(alpha), -std::sin(alpha), -std::sin(alpha) * d,
         std::sin(q) * std::sin(alpha), std::cos(q) * std::sin(alpha), std::cos(alpha), std::cos(alpha) * d,
         0, 0, 0, 1;

    return T;
}

// 计算机械臂的正向运动学
Eigen::Matrix4d EditWholeTaskController::getFkSolution(const std::vector<double>& joint_angles) {
    std::vector<std::vector<double>> dh_params = {
        {0, 0.333, 0, joint_angles[0]},
        {0, 0, -PI / 2, joint_angles[1]},
        {0, 0.316, PI / 2, joint_angles[2]},
        {0.0825, 0, PI / 2, joint_angles[3]},
        {-0.0825, 0.384, -PI / 2, joint_angles[4]},
        {0, 0, PI / 2, joint_angles[5]},
        {0.088, 0, PI / 2, joint_angles[6]},
        {0, 0.107, 0, 0},
        {0, 0, 0, -PI / 4},
        {0.0, 0.1034, 0, 0}
    };

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    for (int i = 0; i < dh_params.size(); ++i) {
        T = T * getTfMat(i, dh_params);
    }

    return T;
}

}  // namespace franka_us_controllers

PLUGINLIB_EXPORT_CLASS(franka_us_controllers::EditWholeTaskController,
                       controller_interface::ControllerBase)
