/* TASK SEQUENCER - Contains all recepies for grasping, and other tasks
Authors: George Jose Pollayil - Mathew Jose Pollayil - Stefano Angeli
Email: gpollayil@gmail.com, mathewjosepollayil@gmail.com, stefano.angeli@ing.unipi.it  */

#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "abb_wrapper_control/TaskSequencer.h"
#include <std_msgs/UInt8.h>

TaskSequencer::TaskSequencer(ros::NodeHandle& nh_){
    
    // Initializing Node Handle
    this->nh = nh_;

    // Parsing the task params
    if(!this->parse_task_params()){
        ROS_ERROR("The parsing of task parameters went wrong. Be careful, using default values...");
    }

    // Initializing Abb Client (TODO: Return error if initialize returns false)
    this->abb_client.initialize(this->nh);

    // Moveit names

	if(!nh_.getParam("/abb/group_name", group_name)){
        ROS_ERROR("Failed to load the move group name!");
    };

    ROS_INFO("The move group name is: %s", group_name.c_str());
 
    //End-effector name use for planning
    
	if(!nh_.getParam("/abb/end_effector_name", end_effector_name)){
        ROS_ERROR("Failed to load the end_effector_name!");
    };

    ROS_INFO("The end-effector name is: %s", end_effector_name.c_str());

    // Initializing other moveit stuff (robot model, kinematic model and state)
    this->robot_model_loader_ptr.reset(new robot_model_loader::RobotModelLoader("robot_description"));
    this->kinematic_model = this->robot_model_loader_ptr->getModel();
    ROS_INFO("Model frame: %s", this->kinematic_model->getModelFrame().c_str());
    this->kinematic_state.reset(new robot_state::RobotState(this->kinematic_model));
    
    // Get the array of active joints
    this->number_of_active_joints = this->kinematic_model->getActiveJointModels();

    // Setting the task service names
    this->example_task_service_name = "example_task_service";
    this->template_task_service_name = "template_task_service";

    // Advertising the services
    this->example_task_server = this->nh.advertiseService(this->example_task_service_name, &TaskSequencer::call_example_task, this);
    this->template_task_server = this->nh.advertiseService(this->template_task_service_name, &TaskSequencer::call_template_task, this);

    // Initializing other control values 
    this->waiting_time = ros::Duration(20.0);
    this->null_joints.resize(this->number_of_active_joints.size());
    std::fill(this->null_joints.begin(), this->null_joints.end(), 0.0);

    //
    this->gripper_close_open.data = true;
    
    // Spinning once
    ros::spinOnce();
}

TaskSequencer::~TaskSequencer(){
    std::cout << "Destructor executed" << std::endl;
    // Nothing to do here yet
}

// Parameters parsing
bool TaskSequencer::parse_task_params(){

    bool success = true;

    if(!ros::param::get("/task_sequencer/grasp_transform", this->grasp_transform)){
		ROS_WARN("The param 'grasp_transform' not found in param server! Using default.");
		this->grasp_transform.resize(6);
        std::fill(this->grasp_transform.begin(), this->grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the grasp_transform vector to geometry_msgs Pose
    this->grasp_T = this->convert_vector_to_pose(this->grasp_transform);

    if(!ros::param::get("/task_sequencer/pre_grasp_transform", this->pre_grasp_transform)){
		ROS_WARN("The param 'pre_grasp_transform' not found in param server! Using default.");
		this->pre_grasp_transform.resize(6);
        std::fill(this->pre_grasp_transform.begin(), this->pre_grasp_transform.end(), 0.0);
		success = false;
	}

    // Converting the pre_grasp_transform vector to geometry_msgs Pose
    this->pre_grasp_T = this->convert_vector_to_pose(this->pre_grasp_transform);
    
    //
    if(!ros::param::get("/task_sequencer/home_joints", this->home_joints)){
		ROS_WARN("The param 'home_joints' not found in param server! Using default.");
		this->home_joints = {0.0,-0.0364, 0.0, 0.5051, 0.0, 1.1522, 0.0};
		success = false;
	}

    if(!ros::param::get("/task_sequencer/joint_position_A", this->joint_pos_A)){
		ROS_WARN("The param 'joint_position_A' not found in param server! Using default.");
		this->joint_pos_A = {0.0,-0.0364, 0.0, 0.5051, 0.0, 1.1522, 0.0};
		success = false;
	}
    return success;
}

bool TaskSequencer::call_example_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    // Checking the request for correctness
    if(!req.data){
        ROS_WARN("Did you really want to call the simple grasp task service with data = false?");
        res.success = true;
        res.message = "The service call_simple_grasp_task done correctly with false request!";
        return true;
    }

    // Eigen::Affine3d grasp_transform_aff; tf::poseMsgToEigen(this->grasp_T, grasp_transform_aff);
    // Eigen::Affine3d pre_grasp_transform_aff; tf::poseMsgToEigen(this->pre_grasp_T, pre_grasp_transform_aff);

    // geometry_msgs::Pose pre_grasp_pose; geometry_msgs::Pose grasp_pose;
    // tf::poseEigenToMsg(grasp_transform_aff * pre_grasp_transform_aff, pre_grasp_pose);
    // tf::poseEigenToMsg(grasp_transform_aff, grasp_pose);

    // // ROS_INFO("GRASP POSE is: %f, %f, %f, %f, %f, %f, %f",grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z, grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w);
    // // ROS_INFO("PRE GRASP POSE is: %f, %f, %f, %f, %f, %f, %f",pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z, pre_grasp_pose.orientation.x, pre_grasp_pose.orientation.y, pre_grasp_pose.orientation.z, pre_grasp_pose.orientation.w);
    
    // // Setting zero pose as starting from present

    // geometry_msgs::Pose present_pose = geometry_msgs::Pose();
    // present_pose.position.x = 0.0; present_pose.position.y = 0.0; present_pose.position.z = 0.0;
    // present_pose.orientation.x = 0.0; present_pose.orientation.y = 0.0; present_pose.orientation.z = 0.0; present_pose.orientation.w = 1.0;

    // /* PLAN 1: Plan to PRE GRASP POSE */

    // if(!this->abb_client.call_pose_service(pre_grasp_pose, present_pose, false, this->tmp_traj_arm, this->tmp_traj_arm)){
    //     ROS_ERROR("Could not plan to the specified pre grasp pose.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }  

    // /* EXEC 1: Going to PRE GRASP POSE*/

    // if(!this->abb_client.call_arm_control_service(this->tmp_traj_arm)){
    //     ROS_ERROR("Could not go to PreGraspPose.");
    //     res.success = false;
    //     res.message = "The service call_arm_control_service was NOT performed correctly! Error in arm control.";
    //     return false;
    // }

    // /* WAIT 1: Wait to finish the previous task(EXEC 1)*/

    // if(!this->abb_client.call_arm_wait_service(this->waiting_time)){ // WAITING FOR END EXEC
    //     ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to Pre Grasp Pose");
    //     res.success = false;
    //     res.message = "The service call_arm_wait_service was NOT performed correctly! Error wait in arm control.";
    //     return false;
    // }

    // /* PLAN 2: Planning to GRASP POSE*/

    // if(!this->abb_client.call_slerp_service(grasp_pose, present_pose, false, this->tmp_traj_arm, this->tmp_traj_arm)){
    //     ROS_ERROR("Could not plan to the specified grasp pose.");
    //     res.success = false;
    //     res.message = "The service call_simple_grasp_task was NOT performed correctly!";
    //     return false;
    // }

    // /* EXEC 2: Going to GRASP POSE*/

    // if(!this->abb_client.call_arm_control_service(this->tmp_traj_arm)){
    //     ROS_ERROR("Could not go to PreGraspPose.");
    //     res.success = false;
    //     res.message = "The service call_arm_control_service was NOT performed correctly! Error in arm control.";
    //     return false;
    // }

    // /* WAIT 2: Wait to finish the Task*/

    // if(!this->abb_client.call_arm_wait_service(this->waiting_time)){ // WAITING FOR END EXEC
    //     ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to Pre Grasp Pose");
    //     res.success = false;
    //     res.message = "The service call_arm_wait_service was NOT performed correctly! Error wait in arm control.";
    //     return false;
    // }

    Eigen::Affine3d grasp_transform_aff; tf::poseMsgToEigen(this->grasp_T, grasp_transform_aff);
    Eigen::Affine3d pre_grasp_transform_aff; tf::poseMsgToEigen(this->pre_grasp_T, pre_grasp_transform_aff);

    geometry_msgs::Pose pre_grasp_pose; geometry_msgs::Pose grasp_pose;
    tf::poseEigenToMsg(grasp_transform_aff * pre_grasp_transform_aff, pre_grasp_pose);
    tf::poseEigenToMsg(grasp_transform_aff, grasp_pose);

    bool success = this->PlanAndExecutePose(grasp_pose, false);

    // Close the gripper
    std_msgs::Bool pippo;
    pippo.data = true;

    if(!this->abb_client.call_closing_gripper(pippo)){ 
        ROS_ERROR("Unable to close the gripper!");
        res.success = false;
        res.message = "The service call_closing_gripper was NOT performed correctly!";
        return false;
    }

    // Now, everything finished well
    res.success = true;
    res.message = "The service call_example_task was correctly performed!";
    return true;   
}

// Callback for simple home task service
bool TaskSequencer::call_template_task(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res){
    
    // Checking the request for correctness

    if(!req.data){
        ROS_WARN("Did you really want to call the simple home task service with data = false?");
        res.success = true;
        res.message = "The service call_simple_home_task done correctly with false request!";
        return true;
    }

    /*PLAN 0: Planning to HOME Joints*/

    if(!this->abb_client.call_joint_service(this->home_joints, true, this->tmp_traj_arm, this->tmp_traj_arm)){
        ROS_ERROR("Could not plan to the specified home joint config.");
        res.success = false;
        res.message = "The service call_simple_home_task was NOT performed correctly!";
        return false;
    }

    /*EXEC 0: Going to HOME JOINTS*/

    if(!this->abb_client.call_arm_control_service(this->tmp_traj_arm)){
        ROS_ERROR("Could not go to HOME joints.");
        res.success = false;
        res.message = "The service call_arm_control_service was NOT performed correctly! Error in arm control.";
        return false;
    } 
    
    /**/

    
    



    /*Compose the high level task by using the joint_plan, slerp_plan, pose_plan, arm_control and arm_wait services*/

    





    /**/
    
    // Now, everything finished well
    res.success = true;
    res.message = "The service call_simple_home_task was correctly performed!";
    return true;
}

bool TaskSequencer::PlanAndExecutePose(geometry_msgs::Pose& pose, bool is_relative){

    std_srvs::SetBool set_bool_srv;
    
    // Setting zero pose as starting from present

    geometry_msgs::Pose present_pose = geometry_msgs::Pose();
    present_pose.position.x = 0.0; present_pose.position.y = 0.0; present_pose.position.z = 0.0;
    present_pose.orientation.x = 0.0; present_pose.orientation.y = 0.0; present_pose.orientation.z = 0.0; present_pose.orientation.w = 1.0;

    /* PLAN 1: Plan to POSE */

    if(!this->abb_client.call_pose_service(pose, present_pose, is_relative, this->tmp_traj_arm, this->tmp_traj_arm)){
        ROS_ERROR("Could not plan to the specified pre grasp pose.");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_pose_service was NOT performed correctly!";
        return false;
    }  

    /* EXEC 1: Going to POSE*/

    if(!this->abb_client.call_arm_control_service(this->tmp_traj_arm)){
        ROS_ERROR("Could not go to PreGraspPose.");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_arm_control_service was NOT performed correctly! Error in arm control.";
        return false;
    }

    /* WAIT 1: Wait to finish the task*/

    if(!this->abb_client.call_arm_wait_service(this->waiting_time)){ // WAITING FOR END EXEC
        ROS_ERROR("TIMEOUT!!! EXEC TOOK TOO MUCH TIME for going to Pre Grasp Pose");
        set_bool_srv.response.success = false;
        set_bool_srv.response.message = "The service call_arm_wait_service was NOT performed correctly! Error wait in arm control.";
        return false;
    }
    return set_bool_srv.response.success = true;
}

// Convert xyzrpy vector to geometry_msgs Pose
geometry_msgs::Pose TaskSequencer::convert_vector_to_pose(std::vector<double> input_vec){
    
    // Creating temporary variables
    geometry_msgs::Pose output_pose;
    Eigen::Affine3d output_affine;

    // Getting translation and rotation
    Eigen::Vector3d translation(input_vec[0], input_vec[1], input_vec[2]);
    output_affine.translation() = translation;
    Eigen::Matrix3d rotation = Eigen::Matrix3d(Eigen::AngleAxisd(input_vec[5], Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(input_vec[4], Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(input_vec[3], Eigen::Vector3d::UnitX()));
    output_affine.linear() = rotation;    
    
    // Converting to geometry_msgs and returning
    tf::poseEigenToMsg(output_affine, output_pose);
    return output_pose;
}

// FK and IK Functions which makes use of MoveIt
geometry_msgs::Pose TaskSequencer::performFK(std::vector<double> joints_in){
    const robot_state::JointModelGroup* joint_model_group = this->kinematic_model->getJointModelGroup(group_name);
    this->kinematic_state->setJointGroupPositions(joint_model_group, joints_in);
    const Eigen::Affine3d& end_effector_eigen = this->kinematic_state->getGlobalLinkTransform(end_effector_name);
    geometry_msgs::Pose end_effector_pose;
    tf::poseEigenToMsg(end_effector_eigen, end_effector_pose);
    return end_effector_pose;
}

bool TaskSequencer::performIK(geometry_msgs::Pose pose_in, double timeout, std::vector<double>& joints_out){
    Eigen::Isometry3d end_effector_state;
    tf::poseMsgToEigen(pose_in, end_effector_state);
    const robot_state::JointModelGroup* joint_model_group = this->kinematic_model->getJointModelGroup(group_name);
    bool found_ik = this->kinematic_state->setFromIK(joint_model_group, end_effector_state, timeout);

    if (!found_ik){
        ROS_ERROR("Could not find IK solution in TaskSequencer...");
        return false;
    }

    const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
    kinematic_state->copyJointGroupPositions(joint_model_group, joints_out);
    this->kinematic_state->copyJointGroupPositions(joint_model_group, joints_out);

    if (DEBUG){
        ROS_INFO("Found an IK solution in TaskSequencer: ");
        for (std::size_t i = 0; i < joint_names.size(); ++i){
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joints_out[i]);
        }
    }

    return true;
}
