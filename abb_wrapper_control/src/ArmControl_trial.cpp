#include "abb_wrapper_control/ArmControl_trial.h"

ArmControlTrial::ArmControlTrial()
    : ac_(std::make_shared<actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>>(
          "/gofa_arm_controller/follow_joint_trajectory")),
          start_(ros::Time::now())
{
    ROS_INFO("Waiting for action server...");
    // ac_->waitForServer();
    // ROS_INFO("Action server found.");
}

ArmControlTrial::~ArmControlTrial(){
    ac_->cancelGoal();
    std::cout << "Destructor executed" << std::endl;
    //Nothing to do here
}

void ArmControlTrial::sendTrajectory(const trajectory_msgs::JointTrajectory & trajectory)
{
    this->sendTrajectory(trajectory,false);
}

void ArmControlTrial::sendTrajectory(const trajectory_msgs::JointTrajectory & trajectory, bool replace_traj)
{   
    control_msgs::FollowJointTrajectoryGoal goal;

    goal.trajectory = trajectory;
    // When to start the trajectory: 0.1s from now
    if(!replace_traj){start_ = ros::Time::now();}
    goal.trajectory.header.stamp = start_;
    std::cout << "stamp " << goal.trajectory.header.stamp << std::endl;
    ROS_INFO_STREAM("In ArmControlTrial::sendJointTrajectory, the traj header stamp is " << goal.trajectory.header.stamp
        << " and the time_from_start of first point is " << goal.trajectory.points[1].time_from_start << ".");
    ac_->sendGoal(goal);
}

bool ArmControlTrial::waitForCompletion()
{   
    ros::Duration time_out(60.0);
    return ac_->waitForResult(time_out);
}
