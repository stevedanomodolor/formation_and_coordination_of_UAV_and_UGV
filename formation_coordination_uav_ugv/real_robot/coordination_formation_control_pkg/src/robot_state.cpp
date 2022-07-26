#include "robot_state.h"
#include <iostream>

/* constructor*/

robotState::robotState(robot_ind_t robot_ind, ros::NodeHandle &n,bool &sucess_initialization):
nh(n),
id(robot_ind.id),
robot_name(robot_ind.robot_name),
t(robot_ind.t)
{
  std::stringstream ss_pose;
  std::stringstream ss_vel;
  std::stringstream ss_acc;

  if (this->t == CRAZYFLIE)
  {
    ss_pose << "/" << robot_name << std::to_string(this->id) << "/position";
    ss_vel << "/" << robot_name << std::to_string(this->id) << "/velocity";
    ss_acc << "/" << robot_name << std::to_string(this->id) << "/acceleration";

    // uav_sync.registerCallback( boost::bind( &robotState::stateCallbackUav, this, _1, _2 ) );
    this->state_sub_pose = nh.subscribe(ss_pose.str(), 1,&robotState::stateCallbackUavPos, this);
    this->state_sub_vel = nh.subscribe(ss_vel.str(), 1,&robotState::stateCallbackUavVel, this);
    this->state_sub_acc = nh.subscribe(ss_acc.str(), 1,&robotState::stateCallbackUavAcc, this);

    ROS_INFO("Crazyflie %d state subcribed", this->id );
    ros::topic::waitForMessage< crazyswarm::GenericLogData >(ss_vel.str(), ros::Duration(5));
    ros::topic::waitForMessage< crazyswarm::GenericLogData>(ss_pose.str(), ros::Duration(5));
    ros::topic::waitForMessage< crazyswarm::GenericLogData>(ss_acc.str(), ros::Duration(5));

    sucess_initialization = true;


  }
  else if(this->t == OMNIWHEELS)
  {
    ss_pose << "/" << robot_name << std::to_string(this->id) << "/pose";
    ss_vel << "/" << robot_name << std::to_string(this->id) << "/vel";
    ss_acc << "/" << robot_name << std::to_string(this->id) << "/acc";

    // ugv_sync.registerCallback( boost::bind( &robotState::stateCallbackUgv, this, _1, _2 ) );
    this->state_sub_pose = nh.subscribe(ss_pose.str(), 1,&robotState::stateCallbackUgvPose, this);
    this->state_sub_vel = nh.subscribe(ss_vel.str(), 1,&robotState::stateCallbackUgvVel, this);
    this->state_sub_acc = nh.subscribe(ss_acc.str(), 1,&robotState::stateCallbackUgvAcc, this);

    ROS_INFO("Omniwheels %d state subcribed", this->id );
    ros::topic::waitForMessage<geometry_msgs::Twist>(ss_vel.str(), ros::Duration(5));
    ros::topic::waitForMessage<geometry_msgs::Pose>(ss_pose.str(), ros::Duration(5));
    ros::topic::waitForMessage<geometry_msgs::Vector3>(ss_acc.str(), ros::Duration(5));

    sucess_initialization = true;

  }
  else
  {
    ROS_ERROR("Wrong vehicle type");
    sucess_initialization = false;
  }


}

robot_state_t robotState::get_state() const
{
  return this->state;
}
/* callbacks*/
void robotState::stateCallbackUgvVel(const geometry_msgs::TwistConstPtr &vel_msg)
{
  this->state.p(0) = vel_msg->linear.x;
  this->state.p(1) = vel_msg->linear.y;
  this->state.p(2) = vel_msg->angular.z;

}
void robotState::stateCallbackUgvPose(const geometry_msgs::PoseConstPtr &pose_msg)
{
  this->state.q(0) = pose_msg->position.x;
  this->state.q(1) = pose_msg->position.y;
  this->state.q(2) = pose_msg->position.z;
  double roll, pitch,yaw;
  tf::Quaternion quat(pose_msg->orientation.x,
                      pose_msg->orientation.y,
                      pose_msg->orientation.z,
                      pose_msg->orientation.w);

  tf::Matrix3x3(quat).getRPY(roll, pitch,yaw);
  this->state.q(3) = yaw*180.0/M_PI;  // degres to mantain consistency
}
void robotState::stateCallbackUavVel(const crazyswarm::GenericLogDataConstPtr &vel_msg)
{
  this->state.p(0) = vel_msg->values[0]/1000.0;
  this->state.p(1) = vel_msg->values[1]/1000.0;
  this->state.p(2) = vel_msg->values[2]/1000.0;
}
void robotState::stateCallbackUavPos(const crazyswarm::GenericLogDataConstPtr &pose_msg)
{
  this->state.q(0) = pose_msg->values[0]/1000.0;
  this->state.q(1) = pose_msg->values[1]/1000.0;
  this->state.q(2) = pose_msg->values[2]/1000.0;
  this->state.q(3) = pose_msg->values[3]; // degres

}

void robotState::stateCallbackUgvAcc(const geometry_msgs::Vector3ConstPtr &acc_msg)
{

  this->state.a(0) = acc_msg->x;
  this->state.a(1) = acc_msg->y;
  this->state.a(2) = acc_msg->z;

}

void robotState::stateCallbackUavAcc(const crazyswarm::GenericLogDataConstPtr &Acc_msg)
{
  this->state.a(0) = Acc_msg->values[0]/1000.0;
  this->state.a(1) = Acc_msg->values[1]/1000.0;
  this->state.a(2) = Acc_msg->values[2]/1000.0;
}
