#ifndef UAV_H
#define UAV_H

#include "shared_definitions.h"
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Vector3.h"
#include "crazyswarm/GenericLogData.h"
#include "robot_state.h"
#include "tf/transform_datatypes.h"
// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>

/* obtain_robot_state*/


class robotState
{
private:
  std::string robot_name;
  robot_type_t t;
  ros::Subscriber state_sub_pose;
  ros::Subscriber state_sub_vel;
  ros::Subscriber state_sub_acc;

  ros::NodeHandle nh;
  /* not an effective way*/
  // message_filters::Subscriber<geometry_msgs::Twist> vel_state_ugv;
  // message_filters::Subscriber<geometry_msgs::Pose> pos_state_ugv;
  // message_filters::Subscriber<crazyswarm::GenericLogData> vel_state_uav;
  // message_filters::Subscriber<crazyswarm::GenericLogData> pos_state_uav;
  // TimeSynchronizer<crazyswarm::GenericLogData, crazyswarm::GenericLogData> uav_sync;
  // TimeSynchronizer<geometry_msgs::Twist, geometry_msgs::Pose> ugv_sync;

  /* not effective, there should be synchrnied*/
  void stateCallbackUgvVel(const geometry_msgs::TwistConstPtr &vel_msg);
  void stateCallbackUgvPose(const geometry_msgs::PoseConstPtr &pose_msg);
  void stateCallbackUgvAcc(const geometry_msgs::Vector3ConstPtr &acc_msg);
  void stateCallbackUavVel(const crazyswarm::GenericLogDataConstPtr &vel_msg);
  void stateCallbackUavPos(const crazyswarm::GenericLogDataConstPtr &pose_msg);
  void stateCallbackUavAcc(const crazyswarm::GenericLogDataConstPtr &Acc_msg);


public:
  int id;
  robotState(robot_ind_t robot_ind, ros::NodeHandle &n,bool &sucess_initialization);
  robot_state_t state;
  robot_state_t get_state() const;  /* the const prevents changes in the state when it is called*/

};



#endif
