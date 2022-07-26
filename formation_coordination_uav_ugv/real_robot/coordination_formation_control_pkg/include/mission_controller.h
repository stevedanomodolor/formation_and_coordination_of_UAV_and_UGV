#ifndef MISSION_CONTROLLER_H
#define MISSION_CONTROLLER_H
#include "ros/ros.h"
#include "shared_definitions.h"
#include "robot_state.h"
#include <dynamic_reconfigure/server.h>
#include <coordination_formation_control_pkg/missionControllerConfig.h>
#include <coordination_formation_control_pkg/coordination.h>
#include <coordination_formation_control_pkg/waypoint.h>
#include "crazyswarm/GenericLogData.h"

#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
using namespace cv;
using namespace cv;

#define ENABLE_LATCH true
/* state machine mission_controller*/
typedef enum fsm_mc
{
  IDLE,
  INITILIAZE_ROBOT,
  START_FORMATION,
  END
}fsm_mc_t;


class missionController
{
  public:
    missionController(ros::NodeHandle &nh);
    ~missionController();

  protected:
    /* execute main process*/
    void executeMainProcess();
    void updateCofig(coordination_formation_control_pkg::missionControllerConfig &config, uint32_t level);
    Eigen::MatrixXf compute_cluster(Eigen::MatrixXf uav_q, Eigen::MatrixXf ugv_q,int num_ugvs, int num_uavs,Mat &previous_labels, Eigen::MatrixXf & previous_centroids);

  private:
    fsm_mc_t state;
    bool shut_down;
    bool initialize_all_robot;
    bool start_formation;
    bool stop;
    bool mantain_position;
    bool start_publishing_waypoint;
    bool cluster_initialized;
    ros::NodeHandle nh;
    Eigen::MatrixXf obstacle_pose;


      /*dynamic reconfigure*/
   dynamic_reconfigure::Server<coordination_formation_control_pkg::missionControllerConfig> server;
   dynamic_reconfigure::Server<coordination_formation_control_pkg::missionControllerConfig>::CallbackType f;

   /* service responsible for the initialization, shut_down of all the robot*/
   ros::Publisher coordination_publisher;
   ros::Publisher waypoint_publisher;
   ros::Subscriber obs_state_publisher;

   float eps;
   float n_obs;

   coordination_formation_control_pkg::coordination current_configuration;
   coordination_formation_control_pkg::waypoint current_waypoint;
   void stateCallbackObsPos(const crazyswarm::GenericLogDataConstPtr &pose_msg);




};



#endif
