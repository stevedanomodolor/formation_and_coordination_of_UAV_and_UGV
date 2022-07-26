#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "coordination_formation_control_pkg/coordination.h"
#include "coordination_formation_control_pkg/waypoint.h"
#include "coordination_formation_control_pkg/logData.h"

#include "robot_state.h"
#include "shared_definitions.h"
#include "swamControllerAlg.h"
#include <rosbag/bag.h>

/* global variables*/
coordination_formation_control_pkg::coordination *coord_config;
coordination_formation_control_pkg::waypoint *next_waypoint;
coordination_formation_control_pkg::logData *logdata;

std::stringstream ss_log_naming;
rosbag::Bag bag;
bool waypoint_received = false;
std::string robot_name;
bool enable_log = false;


void configurationCallback(const coordination_formation_control_pkg::coordination::ConstPtr& msg );
void waypointCallback(const coordination_formation_control_pkg::waypoint::ConstPtr& msg);
void logCallback(const ros::TimerEvent& event);



int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  int robotid = std::atoi(argv[1]);
  ros::NodeHandle nh;
  ROS_INFO("Ugv %d turned ON", robotid);
  int hz_freq, n_ugvs, n_obs;
  bool use_obstacle;
  nh.getParam("/n_ugvs", n_ugvs);
  nh.getParam("/hz_freq", hz_freq);
  nh.getParam("/include_obstacle", use_obstacle);
  nh.getParam("/n_obs", n_obs);

  float dt = 1/(float)hz_freq;

  /* intialization*/
  coord_config = new coordination_formation_control_pkg::coordination();
  coord_config->stop_demo = false;
  coord_config->init_formation = false;
  coord_config->initialize_robot = false;

  /* current waypoint*/
  next_waypoint = new coordination_formation_control_pkg::waypoint();
  next_waypoint->pos[0] = 0;
  next_waypoint->pos[1] = 0;
  next_waypoint->vel[0] = 0;
  next_waypoint->vel[1] = 0;
  next_waypoint->orientation = 0.0;

  logdata = new coordination_formation_control_pkg::logData();
  logdata->current_pose.resize(n_ugvs);
  logdata->current_twist.resize(n_ugvs);
  logdata->current_acc.resize(n_ugvs);
  // logdata->current_acc.resize(n_ugvs);

  /*current command*/
  geometry_msgs::Twist current_command;
  current_command.linear.x = 0;
  current_command.linear.y = 0;
  current_command.angular.z = 0;

  /*current state include cluster and ugv pose*/
  Eigen::MatrixXf current_state_q(2,2);
  Eigen::MatrixXf current_state_p(2,2);

  /* vector of ugvs*/
  Eigen::MatrixXf ugv_state_q(2,n_ugvs);
  Eigen::MatrixXf ugv_state_p(2,n_ugvs);
  Eigen::MatrixXf ugv_state_a(2,n_ugvs);

  /* obstacle = number of obstacle + the other ugvs*/
  int n_obs_ugv = (n_obs) + (n_ugvs-1);
  // obstacle_t *current_obs = new obstacle_t[n_obs_ugv]();
  std::vector<obstacle_t> current_obs;
  current_obs.resize(n_obs_ugv);
  waypoint_t current_waypoint;
  current_waypoint.q(0) = 0 ; /*position 2 position 1 orientation yaw*/
  current_waypoint.q(1) = 0; /*position 2 position 1 orientation yaw*/
  current_waypoint.p(0) = 0 ; /*position 2 position 1 orientation yaw*/
  current_waypoint.p(1) = 0 ; /*position 2 position 1 orientation yaw*/



  /* robot indentification*/
  robot_ind_t robot_ind;

  robot_ind.id = robotid;
  robot_ind.id_m = 1; // there are both one because the sentoid is in the first location

  robot_ind.robot_name = "omniwheel" + std::to_string(robotid);
  robot_name = robot_ind.robot_name + "result";
  robot_ind.t = OMNIWHEELS;
  robot_ind.safe_ditance = 0.15; // to prevent collision
  robot_ind.robot_size = 0.32; // drone radius
  robot_ind.leader = false;
  /* initialization of the formation configuration */
  formation_config_t formation_cfg;
  nh.getParam("/ugv_c1_alpha",formation_cfg.gain.c1_alpha);
  nh.getParam("/ugv_c2_alpha",formation_cfg.gain.c2_alpha);
  nh.getParam("/ugv_c1_beta",formation_cfg.gain.c1_beta);
  nh.getParam("/ugv_c2_beta",formation_cfg.gain.c2_beta);
  nh.getParam("/ugv_c1_gamma",formation_cfg.gain.c1_gamma);
  nh.getParam("/ugv_c2_gamma",formation_cfg.gain.c2_gamma);
  nh.getParam("/ugv_c1_theta",formation_cfg.gain.c1_theta);
  nh.getParam("/ugv_c1_delta",formation_cfg.gain.c1_delta);


  formation_cfg.param.dt = dt;

  nh.getParam("/ugv_d", formation_cfg.param.d);
  nh.getParam("/ugv_formation_t", formation_cfg.param.formation_t);
  nh.getParam("/ugv_k", formation_cfg.param.k);
  nh.getParam("/ugv_ratio", formation_cfg.param.ratio);
  nh.getParam("/ugv_eps", formation_cfg.param.eps);
  nh.getParam("/ugv_a", formation_cfg.param.a);
  nh.getParam("/ugv_b", formation_cfg.param.b);
  nh.getParam("/ugv_h_alpha", formation_cfg.param.h_alpha);
  nh.getParam("/ugv_h_beta", formation_cfg.param.h_beta);
  nh.getParam("/ugv_d_obs", formation_cfg.param.d_obs);
  nh.getParam("/ugv_nav_type", formation_cfg.param.nav_type);
  nh.getParam("/ugv_integrator", formation_cfg.param.integrator);
  nh.getParam("/ugv_int_max", formation_cfg.param.int_max);
  ROS_INFO("Omniwheels %d : variable initialization completed Successfull", robotid);


  /* Initialize Swam controller*/
  swamControllerAlg swam_controller(robot_ind, formation_cfg);
  ROS_INFO("Omniwheels %d : Swam controlller initialzied correctly", robotid);


  /* Subscribers*/
  ros::Subscriber configuration_sub = nh.subscribe("/coordination_config", 1,configurationCallback);
  ros::Subscriber waypoint_sub = nh.subscribe("/goal_waypoint", 1,waypointCallback);

  /* publishers*/
  std::stringstream ss;
  ss << "/omniwheel"  << std::to_string(robotid) << "/vel_cmd";
  ros::Publisher vel_cmd_publisher = nh.advertise<geometry_msgs::Twist>(ss.str(),1);


  /* TODO publisher to publish results and rosbag it*/
  // bag_ptr = new rosbag::Bag;
  std::string log_data_name;
  nh.getParam("/enable_log", enable_log);
  nh.getParam("/log_name", log_data_name);
  ss_log_naming << log_data_name << "_omniwheel_" << std::to_string(robotid) << "_logdata.bag";
  ROS_INFO("Logging to %s", ss_log_naming.str().c_str());
  // rosbag::Bag test;
  ros::Timer writeTobagfileTimer = nh.createTimer(ros::Duration(1.0/10.0), logCallback);

  // rosbag::Bag bag;
  if(enable_log)
  {
    bag.open(ss_log_naming.str(),rosbag::bagmode::Write);
    writeTobagfileTimer.start();


  }




  std::vector<robotState *> ugvs;
  robot_ind_t ugv_def;
  bool sucess_initialization;
  for (int i = 1; i <= n_ugvs; i++)
  {
    ugv_def.robot_name = "omniwheel";
    ugv_def.t = OMNIWHEELS;
    ugv_def.id = i;
    ugv_def.id_m = 1;
    sucess_initialization = false;
    auto * ugv = new robotState(ugv_def, nh, sucess_initialization);
    ugvs.push_back(ugv);
    if (sucess_initialization == false)
    {
      ROS_ERROR("Initialization: ugv  %d produced an error in mission controller", ugv_def.id);
      EXIT_FAILURE;
    }
  }

  input_vector_t input;
  input  << 0.0,0.0,0.0,0.0,0.0,0.0,
                          0.0,0.0,0.0,0.0,0.0,0.0;



  ros::Rate loop_rate(hz_freq);
  /* main loop*/
  /* TODO improve state machine*/
  while(ros::ok())
  {

            /*get current state of all the ugv*/
            for(int i = 1; i <=n_ugvs; i++)
            {
              robot_state_t current_state;
              current_state = ugvs[i-1]->get_state();
              ugv_state_q(0, i-1) =current_state.q(0);
              ugv_state_q(1, i-1) =current_state.q(1);
              ugv_state_p(0, i-1) = current_state.p(0);
              ugv_state_p(1, i-1) = current_state.p(1);
              ugv_state_a(0, i-1) = current_state.a(0);
              ugv_state_a(1, i-1) = current_state.a(1);
              // std::cout <<   ugv_state_q(0, i-1) << std::endl;

            }


    if(coord_config->initialize_robot && coord_config->init_formation ==false && coord_config->stop_demo == false)
    {
      // send the robot to hover at zero velocity in plane
      // std::cout << "Here" << std::endl;
      current_command.linear.x = 0;
      current_command.linear.y = 0;
      current_command.angular.z = 0;
      vel_cmd_publisher.publish(current_command);
    }
    else if(coord_config->initialize_robot && coord_config->init_formation && coord_config->stop_demo == false)
    {
      /* start formation controll*/

      if (waypoint_received)
      {

        /* create a state vector of the cluster and the current agv*/
        current_state_q(0,0) = next_waypoint->cluster_pos[robotid*2 -2];
        current_state_q(1,0) = next_waypoint->cluster_pos[robotid*2 -1];
        current_state_p(0,0) = next_waypoint->cluster_vel[0];
        current_state_p(1,0) = next_waypoint->cluster_vel[1];
        current_state_q.col(1) = ugv_state_q.col(robotid-1);
        current_state_p.col(1) = ugv_state_p.col(robotid-1);

        /* update reference point*/
        current_waypoint.q(0) =next_waypoint->cluster_pos[robotid*2 -2];
        current_waypoint.q(1) = next_waypoint->cluster_pos[robotid*2 -1];
        current_waypoint.p(0) = next_waypoint->cluster_vel[0];
        current_waypoint.p(1) = next_waypoint->cluster_vel[1];

        /* mantain a 180 degress*/
        current_waypoint.orientation = 180;
        if (n_obs_ugv > 0)
        {
          /* add obstacle*/
          if(n_obs != 0)
          {
            int indx = 0;
            for(int n = 0; n < n_obs; n++)
            {

              current_obs[n].q(0) = next_waypoint->obstacle[indx];
              current_obs[n].q(1) = next_waypoint->obstacle[indx+1];
              current_obs[n].radius = next_waypoint->obstacle[indx+2];
              indx = indx+3;
            }
          }
          /* add other ugv as obstacle*/
          if((n_ugvs-1) !=0)
          {
            int ugv_ind = 0;
            for(int n = 0; n < (n_ugvs); n++)
            {
              if ((n+1) != robotid) // exclude current ugv as it own obstacle
              {
                current_obs[n_obs+ugv_ind].q(0) =ugv_state_q(0, n) ;
                current_obs[n_obs+ugv_ind].q(1) =ugv_state_q(1, n) ;
                current_obs[n_obs+ugv_ind].radius = robot_ind.robot_size +0.1;
                ugv_ind = ugv_ind+1;
              }

            }
          }

        }
        else
        {
          current_obs.resize(0);
        }

      //
        /* call controller*/
        // if(robotid == 2)
        // {
        //   std::cout << current_state_q << std::endl;
        //   std::cout << current_state_p << std::endl;
        //   std::cout << current_waypoint.q << std::endl;
        //   std::cout << current_obs[0].q << std::endl;
        // }
        input = swam_controller.controller(current_state_q, current_state_p,current_waypoint,current_obs);
        // ROS_INFO("Five");

        /* integrate acceleration*/
        if(next_waypoint->stop)
        {
          current_command.linear.x = 0;
          current_command.linear.y = 0;
          current_command.angular.z = 0;
        }
        else
        {
          current_command.linear.x = input(0,0);
          current_command.linear.y = input(1,0);
          current_command.angular.z = 0;

        }



      }
      // vel_cmd_publisher.publish(current_command);

    }
    if(coord_config->stop_demo)
    {
      current_command.linear.x = 0;
      current_command.linear.y = 0;
      current_command.angular.z = 0;
      vel_cmd_publisher.publish(current_command);
      sleep(3);
      break;


    }

    if(enable_log)
    {
      logdata->header.stamp = ros::Time::now();
      logdata->header.frame_id = robot_ind.robot_name;

      for(int i = 0; i <n_ugvs; i++)
      {

        logdata->current_pose[i].position.x = ugv_state_q(0,i);
        logdata->current_pose[i].position.y = ugv_state_q(1,i);
        logdata->current_twist[i].linear.x = ugv_state_p(0,i);
        logdata->current_twist[i].linear.y = ugv_state_p(1,i);
        logdata->current_acc[i].linear.x = ugv_state_a(0,i);
        logdata->current_acc[i].linear.y = ugv_state_a(1,i);
      }

      logdata->command_pose.x = current_waypoint.q(0);
      logdata->command_pose.y = current_waypoint.q(1);
      logdata->command_pose.theta = current_waypoint.orientation;
      logdata->command_twist.linear.x = current_waypoint.p(0);
      logdata->command_twist.linear.y = current_waypoint.p(1);
      logdata->input.x = input(0,0);
      logdata->input.y = input(1,0);
      logdata->input_formation.x = input(0,1);
      logdata->input_formation.y = input(1,1);
      logdata->input_obstacle.x = input(0,2);
      logdata->input_obstacle.y = input(1,2);
      logdata->input_navigation.x = input(0,3);
      logdata->input_navigation.y = input(1,3);
      logdata->input_orientation.x = input(0,4);
      logdata->input_orientation.y = input(1,4);
      logdata->input_integration.x = input(0,5);
      logdata->input_integration.y = input(1,5);
      logdata->input_to_system.x = current_command.linear.x;
      logdata->input_to_system.y = current_command.linear.x;
      // bag.write("robot_1",logdata->header.stamp, *logdata);




    }


    ros::spinOnce();
    loop_rate.sleep();
  }


  if(enable_log)
  {
    writeTobagfileTimer.stop();

    bag.close();
  }

  return 0;
}


/* callbacks*/
void configurationCallback(const coordination_formation_control_pkg::coordination::ConstPtr& msg )
{
  // std::cout <<"Here in callback" << std::endl;
  *coord_config = *msg;
}

void waypointCallback(const coordination_formation_control_pkg::waypoint::ConstPtr& msg)
{
  waypoint_received = true;
  *next_waypoint = *msg;
}

void logCallback(const ros::TimerEvent& event)
{
  /* this can be an issue, concurrent read write can be a problem*/
  // ROS_INFO("HEre");
  if(enable_log)
  {
    bag.write(robot_name,logdata->header.stamp, *logdata);
  }

}
