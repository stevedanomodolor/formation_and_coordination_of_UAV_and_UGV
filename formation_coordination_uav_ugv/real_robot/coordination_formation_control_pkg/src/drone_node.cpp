/* drone control node, represents the computation of a drone*/
/* TODO maybe implement in a class format as mission controller*/
#include "ros/ros.h"
#include "robot_state.h"
#include "shared_definitions.h"
#include "swamControllerAlg.h"
#include "coordination_formation_control_pkg/coordination.h"
#include "coordination_formation_control_pkg/waypoint.h"
#include "coordination_formation_control_pkg/logData.h"
#include <rosbag/bag.h>

#include "crazyswarm/Hover.h"
#include "crazyswarm/Land.h"
#include "std_msgs/Empty.h"
#include <unistd.h>
#include "integration.h"
#include <cmath>

/* global variables*/
coordination_formation_control_pkg::coordination *coord_config;
coordination_formation_control_pkg::waypoint *next_waypoint;
coordination_formation_control_pkg::logData *logdata;
std::stringstream ss_log_naming;
rosbag::Bag bag;
std::string robot_name;
bool enable_log;
bool emergency_stop = false;
bool start_logging = false;


bool waypoint_received = false;
row_vector_2d_t saturate(row_vector_2d_t input, row_vector_2d_t range)
{
  row_vector_2d_t return_value;
  for(int i = 0; i <2; i++)
  {
    if(input(i) > range(1))
    {
      return_value(i) = range(i);
    }
    else if(input(i) < range(0))
    {
      return_value(i) = range(0);
    }
    else
    {
      return_value(i) = input(i);

    }
  }
  return return_value;
}

void configurationCallback(const coordination_formation_control_pkg::coordination::ConstPtr& msg );
void waypointCallback(const coordination_formation_control_pkg::waypoint::ConstPtr& msg);
void logCallback(const ros::TimerEvent& event);
int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  int robotid = std::atoi(argv[1]);
  ros::NodeHandle nh;

  ROS_INFO("Drone %d turned ON", robotid);
  int n_drones;
  int hz_freq, n_obs,n_ugvs;
  bool use_obstacle,use_orientation;
  nh.getParam("/n_drones", n_drones);
  std::cout << "NUmber of drones " << n_drones << std::endl;
  nh.getParam("/hz_freq", hz_freq); //* might not be a good idea*/
  nh.getParam("/include_obstacle", use_obstacle);
  nh.getParam("/include_orientation", use_orientation);
  nh.getParam("/n_obs", n_obs);
  nh.getParam("/n_ugvs", n_ugvs);

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
  next_waypoint->obstacle.resize(n_obs*3);
  next_waypoint->cluster_pos.resize(n_ugvs*2);



  logdata = new coordination_formation_control_pkg::logData();
  logdata->current_pose.resize(n_drones);
  logdata->current_twist.resize(n_drones);
  logdata->current_acc.resize(n_drones);
  logdata->current_obstacle.resize(n_obs*3);
  logdata->ugv_current_pose.resize(n_ugvs);
  logdata->ugv_cluster.resize(n_ugvs);

  /* current command*/
  crazyswarm::Hover current_command;
  current_command.vx = 0;
  current_command.vy = 0;
  current_command.yawrate = 0;
  current_command.zDistance = 0;

  /* current state*/
  /* current implementaiton is in 2D*/
  Eigen::MatrixXf uav_state_q(2,n_drones);
  Eigen::MatrixXf uav_state_p(2,n_drones);
  Eigen::MatrixXf uav_state_a(2,n_drones);

  // obstacle_t *current_obs = new obstacle_t[n_obs]();
  std::vector<obstacle_t> current_obs;
  current_obs.resize(n_obs);
  waypoint_t current_waypoint;
  current_waypoint.q(0) = 0 ; /*position 2 position 1 orientation yaw*/
  current_waypoint.q(1) = 0; /*position 2 position 1 orientation yaw*/
  current_waypoint.p(0) = 0 ; /*position 2 position 1 orientation yaw*/
  current_waypoint.p(1) = 0 ; /*position 2 position 1 orientation yaw*/

  integration acc_int(0.0,0.0, dt);


  /* robot indentification*/
  robot_ind_t robot_ind;

  robot_ind.id = robotid;
  robot_ind.id_m = robotid-1;

  robot_ind.robot_name = "crazyflie_" + std::to_string(robotid);
  robot_name = robot_ind.robot_name + "result";
  robot_ind.t = CRAZYFLIE;
  robot_ind.safe_ditance = 0.15; // to prevent collision
  robot_ind.robot_size = 0.15; // drone radius
  /* make drone 1 the leader, this is for orientaiton*/
  if (robotid == 1)
  {
    robot_ind.leader = true;
  }
  else
  {
    robot_ind.leader = true;
  }

  /* initialization of the formation configuration */
  formation_config_t formation_cfg;
  nh.getParam("/uav_c1_alpha",formation_cfg.gain.c1_alpha);
  nh.getParam("/uav_c2_alpha",formation_cfg.gain.c2_alpha);
  nh.getParam("/uav_c1_beta",formation_cfg.gain.c1_beta);
  nh.getParam("/uav_c2_beta",formation_cfg.gain.c2_beta);
  nh.getParam("/uav_c1_gamma",formation_cfg.gain.c1_gamma);
  nh.getParam("/uav_c2_gamma",formation_cfg.gain.c2_gamma);
  nh.getParam("/uav_c1_theta",formation_cfg.gain.c1_theta);
  nh.getParam("/uav_c1_delta",formation_cfg.gain.c1_delta);

  formation_cfg.param.dt = dt;

  nh.getParam("/uav_d", formation_cfg.param.d);
  nh.getParam("/uav_formation_t", formation_cfg.param.formation_t);
  nh.getParam("/uav_k", formation_cfg.param.k);
  nh.getParam("/uav_ratio", formation_cfg.param.ratio);
  nh.getParam("/uav_eps", formation_cfg.param.eps);
  nh.getParam("/uav_a", formation_cfg.param.a);
  nh.getParam("/uav_b", formation_cfg.param.b);
  nh.getParam("/uav_h_alpha", formation_cfg.param.h_alpha);
  nh.getParam("/uav_h_beta", formation_cfg.param.h_beta);
  nh.getParam("/uav_d_obs", formation_cfg.param.d_obs);
  nh.getParam("/uav_nav_type", formation_cfg.param.nav_type);
  nh.getParam("/uav_integrator", formation_cfg.param.integrator);
  nh.getParam("/uav_int_max", formation_cfg.param.int_max);

  /* Initialize Swam controller*/
  swamControllerAlg swam_controller(robot_ind, formation_cfg);




  /*subcribers*/
  ros::Subscriber configuration_sub = nh.subscribe("/coordination_config", 1,configurationCallback);
  std::stringstream ss_t;
  ss_t << "/cf"  << std::to_string(robotid) << "/land";
  ros::Subscriber waypoint_sub = nh.subscribe("/goal_waypoint", 1,waypointCallback);

  ros::ServiceClient land_drone_client = nh.serviceClient <crazyswarm::Land>(ss_t.str());

  /* publisher*/
  std::stringstream ss;
  ss << "/cf"  << std::to_string(robotid) << "/cmd_hover";
  ros::Publisher vel_cmd_publisher = nh.advertise<crazyswarm::Hover>(ss.str(),1);

  std::stringstream ss_stop;
  ss_stop << "/cf"  << std::to_string(robotid) << "/cmd_stop";
  ros::Publisher stop_robot_publisher = nh.advertise<std_msgs::Empty>(ss_stop.str(),1);

  /* TODO publisher to publish results and rosbag it*/

  std::string log_data_name;
  nh.getParam("/enable_log", enable_log);
  nh.getParam("/log_name", log_data_name);
  ss_log_naming << log_data_name << "_crazyflie_" << std::to_string(robotid) << "_logdata.bag";

  // bag_ptr = new rosbag::bag();
  ros::Timer writeTobagfileTimer = nh.createTimer(ros::Duration(1.0/10.0), logCallback);

  if(enable_log)
  {
    bag.open(ss_log_naming.str(),rosbag::bagmode::Write);
    writeTobagfileTimer.start();
    ROS_INFO("Enabled logging");

  }



  /*create a list of drones to subscribe to*/
  std::vector<robotState *> uavs;
  robot_ind_t uav_def;
  bool sucess_initialization;
  input_vector_t input;
  input  << 0.0,0.0,0.0,0.0,0.0,0.0,
                          0.0,0.0,0.0,0.0,0.0,0.0;
  for (int i = 1; i <= n_drones; i++)
  {
    uav_def.robot_name = "cf";
    uav_def.t = CRAZYFLIE;
    uav_def.id = i;
    uav_def.id_m = i-1;
    sucess_initialization = false;
    auto * uav = new robotState(uav_def, nh, sucess_initialization);
    uavs.push_back(uav);
    if (sucess_initialization == false)
    {
      ROS_ERROR("Initialization: Agv %d produced an error in mission controller", uav_def.id);
      EXIT_FAILURE;
    }
  }


  /* create a list of ugvs just for logging data from the same node */
  std::vector<robotState *> ugvs;
  robot_ind_t ugv_def;
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


  ros::Rate loop_rate(hz_freq);


  /* main loop*/
  /* TODO improve state machine*/
  float vxx = 0, vyy = 0, yaw = 0;
  Eigen::Vector2d vel_command(0,0);
  while(ros::ok())
  {
    /*get current state of all the robot*/
    for(int i = 1; i <=n_drones; i++)
    {
      robot_state_t current_state;
      current_state = uavs[i-1]->get_state();
      uav_state_q(0, i-1) =current_state.q(0);
      uav_state_q(1, i-1) =current_state.q(1);
      yaw = current_state.q(3);
      uav_state_p(0, i-1) = current_state.p(0);
      uav_state_p(1, i-1) = current_state.p(1);
      uav_state_a(0, i-1) = current_state.a(0);
      uav_state_a(1, i-1) = current_state.a(1);
    }
    // std::cout << uav_state_q << std::endl;
    if(coord_config->initialize_robot && coord_config->init_formation ==false && coord_config->stop_demo == false)
    {
      // send the robot to hover at zero velocity in plane
      // std::cout << "Here" << std::endl;
      current_command.vx = 0;
      current_command.vy = 0;
      current_command.yawrate = 0;
      current_command.zDistance = 0.5;
      vel_cmd_publisher.publish(current_command);


    }
    else if(coord_config->initialize_robot && coord_config->init_formation && coord_config->stop_demo == false)
    {
      /* start formation controll*/
      // std::cout << waypoint_received <<std::endl;

      if (waypoint_received)
      {
        start_logging = true;


        /* update reference point*/
        current_waypoint.q(0) = next_waypoint->pos[0] ; /*position 2 position 1 orientation yaw*/
        current_waypoint.q(1) = next_waypoint->pos[1] ; /*position 2 position 1 orientation yaw*/
        current_waypoint.p(0) = next_waypoint->vel[0] ; /*position 2 position 1 orientation yaw*/
        current_waypoint.p(1) = next_waypoint->pos[1] ; /*position 2 position 1 orientation yaw*/

        /* use orienation: update orientation*/
        if (use_orientation)
        {
          if(coord_config->start_orientation)
          {
            current_waypoint.orientation = next_waypoint->orientation;
          }
          else
          {
            current_waypoint.orientation = NOORIENTATION;
          }
        }
        else
        {
          current_waypoint.orientation = NOORIENTATION;

        }
        /* use obstacle update obstacle*/
        if(n_obs != 0)
        {
          int indx = 0;
          for(int n = 0; n < n_obs; n++)
          {

            current_obs[n].q(0) = next_waypoint->obstacle[indx];
            current_obs[n].q(1) = next_waypoint->obstacle[indx+1];
            current_obs[n].radius = next_waypoint->obstacle[indx+2];
            indx = indx+3;
            // std::cout << current_obs[n].q(0) << " " << current_obs[n].q(1) << std::endl;
          }
        }
        else
        {
          current_obs.resize(0);
        }

        /* call controller*/
        input = swam_controller.controller(uav_state_q, uav_state_p,current_waypoint,current_obs);
        // std::cout << input << std::endl;
        /* integrate acceleration*/
        vel_command = acc_int.integrate(input(0,0), input(1,0),uav_state_p(0, robotid-1),uav_state_p(1, robotid-1) );
        Eigen::Vector2d vel_command_;
         vel_command_ << input(0,0), input(1,0);
        /* reached a certain waypoint and we want to stop sending command*/
        if(next_waypoint->stop) // change back TODO
        {
          current_command.vx = 0;
          current_command.vy = 0;
        }
        else if((abs(uav_state_q(0, robotid-1)) > 2) || (abs(uav_state_q(1, robotid-1)) > 1.0))
        {
          current_command.vx = 0;
          current_command.vy = 0;
          // emergency_stop = true;
        }
        else
        {
          /* transform from global velocity to local velocity*/
          /* get latest yaw*/
          robot_state_t current_state_;
          current_state_ = uavs[robotid-1]->get_state();
          float yaw_ = current_state_.q(3)*M_PI/180.0;  // TODO confirm that indeed it is in degrees
          // float vxl =  vel_command_(0)*std::cos(yaw) +  vel_command_(1)*std::sin(yaw) ;
          // float vyl = - vel_command_(0)*std::sin(yaw)  +  vel_command_(1)*std::cos(yaw) ;
          float vxl =  vel_command(0)*std::cos(yaw_) +  vel_command(1)*std::sin(yaw_) ;
          float vyl = - vel_command(0)*std::sin(yaw_)  +  vel_command(1)*std::cos(yaw_) ;

          // current_command.vx = vxl;
          // current_command.vy = vyl;
          row_vector_2d_t in;
          in << vxl, vyl;
          row_vector_2d_t range;
          // range << -0.15, 0.15;
          range << -1, 1;

          row_vector_2d_t out = saturate(in, range);
          current_command.vx = out(0);
          current_command.vy = out(1);
          // vxx = vxl;
          // vyy = vyl;
        }



      }
      // /* testing velocity command*/
      // robot_state_t current_state_;
      // float vx = 0.05;
      // float vy = 0;
      // current_state_ = uavs[robotid-1]->get_state();
      // float yaw = current_state_.q(3)*M_PI/180.0;  // TODO confirm that indeed it is in degrees
      // float vxl =  vx*std::cos(yaw) +  0*std::sin(yaw) ;
      // float vyl = -vx*std::sin(yaw)  +  0*std::cos(yaw) ;
      //
      // current_command.vx = vxl;
      // current_command.vy = vyl;
      // std::cout << "vx: normal: " << vx << "vy normal: " << vy << std::endl;
      // std::cout << "***********************************" << std::endl;
      // std::cout << "vy: transformed: " << vxl << "vy transformed: " << vyl << std::endl;


      // vel_cmd_publisher.publish(current_command);

    }

    // std::cout << "here " << ((coord_config->stop_demo)?1:0) << std::endl;
    if(coord_config->stop_demo)
    {
      // land_drone();
      // current_command.vx = 0;
      // current_command.vy = 0;
      // current_command.yawrate = 0;
      // current_command.zDistance = 0;
      // std::cout << "Here" << std::endl;
      crazyswarm::Land land_drone_srv;
      land_drone_srv.request.height = 0;
      ros::Duration d(3);
      land_drone_srv.request.duration = d;

      if(land_drone_client.call(land_drone_srv))
      {
        std::cout << "Shutting down uav: " << std::to_string(robotid) << "Successfull" << std::endl;
      }
      else
      {
        std::cout << "Shutting down uav: " << std::to_string(robotid) << "unsuccessfull" << std::endl;

      }
      sleep(3);
      break;


    }

    if(enable_log)
    {
      logdata->header.stamp = ros::Time::now();
      logdata->header.frame_id = robot_ind.robot_name;
      for(int i = 0; i <n_drones; i++)
      {
        logdata->current_pose[i].position.x = uav_state_q(0,i);
        logdata->current_pose[i].position.y = uav_state_q(1,i);
        // std::cout <<(uav_state_q(1,i)) << std::endl;
        logdata->current_twist[i].linear.x = uav_state_p(0,i);
        logdata->current_twist[i].linear.y = uav_state_p(1,i);
        logdata->current_acc[i].linear.x = uav_state_a(0,i);
        logdata->current_acc[i].linear.y = uav_state_a(1,i);
      }
      /* log the ugv state too, not an efficient approach*/
      for (int i = 0; i<n_ugvs; i++)
      {
        robot_state_t current_ugv_state;
        current_ugv_state = ugvs[i]->get_state();
        logdata->ugv_current_pose[i].position.x = current_ugv_state.q(0);
        logdata->ugv_current_pose[i].position.y = current_ugv_state.q(1);
        logdata->ugv_cluster[i].position.x = next_waypoint->cluster_pos[i*2];
        logdata->ugv_cluster[i].position.y = next_waypoint->cluster_pos[i*2+1];
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
      logdata->input_orientation.x =   input(0,4);
      logdata->input_orientation.y =input(1,4);
      logdata->input_integration.x = input(0,5);
      logdata->input_integration.y = input(1,5);
      logdata->input_to_system.x =vel_command(0);
      logdata->input_to_system.y =vel_command(1);
      logdata->input_before_to_system.x =  current_command.vx;
      logdata->input_before_to_system.y =  current_command.vy;
      logdata->yaw = yaw;
      // TODO add to omni wheel node
      if(n_obs != 0)
      {
        int indx = 0;
        for(int n = 0; n < n_obs; n++)
        {
          //
          logdata->current_obstacle[indx]= next_waypoint->obstacle[indx];
          logdata->current_obstacle[indx+1] = next_waypoint->obstacle[indx+1];
          logdata->current_obstacle[indx+2] = next_waypoint->obstacle[indx+2];
          indx = indx+3;
        }
      }

      // std::cout << yaw << std::endl;

      // uav_state_p(0, robotid-1),uav_state_p(1, robotid-1)



    }

    ros::spinOnce();
    loop_rate.sleep();


  }


  /*stop */
  for(int i = 0; i <= 1000; i++)
  {
    std_msgs::Empty stop_drone;
    stop_robot_publisher.publish(stop_drone);
  }
  if(enable_log)
  {
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
  if(enable_log & start_logging)
  {
    /* this can be an issue, concurrent read write can be a problem*/
    bag.write(robot_name,logdata->header.stamp, *logdata);
  }


}
