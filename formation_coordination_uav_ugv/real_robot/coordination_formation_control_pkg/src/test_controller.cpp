#include "ros/ros.h"
#include "shared_definitions.h"
#include "swamControllerAlg.h"
/* this node is to test and ensure that the controller result a valid result
 * the result is compared with the result obtained from the simulation*/



int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_swam_controller");
  ros::NodeHandle nh("~");
  input_vector_t input;
  std::cout << "Starting test" << std::endl;


  /* remember to rosparam load the configuration file in the conifg path*/
  formation_config_t formation_cfg;
  nh.getParam("/uav_c1_alpha",formation_cfg.gain.c1_alpha);
  nh.getParam("/uav_c2_alpha",formation_cfg.gain.c2_alpha);
  nh.getParam("/uav_c1_beta",formation_cfg.gain.c1_beta);
  nh.getParam("/uav_c2_beta",formation_cfg.gain.c2_beta);
  nh.getParam("/uav_c1_gamma",formation_cfg.gain.c1_gamma);
  nh.getParam("/uav_c2_gamma",formation_cfg.gain.c2_gamma);
  nh.getParam("/uav_c1_theta",formation_cfg.gain.c1_theta);
  nh.getParam("/uav_c1_delta",formation_cfg.gain.c1_delta);
  nh.getParam("/uav_dt",formation_cfg.param.dt);
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
  std::cout << formation_cfg.param.formation_t << std::endl;


  robot_ind_t robot_ind;
  Eigen::MatrixXf uav_state_q(2,3);
  Eigen::MatrixXf uav_state_p(2,3);
  // obstacle_t *obs = new obstacle_t[3]();
  std::vector<obstacle_t> obs;
  obs.resize(3);
  waypoint_t ref;
  std::cout << "Starting test" << std::endl;

  uav_state_q.col(0) << 1 ,-1;
  uav_state_q.col(1) << 1, -1.1;
  uav_state_q.col(2)  << 2, -2;
  uav_state_p.col(0) << 0,0;
  uav_state_p.col(1) << 0,0;
  uav_state_p.col(2)  << 0,0;

  std::cout << "Starting test" << std::endl;

  ref.q << 0,0;
  ref.p << 5,1;
  ref.orientation = 0;
  std::cout << "Starting test" << std::endl;

  obs[0].q << 1, -1.15;
  obs[0].radius = 0.5;
  obs[1].q << 0, 0;
  obs[1].radius = 2;
  obs[2].q << 0, 0;
  obs[2].radius = 3;
  std::cout << "Starting test" << std::endl;


  robot_ind.id = 1;
  robot_ind.id_m = 0;

  robot_ind.robot_name = "crazyflie_" + std::to_string(robot_ind.id);
  robot_ind.t = CRAZYFLIE;
  robot_ind.safe_ditance = 0.15; // to prevent collision
  robot_ind.robot_size = 0.15; // drone radius
  robot_ind.leader = true;
  std::cout << "Starting test" << std::endl;

  swamControllerAlg swam_controller(robot_ind, formation_cfg);
  std::cout << "Starting test" << std::endl;


  input = swam_controller.controller(uav_state_q, uav_state_p,ref,obs);

  std::cout << std::fixed;
  std::cout << std::setprecision(15);
  std::cout << input << std::endl;

  std::cout << "Starting test" << std::endl;

  return 0;
}
