#ifndef SWAM_CONTROLLER_ALG_UAV_H
#define SWAM_CONTROLLER_ALG_UAV_H

#include "ros/ros.h"
#include <eigen3/Eigen/Dense>
#include "math.h"
#include "shared_definitions.h"
#include <vector>

class swamControllerAlg
{
private:
  formation_config_t form_param; /*parameters of the formation*/
  row_vector_2d_t integral_error;
  row_vector_2d_t previous_integral_error;
  row_vector_2d_t range_sat;
  robot_ind_t robot_id;
  formation_matrix_t form_type;
  Eigen::MatrixXf iad; /* iter agent distances*/
  int N; /* number of agents*/


public:
  /* constructor*/
  swamControllerAlg(robot_ind_t indentification, formation_config_t formation_config);
  /* destructor*/
  ~swamControllerAlg();
  /* includes the main controller*/
  input_vector_t controller(Eigen::MatrixXf q, Eigen::MatrixXf p, waypoint_t ref, std::vector<obstacle_t> obs);

protected:
  /* configuration formation information like interagent distances*/
  void configureFormation();
  std::vector<std::pair<int, float>> sortNeigbour(Eigen::MatrixXf M, row_vector_2d_t q);
  /* compute fi function*/
  float computeFi(row_vector_2d_t qj, row_vector_2d_t qi, float a, float b, float h, float eps, float r, float d);
  /* compute sigma norm*/
  float computeSigmaNorm(float z, float eps);
  /*compute ph bumber function*/
  float ph(float z, float h);
  /* compute fi*/
  float computefi(float a, float b, float z);
  /* compute sigmaone*/
  float sigmaOne(float z);
/* sigma norm vector form*/
  row_vector_2d_t sigmaOne(row_vector_2d_t z);
  /* compute nij*/
  row_vector_2d_t computeNij(row_vector_2d_t qj, row_vector_2d_t qi, float eps);
  /*compute aij*/
  float computeAij(row_vector_2d_t qj, row_vector_2d_t qi, float r,float eps, float h);
  /* compute fi function for the obstacle*/
  float computeFiB(row_vector_2d_t q_hat_i, row_vector_2d_t qi, float h_beta, float d_obs, float eps);
  /* compute Bij*/
  float computeBij(row_vector_2d_t q_hat_i, row_vector_2d_t qi, float d_obs, float eps, float h_beta);
  /* saturates values in range [min, max]*/
  row_vector_2d_t saturate(row_vector_2d_t input, row_vector_2d_t range);

};










#endif
