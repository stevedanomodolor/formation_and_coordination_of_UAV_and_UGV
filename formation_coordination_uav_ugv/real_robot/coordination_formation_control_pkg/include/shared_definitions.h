#ifndef SHARED_DEFINITION_H
#define SHARED_DEFINITION_H

#include <eigen3/Eigen/Dense>
#include <iostream>

// #define MAX_CHAR 30
using namespace Eigen;
#define NOORIENTATION -370 // just a random number out of 0 to 360

typedef enum robot_type
{
  CRAZYFLIE,
  OMNIWHEELS,

} robot_type_t;

/* formation types*/
typedef enum formation_matrix
{
  PAIR =2,
  TRIANGLE = 3,
  SQUARE = 4,
  PENTAGON = 5
}formation_matrix_t;

/* robot state-ugv and agv*/
typedef  struct robot_state
{
  Vector4d q; /*position 3 position 1 orientation yaw*/
  Vector3d p; /* velocity*/
  Vector3d a; /* acceleration*/
}robot_state_t;

/* indentify the robot*/
typedef struct robot_ind
{
  int id;   /*d*/
  int id_m; /* position in the distance vector*/
  std::string robot_name; /* robot name*/
  robot_type_t t; /*robot_type*/
  float safe_ditance; /* important to prevent mistakenly putting a very small number*/
  float robot_size; /* also for safety reasons*/
  bool leader;
}robot_ind_t;

/* obstacle definition*/
/* Only applied to spherical obstacles*/
typedef struct obstacle
{
  Vector2d q; /*position of the obstacle*/
  float radius; /*radius of the obstacle*/
}obstacle_t;

/* goal waypoint*/
typedef struct waypoint
{
  Vector2d q; /*position 2 position 1 orientation yaw*/
  Vector2d p; /* velocity*/
  MatrixXf cluster;
  float orientation;

}waypoint_t;

/* formation configuraiton*/




/* formation gain*/
typedef struct form_gains
{
  float c1_alpha; /*formation gains*/
  float c2_alpha;
  float c1_beta; /* consesus gains*/
  float c2_beta;
  float c1_gamma; /* navigation gains*/
  float c2_gamma;
  float c1_theta; /* orientation gains*/
  float c1_delta; /* integral gains*/


}form_gains_t;

/* formation parameters*/
typedef struct form_param
{
  float d; /* distance between agent*/
  int formation_t; /* type must be minimum 2, 3: tuangle formation, 4 suare, 5 pentagon*/
  float k;
  float ratio; /* ratio between dist inter and obstacle*/
  float eps;
  float a;
  float b;
  float h_alpha;
  float h_beta;
  float d_obs;
  int nav_type; /* convergence approach -1, parallel approach 2*/
  int integrator; /* single/ double integrator*/
  float dt;  /* sample time*/
  float int_max; /* maximum integral*/
  float r; /* maximum allowd Neighbourhood radios, this is set based on the k ratio*/
  float r_obs; /* maximum allowd Neighbourhood radios, this is set based on the k ratio*/

}form_param_t;

/* formation configuraiton*/
typedef struct formation_config
{
  form_gains_t gain;
  form_param_t param;
}formation_config_t;



typedef Eigen::Matrix<float, 2,6> input_vector_t;
/* input vector conatains the following
 * totall input x y
 * ofr debugging I print the remaining input
 * input_formation
 * input obstacle
 * input navigation
 * input orientation
 */

 typedef Eigen::Matrix <float, 2, 1>row_vector_2d_t;

 typedef Eigen::Matrix <float,3,1> row_vector_3d_t;




#endif
