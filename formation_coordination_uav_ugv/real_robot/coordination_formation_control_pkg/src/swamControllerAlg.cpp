#include "swamControllerAlg.h"




swamControllerAlg::swamControllerAlg(robot_ind_t indentification, formation_config_t formation_config)
{

  // ROS_INFO("Here");

  this->form_param = formation_config;
  this->robot_id = indentification;
  this->integral_error << 0,0;
  this->previous_integral_error << 0,0;
  this->range_sat << -formation_config.param.int_max,formation_config.param.int_max;

  /* we want to ensure the following condition is met
   * c1_alpha < c1_gamma < c1_beta*/
   // if((this->form_param.gain.c1_alpha > this->form_param.gain.c1_gamma) || (this->form_param.gain.c1_gamma > this->form_param.gain.c1_beta))
   // {
   //   ROS_ERROR("Swam controller: Following condition not met!!!!!! c1_alpha < c1_gamma < c1_beta");
   // //   EXIT_FAILURE;
   // }
   /* make sure that the following codition is met
    *  0 < a <= b*/
    if((0 > this->form_param.param.a) || (this->form_param.param.a > this->form_param.param.b))
    {
      ROS_ERROR("Swam controller: Following condition not met!!!!!! 0 < a <= b");
      EXIT_FAILURE;
    }
    ROS_DEBUG("Navigation term %d", this->form_param.param.nav_type);

    if((this->form_param.param.nav_type != 1) && (this->form_param.param.nav_type != 2))
    {
      ROS_ERROR("Swam Controller: Navigational_type should be 1-converge approach or 2 - parallel approach");
      EXIT_FAILURE;
    }

    /* compute the c2 for each gains*/
    if(this->form_param.param.nav_type != 2)
    {
    this->form_param.gain.c2_alpha = 2*sqrt(this->form_param.gain.c1_alpha);
    this->form_param.gain.c2_gamma = 2*sqrt(this->form_param.gain.c1_gamma);
    this->form_param.gain.c2_beta = 2*sqrt(this->form_param.gain.c1_beta);
    }
    /* formation Neighbourhood distance*/
    this->form_param.param.r = this->form_param.param.k*this->form_param.param.d;
    /* set the obstcale avoidance parameters*/
    this->form_param.param.d_obs = this->form_param.param.d*this->form_param.param.ratio;
    this->form_param.param.r_obs = this->form_param.param.k*this->form_param.param.d_obs;
    /* ensure there is no collision between robots and obstacles*/
    float total_safety_distance = this->robot_id.robot_size/2 + this->robot_id.safe_ditance;
    if((total_safety_distance > this->form_param.param.d) || (total_safety_distance > this->form_param.param.d_obs))
    {
      ROS_ERROR("Swam controller: The distance between the robot and other robot or obstacle might result in collision: total_safety_distance: %f robot_size: %f safe_ditance: %f",
       total_safety_distance, this->robot_id.robot_size, this->robot_id.safe_ditance);
      EXIT_FAILURE;
    }

    this->N = this->form_param.param.formation_t;
    this->form_type = formation_matrix_t(this->form_param.param.formation_t);

    /* formation configuration steps*/
    this->configureFormation();



  ROS_INFO("Robot %s_%d swam controller initialized", this->robot_id.robot_name.c_str(), this->robot_id.id);

}


swamControllerAlg::~swamControllerAlg()
{

}

void swamControllerAlg::configureFormation()
{
  /* this consists of predefined formation*/
  float d = this->form_param.param.d;
  float h = 0;

  if(this->form_type == PAIR)
  {
    this->iad = Eigen::MatrixXf(2,2);
    this->iad << 0, d,
                 d, 0;
  }
  else if(this->form_type == TRIANGLE)
  {
    this->iad = Eigen::MatrixXf(3,3);

    this->iad << 0, d,  d,
                 d, 0, d,
                 d, d, 0;

  }
  else if(this->form_type == SQUARE)
  {
    h = sqrt(2*d*d); // diagonal distance
    this->iad = Eigen::MatrixXf(4,4);
    this->iad << 0, d, h, d,
                  d,0,d,h,
                  h,d,0,d,
                  d,h,d,0;
  }
  else if(this->form_type == PENTAGON)
  {
    h = d + 2*d*cos(72*M_PI/180.0); // diagonal distance
    this->iad = Eigen::MatrixXf(5,5);
    this->iad << 0, d, h, h, d,
                  d,0,d,h,h,
                  h,d,0,d,h,
                  h,h,d,0,d,
                  d,h,h,d,0;
  }
  else
  {
    ROS_ERROR("Swam Controller: Wrong formation type: 2 PAIR, 3 TRIANGLE, 4 SQUARE, 5 PENTAGON ");
    EXIT_FAILURE;
  }
  ROS_INFO("Swam Controller: interagent matrix");
  std::cout << this->iad << std::endl;


}


/* main controller implementation*/
input_vector_t swamControllerAlg::controller(Eigen::MatrixXf q, Eigen::MatrixXf p, waypoint_t ref, std::vector<obstacle_t> obs)
{
  /* implementation intended for control in the x y plane*/
  /* make sure that the number of agent match the matrix*/
  // ROS_INFO("Swam Controller: Entered function");
  if((q.cols() != this->N ) || (p.cols() != this->N ))
  {
    ROS_ERROR("Swam controller: p and q matrix is not correct, Number of agent: %d p/q column size: %d/%d", this->N, (int)q.cols(), (int)p.cols());
    EXIT_FAILURE;
  }
  /* formation computation*/
  row_vector_2d_t qi,pi,qj,pj;
  // std::cout << this->robot_id.id_m << std::endl;
  // ROS_INFO("Swam Controller: Sort inccompleted 1");

  // std::cout << q(0,this->robot_id.id_m) << std::endl;
  // std::cout << p(0,this->robot_id.id_m) << std::endl;


  qi(0) = q(0,this->robot_id.id_m);
  qi(1) = q(1,this->robot_id.id_m);
  pi(0) = p(0,this->robot_id.id_m);
  pi(1) = p(1,this->robot_id.id_m);
  // std::cout << "*******************************************" << std::endl;
  //
  // std::cout << " " << qi << std::endl;
  // std::cout << " " << pi << std::endl;
  // std::cout << "*******************************************" << std::endl;

  // ROS_INFO("Swam Controller: Sort inccompleted");

  std::vector<std::pair<int, float>> neighbour_sorted = this->sortNeigbour(q, qi);
  std::cout << std::fixed;
  std::cout << std::setprecision(15);
  // for (int i = 0; i <this->N; i++)
  // {
  //   std::cout << neighbour_sorted[i].first << std::endl;
  //   std::cout << neighbour_sorted[i].second << std::endl;
  //
  // }
  // ROS_INFO("Swam Controller: Sort completed");

  row_vector_2d_t u_formation;//(0.0,0.0);
  row_vector_2d_t grad_term;//(0.0,0.0);
  row_vector_2d_t cons_term;//(0.0,0.0);
  row_vector_2d_t deriv_term;//(0.0,0.0);
  row_vector_2d_t error_grad_term;//(0.0,0.0);
  u_formation << 0.0,0.0;
  grad_term << 0.0,0.0;
  cons_term << 0.0,0.0;
  deriv_term << 0.0,0.0;
  error_grad_term << 0.0,0.0;
  // ROS_INFO("Swam Controller: Initialization completed");


  float fi,aij,d;
  row_vector_2d_t nij;


  for(int i = 0; i < this->N; i++)
  {
    /* compute input for those in the neighbour*/
    if(neighbour_sorted[i].second > this->form_param.param.r)
    {
      break;
    }
    /* only for neighbours*/
    if(i != this->robot_id.id_m)
    {
      qj(0) =q(0,i);
      qj(1) =q(1,i);
      pj(0) =p(0,i);
      pj(1) =p(1,i);
      d = this->iad(this->robot_id.id_m,i); /* distance between neighbours*/
      // std::cout << "*******************************************" << std::endl;
      // std::cout << "first" << qj << pj << std::endl;
      // std::cout << d << std::endl;
      // std::cout << "*******************************************" << std::endl;

      // compute the gradient term
      fi = this->computeFi(qj,qi,this->form_param.param.a,this->form_param.param.b,this->form_param.param.h_alpha,
        this->form_param.param.eps,this->form_param.param.r,d);
      nij = this->computeNij(qj,qi,this->form_param.param.eps);
      grad_term = grad_term + fi*nij;
      // consesus term
      if (this->form_param.param.integrator == 2)
      {
        aij = this->computeAij(qj,qi,this->form_param.param.r,this->form_param.param.eps,this->form_param.param.h_alpha);
        cons_term = cons_term + aij*(pj-pi);
      }

    }

  }
  error_grad_term = grad_term;

  u_formation = grad_term*this->form_param.gain.c1_alpha+cons_term*this->form_param.gain.c2_alpha;
  // ROS_INFO("Swam Controller: formation completed");


  /* obstacle formation*/
  row_vector_2d_t u_obstacle;
  u_obstacle << 0.0,0.0; //(0.0,0.0);

  if (obs.size() != 0)
  {
    /* if we have obstacle*/
    size_t obs_size = obs.size();
    // std::cout << obs_size << std::endl;

    row_vector_2d_t grad_term_obs;//(0.0,0.0);
    row_vector_2d_t cons_term_obs;//(0.0,0.0);
    grad_term_obs << 0.0,0.0;
    cons_term_obs << 0.0,0.0;
    u_obstacle << 0.0,0.0;
    row_vector_2d_t yk, ak, q_hat_i, p_hat_i;
    float Rk, mu;
    Eigen::MatrixXf a__;

    for(int o = 0; o<obs_size; o++)
    {
      yk(0) = obs[o].q(0);
      yk(1) = obs[o].q(1);
      Rk = obs[o].radius;
      mu = Rk/(qi-yk).norm();
      ak = (qi-yk)/(qi-yk).norm();
      a__ = ak*ak.transpose();
      Eigen::Matrix2f P;
      P << 1.0, 0.0, 1.0,0.0;
      q_hat_i = mu*qi+(1-mu)*yk;
      p_hat_i = mu*(P-a__)*pi;
      float d2obs = (qi-q_hat_i).norm();
      // std::cout << d2obs << std::endl;
      // std::cout << this->form_param.param.r_obs << std::endl;

      if (d2obs < this->form_param.param.r_obs) /* compute for neighbour obstacles*/
      {
        /* gradient term*/
        float fi_beta = this->computeFiB(q_hat_i,qi,this->form_param.param.h_beta,this->form_param.param.d_obs,this->form_param.param.eps);
        row_vector_2d_t nij = this->computeNij(q_hat_i, qi, this->form_param.param.eps);
        grad_term_obs = grad_term_obs+fi_beta*nij;

        /*consesus term*/
        if (this->form_param.param.integrator == 2)
        {
          float bij = this->computeBij(q_hat_i,qi,this->form_param.param.d_obs,this->form_param.param.eps, this->form_param.param.h_beta);
          cons_term_obs = cons_term_obs + bij*(p_hat_i-pi);

        }
      }
    }
    u_obstacle = this->form_param.gain.c1_beta*grad_term_obs + this->form_param.gain.c2_beta*cons_term_obs;



  }


  // ROS_INFO("Swam Controller: obstacle completed");

  /* navigational term*/
  row_vector_2d_t u_navigation;
  u_navigation << 0.0,0.0;
  row_vector_2d_t q_ref,p_ref;
  q_ref << ref.q(0), ref.q(1);
  p_ref << ref.p(0), ref.p(1);
  row_vector_2d_t centroid_q, centroid_p;
  centroid_q << q.row(0).mean(), q.row(1).mean();
  centroid_p << p.row(0).mean(), p.row(1).mean();
  if (this->robot_id.leader == true)
  {



    if(this->form_param.param.nav_type == 1)
    {
      u_navigation = -this->form_param.gain.c1_gamma*this->sigmaOne((qi-q_ref));
      if(this->form_param.param.integrator == 2)
      {
       u_navigation = u_navigation  -this->form_param.gain.c2_gamma*(pi-p_ref);
      }
      // ROS_ERROR("Here: %f:%f "  (qi-q_ref)(0), (qi-q_ref)(1)l;
    }
    else
    {
      // std::cout << centroid_q << std::endl;
      // std::cout << centroid_p << std::endl;
    u_navigation = -this->form_param.gain.c1_gamma*this->sigmaOne(centroid_q-q_ref)-this->form_param.gain.c2_gamma*(centroid_p-p_ref);
    // std::cout <<"q***********" <<  centroid_q-q_ref << std::endl;
    // std::cout <<"p***********" <<  centroid_p-p_ref << std::endl;

    }
  }

  // ROS_INFO("Swam Controller: Navigation completed");


  // orienation term
  row_vector_2d_t u_orientation;
  u_orientation << 0.0,0.0;
  if (ref.orientation != NOORIENTATION)
  {
    // ROS_INFO("ORinetaotn here");
    float angle = ref.orientation * M_PI /180;
    // convert to 3d
    row_vector_3d_t desired_vector(cos(angle), sin(angle),0);
    row_vector_3d_t q_centroid, q_leader;
    if (this->robot_id.leader == true)
    {
      q_centroid << centroid_q(0), centroid_q(1),0;
      q_leader << q(0,0),q(1,0),0;
    }
    else
    {
      q_centroid << q(0,0),q(1,0),0;//centroid_q(0), centroid_q(1),0;
      q_leader << q(0,1),q(1,1),0;
    }
    float dlc = (q_leader-q_centroid).norm(); // distance from centroid to leader drone
    row_vector_3d_t lcv  = q_leader-q_centroid; // vector from centroid to leader
    row_vector_3d_t dcv = dlc*desired_vector; // vector from centroid to desired final pose of leader
    float acl = atan2(lcv(1), lcv(0)); // angle of vector from centroid to leadership dron
    float acc = atan2(dcv(1), dcv(0)); // angle of vector from fron centroid to final drone pose
    if(this->robot_id.leader == false)
    {
      // not necessaty TODO improve to get angle from 0 to 360
      acl = atan(lcv(1)/lcv(0)); // angle of vector from centroid to leadership dron
      acc = atan(dcv(1)/dcv(0)); //
    }
    float alpha = acc-acl; // angle between vector lcv and dcv
    // we want to compute the rotation acceleration
    // direction
    row_vector_3d_t q_current(qi(0), qi(1),0); // current drone pose
    row_vector_3d_t cq_hat = q_current - q_centroid; // vector from cwentroid to current drone
    row_vector_3d_t pot(0,0,-alpha);
    row_vector_3d_t rot_d = cq_hat.cross(pot) ;// rotation direction

    if(rot_d.norm() == 0)
    {
      rot_d << 0,0,0;
    }
    else
    {
      rot_d = rot_d/rot_d.norm(); // convert to unit vector
    }

    row_vector_2d_t final_rot_d(rot_d(0), rot_d(1));
    // if(this->robot_id.id_m == 0)
    // {
      u_orientation = this->form_param.gain.c1_theta*abs(alpha)*final_rot_d;
      // row_vector_2d_t range_ori;
      // range_ori << 0.1, -0.1;

      // u_orientation = this->saturate(u_orientation, range_ori);

      // std::cout << alpha << std::endl;
    //
    // }

  }

  // ROS_INFO("Swam Controller: orientation completed");

  /* integral term*/
  row_vector_2d_t u_integral;
  u_integral << 0.0,0.0;
  this->integral_error =  this->integral_error + (error_grad_term)*this->form_param.param.dt;
  u_integral = this->form_param.gain.c1_delta *this->integral_error;
  u_integral = this->saturate(u_integral, range_sat);

  // std::cout << u_integral << std::endl;

  // ROS_INFO("Swam Controller: Integral completed");


  row_vector_2d_t total_input;
  total_input = u_formation + u_navigation  + u_orientation+u_obstacle+u_integral;
  input_vector_t input2send;
  input2send.col(0) << total_input;
  input2send.col(1) = u_formation;
  input2send.col(2) = u_obstacle;
  input2send.col(3) = u_navigation;
  input2send.col(4) = u_orientation;
  input2send.col(5) = u_integral;
  // ROS_INFO("Swam Controller: input completed");
  // std::cout << total_input <<std::endl;

  return (input2send);



}
// sort the neighbour distances
typedef std::function<bool(std::pair<int, float>, std::pair<int, float>)> Comparator;
Comparator compFunctor =
        [](std::pair<int, float> elem1, std::pair<int, float> elem2) {
            return elem1.second < elem2.second;
        };


std::vector<std::pair<int, float>> swamControllerAlg::sortNeigbour(Eigen::MatrixXf M, row_vector_2d_t q)
{
  // ROS_INFO("Swam Controller: Sort inside 1");

  std::vector<std::pair<int, float>> neighbour_dist;
  float dist;
  /* Compute the distance*/
  row_vector_2d_t qj;
  for (int i = 0; i <this->N;i++)
  {
    qj(0) = M(0,i);
    qj(1) = M(1,i);
    dist = (q-qj).norm();
    // std::cout << std::fixed;
    // std::cout << std::setprecision(15);
    // std::cout << dist << std::endl;
    neighbour_dist.emplace_back(std::pair<int,float>(i, dist));
  }
  // ROS_INFO("Swam Controller: Sort inside 1");

  /* sort the distance*/
  std::sort(neighbour_dist.begin(), neighbour_dist.end(), compFunctor);
  // ROS_INFO("Swam Controller: Sort inside 1");

  return neighbour_dist;

}

float swamControllerAlg::computeFi(row_vector_2d_t qj, row_vector_2d_t qi, float a, float b, float h, float eps, float r, float d)
{
  /* qj neighbour
   * qi drone itself*/
   float z_sn = this->computeSigmaNorm((qj-qi).norm(), eps);
   float r_sn = this->computeSigmaNorm(r,eps);
   float d_sn = this->computeSigmaNorm(d,eps);
   return (this->ph(z_sn/r_sn,h) * this->computefi(a,b,(z_sn-d_sn)));
}

float swamControllerAlg::computeSigmaNorm(float z, float eps)
{
  return((1.0/eps)*(sqrt(1.0+eps*z*z)-1.0));
}

float swamControllerAlg::ph(float z, float h)
{
  if((z>= 0) && (z <h))
  {
    return 1;
  }
  else if((z>=h) && (z<=1))
  {
    return(0.5*(1.0+cos(M_PI*(z-h)/(1-h))));
  }
  else
  {
    return 0;
  }
}

float swamControllerAlg::computefi(float a, float b, float z)
{
  float c = abs(a-b)/sqrt(4*a*b);
  return (0.5*(((a+b)*this->sigmaOne(z+c))+(a-b)));

}

float swamControllerAlg::sigmaOne(float z)
{
  return (z/sqrt(1.0+(z*z)));
}
row_vector_2d_t swamControllerAlg::sigmaOne(row_vector_2d_t z)
{
  float n = z.norm();
  return (z/sqrt(1.0+(n*n)));
}
row_vector_2d_t swamControllerAlg::computeNij(row_vector_2d_t qj, row_vector_2d_t qi, float eps)
{
  row_vector_2d_t z  = qj-qi;
  float n = z.norm();
  return(z/sqrt(1.0+(eps*n*n)));

}

float swamControllerAlg::computeAij(row_vector_2d_t qj, row_vector_2d_t qi, float r, float eps, float h)
{
  float z_sn = this->computeSigmaNorm((qj-qi).norm(),eps);
  float r_sn = this->computeSigmaNorm(r,eps);
  return (this->ph(z_sn/r_sn,h));

}

float swamControllerAlg::computeFiB(row_vector_2d_t q_hat_i,row_vector_2d_t  qi, float h_beta, float d_obs, float eps)
{
  float z_sn = this->computeSigmaNorm((q_hat_i-qi).norm(),eps);
  float d_obs_sn = this->computeSigmaNorm(d_obs,eps);
  return (this->ph(z_sn/d_obs_sn, h_beta)* (this->sigmaOne(z_sn-d_obs_sn)-1));
}


float swamControllerAlg::computeBij(row_vector_2d_t q_hat_i, row_vector_2d_t qi, float d_obs, float eps, float h_beta)
{
  float z_sn = this->computeSigmaNorm((q_hat_i-qi).norm(),h_beta);
  float d_obs_sn = this->computeSigmaNorm(d_obs,eps);
  return (this->ph(z_sn/d_obs_sn,h_beta));
}

row_vector_2d_t swamControllerAlg::saturate(row_vector_2d_t input, row_vector_2d_t range)
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
