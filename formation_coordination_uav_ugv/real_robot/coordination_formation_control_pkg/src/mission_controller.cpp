#include "mission_controller.h"

/*constructor*/
missionController::missionController(ros::NodeHandle &nh):
nh(nh)
{
  this->shut_down = false;
  this->state = IDLE;
  this->shut_down = false;
  this->initialize_all_robot = false;
  this->start_formation = false;
  this->stop = false;
  this->mantain_position = true;

  /*publisher*/
  this->coordination_publisher = this->nh.advertise<coordination_formation_control_pkg::coordination>("coordination_config",1);
  this->current_configuration.stop_demo = false;
  this->current_configuration.init_formation = false;
  this->current_configuration.initialize_robot = false;
  this->current_configuration.start_orientation = false;
  this->waypoint_publisher = this->nh.advertise<coordination_formation_control_pkg::waypoint>("goal_waypoint",1);


  this->current_waypoint.vel[0] = 0;
  this->current_waypoint.vel[1] = 0;
  this->start_publishing_waypoint = false;
  this->eps = 0.01;

  this->nh.getParam("/n_obs", this->n_obs);
  std::cout << this->n_obs << std::endl;
  if (this->n_obs>0)
  {
    this->obstacle_pose.resize(3,this->n_obs);

    this->obs_state_publisher = nh.subscribe("/cf4/position", 1,&missionController::stateCallbackObsPos, this);
    this->current_waypoint.obstacle.resize(this->n_obs*3);

  }


  /*dynamic_reconfigure*/
  this->f = boost::bind(&missionController::updateCofig,this, _1,_2);
  server.setCallback(f);

  /* execute main process*/
  this->executeMainProcess();
  this->cluster_initialized = false;

}
/*destructor*/
missionController::~missionController()
{

}
/* main process*/
void missionController::executeMainProcess()
{
  ros::Rate loop_rate(50);
  /* create a list of drones*/
  int n_drones, n_ugvs;
  this->nh.getParam("/n_drones", n_drones);
  this->nh.getParam("/n_ugvs", n_ugvs);
  this->current_waypoint.cluster_pos.resize(n_ugvs*2);


  std::vector<robotState *> uavs;
  robot_ind_t uav_def;
  bool sucess_initialization;

  for (int i = 1; i <= n_drones; i++)
  {
    uav_def.robot_name = "cf";
    uav_def.t = CRAZYFLIE;
    uav_def.id = i;
    sucess_initialization = false;
    auto * uav = new robotState(uav_def, this->nh, sucess_initialization);
    uavs.push_back(uav);
    if (sucess_initialization == false)
    {
      ROS_ERROR("Initialization: %d produced an error in mission controller", uav_def.id);
      EXIT_FAILURE;
    }
  }
  /* create a list of ugvs*/
  std::vector<robotState *> ugvs;
  robot_ind_t ugv_def;
  for (int i = 1; i <= n_ugvs; i++)
  {
    ugv_def.robot_name = "omniwheel";
    ugv_def.t = OMNIWHEELS;
    ugv_def.id = i;
    sucess_initialization = false;
    auto * ugv = new robotState(ugv_def, nh, sucess_initialization);
    ugvs.push_back(ugv);
    // robot_state_t current_state;
    // current_state = ugvs[i-1]->get_state();
    // std::cout << current_state.q(0) << std::endl;
    if (sucess_initialization == false)
    {
      ROS_ERROR("Initialization: ugv  %d produced an error in mission controller", ugv_def.id);
      EXIT_FAILURE;
    }
  }


  Eigen::MatrixXf uav_state_q(2,n_drones);
  Eigen::MatrixXf uav_state_p(2,n_drones);
  Eigen::MatrixXf ugv_state_q(2,n_ugvs);
  Eigen::MatrixXf ugv_state_p(2,n_ugvs);
  Eigen::MatrixXf current_cluster(2,n_ugvs);
  row_vector_2d_t current_centroid_vel,goal_centroid,current_centroid, error;
  float dist;
  int cluster_ind = 0;
  // std::vector<Point2f> previous_center;
  // for(int i = 0; i<n_ugvs; i++)
  // {
  //   Point2f center;
  //   center.x = ugv_state_q(0,i);
  //   center.y = ugv_state_q(1,i);
  //   previous_center.push_back(center);
  // }
  Mat previous_labels;
  previous_labels = cv::Mat(n_drones,1,CV_32S);

  for (int i = 0; i<n_drones; i++)
  {
    previous_labels.at<int>(i,0) = i;
  }
  Eigen::MatrixXf previous_centroids(2,n_ugvs);

for (int i = 0; i ++; i < n_ugvs)
{
  robot_state_t current_state;
  current_state = ugvs[i-1]->get_state();
  previous_centroids.col(i) << current_state.q(0),current_state.q(1);
}

  while(ros::ok())
  {

    // std::cout <<"here" << std::endl;



    for(int i = 1; i <=n_drones; i++)
    {
      robot_state_t current_state;
      current_state = uavs[i-1]->get_state();
      uav_state_q(0, i-1) =current_state.q(0);
      uav_state_q(1, i-1) =current_state.q(1);
      uav_state_p(0, i-1) = current_state.p(0);
      uav_state_p(1, i-1) = current_state.p(1);

    }

    /*get current state of all the ugv*/
    for(int i = 1; i <=n_ugvs; i++)
    {
      robot_state_t current_state;
      current_state = ugvs[i-1]->get_state();
      ugv_state_q(0, i-1) =current_state.q(0);
      ugv_state_q(1, i-1) =current_state.q(1);
      ugv_state_p(0, i-1) = current_state.p(0);
      ugv_state_p(1, i-1) = current_state.p(1);
      // std::cout << ugv_state_q(0, i-1) << std::endl;

    }

    /* get position of current obstacle*/
    if(this->n_obs>0)
    {
      for(int i = 0; i < this->n_obs; i++)
      {
        this->current_waypoint.obstacle[i*3] = this->obstacle_pose(0,i);
        this->current_waypoint.obstacle[i*3+1] = this->obstacle_pose(1,i);
        this->current_waypoint.obstacle[i*3+2] = 0.25; // radius
        // std::cout << t§his->current_waypoint.obstacle[i*3] << " " << this->current_waypoint.obstacle[i*3+1] << std::endl;
      }
    }




    if(this->shut_down)
    {
      // stop all process here

      break;
    }
    else
    {
      switch (this->state) {
        case IDLE:
          if(this->initialize_all_robot)
          {
            this->current_configuration.initialize_robot = true;
            ROS_INFO("Mission_controller State: INITILIAZE_ROBOT");
            this->state = INITILIAZE_ROBOT;
            this->current_configuration.initialize_robot = true;
            this->current_configuration.stop_demo = false;
            this->current_configuration.init_formation = false;
          }
          else
          {
            this->state = IDLE;
          }

          break;
        case INITILIAZE_ROBOT:
          if(this->start_formation)
          {
            this->current_configuration.init_formation = true;
            this->current_configuration.initialize_robot = true;
            this->current_configuration.stop_demo = false;

            ROS_INFO("Mission_controller State: START_FORMATION");
            /*make sure that we mantain current entroid if no centorid has been sent*/
            // this->current_waypoint.pos[0] = uav_state_q.row(0).mean();
            // this->current_waypoint.pos[1]  = uav_state_q.row(1).mean();


            this->state = START_FORMATION;
          }
          else
          {
            this->state = INITILIAZE_ROBOT;
          }
          if(this->stop)
          {
            this->current_configuration.stop_demo = true;
            this->current_configuration.init_formation = false;
            this->current_configuration.initialize_robot = false;
            if(this->shut_down)
            {
              this->state = END;
            }
          }

        break;
        case START_FORMATION:
        /* testint*/
        // uav_state_q(0, 1-1) =0;
        // uav_state_q(1, 1-1) =0;
        // uav_state_p(0, 1-1) =0;
        // uav_state_p(1, 1-1) = 0;
        //
        // uav_state_q(0, 2-1) =1;
        // uav_state_q(1, 2-1) =0;
        // uav_state_p(0, 2-1) = 0;
        // uav_state_p(1, 2-1) = 0;
        //
        // // uav_state_q(0, 3-1) =0;
        // // uav_state_q(1, 3-1) =1;
        // // uav_state_p(0, 3-1) = 0;
        // // uav_state_p(1, 3-1) = 0;
        //
        // // uav_state_q(0, 4-1) =1;
        // // uav_state_q(1, 4-1) =1;
        // // uav_state_p(0, 4-1) = 0;
        // // uav_state_p(1, 4-1) = 0;
        // //
        // ugv_state_q(0, 1-1) =-1.5;
        // ugv_state_q(1, 1-1) =-0.5;
        // ugv_state_p(0, 1-1) = 0;
        // ugv_state_p(1, 1-1) =0;

        // ugv_state_q(0, 2-1) =1.5;
        // ugv_state_q(1, 2-1) =-0.5;
        // ugv_state_p(0, 2-1) = 0;
        // ugv_state_p(1, 2-1) =0;

        /* compute waypoint formation*/
        /* initial waypoint will be the centorid of the current configuration*/
        if(n_drones >0)
        {


          current_centroid_vel(0) = uav_state_p.row(0).mean();
          current_centroid_vel(1) = uav_state_p.row(1).mean();
          current_centroid(0) = uav_state_q.row(0).mean();
          current_centroid(1) = uav_state_q.row(1).mean();
          goal_centroid(0) = current_waypoint.pos[0];
          goal_centroid(1) = current_waypoint.pos[1];

          error = (goal_centroid-current_centroid);
          dist = error.norm();
          }
          else
          {
            ROS_ERROR("Num of drones should be more than 0");

          }





        /* compute cluster*/
        if(n_drones >0)
        {
          if (n_ugvs >0)
          {
            // std::cout << "outside before previous lable" << previous_labels << std::endl;

            // current_cluster = this->compute_cluster(uav_state_q, ugv_state_q, n_ugvs,n_drones);
            current_cluster = this->compute_cluster(uav_state_q, ugv_state_q, n_ugvs,n_drones, previous_labels, previous_centroids);
            previous_centroids = current_cluster;
            // std::cout << "outside after previous lable" << previous_labels <<std::endl;
            /* assign clusters*/
            cluster_ind = 0;
            // if(n_ugvs > 0)
            // {
              for(int i = 0; i < n_ugvs; i++)
              {

                this->current_waypoint.cluster_pos[cluster_ind] = current_cluster(0,i);
                this->current_waypoint.cluster_pos[cluster_ind+1] = current_cluster(1,i);
                // std::cout << i << ":   " << this->current_waypoint.cluster_pos[cluster_ind] << " " << this->current_waypoint.cluster_pos[cluster_ind+1] << std::endl;
                // previous_center[i].x =  current_cluster(0,i);
                // previous_center[i].y = current_cluster(1,i);
                cluster_ind +=2;
              }
              this->current_waypoint.cluster_vel[0] = current_centroid_vel(0);
              this->current_waypoint.cluster_vel[1] = current_centroid_vel(1);
            // }


          }
        }






        /* update waypoints*/
        if (start_publishing_waypoint)
        {
          // std::cout << dist << std::endl;
          /* send goals*/
          if (dist < this->eps) // waypoint reached TODO, this is hard coded, not a good idea
          {
            // std::cout << "here" << std::endl;
            if(this->mantain_position == false)
            {
              this->current_waypoint.stop = true;
            }
            else
            {
              this->current_waypoint.stop = false;
              /* if we want to mantain centroid, vel to 0*/
              this->current_waypoint.vel[0] = 0;
              this->current_waypoint.vel[1] = 0;

            }
          }
          waypoint_publisher.publish(this->current_waypoint);
        }

        if(this->stop)
        {
          this->current_configuration.stop_demo = true;
          this->current_configuration.init_formation = false;
          this->current_configuration.initialize_robot = false;
          if(this->shut_down)
          {
            this->state = END;
          }
        }


        break;

        case END:
          this->state = IDLE;
          this->shut_down = true;
          ROS_INFO("Mission_controller State: END");

        break;
      }
    }

    this->coordination_publisher.publish(this->current_configuration);

    ros::spinOnce();
    loop_rate.sleep();
  }

}

void missionController::updateCofig(coordination_formation_control_pkg::missionControllerConfig &config, uint32_t level)
{
  ROS_INFO("mission controller: reconfigure callback");
  // demo
  if(config.initialize_all_robot)
  {
    this->initialize_all_robot = true;
    config.initialize_all_robot = false;
  }
  if(config.stop)
  {
    this->stop = true;
    config.stop = false;
  }
  if(config.start_formation)
  {
    this->start_formation = true;
    config.start_formation = false;
  }
  if(config.shut_down)
  {
    this->shut_down = true;
    config.shut_down = false;
  }
  if(config.update_waypoint)
  {
    config.update_waypoint = false;
    current_waypoint.pos[0] = config.x;
    current_waypoint.pos[1] = config.y;
    current_waypoint.orientation = config.orientation;
    current_waypoint.vel[0] = config.vx;
    current_waypoint.vel[1] = config.vy;
    this->mantain_position = config.mantain_position;
    this->start_publishing_waypoint = true;
    config.mantain_position = false;


  }
   this->current_configuration.start_orientation =config.start_orientation;


}

Eigen::MatrixXf missionController::compute_cluster(Eigen::MatrixXf uav_q, Eigen::MatrixXf ugv_q,int num_ugvs, int num_uavs,Mat &previous_labels,Eigen::MatrixXf & previous_centroids)
{

  /* only implemented for two ugv*/
  if(num_ugvs > 2 || num_ugvs <1)
  {
    ROS_INFO("n_ugv is only implemented for 2 ugv, should modify for more ugvs");

    this->stop = true;


  }
  else
  {

    std::vector<Point2f> centers;;
    // for (int i = 0; i <num_ugvs; i++)
    // {
    //   /* make intial ugv be the centroid */
    //   Point2f center;
    //   center.x = ugv_q(0,i);
    //   center.y =  ugv_q(1,i);
    //   centers.push_back(center);
    // }

    /* points*/
    std::vector<Point2f> points;

    for(int t = 0; t < num_uavs; t++)
    {
      Point2f point;
      point.x = uav_q(0,t);
      point.y =  uav_q(1,t);
      points.push_back(point);
    }
    // ROS_INFO("*****************************************Here");

    Mat labels = previous_labels;
    /* set up labels*/
    int cluster_cout = num_ugvs;
    double compactness = kmeans(points, cluster_cout, labels,
            TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 0, 0.1),
               10,  KMEANS_PP_CENTERS, centers);

    // std::cout << "previous lable" << previous_labels << "current labels" << labels << std::endl;
    // std::cout << "Current center" << centers << std::endl;
    previous_labels = labels;
    /* decide to change or not*/
    /* we calculate the enegy to mantain the current confguration or change*/
    Eigen::MatrixXf centroids(2,num_ugvs);
    for(int p = 0; p <num_ugvs; p++)
    {
      centroids(0,p) = centers[p].x;
      centroids(1,p) = centers[p].y;
    }

    // std::cout << "Current center set" << centroids << std::endl;






    /* this is intended for two ugvs, it computed the energy to mantain leader cluster,
    and change, if energy in both ugv to change is higher, the switch is perfomred*/
    if(num_ugvs == 2)
    {
      float man1 = (centroids.col(0) - previous_centroids.col(0)).norm();
      float man2 = (centroids.col(1) - previous_centroids.col(0)).norm();
      float cha1 = (centroids.col(1) - previous_centroids.col(1)).norm();
      float cha2 = (centroids.col(0) - previous_centroids.col(1)).norm();

      if( cha1 < man1 && cha2 <man2)
      {

          Eigen::MatrixXf dummy_centroid(2,num_ugvs);
          dummy_centroid = centroids;
          /* switch centroids*/
          centroids.col(0) = dummy_centroid.col(1);
          centroids.col(1) = dummy_centroid.col(0);



      }
      else
      {
        // bad ondding
        if(this->cluster_initialized)
        {
          centroids = previous_centroids;

        }
      }

    }
    previous_centroids = centroids;
    // std::cout << "final centroids" << centroids << std::endl;
    this->cluster_initialized = true;
    return centroids;

  }

}

// calback for obstacle pose
void missionController::stateCallbackObsPos(const crazyswarm::GenericLogDataConstPtr &pose_msg)
{
  // not the best implementation
  this->obstacle_pose(0,0) = pose_msg->values[0]/1000.0;
  this->obstacle_pose(1,0)  = pose_msg->values[1]/1000.0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mission_controller_node");
  ros::NodeHandle nh("");
  ROS_INFO("Initialize");
  missionController mission_controller(nh);
  return 0;
}
