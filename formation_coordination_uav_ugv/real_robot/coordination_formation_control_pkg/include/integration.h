#ifndef RUNGE_KUTTE_INTEGRATION_H
#define RUNGE_KUTTE_INTEGRATION_H
#include "shared_definitions.h"

class integration
{
private:

Eigen::Vector2d accel;
Eigen::Vector2d vel;
Eigen::Vector2d current_vel;

float dt;


public:
  integration(float ax_initial, float ay_initial,float dt)
  {
    this->accel(0) = ax_initial;
    this->accel(1) = ay_initial;
    this->vel(0) = 0;
    this->vel(1) = 0;
    this->current_vel(0) = 0;
    this->current_vel(1) = 0;
    this->dt = dt;

  }
  ~integration()
  {

  }

  Eigen::Vector2d integrate(float ax, float ay, float vx, float vy)
  {
    /* returns the vx and vy*/
    this->accel(0) = ax;
    this->accel(1) = ay;
    this->current_vel(0) = vx;
    this->current_vel(1) = vy;
    this->vel = this->vel + this->accel*dt;
    return this->vel;
  }


};



#endif
