#include "rosbag/bag.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "ros/ros.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv,"test");
  ros::NodeHandle("");
  // %Tag(WRITE)%
  rosbag::Bag bag;
  bag.open("/home/spot/steven_master_ws/src/coordination_formation_control_pkg/results/test.bag", rosbag::bagmode::Write);

  std_msgs::String str;
  str.data = std::string("foo");

  std_msgs::Int32 i;
  i.data = 42;

  bag.write("chatter", ros::Time::now(), str);
  bag.write("numbers", ros::Time::now(), i);

  bag.close();
  // %EndTag(WRITE)%
}
