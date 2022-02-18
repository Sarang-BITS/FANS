#include <pci/drone.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pci_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  Drone d(nh, nh_private);
  ros::spin();
}
