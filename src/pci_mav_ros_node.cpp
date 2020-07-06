#include <ros/ros.h>
#include "pci_mav/pci_mav.h"
#include "planner_control_interface/planner_control_interface.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "pci_mav_ros_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  std::shared_ptr<explorer::PCIManager> pci_manager =
      std::make_shared<explorer::PCIMAV>(nh, nh_private);
  explorer::PlannerControlInterface interface(nh, nh_private, pci_manager);

  ros::spin();
  return 0;
}
