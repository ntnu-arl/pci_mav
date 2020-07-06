#ifndef pci_mav_H_
#define pci_mav_H_

#include <cmath>
#include <mav_msgs/Status.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "planner_control_interface/pci_manager.h"
#include "planner_msgs/RobotStatus.h"

namespace explorer {

class PCIMAV : public PCIManager {
 public:
  PCIMAV(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);

  bool loadParams(const std::string ns);
  bool initialize();
  bool initMotion();
  bool executePath(const std::vector<geometry_msgs::Pose> &path,
                   std::vector<geometry_msgs::Pose> &modified_path,
                   ExecutionPathType path_type = ExecutionPathType::kLocalPath);
  void setState(const geometry_msgs::Pose &pose);
  void setVelocity(double v);
  double getVelocity(ExecutionPathType path_type);
  bool goToWaypoint(geometry_msgs::Pose &pose);
  bool planAhead() { return ((planner_trigger_lead_time_ > 0) ? true : false); }
  void allocateYawAlongPath(std::vector<geometry_msgs::Pose> &path) const;
  void allocateYawAlongFistSegment(
      std::vector<geometry_msgs::Pose> &path) const;

 private:
  ros::Subscriber status_sub_;
  ros::Publisher robot_status_pub_;
  ros::Publisher trajectory_pub_;
  RunModeType run_mode_;
  bool init_motion_enable_;
  double v_max_;
  double v_init_max_;
  double v_homing_max_;
  double v_narrow_env_max_;
  double yaw_rate_max_;
  double dt_;
  double planner_trigger_lead_time_;
  std::string world_frame_id_;
  bool smooth_heading_enable_;

  double init_z_takeoff_;
  double init_z_drop_;
  double init_x_forward_;

  bool concatenate_path_enable_ = true;
  std::vector<geometry_msgs::Pose> executing_path_;

  trajectory_msgs::MultiDOFJointTrajectory samples_array_;
  mav_msgs::EigenTrajectoryPoint trajectory_point_;
  trajectory_msgs::MultiDOFJointTrajectoryPoint trajectory_point_msg_;
  int n_seq_;

  // Add offset to account for constant tracking error of controller of aerial
  // robot --> for simulation only.
  const double z_control_offset = 0.25;
  // Safety guards for changing velocity of the robot online.
  const double kVelMax = 2.0;
  const double kVelMin = 0.2;

  visualization_msgs::MarkerArray::Ptr generateTrajectoryMarkerArray(
    const trajectory_msgs::MultiDOFJointTrajectory &traj) const;
  void interpolatePath(const std::vector<geometry_msgs::Pose> &path,
                       std::vector<geometry_msgs::Pose> &path_res);
  void interpolatePath(const std::vector<geometry_msgs::Pose> &path,
                       std::vector<geometry_msgs::Pose> &path_res, double v_max,
                       double yaw_rate_max);
  bool reconnectPath(const std::vector<geometry_msgs::Pose> &path,
                     std::vector<geometry_msgs::Pose> &path_new);

  void statusCallback(const mav_msgs::Status &status);
  inline double diffPos(const geometry_msgs::Pose &p1,
                        const geometry_msgs::Pose &p2) {
    return std::sqrt(
        (p1.position.x - p2.position.x) * (p1.position.x - p2.position.x) +
        (p1.position.y - p2.position.y) * (p1.position.y - p2.position.y) +
        (p1.position.z - p2.position.z) * (p1.position.z - p2.position.z));
  }
};

}  // namespace explorer
#endif
