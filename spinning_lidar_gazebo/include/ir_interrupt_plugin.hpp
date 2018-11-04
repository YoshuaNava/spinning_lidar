#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>

namespace gazebo {

class IrInterruptPlugin : public ModelPlugin {
 private:
  const double M_2PI = 2.0 * M_PI;
  std::string joint_name_;
  std::string frame_id_;
  bool half_cycle_int_;
  double angle_tol_;
  double recent_interrupt_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::JointPtr joint_;
  event::ConnectionPtr connection_handler_;

  // ROS transport layer
  std::unique_ptr<ros::NodeHandle> nh_ptr_;
  ros::Publisher ir_interrupt_pub_;
  ros::Publisher spinning_lidar_joint_pub_;

 public:
  IrInterruptPlugin();
  ~IrInterruptPlugin();

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  void CheckAngleAndPublish(double angle, double goal_angle);

  void UpdateChild();
};

GZ_REGISTER_MODEL_PLUGIN(IrInterruptPlugin)

}  // namespace gazebo