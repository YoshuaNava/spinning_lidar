#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include "gazebo/common/common.hh"
#include "gazebo/physics/physics.hh"

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>

namespace gazebo {

class irInterruptPlugin : public ModelPlugin {
 private:
  const double M_2PI = 2.0 * M_PI;
  std::string joint_name_;
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
  irInterruptPlugin() : ModelPlugin() {}

  ~irInterruptPlugin() {}

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) {
    printf("Loading IR interrupt Gazebo plugin\n");
    model_ = _model;
    world_ = model_->GetWorld();
    if (_sdf->HasElement("joint")) {
      joint_name_ = _sdf->GetElement("joint")->Get<std::string>();
    }
    joint_ = model_->GetJoint(joint_name_);

    if (_sdf->HasElement("half_cycle_int")) {
      half_cycle_int_ = _sdf->GetElement("half_cycle_int")->Get<bool>();
    }

    if (_sdf->HasElement("angle_tol")) {
      angle_tol_ = _sdf->GetElement("angle_tol")->Get<double>();
    }
    connection_handler_ = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&irInterruptPlugin::UpdateChild, this));

    if (!ros::isInitialized()) {
      int argc = 0;
      char** argv = NULL;
      ros::init(
          argc, argv, "ir_interrupt_plugin",
          ros::init_options::NoSigintHandler);
    }
    nh_ptr_.reset(new ros::NodeHandle());

    ir_interrupt_pub_ =
        nh_ptr_->advertise<std_msgs::Empty>("spinning_lidar/ir_interrupt", 1);
    spinning_lidar_joint_pub_ =
        nh_ptr_->advertise<sensor_msgs::JointState>("joint_states", 1);
    recent_interrupt_ = true;
  }

  void checkAngleAndPublish(double angle, double goal_angle) {
    double angle_dist = fabs(angle - goal_angle);
    if (angle_dist <= angle_tol_) {
      if (!recent_interrupt_) {
        std_msgs::Empty empty_msg;
        ir_interrupt_pub_.publish(empty_msg);
        // printf("************************************************\n");
        printf("LIDAR angle = %f\n", joint_->Position(0));
        printf("LIDAR normalized angle = %f\n", angle);
        printf("LIDAR angle dist = %f\n", angle_dist);
        printf("LIDAR angle tol = %f\n", angle_tol_);
      }
      recent_interrupt_ = true;
    } else {
      recent_interrupt_ = false;
    }
  }

  void UpdateChild() {
    double angle = fmod(joint_->Position(0), M_2PI);
    double omega = joint_->GetVelocity(0);
    double goal_angle;
    if (half_cycle_int_) {
      goal_angle = M_PI;
      checkAngleAndPublish(goal_angle, angle);
    }

    goal_angle = M_2PI;
    checkAngleAndPublish(goal_angle, angle);

    sensor_msgs::JointState joint_state_msg;
    joint_state_msg.header.stamp = ros::Time::now();
    joint_state_msg.header.frame_id = "laser_mount";
    joint_state_msg.name.push_back(joint_name_);
    joint_state_msg.position.push_back(angle);
    joint_state_msg.velocity.push_back(omega);

    spinning_lidar_joint_pub_.publish(joint_state_msg);
  }
};
GZ_REGISTER_MODEL_PLUGIN(irInterruptPlugin)
}  // namespace gazebo