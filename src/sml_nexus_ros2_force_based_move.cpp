// This code was developed using a template from the gazebo_ros_package for ros2 humble 
// See https://github.com/ros2-gbp/gazebo_ros_pkgs-release/blob/release/humble/gazebo_plugins/src/gazebo_ros_force.cpp for details 

/*
 * \brief  Simple model controller that uses a twist message to move an entity on the xy plane using a PI controller.
 *
 * \author  Gregorio Marchesini (gremar@kth.com)
 *
 * \date  April 2024
 */

#include <gazebo/common/Events.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#ifdef IGN_PROFILER_ENABLE
#include <ignition/common/Profiler.hh>
#endif
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <string>

#include "sml_nexus_plugin_ros2/sml_nexus_ros2_force_based_move.hpp"

namespace gazebo_plugins
{
class GazeboRosPlanarMovePrivate
{
public:
  /// Callback to be called at every simulation iteration.
  /// \param[in] _info Updated simulation info.
  void OnUpdate(const gazebo::common::UpdateInfo & _info);

  /// Callback when a velocity command is received.
  /// \param[in] _msg Twist command message.
  void OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg);

  /// Update odometry.
  /// \param[in] _current_time Current simulation time
  void UpdateOdometry(const gazebo::common::Time & _current_time);

  /// Publish odometry transforms
  /// \param[in] _current_time Current simulation time
  void PublishOdometryTf(const gazebo::common::Time & _current_time);

  /// A pointer to the GazeboROS node.
  gazebo_ros::Node::SharedPtr ros_node_;

  /// Subscriber to command velocities
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  /// Odometry publisher
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

  /// To broadcast TF
  std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

  /// Velocity received on command.
  geometry_msgs::msg::Twist target_cmd_vel_;

  /// Keep latest odometry message
  nav_msgs::msg::Odometry odom_;

  /// Pointer to world.
  gazebo::physics::WorldPtr world_;

  /// Pointer to model.
  gazebo::physics::ModelPtr model_;

  /// Connection to event called at every world iteration.
  gazebo::event::ConnectionPtr update_connection_;

  /// Protect variables accessed on callbacks.
  std::mutex lock_;

  /// Update period in seconds.
  double update_period_;

  /// Publish period in seconds.
  double publish_period_;

  /// Last update time.
  gazebo::common::Time last_update_time_;

  /// Last publish time.
  gazebo::common::Time last_publish_time_;

  /// Odometry frame ID
  std::string odometry_frame_;

  /// Robot base frame ID
  std::string robot_base_frame_;

  /// True to publish odometry messages.
  bool publish_odom_;

  /// True to publish odom-to-world transforms.
  bool publish_odom_tf_;

  /// @brief PID gains for the controller.
  double yaw_velocity_p_gain;
  double x_velocity_p_gain;
  double y_velocity_p_gain;
  
  /// @brief Maximum velocities for the controller.
  double max_x_velocity;
  double max_y_velocity;
  double max_yaw_velocity;

  /// A pointer to the Link, where force is applied
  gazebo::physics::LinkPtr link_;
  
  /// @brief If true, the velocity is in the world frame, otherwise in the link frame.
  bool velocity_on_world_frame_;

  gazebo::common::Time last_cmd_vel_update_time_;
};

GazeboRosPlanarMove::GazeboRosPlanarMove()
: impl_(std::make_unique<GazeboRosPlanarMovePrivate>())
{
}

GazeboRosPlanarMove::~GazeboRosPlanarMove()
{
}

void GazeboRosPlanarMove::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  impl_->model_ = _model;

  impl_->world_ = _model->GetWorld();

  // Initialize ROS node
  impl_->ros_node_ = gazebo_ros::Node::Get(_sdf);

  // Get QoS profiles
  const gazebo_ros::QoS & qos = impl_->ros_node_->get_qos();


  impl_->odometry_frame_ = _sdf->Get<std::string>("odometry_frame", "odom").first;
  impl_->robot_base_frame_ = _sdf->Get<std::string>("robot_base_frame", "base_footprint").first;
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Force will be applied to the link [%s]",impl_->robot_base_frame_.c_str());

  // Target link
  impl_->link_ = impl_->model_->GetLink(impl_->robot_base_frame_);
  RCLCPP_INFO(impl_->ros_node_->get_logger(), "Force will be applied to the link [%s]",impl_->robot_base_frame_.c_str());



  // Update rate
  auto update_rate = _sdf->Get<double>("update_rate", 20.0).first; // default to 20Hz if not given
  if (update_rate > 0.0) {
    impl_->update_period_ = 1.0 / update_rate;
  } else {
    impl_->update_period_ = 0.0;
  }
  impl_->last_update_time_ = impl_->world_->SimTime();

  // Update rate
  auto publish_rate = _sdf->Get<double>("publish_rate", 20.0).first; // default to 20Hz if not given
  if (update_rate > 0.0) {
    impl_->publish_period_ = 1.0 / publish_rate;
  } else {
    impl_->publish_period_ = 0.0;
  }
  impl_->last_publish_time_ = impl_->world_->SimTime();

  impl_->cmd_vel_sub_ = impl_->ros_node_->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel", qos.get_subscription_qos("cmd_vel", rclcpp::QoS(1)),
    std::bind(&GazeboRosPlanarMovePrivate::OnCmdVel, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(
    impl_->ros_node_->get_logger(), "Subscribed to [%s]",
    impl_->cmd_vel_sub_->get_topic_name());

  // Advertise odometry topic
  impl_->publish_odom_ = _sdf->Get<bool>("publish_odom", true).first;
  if (impl_->publish_odom_) {
    impl_->odometry_pub_ = impl_->ros_node_->create_publisher<nav_msgs::msg::Odometry>(
      "odom", qos.get_publisher_qos("odom", rclcpp::QoS(1)));

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(), "Advertise odometry on [%s]",
      impl_->odometry_pub_->get_topic_name());
  }

  // Broadcast TF
  impl_->publish_odom_tf_ = _sdf->Get<bool>("publish_odom_tf", true).first;
  if (impl_->publish_odom_tf_) {
    impl_->transform_broadcaster_ =
      std::make_shared<tf2_ros::TransformBroadcaster>(impl_->ros_node_);// this is just a publisher in disguise

    RCLCPP_INFO(
      impl_->ros_node_->get_logger(),
      "Publishing odom transforms between [%s] and [%s]", impl_->odometry_frame_.c_str(),
      impl_->robot_base_frame_.c_str());
  }

  impl_ -> yaw_velocity_p_gain  = _sdf->Get<double>("yaw_velocity_p_gain",1).first;
  RCLCPP_INFO( impl_->ros_node_->get_logger(), "yaw velocity p gain : %f", impl_ -> yaw_velocity_p_gain);
  impl_ -> x_velocity_p_gain    = _sdf->Get<double>("x_velocity_p_gain",1).first; 
  RCLCPP_INFO( impl_->ros_node_->get_logger(), "x velocity p gain : %f", impl_ -> x_velocity_p_gain);  
  impl_ -> y_velocity_p_gain    = _sdf->Get<double>("y_velocity_p_gain",1).first;    
  RCLCPP_INFO( impl_->ros_node_->get_logger(), "y velocity p gain : %f", impl_ -> y_velocity_p_gain);
  
  /// @brief Maximum velocities for the controller.
  impl_ -> max_x_velocity    = _sdf->Get<double>("max_x_velocity",10).first;
  RCLCPP_INFO( impl_->ros_node_->get_logger(), "max x velocity : %f", impl_ -> max_x_velocity);
  impl_ -> max_y_velocity    = _sdf->Get<double>("max_y_velocity",10).first;  
  RCLCPP_INFO( impl_->ros_node_->get_logger(), "max y velocity : %f", impl_ -> max_y_velocity);
  impl_ -> max_yaw_velocity  = _sdf->Get<double>("max_yaw_velocity",10).first;  
  RCLCPP_INFO( impl_->ros_node_->get_logger(), "max yaw velocity : %f", impl_ -> max_yaw_velocity);


  auto covariance_x = _sdf->Get<double>("covariance_x", 0.00001).first;
  auto covariance_y = _sdf->Get<double>("covariance_y", 0.00001).first;
  auto covariance_yaw = _sdf->Get<double>("covariance_yaw", 0.001).first;

  // Set covariance
  impl_->odom_.pose.covariance[0] = covariance_x;
  impl_->odom_.pose.covariance[7] = covariance_y;
  impl_->odom_.pose.covariance[14] = 1000000000000.0;
  impl_->odom_.pose.covariance[21] = 1000000000000.0;
  impl_->odom_.pose.covariance[28] = 1000000000000.0;
  impl_->odom_.pose.covariance[35] = covariance_yaw;

  impl_->odom_.twist.covariance[0] = covariance_x;
  impl_->odom_.twist.covariance[7] = covariance_y;
  impl_->odom_.twist.covariance[14] = 1000000000000.0;
  impl_->odom_.twist.covariance[21] = 1000000000000.0;
  impl_->odom_.twist.covariance[28] = 1000000000000.0;
  impl_->odom_.twist.covariance[35] = covariance_yaw;


  auto vel_frame = _sdf->Get<std::string>("velocity_frame","world").first;
    if (vel_frame == "world") {
      impl_->velocity_on_world_frame_ = true;
    } else if (vel_frame == "link") {
      impl_->velocity_on_world_frame_ = false;
    } else {
      RCLCPP_ERROR(impl_->ros_node_->get_logger(), "sml_nexus_force plugin  <velocity_frame> can only be 'world' or 'link'");
      return;
    }




  // Set header
  impl_->odom_.header.frame_id = impl_->odometry_frame_;
  impl_->odom_.child_frame_id = impl_->robot_base_frame_;

  // Listen to the update event (broadcast every simulation iteration)
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboRosPlanarMovePrivate::OnUpdate, impl_.get(), std::placeholders::_1));
}

void GazeboRosPlanarMove::Reset()
{
  impl_->last_update_time_ = impl_->world_->SimTime();
  impl_->target_cmd_vel_.linear.x = 0;
  impl_->target_cmd_vel_.linear.y = 0;
  impl_->target_cmd_vel_.angular.z = 0;
}

void GazeboRosPlanarMovePrivate::OnUpdate(const gazebo::common::UpdateInfo & _info)
{
  double seconds_since_last_update = (_info.simTime - last_update_time_).Double();
  double seconds_since_last_cmd_vel_update = (_info.simTime - last_cmd_vel_update_time_).Double();

  std::lock_guard<std::mutex> scoped_lock(lock_);
  #ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE("GazeboRosPlanarMovePrivate::OnUpdate");
    IGN_PROFILE_BEGIN("fill ROS message");
  #endif
  if (seconds_since_last_update >= update_period_) {
    
    if (seconds_since_last_cmd_vel_update > 0.25) {
      /// no updates velocity info was given in a quarter of a second
      target_cmd_vel_.linear.x = 0;
      target_cmd_vel_.linear.y = 0;
      target_cmd_vel_.angular.z = 0;
    }
    else {
      target_cmd_vel_.linear.x = std::min(std::max(target_cmd_vel_.linear.x  , -this->max_x_velocity)  ,this->max_x_velocity);
      target_cmd_vel_.linear.y = std::min(std::max(target_cmd_vel_.linear.y  , -this->max_y_velocity)  ,this->max_y_velocity);
      target_cmd_vel_.angular.z = std::min(std::max(target_cmd_vel_.angular.z, -this->max_yaw_velocity),this->max_yaw_velocity);
    }

   



    if (this->velocity_on_world_frame_) {

      ignition::math::Vector3d angular_vel = model_->WorldAngularVel();
      ignition::math::Vector3d linear_vel  = model_->WorldLinearVel();


      ignition::math::Vector3d force_vector_ = ignition::math::Vector3d(0,0,0);
      force_vector_.X() = (target_cmd_vel_.linear.x - linear_vel.X()) * this-> x_velocity_p_gain;
      force_vector_.Y() = (target_cmd_vel_.linear.y - linear_vel.Y()) * this-> y_velocity_p_gain;
      force_vector_.Z() = 0;

      ignition::math::Vector3d torque_vector_ = ignition::math::Vector3d(0,0,0);
      torque_vector_.Z() = (target_cmd_vel_.angular.z - angular_vel.Z()) * this->yaw_velocity_p_gain;



      // added in the world frame
      this->link_->AddForce(force_vector_);
      this->link_->AddTorque(torque_vector_);
      //
    } 
    else {
      ignition::math::Vector3d angular_vel = model_->RelativeAngularVel();
      ignition::math::Vector3d linear_vel  = model_->RelativeLinearVel();



      // Get the transform between the link's relative frame and the world frame
      ignition::math::Pose3d relative_pose = link_->WorldPose();
      ignition::math::Quaterniond relative_rotation = relative_pose.Rot();

      ignition::math::Vector3d force_vector_ = ignition::math::Vector3d(0,0,0);
      force_vector_.X() = (target_cmd_vel_.linear.x - linear_vel.X()) * this-> x_velocity_p_gain;
      force_vector_.Y() = (target_cmd_vel_.linear.y - linear_vel.Y()) * this-> y_velocity_p_gain;
      force_vector_.Z() = 0;
      
      ignition::math::Vector3d torque_vector_ = ignition::math::Vector3d(0,0,0);
      torque_vector_.Z() = (target_cmd_vel_.angular.z - angular_vel.Z()) * this->yaw_velocity_p_gain;

      // Transform wrench to the world frame
      ignition::math::Vector3d force_world_frame  = relative_rotation.RotateVectorReverse(force_vector_);
      ignition::math::Vector3d torque_world_frame = relative_rotation.RotateVectorReverse(torque_vector_); // not needed for planar robot but left for consistency

      // added in the world frame
      this->link_->AddRelativeForce(force_world_frame);
      this->link_->AddRelativeTorque(torque_world_frame);
    }


    last_update_time_ = _info.simTime;
  }
#ifdef IGN_PROFILER_ENABLE
  IGN_PROFILE_END();
#endif
  if (publish_odom_ || publish_odom_tf_) {
    double seconds_since_last_publish = (_info.simTime - last_publish_time_).Double();

    if (seconds_since_last_publish < publish_period_) {
      return;
    }
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_BEGIN("UpdateOdometry");
#endif
    UpdateOdometry(_info.simTime);
#ifdef IGN_PROFILER_ENABLE
    IGN_PROFILE_END();
#endif
    if (publish_odom_) {
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_BEGIN("publish odometry");
#endif
      odometry_pub_->publish(odom_);
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
#endif
    }
    if (publish_odom_tf_) {
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_BEGIN("publish odometryTF");
#endif
      PublishOdometryTf(_info.simTime);
#ifdef IGN_PROFILER_ENABLE
      IGN_PROFILE_END();
#endif
    }

    last_publish_time_ = _info.simTime;
  }
}

void GazeboRosPlanarMovePrivate::OnCmdVel(const geometry_msgs::msg::Twist::SharedPtr _msg)
{
  std::lock_guard<std::mutex> scoped_lock(lock_);
  target_cmd_vel_ = *_msg;
  
 
  last_cmd_vel_update_time_= world_->SimTime();

}

void GazeboRosPlanarMovePrivate::UpdateOdometry(const gazebo::common::Time & _current_time)
{
  auto pose = model_->WorldPose();
  odom_.pose.pose = gazebo_ros::Convert<geometry_msgs::msg::Pose>(pose);

  // Get velocity in odom frame
  odom_.twist.twist.angular.z = model_->WorldAngularVel().Z();

  // Convert velocity to child_frame_id(aka base_footprint)
  auto linear = model_->WorldLinearVel();
  auto yaw = static_cast<float>(pose.Rot().Yaw());
  odom_.twist.twist.linear.x = cosf(yaw) * linear.X() + sinf(yaw) * linear.Y();
  odom_.twist.twist.linear.y = cosf(yaw) * linear.Y() - sinf(yaw) * linear.X();

  // Set timestamp
  odom_.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
}

void GazeboRosPlanarMovePrivate::PublishOdometryTf(const gazebo::common::Time & _current_time)
{


  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = gazebo_ros::Convert<builtin_interfaces::msg::Time>(_current_time);
  msg.header.frame_id = std::string("world");
  msg.child_frame_id  = ros_node_->get_name();
  msg.transform = gazebo_ros::Convert<geometry_msgs::msg::Transform>(odom_.pose.pose);

  transform_broadcaster_->sendTransform(msg);
}

GZ_REGISTER_MODEL_PLUGIN(GazeboRosPlanarMove)
}  // namespace gazebo_plugins
