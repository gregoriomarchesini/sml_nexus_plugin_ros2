// This code was developed using a template from the gazebo_ros_package for ros2 humble 
// See https://github.com/ros2-gbp/gazebo_ros_pkgs-release/blob/release/humble/gazebo_plugins/src/gazebo_ros_force.cpp for details 

/*
 * \brief  Simple model controller that uses a twist message to move an entity on the xy plane using a PI controller.
 *         the plugins gazebo_planar_move and gazebo_force where used as reference for the development of this plugin
 *
 * \author  Gregorio Marchesini (gremar@kth.com)
 *
 * \date  April 2024
 */

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_PLANAR_MOVE_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_PLANAR_MOVE_HPP_

#include <gazebo/common/Plugin.hh>

#include <memory>

namespace gazebo_plugins
{
class GazeboRosPlanarMovePrivate;

/*
 * \brief  Simple model controller that uses a twist message to move an entity on the xy plane using a PI controller.
 *
 * \author  Gregorio Marchesini (gremar@kth.com)
 *
 * \date  April 2024
 */


/**
  Example Usage:
  \code{.xml}
    <plugin name="gazebo_ros_planar_move" filename="libgazebo_ros_planar_move.so">

      <ros>

        <!-- Add a namespace -->
        <namespace>/demo</namespace>

        <!-- Remap the default topic -->
        <remapping>cmd_vel:=custom_cmd_vel</remapping>
        <remapping>odom:=custom_odom</remapping>

      </ros>

      <yaw_velocity_p_gain>1</yaw_velocity_p_gain>
      <x_velocity_p_gain>15.0</x_velocity_p_gain>
      <y_velocity_p_gain>15.0</y_velocity_p_gain>

      <max_x_velocity>0.7</max_x_velocity>
      <max_y_velocity>0.7</max_y_velocity>
      <max_yaw_velocity>0.5</max_yaw_velocity>

      <velocity_frame>world</velocity_frame>  link: velcity is read an published in the link frame 
                                              world : velocity is read and published in the world frame

      <update_rate>100</update_rate>
      <publish_rate>10</publish_rate>

      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>

      <odometry_frame>odom_demo</odometry_frame>
      <robot_base_frame>link</robot_base_frame>
      

      <covariance_x>0.0001</covariance_x>
      <covariance_y>0.0001</covariance_y>
      <covariance_yaw>0.01</covariance_yaw>

    </plugin>
  \endcode
*/

class GazeboRosPlanarMove : public gazebo::ModelPlugin
{
public:
  /// Constructor
  GazeboRosPlanarMove();

  /// Destructor
  ~GazeboRosPlanarMove();

protected:
  // Documentation inherited
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

  // Documentation inherited
  void Reset() override;

private:
  /// Private data pointer
  std::unique_ptr<GazeboRosPlanarMovePrivate> impl_;
};
}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_PLANAR_MOVE_HPP_
