// Copyright (c) 2024 Shantanu Parab. MIT License.
//
// Gazebo Classic model plugin for the delivery drone.
//
//  * Flight: subscribes to `motor_speed` (drone_plugin/MotorSpeed) and, every
//    physics step, applies thrust (k_t * w^2, along each rotor's spin axis) and
//    reaction torque (k_q * w * |w|) to the named rotor links, with the summed
//    reaction torque applied to the body as yaw.
//  * Feedback: publishes the body pose on `drone_pose` (PoseStamped) and its
//    twist on `drone_velocity` (Twist) at <updateRate> Hz, plus a world->body
//    TF when <publishTf> is true.
//  * Vacuum gripper: when enabled via `enable_gripper` (Bool) and all four
//    landing legs (stand_1..stand_4) touch the same "box" collision, the box is
//    welded to the body with a fixed joint and `part_attached` (Bool) goes
//    true. Disabling the gripper releases the box and publishes false.
//
// SDF parameters (all optional):
//   <drone_frame>        body link name                 default: base_link
//   <updateRate>         pose/TF publish rate in Hz     default: 100
//   <publishTf>          broadcast world->body TF       default: true
//   <rotorThrustCoeff>   k_t, N / (rad/s)^2             default: 0.00025
//   <rotorTorqueCoeff>   k_q, N*m / (rad/s)^2           default: 0.0000074

#ifndef DRONE_PLUGIN__DRONE_PLUGIN_HPP_
#define DRONE_PLUGIN__DRONE_PLUGIN_HPP_

#include <memory>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>

namespace drone_plugin
{

class DronePluginPrivate;

class DronePlugin : public gazebo::ModelPlugin
{
public:
  DronePlugin();
  ~DronePlugin() override;

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

private:
  std::unique_ptr<DronePluginPrivate> impl_;
};

}  // namespace drone_plugin

#endif  // DRONE_PLUGIN__DRONE_PLUGIN_HPP_
