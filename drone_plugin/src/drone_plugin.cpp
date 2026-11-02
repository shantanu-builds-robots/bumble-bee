// Copyright (c) 2024 Shantanu Parab. MIT License.
//
// Implementation of the delivery-drone Gazebo plugin. See the header for the
// ROS interface and SDF parameters.

#include "drone_plugin/drone_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "drone_plugin/msg/motor_speed.hpp"

namespace drone_plugin
{

namespace
{
constexpr int kNumLegs = 4;
// Landing-leg link names as they appear in Gazebo collision names. The URDF's
// fixed joints lump them into the body, giving e.g.
// "drone::base_link::base_link_fixed_joint_lump__stand_1_collision".
constexpr const char * kLegNames[kNumLegs] = {"stand_1", "stand_2", "stand_3", "stand_4"};
// Delivery box models are named box_0 .. box_4.
constexpr const char * kBoxTag = "box";
}  // namespace

class DronePluginPrivate
{
public:
  void OnUpdate(const gazebo::common::UpdateInfo & info);
  void OnMotorSpeed(const drone_plugin::msg::MotorSpeed::SharedPtr msg);
  void OnEnableGripper(const std_msgs::msg::Bool::SharedPtr msg);
  void OnContacts(ConstContactsPtr & contacts);

  void UpdateThrust();
  void PublishDronePose();
  void UpdateGripper();
  void PublishPartAttached(bool attached);

  double CalculateThrust(double w) const {return thrust_coeff_ * w * w;}
  double CalculateTorque(double w) const {return torque_coeff_ * w * std::abs(w);}

  // Gazebo handles.
  gazebo::physics::ModelPtr model_;
  gazebo::physics::LinkPtr base_link_;
  gazebo::physics::JointPtr picked_part_joint_;
  gazebo::event::ConnectionPtr update_connection_;
  gazebo::transport::NodePtr gz_node_;
  gazebo::transport::SubscriberPtr contacts_sub_;

  // ROS handles.
  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Subscription<drone_plugin::msg::MotorSpeed>::SharedPtr motor_speed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_gripper_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr part_attached_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Parameters.
  std::string base_link_name_{"base_link"};
  std::string world_frame_name_{"world"};
  double update_period_{0.01};
  bool publish_tf_{true};
  double thrust_coeff_{0.00025};
  double torque_coeff_{0.0000074};

  // State shared between ROS / transport callbacks and the physics thread.
  std::mutex mutex_;
  drone_plugin::msg::MotorSpeed::SharedPtr motor_speed_msg_;
  bool enable_gripper_{false};
  bool picked_part_{false};
  std::string pending_box_collision_;  // set by OnContacts, consumed in OnUpdate

  gazebo::common::Time last_publish_time_;
};

DronePlugin::DronePlugin()
: impl_(std::make_unique<DronePluginPrivate>())
{
}

DronePlugin::~DronePlugin() = default;

void DronePlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{
  impl_->model_ = model;
  impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
  auto logger = impl_->ros_node_->get_logger();

  // Parameters from the <plugin> block in the URDF.
  if (sdf->HasElement("drone_frame")) {
    impl_->base_link_name_ = sdf->Get<std::string>("drone_frame");
  }
  if (sdf->HasElement("updateRate")) {
    const double rate = sdf->Get<double>("updateRate");
    impl_->update_period_ = rate > 0.0 ? 1.0 / rate : 0.0;
  }
  if (sdf->HasElement("publishTf")) {
    impl_->publish_tf_ = sdf->Get<bool>("publishTf");
  }
  if (sdf->HasElement("rotorThrustCoeff")) {
    impl_->thrust_coeff_ = sdf->Get<double>("rotorThrustCoeff");
  }
  if (sdf->HasElement("rotorTorqueCoeff")) {
    impl_->torque_coeff_ = sdf->Get<double>("rotorTorqueCoeff");
  }

  impl_->base_link_ = model->GetLink(impl_->base_link_name_);
  if (!impl_->base_link_) {
    RCLCPP_ERROR(logger, "drone_plugin: link '%s' not found; plugin disabled.",
      impl_->base_link_name_.c_str());
    return;
  }

  // ROS interface (topic names match drone_delivery's fly and deliver nodes).
  impl_->motor_speed_sub_ = impl_->ros_node_->create_subscription<drone_plugin::msg::MotorSpeed>(
    "motor_speed", 10,
    std::bind(&DronePluginPrivate::OnMotorSpeed, impl_.get(), std::placeholders::_1));
  impl_->enable_gripper_sub_ = impl_->ros_node_->create_subscription<std_msgs::msg::Bool>(
    "enable_gripper", 10,
    std::bind(&DronePluginPrivate::OnEnableGripper, impl_.get(), std::placeholders::_1));
  impl_->pose_pub_ =
    impl_->ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>("drone_pose", 10);
  impl_->velocity_pub_ =
    impl_->ros_node_->create_publisher<geometry_msgs::msg::Twist>("drone_velocity", 10);
  impl_->part_attached_pub_ =
    impl_->ros_node_->create_publisher<std_msgs::msg::Bool>("part_attached", 10);
  if (impl_->publish_tf_) {
    impl_->tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(impl_->ros_node_);
  }

  // Vacuum gripper: a fixed joint that gets re-targeted at whichever box is picked.
  gazebo::physics::WorldPtr world = model->GetWorld();
  impl_->picked_part_joint_ = world->Physics()->CreateJoint("fixed", model);
  impl_->picked_part_joint_->SetName("picked_part_fixed_joint");

  // Gazebo publishes every contact on this topic while something subscribes.
  impl_->gz_node_ = boost::make_shared<gazebo::transport::Node>();
  impl_->gz_node_->Init(world->Name());
  impl_->contacts_sub_ = impl_->gz_node_->Subscribe(
    "~/physics/contacts", &DronePluginPrivate::OnContacts, impl_.get());

  impl_->last_publish_time_ = world->SimTime();
  impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&DronePluginPrivate::OnUpdate, impl_.get(), std::placeholders::_1));

  RCLCPP_INFO(logger,
    "drone_plugin loaded: body '%s', k_t=%g, k_q=%g, pose at %.0f Hz, tf %s",
    impl_->base_link_name_.c_str(), impl_->thrust_coeff_, impl_->torque_coeff_,
    impl_->update_period_ > 0.0 ? 1.0 / impl_->update_period_ : 0.0,
    impl_->publish_tf_ ? "on" : "off");
}

void DronePluginPrivate::OnUpdate(const gazebo::common::UpdateInfo & info)
{
  // Forces are cleared every step, so thrust must be re-applied every step.
  UpdateThrust();
  UpdateGripper();

  if ((info.simTime - last_publish_time_).Double() >= update_period_) {
    last_publish_time_ = info.simTime;
    PublishDronePose();
  }
}

void DronePluginPrivate::OnMotorSpeed(const drone_plugin::msg::MotorSpeed::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  motor_speed_msg_ = msg;
}

void DronePluginPrivate::OnEnableGripper(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  enable_gripper_ = msg->data;
}

void DronePluginPrivate::UpdateThrust()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!motor_speed_msg_) {
    return;
  }
  const size_t n = std::min(motor_speed_msg_->name.size(), motor_speed_msg_->velocity.size());
  double resultant_torque = 0.0;
  for (size_t i = 0; i < n; ++i) {
    const double w = motor_speed_msg_->velocity[i];
    const double thrust = CalculateThrust(w);
    const double torque = CalculateTorque(w);
    resultant_torque += torque;

    gazebo::physics::LinkPtr link = model_->GetLink(motor_speed_msg_->name[i]);
    if (link) {
      // Rotor joints are rolled 90 degrees, so each rotor link's +Y is its spin axis.
      link->AddLinkForce(ignition::math::Vector3d(0, thrust, 0));
      link->AddRelativeTorque(ignition::math::Vector3d(0, torque, 0));
    }
  }
  // Net rotor reaction torque yaws the body.
  base_link_->AddRelativeTorque(ignition::math::Vector3d(0, 0, resultant_torque));
}

void DronePluginPrivate::PublishDronePose()
{
  const ignition::math::Pose3d pose = base_link_->WorldPose();
  const ignition::math::Vector3d lin = base_link_->WorldLinearVel();
  const ignition::math::Vector3d ang = base_link_->WorldAngularVel();
  const rclcpp::Time stamp = ros_node_->now();

  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.stamp = stamp;
  pose_msg.header.frame_id = world_frame_name_;
  pose_msg.pose.position.x = pose.Pos().X();
  pose_msg.pose.position.y = pose.Pos().Y();
  pose_msg.pose.position.z = pose.Pos().Z();
  pose_msg.pose.orientation.x = pose.Rot().X();
  pose_msg.pose.orientation.y = pose.Rot().Y();
  pose_msg.pose.orientation.z = pose.Rot().Z();
  pose_msg.pose.orientation.w = pose.Rot().W();
  pose_pub_->publish(pose_msg);

  geometry_msgs::msg::Twist twist;
  twist.linear.x = lin.X();
  twist.linear.y = lin.Y();
  twist.linear.z = lin.Z();
  twist.angular.x = ang.X();
  twist.angular.y = ang.Y();
  twist.angular.z = ang.Z();
  velocity_pub_->publish(twist);

  if (tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = world_frame_name_;
    t.child_frame_id = base_link_name_;
    t.transform.translation.x = pose.Pos().X();
    t.transform.translation.y = pose.Pos().Y();
    t.transform.translation.z = pose.Pos().Z();
    t.transform.rotation = pose_msg.pose.orientation;
    tf_broadcaster_->sendTransform(t);
  }
}

void DronePluginPrivate::OnContacts(ConstContactsPtr & contacts)
{
  // Runs on Gazebo's transport thread: only record what to pick; the joint is
  // attached on the physics thread in UpdateGripper().
  std::string leg_box[kNumLegs];

  for (int i = 0; i < contacts->contact_size(); ++i) {
    const std::string & c1 = contacts->contact(i).collision1();
    const std::string & c2 = contacts->contact(i).collision2();
    const bool c1_box = c1.find(kBoxTag) != std::string::npos;
    const bool c2_box = c2.find(kBoxTag) != std::string::npos;
    if (!c1_box && !c2_box) {
      continue;
    }
    for (int leg = 0; leg < kNumLegs; ++leg) {
      if (c1.find(kLegNames[leg]) != std::string::npos ||
        c2.find(kLegNames[leg]) != std::string::npos)
      {
        leg_box[leg] = c1_box ? c1 : c2;
      }
    }
  }

  // Pick only when every leg rests on the same box.
  for (int leg = 0; leg < kNumLegs; ++leg) {
    if (leg_box[leg].empty() || leg_box[leg] != leg_box[0]) {
      return;
    }
  }

  std::lock_guard<std::mutex> lock(mutex_);
  if (enable_gripper_ && !picked_part_ && pending_box_collision_.empty()) {
    pending_box_collision_ = leg_box[0];
  }
}

void DronePluginPrivate::UpdateGripper()
{
  std::string box_collision;
  bool enabled;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    enabled = enable_gripper_;
    box_collision.swap(pending_box_collision_);
  }

  if (picked_part_ && !enabled) {
    picked_part_joint_->Detach();
    picked_part_ = false;
    RCLCPP_INFO(ros_node_->get_logger(), "drone_plugin: box released");
    PublishPartAttached(false);
    return;
  }

  if (!picked_part_ && enabled && !box_collision.empty()) {
    auto collision = boost::dynamic_pointer_cast<gazebo::physics::Collision>(
      model_->GetWorld()->EntityByName(box_collision));
    if (!collision) {
      RCLCPP_WARN(ros_node_->get_logger(), "drone_plugin: collision '%s' not found",
        box_collision.c_str());
      return;
    }
    picked_part_joint_->Load(base_link_, collision->GetLink(), ignition::math::Pose3d());
    picked_part_joint_->Init();
    picked_part_ = true;
    RCLCPP_INFO(ros_node_->get_logger(), "drone_plugin: picked up '%s'",
      collision->GetLink()->GetScopedName().c_str());
    PublishPartAttached(true);
  }
}

void DronePluginPrivate::PublishPartAttached(bool attached)
{
  std_msgs::msg::Bool msg;
  msg.data = attached;
  part_attached_pub_->publish(msg);
}

GZ_REGISTER_MODEL_PLUGIN(DronePlugin)

}  // namespace drone_plugin
