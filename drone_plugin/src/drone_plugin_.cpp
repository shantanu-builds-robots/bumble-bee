
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <drone_plugin/drone_plugin_.hpp>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include "drone_plugin/msg/motor_speed.hpp"

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <gazebo/common/Console.hh>
#include <tf2_ros/transform_broadcaster.h>
#include <memory>
#include <mutex>

#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/transport/Subscriber.hh>
#include <gazebo/transport/Node.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/PhysicsEngine.hh>

std::mutex mutex_;

namespace gazebo{


class DronePluginPrivate
{
  /**
   * @brief Class to hold data members and methods for plugin
   * 
   */
public:
  /// Connection to world update event. Callback is called while this is alive.
  gazebo::event::ConnectionPtr update_connection_;

  /// Node for ROS communication.
  gazebo_ros::Node::SharedPtr ros_node_;

  gazebo::physics::ModelPtr _model;

  gazebo::physics::JointPtr picked_part_joint_;
  gazebo::physics::CollisionPtr model_collision_;
  gazebo::physics::LinkPtr gripper_link_;


  gazebo::transport::SubscriberPtr contact1_sub_;
  gazebo::transport::SubscriberPtr contact2_sub_;
  gazebo::transport::SubscriberPtr contact3_sub_;
  gazebo::transport::SubscriberPtr contact4_sub_;
  gazebo::transport::NodePtr gznode_;



  /// Publisher to publsih the pose to /odom topic
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr droneOdomPublisher;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr partAttachedPublisher;

  /// Publisher to publsih the velocity to /velocity topic
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocityPublisher;

  /// Publisher to publsih the velocity to /velocity topic
  rclcpp::Subscription<drone_plugin::msg::MotorSpeed>::SharedPtr motor_speed_subscriber;

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pick_part_subscriber;

  /// TF Broadcaster to pubshil the transform between 2 frames
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Callback function to perform task with each iteration in Gazebo
  void OnUpdate();

  void publishDronePose() ;

  void onMotorSpeedsMsg(const drone_plugin::msg::MotorSpeed::SharedPtr msg);

  void OnPickPart(const std_msgs::msg::Bool::SharedPtr msg);

  void updateThrust() ;

  double calculateThrust(double w) ;

  double calculateTorque(double w) ;

  void OnContactUpdate(ConstContactsPtr &contacts);




    //   Variable
  double _rate;
  bool _publish_tf;
  double _rotor_thrust_coeff;
  double _rotor_torque_coeff;

  bool picked_part = false;

  bool enable_gripper = false;

  gazebo::physics::LinkPtr baseLink;

  std::string base_link_name;
  std::string world_frame_name;

  std::shared_ptr<drone_plugin::msg::MotorSpeed> _motor_speed_msg;

};


/**
 * @brief Construct a new Drone Plugin:: Drone Plugin object
 * 
 */
DronePlugin::DronePlugin()
: impl_(std::make_unique<DronePluginPrivate>())
{
  std::cout << "Starting drone_plugin" << std::endl;
}

/**
 * @brief Destroy the Drone Plugin:: Drone Plugin object
 * 
 */
DronePlugin::~DronePlugin()
{
    std::cout << "Closing drone_plugin" << std::endl;
}

/**
 * @brief Load the SDF/URDF model of the robot and access the links/joints.
 * 
 * @param model 
 * @param sdf 
 */
void DronePlugin::Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf)
{   


    impl_->_model = model;
    impl_->ros_node_ = gazebo_ros::Node::Get(sdf);
    impl_->droneOdomPublisher = impl_->ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>("/drone_pose", 10);
    impl_->velocityPublisher = impl_->ros_node_->create_publisher<geometry_msgs::msg::Twist>("/drone_velocity", 10);
    impl_->partAttachedPublisher = impl_->ros_node_->create_publisher<std_msgs::msg::Bool>("/part_attached", 10);
    // Create subscriber
    impl_->motor_speed_subscriber = impl_->ros_node_->create_subscription<drone_plugin::msg::MotorSpeed>(
    "/motor_speed", 10, 
    std::bind(&DronePluginPrivate::onMotorSpeedsMsg, impl_.get(), std::placeholders::_1));

    impl_->pick_part_subscriber = impl_->ros_node_->create_subscription<std_msgs::msg::Bool>("enable_gripper", 10, std::bind(&DronePluginPrivate::OnPickPart, impl_.get(), std::placeholders::_1));


    gazebo::physics::WorldPtr world = impl_->_model->GetWorld();
    impl_->picked_part_joint_ = world->Physics()->CreateJoint("fixed", impl_->_model);
    impl_->picked_part_joint_->SetName("picked_part_fixed_joints");

    std::cout << "Joint Created"<< std::endl;

    // Initialize Gazebo Node and subscribe to contact sensor
    impl_->gznode_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
    impl_->gznode_->Init(impl_->_model->GetWorld()->Name());


    // Get gripper link
    std::string link_name = "base_link";
    impl_->gripper_link_ = impl_->_model->GetLink(link_name);

    std::string contact_topic = "/gazebo/default/physics/contacts";

    impl_->contact1_sub_ = impl_->gznode_->Subscribe(contact_topic, &DronePluginPrivate::OnContactUpdate, impl_.get());
   
    
    impl_->tf_broadcaster_ =std::make_unique<tf2_ros::TransformBroadcaster>(impl_->ros_node_);

    /// Check for the frame element in the URDF
    if (sdf->HasElement("drone_frame")) {
        std::cout  << "Reference link  found!" << std::endl;
        impl_->base_link_name  = sdf->GetElement("drone_frame")->Get<std::string>();

        std::cout  << "Base link  name: " <<impl_->base_link_name<< std::endl;
    } else {
        // Handle the case where the element doesn't exist
        std::cerr << "Reference link not found!" << std::endl;
        return;
    }
    // Access the base link of the model .
    impl_->baseLink = model->GetLink(impl_->base_link_name);
    

    // Check if the base link exists.
    if (!impl_->baseLink) {
        std::cerr  << "Base link not found!" << std::endl;
        return;
    }

    if (sdf->HasElement("updateRate")) {
      impl_->_rate = sdf->GetElement("updateRate")->Get<double>();
    } else {
      impl_->_rate = 100.0;
    }
   
    if (sdf->HasElement("publishTf")) {
      impl_->_publish_tf = sdf->GetElement("publishTf")->Get<bool>();
    } else {
      impl_->_publish_tf = true;
    }
   
    if (sdf->HasElement("rotorThrustCoeff")) {
      impl_->_rotor_thrust_coeff =
         sdf->GetElement("rotorThrustCoeff")->Get<double>();
    } else {
      impl_->_rotor_thrust_coeff = 0.00025;
    }
    std::cout << "Rotor thrust coeff: " << impl_->_rotor_thrust_coeff << std::endl;

   
    if (sdf->HasElement("rotorTorqueCoeff")) {
      impl_->_rotor_torque_coeff =
        sdf->GetElement("rotorTorqueCoeff")->Get<double>();

    } else {
      impl_->_rotor_torque_coeff = 0.0000074;
    }
    std::cout << "Rotor torque coeff: " << impl_->_rotor_torque_coeff << std::endl;
    
    // Set up a Gazebo update callback function.
    impl_->update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&DronePluginPrivate::OnUpdate, impl_.get()));

    if (impl_->update_connection_) {
        std::cout << "Successfully connected to the world update event!" << std::endl;
    } else {
        std::cerr << "Failed to connect to the world update event!" << std::endl;
    }

}

void DronePluginPrivate::OnPickPart(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    std::cout << "Gripper Enabled" << std::endl;
    enable_gripper = true;
  } else if (!msg->data) {
    std::cout << "Gripper Disabled" << std::endl;
    enable_gripper = false;
    if (picked_part) {
      picked_part = false;
      picked_part_joint_->Detach();

      std_msgs::msg::Bool msg;
      msg.data = false;
      partAttachedPublisher->publish(msg);

    }
  }

}


void DronePluginPrivate::OnContactUpdate(ConstContactsPtr &contacts) {
  bool stand_1_collision = false;
  bool stand_2_collision = false;
  bool stand_3_collision = false;
  bool stand_4_collision = false;

  std::string model1_in_contact;
  std::string model2_in_contact;
  std::string model3_in_contact;
  std::string model4_in_contact;

  std::string model_in_contact;

  for (int i=0; i<contacts->contact_size(); i++) {
    if (contacts->contact(i).collision1().find("stand_1") != std::string::npos || contacts->contact(i).collision2().find("stand_1") != std::string::npos) {
      if (contacts->contact(i).collision1().find("box") != std::string::npos || contacts->contact(i).collision2().find("box") != std::string::npos) {
        stand_1_collision = true;
        model1_in_contact = contacts->contact(i).collision1().find("box") != std::string::npos ? contacts->contact(i).collision1() : contacts->contact(i).collision2();
      }
    }
    if (contacts->contact(i).collision1().find("stand_2") != std::string::npos || contacts->contact(i).collision2().find("stand_2") != std::string::npos) {
      if (contacts->contact(i).collision1().find("box") != std::string::npos || contacts->contact(i).collision2().find("box") != std::string::npos) {
        stand_2_collision = true;
        model2_in_contact = contacts->contact(i).collision1().find("box") != std::string::npos ? contacts->contact(i).collision1() : contacts->contact(i).collision2();
      }
    }
    if (contacts->contact(i).collision1().find("stand_3") != std::string::npos || contacts->contact(i).collision2().find("stand_3") != std::string::npos) {
      if (contacts->contact(i).collision1().find("box") != std::string::npos || contacts->contact(i).collision2().find("box") != std::string::npos) {
        stand_3_collision = true;
        model3_in_contact = contacts->contact(i).collision1().find("box") != std::string::npos ? contacts->contact(i).collision1() : contacts->contact(i).collision2();
      }
    }
    if (contacts->contact(i).collision1().find("stand_4") != std::string::npos || contacts->contact(i).collision2().find("stand_4") != std::string::npos) {
      if (contacts->contact(i).collision1().find("box") != std::string::npos || contacts->contact(i).collision2().find("box") != std::string::npos) {
        stand_4_collision = true;
        model4_in_contact = contacts->contact(i).collision1().find("box") != std::string::npos ? contacts->contact(i).collision1() : contacts->contact(i).collision2();
      }
    }
    std::cout << "Stand 1 " << model1_in_contact << std::endl;
    std::cout << "Stand 2 " << model2_in_contact << std::endl;
    std::cout << "Stand 3 " << model3_in_contact << std::endl;
    std::cout << "Stand 4 " << model4_in_contact << std::endl;
    
    model_in_contact = model1_in_contact;

  }
  if (stand_1_collision && stand_2_collision && stand_3_collision && stand_4_collision && !picked_part && enable_gripper) {

    if(model_in_contact == model2_in_contact || model_in_contact == model3_in_contact || model_in_contact == model4_in_contact) {
      std::cout << "Ready to pick" << std::endl;
    std::cout << "Model in contact: " << model_in_contact << std::endl;
    model_collision_ = boost::dynamic_pointer_cast<gazebo::physics::Collision>(_model->GetWorld()->EntityByName(model_in_contact));
    if (model_collision_ == NULL) {
      std::cerr << "Model collision not found!" << std::endl;
      return;
    }
    // Define the specific x, y, z position and roll, pitch, yaw orientation
    double x = 0.0; // specify your x position here
    double y = 0.0; // specify your y position here
    double z = 0.012; // specify your z position here (optional)
    double roll = 0.0; // specify your roll angle here
    double pitch = 0.0; // specify your pitch angle here
    double yaw = 0.0; // specify your yaw angle here

    // Create the Pose3d object with the specified position and orientation
    ignition::math::Pose3d pose(x, y, z, roll, pitch, yaw);

    picked_part_joint_->Load(gripper_link_, model_collision_->GetLink(), pose);
    picked_part_joint_->Init();

    picked_part = true;
    std_msgs::msg::Bool msg;
    msg.data = true;
    partAttachedPublisher->publish(msg);
    }

    
  }
}

void DronePluginPrivate::onMotorSpeedsMsg(const drone_plugin::msg::MotorSpeed::SharedPtr msg)  {
    _motor_speed_msg = msg;
}



void DronePluginPrivate::publishDronePose() {

    geometry_msgs::msg::PoseStamped odometryMessage;
    // Get the current pose of the base link.
    ignition::math::Pose3 baseLinkPose = baseLink->WorldPose();

    // Get the linear and angular velocities of the base link.
    ignition::math::Vector3d linearVelocity = baseLink->WorldLinearVel();
    ignition::math::Vector3d angularVelocity = baseLink->WorldAngularVel();

    // Extract position and orientation information
    odometryMessage.header.stamp = ros_node_->now();
    odometryMessage.header.frame_id = base_link_name;  
    odometryMessage.pose.position.x = baseLinkPose.Pos().X();
    odometryMessage.pose.position.y = baseLinkPose.Pos().Y();
    odometryMessage.pose.position.z = baseLinkPose.Pos().Z();
    odometryMessage.pose.orientation.x = baseLinkPose.Rot().X();
    odometryMessage.pose.orientation.y = baseLinkPose.Rot().Y();
    odometryMessage.pose.orientation.z = baseLinkPose.Rot().Z();
    odometryMessage.pose.orientation.w = baseLinkPose.Rot().W();

    // Create and set linear and angular velocities in the Twist message
    geometry_msgs::msg::Twist twist;

    twist.linear.x = linearVelocity.X();
    twist.linear.y = linearVelocity.Y();
    twist.linear.z = linearVelocity.Z();
    twist.angular.x = angularVelocity.X();
    twist.angular.y = angularVelocity.Y();
    twist.angular.z = angularVelocity.Z();

    

    velocityPublisher->publish(twist);

    droneOdomPublisher->publish(odometryMessage);
 
    if (_publish_tf) {
        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = ros_node_->now();
        t.header.frame_id = world_frame_name;
        t.child_frame_id = base_link_name;

        t.transform.translation.x = baseLinkPose.Pos().X();
        t.transform.translation.y = baseLinkPose.Pos().Y();
        t.transform.translation.z = baseLinkPose.Pos().Z();
        t.transform.rotation.x = baseLinkPose.Rot().X();
        t.transform.rotation.y = baseLinkPose.Rot().Y();
        t.transform.rotation.z = baseLinkPose.Rot().Z();
        t.transform.rotation.w = baseLinkPose.Rot().W();

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
    }
}


/**
 * @brief This method is called at every time interval in Gazebo
 * 
 */
void DronePluginPrivate::OnUpdate()
{   
    publishDronePose();
    updateThrust();

}



void DronePluginPrivate::updateThrust() {
  
  if(!_motor_speed_msg){
    
  }
  else{
  std::unique_lock<std::mutex> lock(mutex_);
  int n = _motor_speed_msg->name.size();
  double resulant_torque = 0.0;
  for (int i = 0; i < n; ++i) {
    // Calculate thrust and torque
    double thrust = calculateThrust(_motor_speed_msg->velocity[i]);
    double torque = calculateTorque(_motor_speed_msg->velocity[i]);

    // Add up the individual torques
    resulant_torque += torque;

    // Get the link from the model using the name
    gazebo::physics::LinkPtr link = _model->GetLink(_motor_speed_msg->name[i]);
    
    // Apply the thrust and torque to the link
    if (link != NULL) {
      link->AddLinkForce(ignition::math::Vector3d(0, thrust, 0));
      link->AddRelativeTorque(ignition::math::Vector3d(0, torque, 0));

    }
  }
  // Apply the resultant torque to the base link
  baseLink->AddRelativeTorque(ignition::math::Vector3d(0,0,resulant_torque));
  lock.unlock();
  }

 
  
}


double DronePluginPrivate::calculateThrust(double w) {
  double thrust = _rotor_thrust_coeff * w * w;
  // std::cout << "Thrust: " << thrust << std::endl;
  return thrust;
}

double DronePluginPrivate::calculateTorque(double w) {
  double torque = copysign(_rotor_torque_coeff * w * w, w);
  return torque;
}


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(DronePlugin)

}