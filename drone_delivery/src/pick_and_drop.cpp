#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <drone_plugin/msg/motor_speed.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class Deliver : public rclcpp::Node
{

public:
  Deliver() : Node("Deliver")
  {
    RCLCPP_INFO(get_logger(), "Deliver node has been created.");

    pose_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    part_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions pose_subscription_options;
    pose_subscription_options.callback_group = pose_callback_group_;


    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("drone_pose", 10,std::bind(&Deliver::pose_callback, this, std::placeholders::_1), pose_subscription_options);
    
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&Deliver::timer_callback, this), timer_callback_group_);


    height_pub_ = this->create_publisher<std_msgs::msg::Float64>("height", 10);
    x_target_pub_ = this->create_publisher<std_msgs::msg::Float64>("target_x", 10);
    y_target_pub_ = this->create_publisher<std_msgs::msg::Float64>("target_y", 10);
    base_speed_pub_ = this->create_publisher<std_msgs::msg::Float64>("base_speed", 10);


    rclcpp::SubscriptionOptions part_attached_subscription_options;
    part_attached_subscription_options.callback_group = part_callback_group_;

    part_attached_sub_ = this->create_subscription<std_msgs::msg::Bool>("part_attached", 10, std::bind(&Deliver::part_attached_callback, this, std::placeholders::_1), part_attached_subscription_options);

    enable_gripper_pub_ = this->create_publisher<std_msgs::msg::Bool>("enable_gripper", 10);

  }

    void part_attached_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        part_attached = msg->data;
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_ = *msg;
        x = pose_.pose.position.x;
        y = pose_.pose.position.y;
        height = pose_.pose.position.z;
    }


void timer_callback()
{   
    if (pick_up){
        // RCLCPP_INFO(get_logger(), "Pick up");
        // Go to pick up location
        if(go_to_location){
            RCLCPP_INFO(get_logger(), "Go to pick up location");
            std_msgs::msg::Float64 height_msg;
            height_msg.data = 0.7;
            height_pub_->publish(height_msg);

            std_msgs::msg::Float64 x_target_msg;
            x_target_msg.data = pick_up_x;
            x_target_pub_->publish(x_target_msg);

            std_msgs::msg::Float64 y_target_msg;
            y_target_msg.data = pick_up_y;
            y_target_pub_->publish(y_target_msg);
            // rclcpp::sleep_for(std::chrono::milliseconds(100));
            if (std::abs(x - pick_up_x) > 0.01 || std::abs(y - pick_up_y) > 0.01){
                
                RCLCPP_INFO(get_logger(), "x: %f, y: %f", x, y);
                return;
            }
            go_to_location = false;

            //Enable gripper
            std_msgs::msg::Bool enable_gripper_msg;
            enable_gripper_msg.data = true;
            enable_gripper_pub_->publish(enable_gripper_msg);
        }

        

        
        if(!part_attached){
            RCLCPP_INFO_STREAM(get_logger(), "Lowering Drone Height: " << height);
            std_msgs::msg::Float64 height_msg;
            height_msg.data = target_height;
            height_pub_->publish(height_msg);
            target_height -= offset;
            return;
        }
        target_height = 0.7;


        // Increse the base speed to compensate for the weight of the part
        // std_msgs::msg::Float64 base_speed_msg;
        // base_speed_msg.data = base_speed_ + 1;
        // base_speed_pub_->publish(base_speed_msg);

        // Raise the robot to a safe height
        std_msgs::msg::Float64 height_msg;
        height_msg.data = 0.4;
        height_pub_->publish(height_msg);
        if(height < 0.38 || height > 0.42){
            RCLCPP_INFO(get_logger(), "height: %f", height);
            return;
        }

        // Make pick_up = false
        pick_up = false;
        // Make drop_off = true
        drop_off = true;
        // Make go_to_location = true
        go_to_location = true;


    }
    else if (drop_off){
        // Go to drop off location
        if(go_to_location){
            RCLCPP_INFO(get_logger(), "Go to drop off location");
            std_msgs::msg::Float64 height_msg;
            height_msg.data = 0.70;
            height_pub_->publish(height_msg);

            std_msgs::msg::Float64 x_target_msg;
            x_target_msg.data = drop_off_x;
            x_target_pub_->publish(x_target_msg);

            std_msgs::msg::Float64 y_target_msg;
            y_target_msg.data = drop_off_y;
            y_target_pub_->publish(y_target_msg);
            if (std::abs(x - drop_off_x) > 0.2 || std::abs(y - drop_off_y) > 0.2){
                // rclcpp::sleep_for(std::chrono::milliseconds(100));
                RCLCPP_INFO(get_logger(), "x: %f, y: %f", x, y);
                return;
            }
            go_to_location = false;
        }

        // Lower the robot to the drop off location

        std_msgs::msg::Float64 height_msg;
        height_msg.data = drop_off_height;
        height_pub_->publish(height_msg);
        if(height > drop_off_height){
            return;
        }


        // Disable the gripper
        std_msgs::msg::Bool enable_gripper_msg;
        enable_gripper_msg.data = false;
        enable_gripper_pub_->publish(enable_gripper_msg);

        // Base speed back to normal
        std_msgs::msg::Float64 base_speed_msg;
        base_speed_msg.data = base_speed_;
        base_speed_pub_->publish(base_speed_msg);

        // Make drop_off = false
        drop_off = false;
        // Make go_to_location = true
        go_to_location = true;


    }
    else{
        // Next parcel while any are left. Checking first matters: indexing past the end of the
        // vectors is undefined behaviour and used to send the drone to a garbage target.
        if (index + 1 < static_cast<int>(pick_up_locations.size())) {
            index++;
            pick_up_x = pick_up_locations[index].first;
            pick_up_y = pick_up_locations[index].second;
            drop_off_x = drop_off_locations[index].first;
            drop_off_y = drop_off_locations[index].second;
            pick_up = true;
            go_to_location = true;
            return;
        }

        // All parcels delivered: fly home and land.
        {
            // Go to home
            if(go_to_location){
                RCLCPP_INFO(get_logger(), "Go home");
                std_msgs::msg::Float64 height_msg;
                height_msg.data = home_height;
                height_pub_->publish(height_msg);

                std_msgs::msg::Float64 x_target_msg;
                x_target_msg.data = home_x;
                x_target_pub_->publish(x_target_msg);

                std_msgs::msg::Float64 y_target_msg;
                y_target_msg.data = home_y;
                y_target_pub_->publish(y_target_msg);
                if (std::abs(x - home_x) > 0.11 || std::abs(y - home_y) > 0.11){
                    // rclcpp::sleep_for(std::chrono::milliseconds(100));
                    RCLCPP_INFO(get_logger(), "x: %f, y: %f", x, y);
                    return;
                }
                go_to_location = false;
            }
            // Settle just above the pad first: dropping the height target straight from cruise to
            // zero brings the drone down at about 2 m/s.
            std_msgs::msg::Float64 height_msg;
            if (!over_home_pad) {
                height_msg.data = landing_approach_height;
                height_pub_->publish(height_msg);
                if (height > landing_approach_height + 0.03) {
                    return;
                }
                over_home_pad = true;
            }
            // Then set it down on the pad
            RCLCPP_INFO(get_logger(), "Landing");
            height_msg.data = 0.0;
            height_pub_->publish(height_msg);
            timer_->cancel();
        }
        
    
        

    }

    
    
    
}

    

private:
  
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr height_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr x_target_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr y_target_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr base_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr enable_gripper_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr part_attached_sub_;

    rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
    rclcpp::CallbackGroup::SharedPtr pose_callback_group_;
    rclcpp::CallbackGroup::SharedPtr part_callback_group_;


    std::vector<std::pair<double, double>> pick_up_locations = {{0.5, 2.0}, {1.5, 2.0}, {1.5, 3.0}, {0.5, 3.0}};
    std::vector<std::pair<double, double>> drop_off_locations = {{0.5, 9.5}, {9.5, 9.5}, {4.5, 5.5}, {9.5, 0.5}};

    int index = 0;
    
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PoseStamped pose_;

    // Informational only: published on /base_speed, which nothing subscribes to. fly.cpp owns the hover speed.
    double base_speed_ = 738.4;

    bool pick_up = true;
    bool drop_off = false;

    bool part_attached = false;

    bool go_to_location = true;



    // Current position of the drone
    double x = 0.0;
    double y = 0.0;
    double height = 0.0;

    // Pick up location
    double pick_up_x = pick_up_locations[0].first;
    double pick_up_y = pick_up_locations[0].second;


    // Drop off location
    double drop_off_x = drop_off_locations[0].first;
    double drop_off_y = drop_off_locations[0].second;
    double drop_off_height = 0.4;


    // Lower the drone till part is attached
    double offset = 0.05;
    double target_height = 0.5;

    // Home location
    // Centre of the home landing pad, where the drone spawns (the pad spans 0..1 m in x and y).
    double home_x = 0.5;
    double home_y = 0.5;
    double home_height = 0.7;  // cruise home at the same height as every other leg
    double landing_approach_height = 0.25;  // about 0.1 m above the 0.1 m tall pad
    bool over_home_pad = false;

    
    
    
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Deliver>());
  rclcpp::shutdown();
  return 0;
}