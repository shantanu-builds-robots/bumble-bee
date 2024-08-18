#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <drone_plugin/msg/motor_speed.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class Fly : public rclcpp::Node
{

public:
  Fly() : Node("fly")
  {
    RCLCPP_INFO(get_logger(), "Fly node has been created.");
    publisher_ = this->create_publisher<drone_plugin::msg::MotorSpeed>("motor_speed", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("drone_pose", 10,std::bind(&Fly::pose_callback, this, std::placeholders::_1));
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Fly::timer_callback, this));


    height_sub_ = this->create_subscription<std_msgs::msg::Float64>("height", 10, std::bind(&Fly::height_callback, this, std::placeholders::_1));
    x_sub_ = this->create_subscription<std_msgs::msg::Float64>("target_x", 10, std::bind(&Fly::x_callback, this, std::placeholders::_1));
    y_sub_ = this->create_subscription<std_msgs::msg::Float64>("target_y", 10, std::bind(&Fly::y_callback, this, std::placeholders::_1));

    part_attached_sub_ = this->create_subscription<std_msgs::msg::Bool>("part_attached", 10, std::bind(&Fly::part_attached_callback, this, std::placeholders::_1));

    kp_sub_ = this->create_subscription<std_msgs::msg::Float64>("kp", 10, std::bind(&Fly::kp_callback, this, std::placeholders::_1));
    ki_sub_ = this->create_subscription<std_msgs::msg::Float64>("ki", 10, std::bind(&Fly::ki_callback, this, std::placeholders::_1));
    kd_sub_ = this->create_subscription<std_msgs::msg::Float64>("kd", 10, std::bind(&Fly::kd_callback, this, std::placeholders::_1));

  }

    void part_attached_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        part_attached = msg->data;
        if (part_attached){
            RCLCPP_INFO_STREAM(get_logger(), "Part attached Gain Changed");
            base_speed_ = 118.0;
            kp = 12.5;
            ki = 0.3;
            kd = 7.1;

        } else {
            RCLCPP_INFO_STREAM(get_logger(), "Part detached Gain Changed");
            base_speed_ = 117.0;
            kp = 11.5;
            ki = 0.2;
            kd = 7.1;
        }
    }

    void kp_callback(const std_msgs::msg::Float64::SharedPtr msg)
        {
            // roll_kp = msg->data;
            // pitch_kp = msg->data;
            x_kp = msg->data;
            y_kp = msg->data;
            // kp = msg->data;
        }

    void ki_callback(const std_msgs::msg::Float64::SharedPtr msg)
        {
            // roll_ki = msg->data;
            // pitch_ki = msg->data;
            x_ki = msg->data;
            y_ki = msg->data;
            // ki = msg->data;
        }
    
    void kd_callback(const std_msgs::msg::Float64::SharedPtr msg)
        {
            // roll_kd = msg->data;
            // pitch_kd = msg->data;
            x_kd = msg->data;
            y_kd = msg->data;
            // kd = msg->data;
        }

    
        
    void x_callback(const std_msgs::msg::Float64::SharedPtr msg)
        {
            target_x = msg->data;
            // yaw_kp = msg->data;
            // y_kp = msg->data;
        }
    
    void y_callback(const std_msgs::msg::Float64::SharedPtr msg)
        {
            target_y = msg->data;
            // yaw_ki = msg->data;
            // y_ki = msg->data;
        }
    
    void height_callback(const std_msgs::msg::Float64::SharedPtr msg)
        {   
            target_height = msg->data;
            // yaw_kd = msg->data;
            // x_kd = msg->data;
            // y_kd = msg->data;
        }


    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_ = *msg;
    }

std::array<double, 3> get_euler_from_quaternion(tf2::Quaternion quaternion)
    {
        double roll;
        double pitch;
        double yaw;
        tf2::Matrix3x3 matrix(quaternion);
        matrix.getRPY(roll, pitch, yaw);
        // create an empty array of doubles to store the roll, pitch, yaw
        std::array<double, 3> rpy;
        rpy.at(0) = roll;
        rpy.at(1) = pitch;
        rpy.at(2) = yaw;

        return rpy;
    }

void timer_callback()
{   
    
    // Target height to hover at
    
    if (target_height == 0.0){
        float prop_1 = 0.0;
        float prop_2 = 0.0;
        float prop_3 = 0.0;
        float prop_4 = 0.0;

        auto message = drone_plugin::msg::MotorSpeed();
        message.name = std::vector<std::string>{"prop_1","prop_2","prop_3","prop_4"};
        message.velocity = std::vector<float>{prop_1, -prop_2, prop_3, -prop_4};
        publisher_->publish(message);
    } 
    else {
        // Calculate the control signal for height control
        double velocity_ = PID_height_control(target_height);

        std::array<double, 3> rpy = getRollPitchYaw(target_x, target_y);
        // rpy = {0.0, 0.0, 0.0};
        RCLCPP_INFO_STREAM(get_logger(), "Roll: " << rpy.at(0) << " " << "Pitch: " << rpy.at(1) << " " << "Yaw: " << rpy.at(2));
        // Calculate the control signal for orientation control
        std::array<double, 6> orientation_velocity = PID_orientation_control(rpy.at(0), rpy.at(1), rpy.at(2));
        // std::array<double, 3> orientation_velocity = {0.0, 0.0, 0.0};

        float prop_1 = static_cast<float>(base_speed_+ velocity_  + orientation_velocity.at(0) + orientation_velocity.at(1) + orientation_velocity.at(2));
        float prop_2 = static_cast<float>(base_speed_ + velocity_ + orientation_velocity.at(0) - orientation_velocity.at(1) - orientation_velocity.at(2));
        float prop_3 = static_cast<float>(base_speed_ + velocity_ - orientation_velocity.at(0) - orientation_velocity.at(1) + orientation_velocity.at(2));
        float prop_4 = static_cast<float>(base_speed_ + velocity_ - orientation_velocity.at(0) + orientation_velocity.at(1) - orientation_velocity.at(2));

        auto message = drone_plugin::msg::MotorSpeed();
        message.name = std::vector<std::string>{"prop_1","prop_2","prop_3","prop_4"};
        message.velocity = std::vector<float>{prop_1, -prop_2, prop_3, -prop_4};
        publisher_->publish(message);
        RCLCPP_INFO_STREAM(get_logger(), "Prop 1: " << prop_1 << " " << "Prop 2: " << prop_2 << " " << "Prop 3: " << prop_3 << " " << "Prop 4: " << prop_4);
        RCLCPP_INFO_STREAM(get_logger(), "Heigth: "<<pose_.pose.position.z<<" Roll: " << orientation_velocity.at(3) << " " << "Pitch: " << orientation_velocity.at(4) << " " << "Yaw: " << orientation_velocity.at(5));
        RCLCPP_INFO_STREAM(get_logger(), "Error: "<< rpy.at(2)- orientation_velocity.at(5) << " Control Signal: "<< orientation_velocity.at(2));
        RCLCPP_INFO_STREAM(get_logger(), "X: " <<target_x << " " << "Y: " << target_y << " " << "Z: " << target_height);
    }
    
}

std::array<double, 3> getRollPitchYaw(double target_x, double target_y){
    double x = pose_.pose.position.x;
    double y = pose_.pose.position.y;
    
    // RCLCPP_INFO_STREAM(get_logger(), "X: " << x << " " << "Y: " << y);
    double x_error = target_x - x;
    double y_error = target_y - y;
    // RCLCPP_INFO_STREAM(get_logger(), "X error: " << x_error << " " << "Y error: " << y_error);

    rclcpp::Time current_time = this->get_clock()->now();

    rclcpp::Duration dt = current_time - this->current_time;
    this->current_time = current_time;



    x_integral += x_error * dt.seconds();
    y_integral += y_error * dt.seconds();

    double x_derivative = (x_error - x_prev_error) / dt.seconds();
    double y_derivative = (y_error - y_prev_error) / dt.seconds();

    x_prev_error = x_error;
    y_prev_error = y_error;

    double x_control_signal = x_kp * x_error + x_ki * x_integral + x_kd * x_derivative;
    double y_control_signal = y_kp * y_error + y_ki * y_integral + y_kd * y_derivative;

    if (x_control_signal >  0.35){
        x_control_signal =  0.35;
    } else if (x_control_signal < -0.35){
        x_control_signal = -0.35;
    }
    if (y_control_signal >  0.35){
        y_control_signal =  0.35;
    } else if (y_control_signal < -0.35){
        y_control_signal = -0.35;
    }
    std::array<double, 3> rpy = { -y_control_signal, x_control_signal, 0.0};

    return rpy;



}
double PID_height_control(double target_height)
{       
    rclcpp::Time current_time = this->get_clock()->now();

    // Calculate the error between the target height and current height
    double error = target_height - pose_.pose.position.z;

    // Update the integral term
    rclcpp::Duration dt = current_time - this->current_time;
    this->current_time = current_time;
    integral += error * dt.seconds();

    // Calculate the derivative term
    double derivative = (error - prev_error) / dt.seconds();

    // Calculate the control signal using PID control
    double control_signal = kp * error + ki * integral + kd * derivative;

    // Adjust the base speed based on the control signal
    double velocity_ = control_signal;

    // Update the previous error for the next iteration
    prev_error = error;

    return velocity_;
}

std::array<double,6> PID_orientation_control(double roll, double pitch, double yaw)
{
    // Get the current orientation of the drone
    tf2::Quaternion quaternion;
    tf2::fromMsg(pose_.pose.orientation, quaternion);
    std::array<double, 3> rpy = get_euler_from_quaternion(quaternion);

    double roll_error = roll - rpy.at(0);
    double pitch_error = pitch - rpy.at(1);
    double yaw_error = yaw - rpy.at(2);

    rclcpp::Time current_time = this->get_clock()->now();

    // Update the integral term
    rclcpp::Duration dt = current_time - this->current_time;
    this->current_time = current_time;
    
    roll_integral += roll_error * dt.seconds();
    pitch_integral += pitch_error * dt.seconds();
    yaw_integral += yaw_error * dt.seconds();

    // Calculate the derivative term
    double roll_derivative = (roll_error - roll_prev_error) / dt.seconds();
    double pitch_derivative = (pitch_error - pitch_prev_error) / dt.seconds();
    double yaw_derivative = (yaw_error - yaw_prev_error) / dt.seconds();

    // Calculate the control signal using PID control
    double roll_control_signal = roll_kp * roll_error + roll_ki * roll_integral + roll_kd * roll_derivative;
    double pitch_control_signal = pitch_kp * pitch_error + pitch_ki * pitch_integral + pitch_kd * pitch_derivative;
    double yaw_control_signal = yaw_kp * yaw_error + yaw_ki * yaw_integral + yaw_kd * yaw_derivative;
    

    roll_prev_error = roll_error;
    pitch_prev_error = pitch_error;
    yaw_prev_error = yaw_error;

    std::array<double, 6> velocity_ = {roll_control_signal, pitch_control_signal, yaw_control_signal, rpy.at(0), rpy.at(1), rpy.at(2)};

    return velocity_;
    
}

private:
    rclcpp::Publisher<drone_plugin::msg::MotorSpeed>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr kd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr ki_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr kp_sub_;


    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr height_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr x_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr y_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr part_attached_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped pose_;
    double base_speed_ = 117.0;
    // Get the current ROS time
    rclcpp::Time current_time = this->get_clock()->now();
    // Initialize variables for PID control
    double integral = 0.0;
    double prev_error = 0.0;

    // Roll
    double roll_integral = 0.0;
    double roll_prev_error = 0.0;

    // Pitch
    double pitch_integral = 0.0;
    double pitch_prev_error = 0.0;

    // Yaw
    double yaw_integral = 0.0;
    double yaw_prev_error = 0.0;

    //Roll PID
    double roll_kp = 2.5;
    double roll_ki = 0.00001;
    double roll_kd = 0.001;

    //Pitch PID
    double pitch_kp = 2.5;
    double pitch_ki = 0.00001;
    double pitch_kd = 0.001;

    //Yaw PID
    double yaw_kp = 3.5;
    double yaw_ki = 0.00001;
    double yaw_kd = 0.004;


    // X pose 
    double x_integral = 0.0;
    double x_prev_error = 0.0;

    // Y pose
    double y_integral = 0.0;
    double y_prev_error = 0.0;


    double x_kp = 0.03;
    double x_ki = 0.0000000;
    double x_kd = 0.0000008;

    double y_kp = 0.013;
    double y_ki = 0.0000000;
    double y_kd = 0.0000008;

    double kp = 11.5;
    double ki = 0.2;
    double kd = 7.1;

    double target_x = 0.0;
    double target_y = 0.0;
    double target_height = 1.0;

    bool part_attached = false;
    
    
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Fly>());
  rclcpp::shutdown();
  return 0;
}