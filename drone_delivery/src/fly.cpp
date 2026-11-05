#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <drone_plugin/msg/motor_speed.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <algorithm>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


class Fly : public rclcpp::Node
{

public:
  Fly() : Node("fly")
  {
    RCLCPP_INFO(get_logger(), "Fly node has been created.");
    publisher_ = this->create_publisher<drone_plugin::msg::MotorSpeed>("motor_speed", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("drone_pose", 10,std::bind(&Fly::pose_callback, this, std::placeholders::_1));
    // 100 Hz control loop: attitude control on a 0.73 kg quad needs a fast loop
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&Fly::timer_callback, this));


    height_sub_ = this->create_subscription<std_msgs::msg::Float64>("height", 10, std::bind(&Fly::height_callback, this, std::placeholders::_1));
    x_sub_ = this->create_subscription<std_msgs::msg::Float64>("target_x", 10, std::bind(&Fly::x_callback, this, std::placeholders::_1));
    y_sub_ = this->create_subscription<std_msgs::msg::Float64>("target_y", 10, std::bind(&Fly::y_callback, this, std::placeholders::_1));
    // Moving reference from a trajectory: [x, y, vx, vy, ax, ay]. Sets the target and feeds its velocity
    // and acceleration forward. Plain target_x / target_y messages leave both at zero.
    ref_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("target_ref", 10, std::bind(&Fly::ref_callback, this, std::placeholders::_1));

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
            base_speed_ = kHoverSpeedLoaded;
            kp = 143.3;
            ki = 45.5;
            kd = 113.7;

        } else {
            RCLCPP_INFO_STREAM(get_logger(), "Part detached Gain Changed");
            base_speed_ = kHoverSpeed;
            kp = 126.0;
            ki = 40.0;
            kd = 100.0;
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
            ref_vx_ = ref_vy_ = ref_ax_ = ref_ay_ = 0.0;  // a fixed target: nothing to feed forward
            // y_kp = msg->data;
        }
    
    void y_callback(const std_msgs::msg::Float64::SharedPtr msg)
        {
            target_y = msg->data;
            ref_vx_ = ref_vy_ = ref_ax_ = ref_ay_ = 0.0;
            // y_ki = msg->data;
        }
    
    void ref_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
        {
            if (msg->data.size() != 6) {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "target_ref needs 6 values, got %zu", msg->data.size());
                return;
            }
            target_x = msg->data[0];
            target_y = msg->data[1];
            ref_vx_ = msg->data[2];
            ref_vy_ = msg->data[3];
            ref_ax_ = msg->data[4];
            ref_ay_ = msg->data[5];
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
    // One time step per tick, shared by every loop. Time is the pose message's stamp, the sim time of
    // that state: the wall clock drifts from it whenever Gazebo runs slower than real time, and then
    // every velocity estimate (and so all the damping) is off by the same factor.
    const rclcpp::Time now(pose_.header.stamp);
    if (have_tick_ && now == last_tick_) {
        // No new state since the last tick: repeat the last command rather than differentiate zero motion.
        if (have_command_) publisher_->publish(last_command_);
        return;
    }
    const bool first_tick = !have_tick_;
    double dt = first_tick ? kNominalDt : (now - last_tick_).seconds();
    last_tick_ = now;
    have_tick_ = true;
    if (first_tick || dt <= 0.0 || dt > 0.5) {
        dt = kNominalDt;
    }

    if (target_height == 0.0){
        float prop_1 = 0.0;
        float prop_2 = 0.0;
        float prop_3 = 0.0;
        float prop_4 = 0.0;

        auto message = drone_plugin::msg::MotorSpeed();
        message.name = std::vector<std::string>{"prop_1","prop_2","prop_3","prop_4"};
        message.velocity = std::vector<float>{prop_1, -prop_2, prop_3, -prop_4};
        publish_command(message);
    }
    else {
        // Calculate the control signal for height control
        double velocity_ = PID_height_control(target_height, dt, first_tick);

        std::array<double, 3> rpy = getRollPitchYaw(target_x, target_y, dt, first_tick);
        // Calculate the control signal for orientation control
        std::array<double, 6> orientation_velocity = PID_orientation_control(rpy.at(0), rpy.at(1), rpy.at(2), dt, first_tick);

        float prop_1 = static_cast<float>(base_speed_+ velocity_  + orientation_velocity.at(0) + orientation_velocity.at(1) + orientation_velocity.at(2));
        float prop_2 = static_cast<float>(base_speed_ + velocity_ + orientation_velocity.at(0) - orientation_velocity.at(1) - orientation_velocity.at(2));
        float prop_3 = static_cast<float>(base_speed_ + velocity_ - orientation_velocity.at(0) - orientation_velocity.at(1) + orientation_velocity.at(2));
        float prop_4 = static_cast<float>(base_speed_ + velocity_ - orientation_velocity.at(0) + orientation_velocity.at(1) - orientation_velocity.at(2));

        auto message = drone_plugin::msg::MotorSpeed();
        message.name = std::vector<std::string>{"prop_1","prop_2","prop_3","prop_4"};
        message.velocity = std::vector<float>{prop_1, -prop_2, prop_3, -prop_4};
        publish_command(message);

        // Log once a second; the loop runs at 100 Hz
        RCLCPP_INFO_STREAM_THROTTLE(get_logger(), *get_clock(), 1000,
            "Props: " << prop_1 << " " << prop_2 << " " << prop_3 << " " << prop_4
            << " | Height: " << pose_.pose.position.z
            << " | Roll/Pitch/Yaw: " << orientation_velocity.at(3) << " " << orientation_velocity.at(4) << " " << orientation_velocity.at(5)
            << " | Target: " << target_x << " " << target_y << " " << target_height);
    }

}

// Position loop: x/y error -> desired tilt [rad], clamped to +-0.35 rad.
// Derivative is taken on the measurement so a new waypoint does not cause a kick.
std::array<double, 3> getRollPitchYaw(double target_x, double target_y, double dt, bool first_tick){
    const double x = pose_.pose.position.x;
    const double y = pose_.pose.position.y;
    if (first_tick) {
        x_prev_ = x;
        y_prev_ = y;
    }

    const double x_error = target_x - x;
    const double y_error = target_y - y;

    x_integral += x_error * dt;
    y_integral += y_error * dt;

    // Velocity error against the reference's velocity (zero when holding a fixed target, which makes this
    // derivative-on-measurement), so a moving target is tracked without a lag of kd/kp times its speed.
    const double x_derivative = ref_vx_ - (x - x_prev_) / dt;
    const double y_derivative = ref_vy_ - (y - y_prev_) / dt;
    x_prev_ = x;
    y_prev_ = y;

    // Feedforward: the tilt that produces the reference's acceleration, a = g tan(tilt).
    double x_control_signal = x_kp * x_error + x_ki * x_integral + x_kd * x_derivative + std::atan(ref_ax_ / kGravity);
    double y_control_signal = y_kp * y_error + y_ki * y_integral + y_kd * y_derivative + std::atan(ref_ay_ / kGravity);

    x_control_signal = std::clamp(x_control_signal, -kMaxTilt, kMaxTilt);
    y_control_signal = std::clamp(y_control_signal, -kMaxTilt, kMaxTilt);

    std::array<double, 3> rpy = { -y_control_signal, x_control_signal, 0.0};

    return rpy;
}

// Height loop: height error -> change in all four rotor speeds [rad/s]
double PID_height_control(double target_height, double dt, bool first_tick)
{
    const double z = pose_.pose.position.z;
    if (first_tick) {
        z_prev_ = z;
    }
    const double error = target_height - z;

    integral = std::clamp(integral + error * dt, -kMaxHeightIntegral, kMaxHeightIntegral);
    const double derivative = -(z - z_prev_) / dt;
    z_prev_ = z;

    return kp * error + ki * integral + kd * derivative;
}

// Attitude loop: desired roll/pitch/yaw -> rotor-speed differences [rad/s].
// Returns {roll, pitch, yaw outputs, measured roll, pitch, yaw}.
std::array<double,6> PID_orientation_control(double roll, double pitch, double yaw, double dt, bool first_tick)
{
    // Get the current orientation of the drone
    tf2::Quaternion quaternion;
    tf2::fromMsg(pose_.pose.orientation, quaternion);
    std::array<double, 3> rpy = get_euler_from_quaternion(quaternion);
    if (first_tick) {
        roll_prev_ = rpy.at(0);
        pitch_prev_ = rpy.at(1);
        yaw_prev_ = rpy.at(2);
    }

    const double roll_error = roll - rpy.at(0);
    const double pitch_error = pitch - rpy.at(1);
    const double yaw_error = std::remainder(yaw - rpy.at(2), 2.0 * M_PI);

    roll_integral = std::clamp(roll_integral + roll_error * dt, -kMaxAttitudeIntegral, kMaxAttitudeIntegral);
    pitch_integral = std::clamp(pitch_integral + pitch_error * dt, -kMaxAttitudeIntegral, kMaxAttitudeIntegral);
    yaw_integral = std::clamp(yaw_integral + yaw_error * dt, -kMaxAttitudeIntegral, kMaxAttitudeIntegral);

    const double roll_derivative = -(rpy.at(0) - roll_prev_) / dt;
    const double pitch_derivative = -(rpy.at(1) - pitch_prev_) / dt;
    const double yaw_derivative = -std::remainder(rpy.at(2) - yaw_prev_, 2.0 * M_PI) / dt;
    roll_prev_ = rpy.at(0);
    pitch_prev_ = rpy.at(1);
    yaw_prev_ = rpy.at(2);

    const double roll_control_signal = roll_kp * roll_error + roll_ki * roll_integral + roll_kd * roll_derivative;
    const double pitch_control_signal = pitch_kp * pitch_error + pitch_ki * pitch_integral + pitch_kd * pitch_derivative;
    const double yaw_control_signal = yaw_kp * yaw_error + yaw_ki * yaw_integral + yaw_kd * yaw_derivative;

    return {roll_control_signal, pitch_control_signal, yaw_control_signal, rpy.at(0), rpy.at(1), rpy.at(2)};
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
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr ref_sub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr part_attached_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::PoseStamped pose_;
    // Rotor speeds [rad/s] that hold the drone in a hover, from the plugin's thrust map
    // (c2 w^2 + c1 w + c0 per rotor) with a 0.73 kg drone, and with a 0.1 kg parcel attached, at g = 9.8 m/s^2
    // (the world's gravity).
    static constexpr double kHoverSpeed = 738.4;
    static constexpr double kHoverSpeedLoaded = 780.2;
    double base_speed_ = kHoverSpeed;
    // Loop timing
    static constexpr double kNominalDt = 0.01;
    rclcpp::Time last_tick_;
    drone_plugin::msg::MotorSpeed last_command_;
    bool have_command_ = false;

    void publish_command(const drone_plugin::msg::MotorSpeed &message)
    {
        publisher_->publish(message);
        last_command_ = message;
        have_command_ = true;
    }
    bool have_tick_ = false;

    // Limits
    static constexpr double kMaxTilt = 0.35;               // rad
    static constexpr double kGravity = 9.8;                // m/s^2, as in the worlds
    static constexpr double kMaxHeightIntegral = 1.0;      // m s
    static constexpr double kMaxAttitudeIntegral = 0.5;    // rad s

    // Previous measurements, for derivatives on the measurement
    double z_prev_ = 0.0;
    double x_prev_ = 0.0;
    double y_prev_ = 0.0;
    double roll_prev_ = 0.0;
    double pitch_prev_ = 0.0;
    double yaw_prev_ = 0.0;
    // Initialize variables for PID control
    double integral = 0.0;

    // Roll
    double roll_integral = 0.0;

    // Pitch
    double pitch_integral = 0.0;

    // Yaw
    double yaw_integral = 0.0;

    //Roll PID
    double roll_kp = 128;
    double roll_ki = 40;
    double roll_kd = 25.6;

    //Pitch PID
    double pitch_kp = 128;
    double pitch_ki = 40;
    double pitch_kd = 25.6;

    //Yaw PID
    double yaw_kp = 148;
    double yaw_ki = 10;
    double yaw_kd = 79;


    // X pose 
    double x_integral = 0.0;

    // Y pose
    double y_integral = 0.0;


    double x_kp = 0.065;
    double x_ki = 0.0;
    double x_kd = 0.147;

    double y_kp = 0.065;
    double y_ki = 0.0;
    double y_kd = 0.147;

    // Gains come from the plant model: near hover one rotor gives dT/dw = 0.0058 N per rad/s, the arm is
    // 0.12 m and the drone is 0.73 kg with inertia diag(0.003942, 0.003942, 0.006132) kg m^2.
    //   height:      z''   = 0.0318 * u      -> wn 2 rad/s, damping 0.8
    //   roll, pitch: acc   = 0.5 * u         -> wn 8 rad/s, damping 0.8
    //   yaw:         acc   = 0.0606 * u      -> wn 3 rad/s, damping 0.8
    //   x, y:        x''   = 9.81 * tilt     -> wn 0.8 rad/s, damping 0.9
    // Carrying a 0.1 kg parcel scales the height gains by 0.83 / 0.73.
    double kp = 126.0;
    double ki = 40.0;
    double kd = 100.0;

    double target_x = 0.0;
    double target_y = 0.0;
    double ref_vx_ = 0.0, ref_vy_ = 0.0;   // [m/s] reference velocity (trajectory following only)
    double ref_ax_ = 0.0, ref_ay_ = 0.0;   // [m/s^2] reference acceleration
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