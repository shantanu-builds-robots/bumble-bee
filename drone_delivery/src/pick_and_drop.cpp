#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <drone_plugin/msg/motor_speed.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <kdl/frames.hpp>
#include <tf2_kdl/tf2_kdl.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <array>
#include <string>
#include <utility>
#include <vector>

// One leg's minimum-snap trajectory, as written by bumble-bee-gym's plan_mission.py: per segment a duration
// and 8 polynomial coefficients per axis in normalised time s = (t - segment start) / duration.
struct Trajectory
{
    std::vector<double> durations;
    std::vector<std::array<double, 8>> cx, cy;

    double total() const
    {
        double t = 0.0;
        for (double d : durations) t += d;
        return t;
    }

    // Position, velocity and acceleration at time t (clamped to the trajectory): {x, y, vx, vy, ax, ay}.
    std::array<double, 6> at(double t) const
    {
        t = std::clamp(t, 0.0, total());
        size_t i = 0;
        while (i + 1 < durations.size() && t > durations[i]) {
            t -= durations[i];
            ++i;
        }
        const double T = durations[i];
        const double s = std::min(t / T, 1.0);
        std::array<double, 6> out{};
        for (int k = 0; k < 8; ++k) {
            const double p = std::pow(s, k);
            const double dp = k >= 1 ? k * std::pow(s, k - 1) : 0.0;
            const double ddp = k >= 2 ? k * (k - 1) * std::pow(s, k - 2) : 0.0;
            out[0] += cx[i][k] * p;
            out[1] += cy[i][k] * p;
            out[2] += cx[i][k] * dp / T;
            out[3] += cy[i][k] * dp / T;
            out[4] += cx[i][k] * ddp / (T * T);
            out[5] += cy[i][k] * ddp / (T * T);
        }
        return out;
    }
};


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
    route_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Deliver::route_callback, this), timer_callback_group_);
    traj_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&Deliver::trajectory_callback, this), timer_callback_group_);

    // How legs are flown: "waypoints" steers at each corner of the planned route in turn; "trajectory"
    // follows the mission's minimum-snap curve as a moving target with velocity and acceleration.
    route_mode_ = this->declare_parameter<std::string>("route_mode", "waypoints");
    if (route_mode_ != "waypoints" && route_mode_ != "trajectory") {
        throw std::invalid_argument("route_mode must be waypoints or trajectory, got " + route_mode_);
    }

    // Optional mission file (planned by bumble-bee-gym's scripts/plan_mission.py): pickups, drops and
    // the waypoints that route each leg around obstacles. Without one, every leg is a straight line.
    const std::string mission = this->declare_parameter<std::string>("mission", "");
    if (!mission.empty()) {
        load_mission(mission);
    }
    if (route_mode_ == "trajectory" && (traj_to_home_.durations.empty() || traj_to_drop_.empty())) {
        throw std::invalid_argument("route_mode trajectory needs a mission file with trajectories");
    }
    RCLCPP_INFO(get_logger(), "Route mode: %s", route_mode_.c_str());
    for (size_t i = 0; i < pick_up_locations.size(); ++i) {
        if (to_pickup_.size() <= i) to_pickup_.push_back({pick_up_locations[i]});
        if (to_drop_.size() <= i) to_drop_.push_back({drop_off_locations[i]});
    }
    if (to_home_.empty()) to_home_ = {{home_x, home_y}};
    pick_up_x = pick_up_locations[0].first;
    pick_up_y = pick_up_locations[0].second;
    drop_off_x = drop_off_locations[0].first;
    drop_off_y = drop_off_locations[0].second;


    height_pub_ = this->create_publisher<std_msgs::msg::Float64>("height", 10);
    ref_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("target_ref", 10);
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

    using XY = std::pair<double, double>;

    static std::vector<XY> read_points(const YAML::Node &node)
    {
        std::vector<XY> pts;
        for (const auto &p : node) {
            pts.emplace_back(p[0].as<double>(), p[1].as<double>());
        }
        return pts;
    }

    static Trajectory read_trajectory(const YAML::Node &node)
    {
        Trajectory tr;
        tr.durations = node["durations"].as<std::vector<double>>();
        for (const auto &c : node["x"]) {
            tr.cx.push_back(c.as<std::array<double, 8>>());
        }
        for (const auto &c : node["y"]) {
            tr.cy.push_back(c.as<std::array<double, 8>>());
        }
        if (tr.cx.size() != tr.durations.size() || tr.cy.size() != tr.durations.size()) {
            throw std::runtime_error("trajectory: one duration and one coefficient row per axis per segment");
        }
        return tr;
    }

    void load_mission(const std::string &path)
    {
        const YAML::Node m = YAML::LoadFile(path);
        cruise_height = m["cruise"].as<double>();
        home_height = cruise_height;
        home_x = m["home"][0].as<double>();
        home_y = m["home"][1].as<double>();
        pick_up_locations.clear();
        drop_off_locations.clear();
        for (const auto &d : m["deliveries"]) {
            pick_up_locations.emplace_back(d["pickup"][0].as<double>(), d["pickup"][1].as<double>());
            drop_off_locations.emplace_back(d["drop"][0].as<double>(), d["drop"][1].as<double>());
            to_pickup_.push_back(read_points(d["to_pickup"]));
            to_drop_.push_back(read_points(d["to_drop"]));
            if (d["traj_to_pickup"] && d["traj_to_drop"]) {
                traj_to_pickup_.push_back(read_trajectory(d["traj_to_pickup"]));
                traj_to_drop_.push_back(read_trajectory(d["traj_to_drop"]));
            }
        }
        to_home_ = read_points(m["to_home"]);
        if (m["traj_to_home"]) {
            traj_to_home_ = read_trajectory(m["traj_to_home"]);
        }
        RCLCPP_INFO(get_logger(), "Mission %s: %zu deliveries, cruise %.2f m", path.c_str(),
                    pick_up_locations.size(), cruise_height);
    }

    // Fly a leg. Waypoint mode: the route timer steers through the waypoints, ending on the leg's target.
    // Trajectory mode: the trajectory timer plays the leg's curve from now on.
    void start_leg(const std::string &name, const std::vector<XY> &waypoints, const Trajectory &trajectory)
    {
        leg_name_ = name;
        leg_start_ = pose_time_;
        RCLCPP_INFO(get_logger(), "LEG start %s t=%.3f", name.c_str(), leg_start_);
        if (route_mode_ == "trajectory") {
            trajectory_ = &trajectory;
            waiting_for_cruise_ = true;
            trajectory_callback();
            return;
        }
        route_ = waypoints;
        route_index_ = 0;
        route_callback();
    }

    void end_leg()
    {
        const double t = pose_time_;
        RCLCPP_INFO(get_logger(), "LEG arrive %s t=%.3f took=%.3f", leg_name_.c_str(), t, t - leg_start_);
    }

    void trajectory_callback()
    {
        if (trajectory_ == nullptr) {
            return;
        }
        // Hold the start point until the drone is at cruise height. Legs from a pickup start low, and
        // climbing while tilted adds forward thrust the trajectory does not know about.
        if (waiting_for_cruise_) {
            if (std::abs(height - cruise_height) > kCruiseTolerance) {
                const auto r0 = trajectory_->at(0.0);
                std_msgs::msg::Float64MultiArray hold;
                hold.data.assign(r0.begin(), r0.end());
                ref_pub_->publish(hold);
                return;
            }
            waiting_for_cruise_ = false;
            traj_start_ = pose_time_;
            RCLCPP_INFO(get_logger(), "LEG play %s t=%.3f", leg_name_.c_str(), traj_start_);
        }
        // Past the end this keeps publishing the final point at rest, which holds the drone there.
        const auto r = trajectory_->at(pose_time_ - traj_start_);
        std_msgs::msg::Float64MultiArray msg;
        msg.data.assign(r.begin(), r.end());
        ref_pub_->publish(msg);
    }

    void route_callback()
    {
        if (route_.empty()) {
            return;
        }
        const bool last = route_index_ + 1 >= route_.size();
        const XY &wp = route_[route_index_];
        if (!last && std::hypot(x - wp.first, y - wp.second) < kWaypointTolerance) {
            ++route_index_;
            RCLCPP_INFO(get_logger(), "Waypoint %zu/%zu", route_index_ + 1, route_.size());
        }
        std_msgs::msg::Float64 x_msg, y_msg;
        x_msg.data = route_[route_index_].first;
        y_msg.data = route_[route_index_].second;
        x_target_pub_->publish(x_msg);
        y_target_pub_->publish(y_msg);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        pose_ = *msg;
        pose_time_ = rclcpp::Time(msg->header.stamp).seconds();
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
            if (!leg_started_) {
                start_leg("to_pickup_" + std::to_string(index), to_pickup_[index], traj_or_empty(traj_to_pickup_));
                leg_started_ = true;
            }
            std_msgs::msg::Float64 height_msg;
            height_msg.data = cruise_height;
            height_pub_->publish(height_msg);
            // rclcpp::sleep_for(std::chrono::milliseconds(100));
            if (std::abs(x - pick_up_x) > 0.01 || std::abs(y - pick_up_y) > 0.01){
                
                RCLCPP_INFO(get_logger(), "x: %f, y: %f", x, y);
                return;
            }
            go_to_location = false;
            leg_started_ = false;
            end_leg();

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
            if (!leg_started_) {
                start_leg("to_drop_" + std::to_string(index), to_drop_[index], traj_or_empty(traj_to_drop_));
                leg_started_ = true;
            }
            std_msgs::msg::Float64 height_msg;
            height_msg.data = cruise_height;
            height_pub_->publish(height_msg);
            if (std::abs(x - drop_off_x) > 0.2 || std::abs(y - drop_off_y) > 0.2){
                // rclcpp::sleep_for(std::chrono::milliseconds(100));
                RCLCPP_INFO(get_logger(), "x: %f, y: %f", x, y);
                return;
            }
            go_to_location = false;
            leg_started_ = false;
            end_leg();
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
                if (!leg_started_) {
                    start_leg("to_home", to_home_, traj_to_home_);
                    leg_started_ = true;
                }
                std_msgs::msg::Float64 height_msg;
                height_msg.data = home_height;
                height_pub_->publish(height_msg);
                if (std::abs(x - home_x) > 0.11 || std::abs(y - home_y) > 0.11){
                    // rclcpp::sleep_for(std::chrono::milliseconds(100));
                    RCLCPP_INFO(get_logger(), "x: %f, y: %f", x, y);
                    return;
                }
                go_to_location = false;
                leg_started_ = false;
                end_leg();
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
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr ref_pub_;
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
    rclcpp::TimerBase::SharedPtr route_timer_;
    rclcpp::TimerBase::SharedPtr traj_timer_;

    const Trajectory &traj_or_empty(const std::vector<Trajectory> &legs) const
    {
        return static_cast<size_t>(index) < legs.size() ? legs[index] : kNoTrajectory;
    }

    inline static const Trajectory kNoTrajectory{};
    std::string route_mode_;
    std::vector<Trajectory> traj_to_pickup_, traj_to_drop_;
    Trajectory traj_to_home_;
    const Trajectory *trajectory_ = nullptr;   // leg being flown in trajectory mode
    std::string leg_name_;
    // Time base for legs: the pose message's stamp, which the plugin sets to the sim time of that state. The
    // node clock is no substitute: with use_sim_time it only advances when /clock does (10 Hz by default).
    double pose_time_ = 0.0;                   // [s]
    double leg_start_ = 0.0;                   // [s] pose time when the current leg was requested
    double traj_start_ = 0.0;                  // [s] pose time when its trajectory started playing
    bool waiting_for_cruise_ = false;
    static constexpr double kCruiseTolerance = 0.05;  // [m]

    // Route of the current leg (one waypoint per straight segment; the last is the leg's target)
    static constexpr double kWaypointTolerance = 0.3;  // [m] switch to the next waypoint this close
    std::vector<XY> route_;
    size_t route_index_ = 0;
    bool leg_started_ = false;
    std::vector<std::vector<XY>> to_pickup_, to_drop_;
    std::vector<XY> to_home_;
    double cruise_height = 0.7;  // transit height [m]

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