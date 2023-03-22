#include <chrono>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "r4c_interfaces/action/nav.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

using Nav = r4c_interfaces::action::Nav;
using NavGoalHandle = rclcpp_action::ClientGoalHandle<Nav>;

class GetThePosition : public rclcpp::Node {
 public:
  GetThePosition() : Node("get_the_position") {
    pub_ = this->create_publisher<geometry_msgs::msg::Pose>("/pose/local", 10);
    pose_sub_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry>>(this, "/odometry/global");
    fix_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::NavSatFix>>(this, "/gps/fix");
    sync_ = std::make_shared<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::NavSatFix>>(
        *pose_sub_, *fix_sub_, 10);
    sync_->registerCallback(&GetThePosition::pose_callback, this);
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pub_;
  message_filters::Subscriber<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::NavSatFix>> sync_;

  void pose_callback(const nav_msgs::msg::Odometry::ConstPtr& pose_sub, const sensor_msgs::msg::NavSatFix::ConstPtr& fix_sub) {
    auto pose = geometry_msgs::msg::Pose();
    pose.position = pose_sub->pose.pose.position;
    pose.orientation = pose_sub->pose.pose.orientation;
    pub_->publish(pose);
  }
};

class GoToPosition : public rclcpp::Node {
 public:
  GoToPosition() : Node("go_to_position") {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    _action_server = rclcpp_action::create_server<Nav>(
        this, "/path_server", std::bind(&GoToPosition::execute_callback, this, _1),
        std::bind(&GoToPosition::cancel_callback, this, _1),
        std::bind(&GoToPosition::feedback_callback, this, _1, _2));
  }

 private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::Point current_pose_;
  geometry_msgs::msg::Point target_pose_;
  geometry_msgs::msg::Quaternion current_orientation_;
  rclcpp_action::Server<Nav>::SharedPtr _action_server;

  rclcpp_action::GoalResponse execute_callback(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Nav
