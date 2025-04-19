#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"
#include "angles/angles.h"
#include <vector>
#include <cmath>

class PIDController {
public:
    PIDController(double kp, double ki, double kd)
	: kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}
	
    double compute(double error){
	integral_ += error;
	double derivative = error - prev_error_;
	prev_error_ = error;
	return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_;
    double integral_;
};

class PathFollower : public rclcpp::Node {
public:
  PathFollower() : Node("path_follower_node"), current_index_(0),
                   tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
                   pid_linear_(1.8, 0.0, 0.1), pid_angular_(1.0, 0.0, 0.1),
                   last_linear_speed_(0.0) {

    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/planned_path", 10, std::bind(&PathFollower::pathCallback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(66),  // ~15 Hz
      std::bind(&PathFollower::followPath, this));
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<geometry_msgs::msg::PoseStamped> poses_;
  size_t current_index_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  PIDController pid_linear_;
  PIDController pid_angular_;
  double last_linear_speed_;
  std::vector<double> velocity_log_;

  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    poses_ = msg->poses;
    current_index_ = 0;
    velocity_log_.clear();
    RCLCPP_INFO(this->get_logger(), "Received path with %zu points!", poses_.size());

    if (poses_.empty()) {
      geometry_msgs::msg::Twist stop_cmd;
      vel_pub_->publish(stop_cmd);
      return;
    }
  }

  void followPath() {
    if (poses_.empty() || current_index_ >= poses_.size()) return;

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
      return;
    }

    double robot_x = transform.transform.translation.x;
    double robot_y = transform.transform.translation.y;

    tf2::Quaternion q(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    const auto& target = poses_[current_index_].pose.position;
    double dx = target.x - robot_x;
    double dy = target.y - robot_y;
    double dist = std::sqrt(dx * dx + dy * dy);

    if (dist < 0.1) {
      current_index_++;
      if (current_index_ >= poses_.size()) {
        geometry_msgs::msg::Twist stop_cmd;
        vel_pub_->publish(stop_cmd);
        RCLCPP_INFO(this->get_logger(), "Reached goal!");
        if (!velocity_log_.empty()) {
          double sum = 0.0;
          for (double v : velocity_log_) sum += v;
          double avg = sum / velocity_log_.size();
          RCLCPP_INFO(this->get_logger(), "Average linear velocity: %.3f m/s", avg);
        }
      }
      return;
    }

    double target_yaw = std::atan2(dy, dx);
    double angle_error = angles::shortest_angular_distance(yaw, target_yaw);

    double angular_cmd = pid_angular_.compute(angle_error);
    double linear_cmd = pid_linear_.compute(dist);

    // adaptive scaling
    double turn_scale = std::max(0.1, 1.0 - std::abs(angle_error));  // slowing down for sharper turns
    double goal_scale = std::min(1.0, dist / 0.5);  // slowing down near goal 
    double scale = std::min(turn_scale, goal_scale);

    linear_cmd *= scale;

    geometry_msgs::msg::Twist cmd_vel;
    if (std::abs(angle_error) > 0.5) {
      cmd_vel.linear.x = 0.0;
      cmd_vel.angular.z = std::clamp(angular_cmd, -0.6, 0.6);
    } else {
      cmd_vel.linear.x = std::clamp(linear_cmd, 0.22, 0.35);
      cmd_vel.angular.z = std::clamp(angular_cmd, -0.5, 0.5);
    }

    // velocity smoothing
    double alpha = 0.3;
    last_linear_speed_ = alpha * cmd_vel.linear.x + (1 - alpha) * last_linear_speed_;
    cmd_vel.linear.x = last_linear_speed_;

    vel_pub_->publish(cmd_vel);

    velocity_log_.push_back(cmd_vel.linear.x);
    RCLCPP_INFO(this->get_logger(), "Velocity: %.3f m/s", cmd_vel.linear.x);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathFollower>());
  rclcpp::shutdown();
  return 0;
}
