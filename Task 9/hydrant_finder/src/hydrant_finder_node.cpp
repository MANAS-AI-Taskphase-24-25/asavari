#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/laser_scan.hpp" 
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>

class HydrantFinder : public rclcpp::Node
{
public:
    HydrantFinder() : Node("hydrant_finder_node"),
                      tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_),
                      hydrant_found_(false), camera_info_received_(false)
    {
        image_sub_ = image_transport::create_subscription(
            this, "/camera/image_raw",
            std::bind(&HydrantFinder::imageCallback, this, std::placeholders::_1),
            "raw");

        cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", 10,
            std::bind(&HydrantFinder::cameraInfoCallback, this, std::placeholders::_1));

        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&HydrantFinder::lidarCallback, this, std::placeholders::_1));

        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    }

private:
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    int hydrant_u_, hydrant_v_;
    bool hydrant_found_, camera_info_received_;
    float fx_, fy_, cx_, cy_;   
    std::vector<float> lidar_ranges_;

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        camera_info_received_ = true;
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
    {
        cv::Mat frame;
        try
        {
            frame = cv_bridge::toCvShare(msg, "bgr8")->image;
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        cv::Scalar lower_red(0, 220, 40);
        cv::Scalar upper_red(5, 255, 170);
        cv::Mat mask;
        cv::inRange(hsv, lower_red, upper_red, mask);

        cv::erode(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty())
        {
            auto largest = *std::max_element(contours.begin(), contours.end(),
                                             [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                                             { return cv::contourArea(a) < cv::contourArea(b); });

            cv::Rect bbox = cv::boundingRect(largest);
            hydrant_u_ = bbox.x + bbox.width / 2;
            hydrant_v_ = bbox.y + bbox.height / 2;
            hydrant_found_ = true;

            RCLCPP_INFO(this->get_logger(), "Hydrant detected at pixel (%d, %d)", hydrant_u_, hydrant_v_);

            if (camera_info_received_ && !lidar_ranges_.empty())
            {
                // pixel coordinates -> 3D coordinates
                float u = hydrant_u_;
                float v = hydrant_v_;
                
                float angle = (u - cx_) / fx_;  // Camera to LiDAR angle relation
                size_t lidar_index = static_cast<size_t>(angle);
                
                if (lidar_index < lidar_ranges_.size())
                {
                    float depth = lidar_ranges_[lidar_index]; // depth
                    
                    float x = (u - cx_) * depth / fx_;
                    float y = (v - cy_) * depth / fy_;
                    float z = depth;  
                    
                    geometry_msgs::msg::PointStamped hydrant_point_camera;
                    hydrant_point_camera.header.frame_id = "camera_link";
                    hydrant_point_camera.point.x = x;
                    hydrant_point_camera.point.y = y;
                    hydrant_point_camera.point.z = z;

                    try
                    {                        
                        geometry_msgs::msg::PointStamped hydrant_point_map = tf_buffer_.transform(hydrant_point_camera, "map", tf2::durationFromSec(0.5));
                        RCLCPP_INFO(this->get_logger(), "Hydrant point in map frame: (%.2f, %.2f)", hydrant_point_map.point.x, hydrant_point_map.point.y);
                        
                        publishGoal(hydrant_point_map.point);
                    }
                    catch (tf2::TransformException &ex)
                    {
                        RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
                    }
                }
            }
        }
    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        lidar_ranges_ = msg->ranges; // to store LiDAR ranges
    }

    void publishGoal(const geometry_msgs::msg::Point &hydrant_point)
    {
        geometry_msgs::msg::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = this->now();

        double dx = hydrant_point.x;
        double dy = hydrant_point.y;
        double dist = std::sqrt(dx * dx + dy * dy);

        // 3 feet away (0.91 meters) from the hydrant
        if (dist < 0.91)
        {
            RCLCPP_WARN(this->get_logger(), "Hydrant too close (%.2f m), skipping goal.", dist);
            return;
        }

        double scale = (dist - 0.91) / dist;
        goal.pose.position.x = dx * scale;
        goal.pose.position.y = dy * scale;
        goal.pose.orientation.w = 1.0;

        goal_pub_->publish(goal);
        RCLCPP_INFO(this->get_logger(), "Published goal 3 feet from hydrant (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HydrantFinder>());
    rclcpp::shutdown();
    return 0;
}
