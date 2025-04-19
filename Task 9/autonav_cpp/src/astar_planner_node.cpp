#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "autonav_cpp/cubic_spline.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include <unordered_map>

using autonav_cpp::CubicSpline2D;

struct AStarNode
{
    int x, y;
    float g_cost, h_cost;
    AStarNode *parent;
    float total_cost() const { return g_cost + h_cost; }
    bool operator>(const AStarNode &other) const { return total_cost() > other.total_cost(); }
};

class Planner : public rclcpp::Node
{
public:
    Planner() : Node("planner"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
    {
        heuristic_type_ = declare_parameter("heuristic", "manhattan");

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&Planner::mapCallback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&Planner::goalCallback, this, std::placeholders::_1));

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);   
                
    }

private:
    float map_resolution_;
    float origin_x_, origin_y_;
    int goal_x_ = -1, goal_y_ = -1;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;    

    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    geometry_msgs::msg::PoseStamped::SharedPtr cached_goal_;
    
    std::string heuristic_type_;    
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;     
    
    // graded inflation to avoid obstacles
    void applyGradedInflation(std::vector<int8_t> &data, int width, int height, int inflation_radius)
    {
	std::vector<int8_t> inflated = data;
        int max_cost = 100;

	for (int y = 0; y < height; ++y)
	{
	    for (int x = 0; x < width; ++x)
	    {
		int index = y * width + x;
		if (data[index] >= 100)
		{
		    for (int dy = -inflation_radius; dy <= inflation_radius; ++dy)
		    {
		        for (int dx = -inflation_radius; dx <= inflation_radius; ++dx)
		        {
		            int nx = x + dx, ny = y + dy;
		            if (nx < 0 || ny < 0 || nx >= width || ny >= height)
		               continue;

		            double dist = std::sqrt(dx * dx + dy * dy);
		            if (dist <= inflation_radius)
		            {
		                int cost = std::max(1, static_cast<int>(max_cost - (dist * (max_cost / inflation_radius))));
		                int nidx = ny * width + nx;
		                inflated[nidx] = std::max(inflated[nidx], static_cast<int8_t>(cost));
		            }
		        }
		    }
		}
	    }
	}data = inflated;
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        latest_map_ = msg;
        map_resolution_ = msg->info.resolution;
        RCLCPP_INFO(this->get_logger(), "Map resolution: %f meters", map_resolution_);
        origin_x_ = msg->info.origin.position.x;
        origin_y_ = msg->info.origin.position.y;
        attemptReplan();
    }

    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        cached_goal_=msg;
        worldToGrid(msg->pose.position.x, msg->pose.position.y, goal_x_, goal_y_);
        RCLCPP_INFO(this->get_logger(), "Goal received: (%d, %d)", goal_x_, goal_y_);
        attemptReplan();
    }
    
    // checking if goal is valid ie outside world boundary or near an obstacle
    bool isValidGoal(double goal_x, double goal_y) 
    {
        if (!latest_map_) 
        {
            RCLCPP_WARN(this->get_logger(), "Map not received yet.");
            return false;
        }

        const auto& map = latest_map_->info;
        
        int mx = static_cast<int>((goal_x - map.origin.position.x) / map.resolution);
        int my = static_cast<int>((goal_y - map.origin.position.y) / map.resolution);

        if (mx < 0 || my < 0 || mx >= static_cast<int>(map.width) || my >=      static_cast<int>(map.height)) 
        {
            RCLCPP_WARN(this->get_logger(), "Goal is outside map boundaries.");
            return false;
        }

        int idx = my * map.width + mx;
        int cost = latest_map_->data[idx];

        if (cost > 50) 
        {  
            RCLCPP_WARN(this->get_logger(), "Goal is dangerously near an obstacle.");
            return false;
        }

        return true;
    }    
          
    void attemptReplan()
    {
        if (!latest_map_)
        {
            RCLCPP_WARN(this->get_logger(), "Map not received.");
            return;
        }

        if (!cached_goal_)
        {
            RCLCPP_WARN(this->get_logger(), "Goal not set yet.");
            return;
        }
        
        double goal_wx=cached_goal_->pose.position.x;
        double goal_wy=cached_goal_->pose.position.y;        
        worldToGrid(goal_wx, goal_wy, goal_x_, goal_y_);
        
        if (!isValidGoal(goal_wx, goal_wy)) 
        {
            RCLCPP_WARN(this->get_logger(), "Invalid goal. Not planning path.");
            return;
        }

        geometry_msgs::msg::TransformStamped transform;
        try
        {
            transform = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
            return;
        }

        int start_x, start_y;
        worldToGrid(transform.transform.translation.x, transform.transform.translation.y, start_x, start_y);
        RCLCPP_INFO(this->get_logger(), "Current pose: (%d, %d)", start_x, start_y);  
        
        applyGradedInflation(latest_map_->data, latest_map_->info.width, latest_map_->info.height, 11);      
        
        auto path = findPath(latest_map_, start_x, start_y, goal_x_, goal_y_);        
        publishPath(path);
    }

    void worldToGrid(float wx, float wy, int &gx, int &gy)
    {
        gx = static_cast<int>((wx - origin_x_) / map_resolution_);
        gy = static_cast<int>((wy - origin_y_) / map_resolution_);
    }

    // A star
    float calculateHeuristic(int x1, int y1, int x2, int y2)
    {
        if (heuristic_type_ == "euclidean")
            return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
        return std::abs(x2 - x1) + std::abs(y2 - y1);
    }

    std::vector<AStarNode> findPath(const nav_msgs::msg::OccupancyGrid::SharedPtr &map, int start_x, int start_y, int goal_x, int goal_y)
    {
        int width = map->info.width, height = map->info.height;
        std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> open_set;
        std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));
        open_set.push({start_x, start_y, 0, calculateHeuristic(start_x, start_y, goal_x, goal_y), nullptr});

        while (!open_set.empty())
        {
            AStarNode current = open_set.top();
            open_set.pop();

            if (current.x == goal_x && current.y == goal_y)
            {
                std::vector<AStarNode> path;
                while (current.parent)
                {
                    path.push_back(current);
                    current = *current.parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }

            if (visited[current.x][current.y])
                continue;
            visited[current.x][current.y] = true;

            for (auto [dx, dy] : std::vector<std::pair<int, int>>{{1, 0}, {-1, 0}, {0, 1}, {0, -1}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}})
            {
                int nx = current.x + dx, ny = current.y + dy;
                if (nx >= 0 && ny >= 0 && nx < width && ny < height)
                {
                    int index = ny * width + nx;
                    if (map->data[index] < 50)
                    {
                        float cost = (dx != 0 && dy != 0) ? 1.414 : 1.0;
                        open_set.push({nx, ny, current.g_cost + cost, calculateHeuristic(nx, ny, goal_x, goal_y), new AStarNode(current)});
                    }
                }
            }
        }
        return {};
    }

    void publishPath(const std::vector<AStarNode> &path)
    {
        nav_msgs::msg::Path ros_path;
        ros_path.header.stamp = this->now();
        ros_path.header.frame_id = "map";

        if (path.size() < 3)
        {            
            for (const auto &node : path)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = ros_path.header;
                pose.pose.position.x = node.x * map_resolution_ + origin_x_;
                pose.pose.position.y = node.y * map_resolution_ + origin_y_;
                pose.pose.orientation.w = 1.0;
                ros_path.poses.push_back(pose);
            }
            path_pub_->publish(ros_path);
            return;
         }

        // path to world 
        std::vector<double> xs, ys;
        for (const auto &node : path)
        {
            xs.push_back(node.x * map_resolution_ + origin_x_);
            ys.push_back(node.y * map_resolution_ + origin_y_);
        }

        // making spline
        CubicSpline2D spline(xs, ys);

        // sampling the spline
        double step = 0.025;  // adjust
        for (double t = 0.0; t <= spline.calcLength(); t += step)
        {
            auto [x, y] = spline.evaluate(t);
            geometry_msgs::msg::PoseStamped p;
            p.header = ros_path.header;
            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.orientation.w = 1.0;
            ros_path.poses.push_back(p);
        }

        path_pub_->publish(ros_path);        
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Planner>());
    rclcpp::shutdown();
    return 0;
}
