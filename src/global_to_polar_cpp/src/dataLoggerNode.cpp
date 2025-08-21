#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip> // For std::setprecision
#include <cmath>

// Custom message
#include "global_to_polar_cpp/msg/polar_grid.hpp"
#include "planning_custom_msgs/msg/path_with_velocity.hpp"

class DataLoggerNode : public rclcpp::Node
{
public:
    DataLoggerNode() : Node("data_logger_node"), scan_received_(false), grid_received_(false), path_received_(false), pose_received_(false)
    {
        // Declare and get parameter for the output CSV file path
        output_csv_file_ = this->declare_parameter<std::string>("output_csv_file", "/home/yongwoo/sim_ws/src/global_to_polar_cpp/dataSet/new_map1.csv");

        // Open the CSV file for writing
        csv_file_.open(output_csv_file_, std::ios::out | std::ios::trunc);
        if (!csv_file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open output file: %s", output_csv_file_.c_str());
            rclcpp::shutdown();
            return;
        }
        
        // Write the header row to the CSV file
        writeHeader();

        // Subscribers
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&DataLoggerNode::laserScanCallback, this, std::placeholders::_1));

        polar_grid_sub_ = this->create_subscription<global_to_polar_cpp::msg::PolarGrid>(
            "/polar_grid", 10,
            std::bind(&DataLoggerNode::polarGridCallback, this, std::placeholders::_1));

        path_with_velocity_sub_ = this->create_subscription<planning_custom_msgs::msg::PathWithVelocity>(
            "/planned_path_with_velocity", 10,
            std::bind(&DataLoggerNode::pathWithVelocityCallback, this, std::placeholders::_1));

        // Add localization subscriber
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/pf/pose/odom", 10,
            std::bind(&DataLoggerNode::poseCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Data Logger Node initialized. Logging to %s", output_csv_file_.c_str());
    }

    ~DataLoggerNode()
    {
        if (csv_file_.is_open()) {
            csv_file_.close();
        }
    }

private:
    void writeHeader()
    {
        // LaserScan headers (assuming 1080 points)
        for (int i = 0; i < 1080; ++i) {
            csv_file_ << "scan_" << i << ",";
        }
        // PolarGrid headers (1080 points)
        for (int i = 0; i < 1080; ++i) {
            csv_file_ << "grid_" << i << ",";
        }
        // PathPointArray headers (16 points with x, y, v, yaw)
        for (int i = 0; i < 16; ++i) {
            csv_file_ << "x" << (i+1) << ",";
            csv_file_ << "y" << (i+1) << ",";
            csv_file_ << "v" << (i+1) << ",";
            csv_file_ << "yaw" << (i+1) << (i == 15 ? "" : ",");
        }
        csv_file_ << "\n";
    }

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Create a new LaserScan message with exactly 1080 points
        auto processed_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
        
        if (msg->ranges.size() == 1081) {
            // Remove the last element to make it 1080
            processed_scan->ranges.pop_back();
            RCLCPP_DEBUG(this->get_logger(), "Trimmed LaserScan from 1081 to 1080 points");
        } else if (msg->ranges.size() != 1080) {
            RCLCPP_WARN(this->get_logger(), "Received LaserScan with %zu points, expected 1080 or 1081. CSV columns might not align.", msg->ranges.size());
        }
        
        last_scan_ = processed_scan;
        scan_received_ = true;
        writeData(); // Attempt to write data every time a new scan arrives
    }

    void polarGridCallback(const global_to_polar_cpp::msg::PolarGrid::SharedPtr msg)
    {
        if (msg->ranges.size() != 1080) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Received PolarGrid with %zu points, expected 1080. CSV columns might not align.", msg->ranges.size());
        }
        last_grid_ = msg;
        grid_received_ = true;
    }

    void pathWithVelocityCallback(const planning_custom_msgs::msg::PathWithVelocity::SharedPtr msg)
    {
        last_path_ = msg;
        path_received_ = true;
        RCLCPP_INFO(this->get_logger(), "Received PathWithVelocity with %zu points", msg->points.size());
    }

    void poseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        last_pose_ = msg;
        pose_received_ = true;
    }

    // Convert global coordinates to local coordinates
    void globalToLocal(double global_x, double global_y, double robot_x, double robot_y, double robot_yaw,
                       double& local_x, double& local_y)
    {
        // Translate to robot origin
        double translated_x = global_x - robot_x;
        double translated_y = global_y - robot_y;
        
        // Rotate to robot coordinate frame
        double cos_yaw = std::cos(-robot_yaw);
        double sin_yaw = std::sin(-robot_yaw);
        
        local_x = translated_x * cos_yaw - translated_y * sin_yaw;
        local_y = translated_x * sin_yaw + translated_y * cos_yaw;
    }

    // Extract yaw from quaternion
    double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q)
    {
        // Convert quaternion to yaw angle
        double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
        double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    void writeData()
    {
        // Only write to file if we have received at least one of each message type
        if (!scan_received_ || !grid_received_ || !path_received_ || !pose_received_) {
            return;
        }

        // Set precision for floating point numbers
        csv_file_ << std::fixed << std::setprecision(5);

        // Write LaserScan ranges
        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            csv_file_ << last_scan_->ranges[i] << ",";
        }

        // Write PolarGrid ranges
        for (size_t i = 0; i < last_grid_->ranges.size(); ++i) {
            csv_file_ << last_grid_->ranges[i] << ",";
        }

        // Get robot pose for coordinate transformation
        double robot_x = last_pose_->pose.pose.position.x;
        double robot_y = last_pose_->pose.pose.position.y;
        double robot_yaw = getYawFromQuaternion(last_pose_->pose.pose.orientation);

        // Write PathPointArray data (16 points with x, y, v, yaw) converted to local coordinates
        for (int i = 0; i < 16; ++i) {
            if (i < static_cast<int>(last_path_->points.size())) {
                const auto& point = last_path_->points[i];
                
                // Convert global coordinates to local coordinates
                double local_x, local_y;
                globalToLocal(point.x, point.y, robot_x, robot_y, robot_yaw, local_x, local_y);
                
                // Convert global yaw to local yaw
                double local_yaw = point.yaw - robot_yaw;
                // Normalize yaw to [-pi, pi]
                while (local_yaw > M_PI) local_yaw -= 2.0 * M_PI;
                while (local_yaw < -M_PI) local_yaw += 2.0 * M_PI;
                
                csv_file_ << local_x << ",";
                csv_file_ << local_y << ",";
                csv_file_ << point.velocity << ",";
                csv_file_ << local_yaw << (i == 15 ? "" : ",");
            } else {
                // Pad with zeros for missing points
                csv_file_ << "0,0,0,0" << (i == 15 ? "" : ",");
            }
        }
        csv_file_ << "\n";
        
        // Flush the buffer to ensure data is written to disk immediately
        csv_file_.flush();

        // Reset flags to ensure we wait for a new pair of messages
        scan_received_ = false;
        grid_received_ = false;
        path_received_ = false;
        pose_received_ = false;
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<global_to_polar_cpp::msg::PolarGrid>::SharedPtr polar_grid_sub_;
    rclcpp::Subscription<planning_custom_msgs::msg::PathWithVelocity>::SharedPtr path_with_velocity_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;

    // Data storage
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    global_to_polar_cpp::msg::PolarGrid::SharedPtr last_grid_;
    planning_custom_msgs::msg::PathWithVelocity::SharedPtr last_path_;
    nav_msgs::msg::Odometry::SharedPtr last_pose_;
    bool scan_received_;
    bool grid_received_;
    bool path_received_;
    bool pose_received_;

    // File handling
    std::ofstream csv_file_;
    std::string output_csv_file_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataLoggerNode>());
    rclcpp::shutdown();
    return 0;
}
