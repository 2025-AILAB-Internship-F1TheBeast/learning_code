#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <memory>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip> // For std::setprecision

// Custom message
#include "global_to_polar_cpp/msg/polar_grid.hpp"
#include "planning_custom_msgs/msg/path_with_velocity.hpp"

class DataLoggerNode : public rclcpp::Node
{
public:
    DataLoggerNode() : Node("data_logger_node"), scan_received_(false), grid_received_(false), path_received_(false)
    {
        // Declare and get parameter for the output CSV file path
        output_csv_file_ = this->declare_parameter<std::string>("output_csv_file", "/home/yongwoo/sim_ws/src/global_to_polar_cpp/dataSet/datalog.csv");

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
        // We assume the laser scan also has 1081 points.
        // If not, you might need to adjust the logic or header.
        if (msg->ranges.size() != 1080) {
             RCLCPP_WARN_ONCE(this->get_logger(), "Received LaserScan with %zu points, expected 1081. CSV columns might not align.", msg->ranges.size());
        }
        last_scan_ = msg;
        scan_received_ = true;
        writeData(); // Attempt to write data every time a new scan arrives
    }

    void polarGridCallback(const global_to_polar_cpp::msg::PolarGrid::SharedPtr msg)
    {
        if (msg->ranges.size() != 1080) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Received PolarGrid with %zu points, expected 1081. CSV columns might not align.", msg->ranges.size());
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

    void writeData()
    {
        // Only write to file if we have received at least one of each message type
        if (!scan_received_ || !grid_received_ || !path_received_) {
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

        // Write PathPointArray data (16 points with x, y, v, yaw)
        for (int i = 0; i < 16; ++i) {
            if (i < static_cast<int>(last_path_->points.size())) {
                const auto& point = last_path_->points[i];
                csv_file_ << point.x << ",";
                csv_file_ << point.y << ",";
                csv_file_ << point.velocity << ",";
                csv_file_ << point.yaw << (i == 15 ? "" : ",");
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
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Subscription<global_to_polar_cpp::msg::PolarGrid>::SharedPtr polar_grid_sub_;
    rclcpp::Subscription<planning_custom_msgs::msg::PathWithVelocity>::SharedPtr path_with_velocity_sub_;

    // Data storage
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    global_to_polar_cpp::msg::PolarGrid::SharedPtr last_grid_;
    planning_custom_msgs::msg::PathWithVelocity::SharedPtr last_path_;
    bool scan_received_;
    bool grid_received_;
    bool path_received_;

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
