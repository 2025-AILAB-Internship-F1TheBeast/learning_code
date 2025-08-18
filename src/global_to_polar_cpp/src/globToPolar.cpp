#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <limits>
#include <algorithm>

// 사용자 정의 메시지
#include "global_to_polar_cpp/msg/polar_grid.hpp"

// 경로점을 저장하기 위한 간단한 구조체
struct Waypoint {
    double x, y;
};

class GlobalToPolarNode : public rclcpp::Node
{
public:
    GlobalToPolarNode() : Node("global_to_polar_node")
    {
        // 파라미터 선언 및 값 가져오기
        std::string path_csv_file = this->declare_parameter<std::string>("path_csv_file", "/home/yongwoo/sim_ws/src/global_to_polar_cpp/line/slam_tool_box.csv");
        lookahead_points_ = this->declare_parameter<int>("lookahead_points", 20);
        search_window_ = this->declare_parameter<int>("search_window", 10);

        // CSV 파일로부터 전역 경로 불러오기
        if (!loadPathFromCSV(path_csv_file)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load path from CSV. Shutting down.");
            rclcpp::shutdown();
            return;
        }

        // Subscriber (Odometry 정보 수신)
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/pf/pose/odom", 10,
            std::bind(&GlobalToPolarNode::odomCallback, this, std::placeholders::_1));

        // Publisher (PolarGrid 메시지 발행)
        polar_grid_pub_ = this->create_publisher<global_to_polar_cpp::msg::PolarGrid>(
            "/polar_grid", 10);

        // 변수 초기화
        current_x_ = 0.0;
        current_y_ = 0.0;
        current_yaw_ = 0.0;
        last_closest_idx_ = 0;

        RCLCPP_INFO(this->get_logger(), "Global to Polar Node initialized successfully.");
    }

private:
    // 고정된 형식의 CSV 파일을 읽는 함수
    bool loadPathFromCSV(const std::string& file_path)
    {
        std::ifstream file(file_path);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot open CSV file: %s", file_path.c_str());
            return false;
        }

        global_path_.clear();
        std::string line;

        // 첫 줄(헤더)은 무조건 읽고 건너뜀
        if (!std::getline(file, line)) {
            RCLCPP_ERROR(this->get_logger(), "CSV file is empty.");
            return false;
        }
        RCLCPP_INFO(this->get_logger(), "CSV header skipped: %s", line.c_str());


        // 나머지 데이터 줄 읽기
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string x_str, y_str;
            std::string tmp;

            std::getline(ss, tmp, ',');

            // x_m, y_m 값만 읽고 나머지는 무시
            if (std::getline(ss, x_str, ',') && std::getline(ss, y_str, ',')) {
                try {
                    Waypoint wp;
                    wp.x = std::stod(x_str);
                    wp.y = std::stod(y_str);
                    global_path_.push_back(wp);
                } catch (const std::invalid_argument& e) {
                    RCLCPP_WARN(this->get_logger(), "Skipping malformed line in CSV: %s", line.c_str());
                }
            }
        }

        file.close();
        
        if (global_path_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No valid waypoints loaded from CSV file.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", global_path_.size(), file_path.c_str());
        return true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // 위치 정보 추출
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;

        // 쿼터니언으로부터 Yaw 각도 추출
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);

        // 새로운 Odometry 정보로 경로 처리
        processPathToPolarGrid();
    }

    void processPathToPolarGrid()
    {
        if (global_path_.empty()) {
            return;
        }

        const size_t path_size = global_path_.size();

        // 1. 현재 위치에서 가장 가까운 경로점의 인덱스 찾기
        // 효율성을 위해 이전 인덱스 주변의 윈도우 내에서만 탐색
        double min_dist = std::numeric_limits<double>::max();
        size_t closest_waypoint_idx = last_closest_idx_;

        for (int j = 0; j < search_window_; ++j) {
            size_t current_idx = (last_closest_idx_ + j) % path_size;
            double dx = global_path_[current_idx].x - current_x_;
            double dy = global_path_[current_idx].y - current_y_;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < min_dist) {
                min_dist = dist;
                closest_waypoint_idx = current_idx;
            }
        }
        last_closest_idx_ = closest_waypoint_idx;

        // 2. PolarGrid 메시지 생성 및 초기화
        auto polar_grid_msg = global_to_polar_cpp::msg::PolarGrid();
        polar_grid_msg.header.stamp = this->get_clock()->now();
        polar_grid_msg.header.frame_id = "base_link"; // 로봇의 지역 좌표계
        polar_grid_msg.ranges.resize(1080, 0.0f); // 1080개의 요소를 0으로 초기화

        // 극좌표 그리드의 각도 범위 정의 (-135도 ~ +135도)
        const double min_angle_rad = -135.0 * M_PI / 180.0;
        const double max_angle_rad = 135.0 * M_PI / 180.0;
        const double angle_increment = (max_angle_rad - min_angle_rad) / (1080.0 - 1.0);

        // 3. 전방 'lookahead_points_' 개수만큼의 경로점을 극좌표 그리드에 채우기
        for (int j = 0; j < lookahead_points_; ++j) {
            size_t current_idx = (closest_waypoint_idx + j) % path_size;
            const auto& waypoint = global_path_[current_idx];

            // 로봇 기준 경로점의 상대 위치 계산
            double dx = waypoint.x - current_x_;
            double dy = waypoint.y - current_y_;

            // 로봇의 현재 자세를 기준으로 극좌표로 변환
            double distance = std::sqrt(dx * dx + dy * dy);
            double global_angle = std::atan2(dy, dx);
            double relative_angle = global_angle - current_yaw_;

            // 각도를 [-pi, pi] 범위로 정규화
            while (relative_angle > M_PI) relative_angle -= 2.0 * M_PI;
            while (relative_angle < -M_PI) relative_angle += 2.0 * M_PI;

            // 경로점이 우리가 원하는 각도 범위 내에 있는지 확인
            if (relative_angle >= min_angle_rad && relative_angle <= max_angle_rad) {
                // 1081개 배열에서 해당 각도에 맞는 인덱스 계산
                int index = static_cast<int>(std::round((relative_angle - min_angle_rad) / angle_increment));

                if (index >= 0 && index < 1081) {
                    // 해당 인덱스가 비어있거나, 새로 계산된 점이 더 가까우면 거리 값을 업데이트
                    if (polar_grid_msg.ranges[index] == 0.0f || distance < polar_grid_msg.ranges[index]) {
                        polar_grid_msg.ranges[index] = static_cast<float>(distance);
                    }
                }
            }
        }

        // 4. 메시지 발행
        polar_grid_pub_->publish(polar_grid_msg);
    }

    // ROS 통신 관련
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<global_to_polar_cpp::msg::PolarGrid>::SharedPtr polar_grid_pub_;

    // 경로 데이터
    std::vector<Waypoint> global_path_;

    // 상태 변수
    double current_x_, current_y_, current_yaw_;
    size_t last_closest_idx_;

    // 파라미터
    int lookahead_points_;
    int search_window_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalToPolarNode>());
    rclcpp::shutdown();
    return 0;
}
