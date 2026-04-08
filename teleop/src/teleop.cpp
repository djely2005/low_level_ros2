#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <memory>
#include <thread>
#include <mutex>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

/**
 * @brief Helper function to get a single character from the terminal without echo
 */
char getch() {
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0) perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) perror("tcsetattr ~ICANON");
    return buf;
}

class SmoothTeleopNode : public rclcpp::Node {
public:
    SmoothTeleopNode() : Node("teleop") {
        this->declare_parameter("accel_rate", 0.01);
        this->declare_parameter("decay_rate", 0.02);
        this->declare_parameter("steer_rate", 0.10);

        speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("/cmd_vel", 10);
        dir_pub_   = this->create_publisher<std_msgs::msg::Float32>("/cmd_dir", 10);

        timer_ = this->create_wall_timer(50ms, std::bind(&SmoothTeleopNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "--- Smooth Teleop Initiated ---");
        RCLCPP_INFO(this->get_logger(), "Arrows: Drive | S: Brake | N: Neutral | Q: Quit");
    }

    void handle_keyboard() {
        char c = getch();

        std::lock_guard<std::mutex> lock(state_mutex_);

        if (c == '\033') {
            getch();
            switch(getch()) {
                case 'A': target_speed_ = std::clamp(target_speed_ + 0.2f, -1.0f, 1.0f); break;
                case 'B': target_speed_ = std::clamp(target_speed_ - 0.2f, -1.0f, 1.0f); break;
                case 'C': target_dir_   = std::clamp(target_dir_   + 0.3f, -1.0f, 1.0f); break;
                case 'D': target_dir_   = std::clamp(target_dir_   - 0.3f, -1.0f, 1.0f); break;
            }
        } else {
            switch(c) {
                case 's': target_speed_ = 0.0f; current_speed_ = 0.0f; break; // Hard brake
                case 'n': target_speed_ = 0.0f; target_dir_ = 0.0f;    break; // Neutral coast
                case 'q': shutdown_node(); break;
            }
        }
    }

private:
    void control_loop() {
        float accel      = get_parameter("accel_rate").as_double();
        float steer_rate = get_parameter("steer_rate").as_double();
        float decay      = get_parameter("decay_rate").as_double();

        std::lock_guard<std::mutex> lock(state_mutex_);

        // Speed interpolation toward target
        if (current_speed_ < target_speed_)
            current_speed_ = std::min(current_speed_ + accel, target_speed_);
        else if (current_speed_ > target_speed_)
            current_speed_ = std::max(current_speed_ - accel, target_speed_);

        // Steering interpolation toward target
        if (current_dir_ < target_dir_)
            current_dir_ = std::min(current_dir_ + steer_rate, target_dir_);
        else if (current_dir_ > target_dir_)
            current_dir_ = std::max(current_dir_ - steer_rate, target_dir_);

        // Natural speed decay (friction) when no target speed is set
        if (target_speed_ == 0.0f && std::abs(current_speed_) > 0.01f) {
            if (current_speed_ > 0) current_speed_ -= decay;
            else                    current_speed_ += decay;
        }

        publish_msgs();
    }

    void publish_msgs() {
        // Note: must be called with state_mutex_ already held
        auto s_msg = std_msgs::msg::Float32();
        s_msg.data = current_speed_;

        auto d_msg = std_msgs::msg::Float32();
        // FIX: exponential curve preserving sign — x*|x| is equivalent to sign(x)*x^2
        d_msg.data = current_dir_ * std::abs(current_dir_);

        speed_pub_->publish(s_msg);
        dir_pub_->publish(d_msg);
    }

    void shutdown_node() {
        // Note: must be called with state_mutex_ already held (called from handle_keyboard)
        target_speed_ = 0.0f;
        current_speed_ = 0.0f;
        // FIX: removed exit(0) — rclcpp::shutdown() lets main() fall through cleanly
        rclcpp::shutdown();
    }

    // State variables — access protected by state_mutex_
    float target_speed_  = 0.0f;
    float current_speed_ = 0.0f;
    float target_dir_    = 0.0f;
    float current_dir_   = 0.0f;

    std::mutex state_mutex_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dir_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SmoothTeleopNode>();

    std::thread spin_thread([node]() { rclcpp::spin(node); });

    while (rclcpp::ok()) {
        node->handle_keyboard();
    }

    if (spin_thread.joinable()) spin_thread.join();
    return 0;
}