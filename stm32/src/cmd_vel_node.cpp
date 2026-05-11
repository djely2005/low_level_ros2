#include <chrono>
#include <memory>
#include <cmath>
#include <mutex>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

// ── ESC Nanosecond Mapping ───────────────────────────────────────────────────
// These represent the raw pulse widths in nanoseconds.
// Neutral: 1500ns
// Forward: 1400 (slow) to 1300 (fast)
// Reverse: 1600 (slow) to 1800 (fast)

static constexpr double NS_FWD_FAST = 1300.0;
static constexpr double NS_FWD_SLOW = 1500.0;
static constexpr double NS_NEUTRAL  = 1500.0;
static constexpr double NS_REV_SLOW = 1600.0;
static constexpr double NS_REV_FAST = 1800.0;

// Conversion factors for the STM32 "Ratio" unit
static constexpr double ESC_PERIOD_NS = 20000.0; // 20 ms PWM period
static constexpr double DUTY_TO_NS    = 0.00938;

class CommandSpeedNode : public rclcpp::Node
{
public:
    CommandSpeedNode() : Node("cmd_vel_node"), ready_(false)
    {
        this->declare_parameter("safety_timeout_ms", 200);

        stm32_pub_ = this->create_publisher<std_msgs::msg::Int16>("/stm32_data", 10);

        sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/cmd_vel", 10,
            std::bind(&CommandSpeedNode::cmd_callback, this, std::placeholders::_1));

        auto timeout = std::chrono::milliseconds(this->get_parameter("safety_timeout_ms").as_int());
        timer_safety_ = this->create_wall_timer(timeout, std::bind(&CommandSpeedNode::emergency_stop, this));
        timer_safety_->cancel();

        ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Command Speed Node initialized with direct NS mapping.");
    }

private:
    /**
     * @brief Converts nanoseconds back into the "Ratio" expected by the firmware.
     * Formula: Ratio = ns / (DUTY_TO_NS * ESC_PERIOD_NS)
     */
    double convert_ns_to_ratio(double ns)
    {
        return ns / (DUTY_TO_NS * ESC_PERIOD_NS);
    }

    void cmd_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        timer_safety_->cancel();

        float cmd = std::clamp(msg->data, -1.0f, 1.0f);
        double target_ns;

        if (cmd > 0.01f)
        {
            // FORWARD: maps [0, 1] to [1400ns, 1300ns]
            target_ns = NS_FWD_SLOW + (double)cmd * (NS_FWD_FAST - NS_FWD_SLOW);
        }
        else if (cmd < -0.01f)
        {
            // REVERSE: maps [0, 1] to [1600ns, 1800ns]
            double magnitude = std::abs((double)cmd);
            target_ns = NS_REV_SLOW + magnitude * (NS_REV_FAST - NS_REV_SLOW);
        }
        else
        {
            target_ns = NS_NEUTRAL;
        }

        publish_pulse(convert_ns_to_ratio(target_ns));
        timer_safety_->reset();
    }

    void emergency_stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        publish_pulse(convert_ns_to_ratio(NS_NEUTRAL));
        RCLCPP_WARN(this->get_logger(), "Watchdog triggered: Neutral sent.");
    }

    void publish_pulse(double ratio)
    {
        if (!ready_) return;

        auto msg = std_msgs::msg::Int16();
        
        // Final pulse calculation in nanoseconds
        double ns = ratio * DUTY_TO_NS * ESC_PERIOD_NS;

        // Safety clamp based on your specific motor limits
        msg.data = static_cast<int16_t>(std::clamp(ns, 1250.0, 1850.0));

        RCLCPP_DEBUG(this->get_logger(), "Pulse: %d ns", msg.data);
        stm32_pub_->publish(msg);
    }

    bool ready_;
    std::mutex mutex_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr stm32_pub_;
    rclcpp::TimerBase::SharedPtr timer_safety_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CommandSpeedNode>();
    try {
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(node->get_logger(), "Crash: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}