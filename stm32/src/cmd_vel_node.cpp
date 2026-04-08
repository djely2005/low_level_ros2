#include <chrono>
#include <memory>
#include <cmath>
#include <mutex>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

template <typename T>
int get_sign(T val)
{
    return (T(0) < val) - (val < T(0));
}

// ── ESC pulse configuration ───────────────────────────────────────────────────
// The STM32 expects a pulse width in nanoseconds derived from a duty cycle ratio.
// Formula: pulse_ns = ratio (%) * DUTY_TO_NS_FACTOR * ESC_PERIOD_NS
//
// DUTY_TO_NS_FACTOR = 0.00938 converts a percentage-style ratio to a
// fractional duty cycle (e.g. 8.0 * 0.00938 ≈ 0.075 → 7.5% duty cycle).
//
// ESC neutral is ~1500 µs = 7.5% of a 20 ms period, which maps to ratio 8.0.
// Forward max is ~2000 µs = 10.0% → ratio 10.0.
// Reverse min is ~1490 µs ≈ 7.7% → ratio 7.7  (just below neutral).
// Reverse max is ~1300 µs ≈ 6.5% → ratio 6.5.

static constexpr double ESC_PERIOD_NS = 20000.0; // 20 ms PWM period
static constexpr double DUTY_TO_NS = 0.00938;
static constexpr double NEUTRAL_RATIO = 8.0;
static constexpr double FWD_MAX_RATIO = 10.0; // # Play with this 10.0
static constexpr double REV_IDLE_RATIO = 7.7; // slowest reverse (just below neutral)
static constexpr double REV_FULL_RATIO = 6.5; // fastest reverse
static constexpr double CMD_DEADBAND = 0.01;

#include <chrono>
#include <memory>
#include <cmath>
#include <mutex>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

// ── Hardware Mapping Constants ──────────────────────────────────────────────
// Based on your tests:
// Forward: 1400 (slow) to 1300 (fast)
// Reverse: 1600 (slow) to 1800 (fast)
// Neutral: 1500

// Math: Ratio = ns / (DUTY_TO_NS * ESC_PERIOD_NS)
static constexpr double RATIO_1300 = 6.93;
static constexpr double RATIO_1400 = 7.46;
static constexpr double RATIO_1500 = 8.00;
static constexpr double RATIO_1600 = 8.53;
static constexpr double RATIO_1800 = 9.59;

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
        RCLCPP_INFO(this->get_logger(), "Command Speed Node initialized for custom ESC mapping.");
    }

private:
    void cmd_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        timer_safety_->cancel();

        float cmd = std::clamp(msg->data, -1.0f, 1.0f);
        // Inside cmd_callback
        if (cmd > 0.01f)
        {
            // FORWARD logic: maps [0, 1] to [1400ns, 1300ns]
            // Note: As speed increases, ratio decreases
            double ratio = RATIO_1400 + (double)cmd * (RATIO_1300 - RATIO_1400);
            publish_pulse(ratio);
        }
        else if (cmd < -0.01f)
        {
            // REVERSE logic: maps [0, 1] to [1600ns, 1800ns]
            double magnitude = std::abs((double)cmd);
            double ratio = RATIO_1600 + magnitude * (RATIO_1800 - RATIO_1600);
            publish_pulse(ratio);
        }
        else
        {
            publish_pulse(RATIO_1500);
        }

        timer_safety_->reset();
    }

    void emergency_stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        publish_pulse(RATIO_1500);
        RCLCPP_WARN(this->get_logger(), "Watchdog triggered: Neutral sent.");
    }

    void publish_pulse(double ratio)
    {
        if (!ready_)
            return;

        auto msg = std_msgs::msg::Int16();
        // Calculate raw nanoseconds
        double ns = ratio * DUTY_TO_NS * ESC_PERIOD_NS;

        // Final safety clamp to prevent the "plummet" (out-of-range signals)
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
    try
    {
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(node->get_logger(), "Crash: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}