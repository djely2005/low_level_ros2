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
int get_sign(T val) {
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
// Reverse min is ~1490 µs ≈ 7.7% → ratio 7.7  (just below neutral).
// Reverse max is ~1300 µs ≈ 6.5% → ratio 6.5.

static constexpr double ESC_PERIOD_NS    = 20000.0; // 20 ms PWM period
static constexpr double DUTY_TO_NS       = 0.00938; // ratio → fractional duty
static constexpr double NEUTRAL_RATIO    = 8.0;
static constexpr double FWD_MAX_RATIO    = 10.0 * 1e-2; //# Play with this 10.0
static constexpr double REV_IDLE_RATIO   = 7.7;  // slowest reverse (just below neutral)
static constexpr double REV_FULL_RATIO   = 6.5;  // fastest reverse
static constexpr double CMD_DEADBAND     = 0.01; // |cmd| below this → neutral

class CommandSpeedNode : public rclcpp::Node {
public:
    CommandSpeedNode() : Node("cmd_vel_node"), curr_dir_(1), ready_(false)
    {
        // ── Parameters ──────────────────────────────────────────────────────
        this->declare_parameter("debug",               false);
        this->declare_parameter("minimal_speed_ratio", 8.3);   // ESC ratio at minimum forward speed
        this->declare_parameter("safety_timeout_ms",   200);
        this->declare_parameter("raw_degrees_mode",    false);  // true = cmd is already in degrees, false = normalized [-1, 1]
        this->declare_parameter("vel_threshold_m_s",   0.05);   // max speed allowed when switching direction

        update_parameters();

        // ── Pub / Sub ────────────────────────────────────────────────────────
        sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/cmd_vel", 10,
            std::bind(&CommandSpeedNode::cmd_callback, this, std::placeholders::_1));

        // Uncomment to enable real velocity feedback for direction-change guard:
        // vel_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        //     "/odom_velocity", 10,
        //     [this](const std_msgs::msg::Float32::SharedPtr msg) {
        //         curr_velocity_m_s_ = msg->data;
        //     });

        stm32_pub_ = this->create_publisher<std_msgs::msg::Int16>("/stm32_data", 10);

        // ── Safety watchdog ──────────────────────────────────────────────────
        // Fires once if no command arrives within the timeout window.
        // Timer is cancelled on each valid command and reset at its end.
        auto timeout = std::chrono::milliseconds(
            this->get_parameter("safety_timeout_ms").as_int());
        timer_safety_ = this->create_wall_timer(
            timeout, std::bind(&CommandSpeedNode::emergency_stop, this));
        timer_safety_->cancel();

        ready_ = true;
        RCLCPP_INFO(this->get_logger(), "Command Speed Node started.");
    }

private:
    // ── Parameter reload ─────────────────────────────────────────────────────
    void update_parameters()
    {
        debug_             = this->get_parameter("debug").as_bool();
        min_speed_ratio_   = this->get_parameter("minimal_speed_ratio").as_double();
        raw_degrees_mode_  = this->get_parameter("raw_degrees_mode").as_bool();
        vel_threshold_     = this->get_parameter("vel_threshold_m_s").as_double();

        if (debug_) {
            rcutils_logging_set_logger_level(
                this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
        }
    }

    // ── Command callback ─────────────────────────────────────────────────────
    void cmd_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        timer_safety_->cancel();
        watchdog_triggered_ = false;

        float cmd = msg->data;
        int   cmd_sign = get_sign(cmd);

        // Direction-change guard: only switch if we are slow enough.
        // Falls back to immediate switch when velocity feedback is unavailable.
        if (cmd_sign != 0 && cmd_sign != curr_dir_) {
            if (std::abs(curr_velocity_m_s_) < vel_threshold_) {
                curr_dir_ = cmd_sign;
            } else {
                // Still moving in the previous direction — force neutral this cycle.
                RCLCPP_DEBUG(this->get_logger(),
                    "Direction change blocked (v=%.2f m/s > threshold=%.2f)",
                    curr_velocity_m_s_, vel_threshold_);
                cmd = 0.0f;
            }
        }

        command(cmd);
        timer_safety_->reset();
    }

    // ── Watchdog ─────────────────────────────────────────────────────────────
    void emergency_stop()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!watchdog_triggered_) {
            watchdog_triggered_ = true;
            RCLCPP_WARN(this->get_logger(),
                "Watchdog triggered: no command received — sending neutral.");
        }
        command(0.0f);
        // Timer stays cancelled; it will be re-armed by the next cmd_callback.
    }

    // ── ESC command dispatch ─────────────────────────────────────────────────
    void command(float cmd_speed)
    {
        float clamped = std::clamp(cmd_speed, -1.0f, 1.0f);

        if (clamped > CMD_DEADBAND) {
            forward(clamped);
        } else if (clamped < -CMD_DEADBAND) {
            reverse(clamped);
        } else {
            neutral();
        }
    }

    void forward(float speed)
    {
        // Map [0, 1] → [min_speed_ratio_, FWD_MAX_RATIO]
        double ratio = min_speed_ratio_ + speed * (FWD_MAX_RATIO - min_speed_ratio_);
        publish_pulse(ratio);
    }

    void reverse(float speed)
    {
        // Map [0, 1] (magnitude) → [REV_IDLE_RATIO, REV_FULL_RATIO]
        // REV_IDLE_RATIO > REV_FULL_RATIO numerically, so as magnitude increases
        // the ratio decreases (further from neutral = faster in reverse).
        double magnitude = std::abs(speed);
        double ratio = REV_IDLE_RATIO + magnitude * (REV_FULL_RATIO - REV_IDLE_RATIO);
        publish_pulse(ratio);
    }

    void neutral()
    {
        publish_pulse(NEUTRAL_RATIO);
    }

    // ── STM32 publisher ───────────────────────────────────────────────────────
    void publish_pulse(double ratio)
    {
        if (!ready_) return;

        auto msg   = std_msgs::msg::Int16();
        msg.data   = static_cast<int16_t>(ratio * DUTY_TO_NS * ESC_PERIOD_NS);

        RCLCPP_DEBUG(this->get_logger(),
            "ESC pulse: %d ns  (ratio=%.3f)", msg.data, ratio);

        stm32_pub_->publish(msg);
    }

    // ── Members ───────────────────────────────────────────────────────────────
    bool   debug_             = false;
    bool   raw_degrees_mode_  = false;
    bool   watchdog_triggered_ = false;
    bool   ready_             = false;

    double min_speed_ratio_   = 8.3 * 1e-2; // # play with this
    double vel_threshold_     = 0.05;
    double curr_velocity_m_s_ = 0.0; // updated by velocity subscriber when enabled

    int    curr_dir_          = 1;

    std::mutex mutex_;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
    // rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr      stm32_pub_;
    rclcpp::TimerBase::SharedPtr                            timer_safety_;
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CommandSpeedNode>();
    try {
        rclcpp::spin(node);
    } catch (const std::exception & e) {
        RCLCPP_FATAL(node->get_logger(), "Node crashed: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}