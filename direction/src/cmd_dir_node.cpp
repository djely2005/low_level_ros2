/**
 * @file cmd_dir_node.cpp
 * @brief Node to command the steering direction via Dynamixel servo
 */

#include <cmath>
#include <memory>
#include <string>

#include "dynamixel_sdk/dynamixel_sdk.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

// ─── Math helpers ────────────────────────────────────────────────────────────

/**
 * @brief Transform the motor position (DXL units) into a steering angle (rad).
 *        psi   = steering angle (rad)
 *        theta = motor angle (rad)
 *        pos   = motor angle (DXL units)
 */
double pos2psi(double pos)
{
  double theta_rad = (pos * (5.24 / 1023.0)) + 0.524;
  double A = std::cos(theta_rad) * 15.0 - 4.35;
  return std::asin(A / 25.0);
}

/**
 * @brief Convert an angle in degrees to a motor position in DXL units.
 */
int degrees2pos(double degrees)
{
  return static_cast<int>((degrees - 30.0) * (1023.0 / 300.0));
}

/**
 * @brief Convert a steering angle (deg) to a motor position (DXL units).
 */
int set_dir_deg(double angle_deg)
{
  double psi   = angle_deg * M_PI / 180.0;
  double A     = 25.0 * std::sin(psi) + 4.35;
  double theta = std::acos(A / 15.0);
  return degrees2pos(theta * 180.0 / M_PI);
}

// ─── Node ────────────────────────────────────────────────────────────────────

class CommandDirection : public rclcpp::Node
{
public:
  CommandDirection()
  : Node("cmd_dir_node"),
    target_steering_angle_deg_(0.0),
    curr_steering_angle_deg_(0.0),
    ms_(false)
  {
    // ── Parameters ──────────────────────────────────────────────────────────
    this->declare_parameter<bool>("debug", false);
    debug_ = this->get_parameter("debug").as_bool();

    if (debug_) {
      // Set logger level to DEBUG
      auto ret = rcutils_logging_set_logger_level(
        this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
      (void)ret;
    }

    // ── Dynamixel configuration ──────────────────────────────────────────────
    constexpr float  PROTOCOL_VERSION = 1.0f;
    constexpr int    DXL_ID           = 1;
    constexpr int    BAUDRATE         = 115200;
    const std::string DEVICENAME      = "/dev/ttyU2D2";
    // udev rule in /etc/udev/rules.d/99-usb-dynamixel.rules:
    // SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6014",
    //   SYMLINK+="ttyU2D2", MODE="0777"

    DXL_ID_           = DXL_ID;
    MAX_STEERING_DEG_ = 15.5;

    portHandler_   = std::unique_ptr<dynamixel::PortHandler>(
      dynamixel::PortHandler::getPortHandler(DEVICENAME.c_str()));
    packetHandler_ = std::unique_ptr<dynamixel::PacketHandler>(
      dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));

    if (portHandler_->openPort()) {
      RCLCPP_INFO(this->get_logger(), "[INFO] -- Succeeded to open the port");
    } else {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] -- Failed to open the port");
    }

    if (portHandler_->setBaudRate(BAUDRATE)) {
      RCLCPP_INFO(this->get_logger(), "[INFO] -- Succeeded to change the baudrate");
    } else {
      RCLCPP_ERROR(this->get_logger(), "[ERROR] -- Failed to change the baudrate");
    }

    // ── Subscription ────────────────────────────────────────────────────────
    sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "/cmd_dir", 10,
      [this](const std_msgs::msg::Float32::SharedPtr msg) { cmd_callback(msg); });

    // ── Timer (30 ms → ~33 Hz) ───────────────────────────────────────────────
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(30),
      [this]() { dxl_callback(); });
  }

private:
  // ── Dynamixel timer callback ──────────────────────────────────────────────
  void dxl_callback()
  {
    // Clamp target angle within hardware limits
    target_steering_angle_deg_ = std::max(
      std::min(target_steering_angle_deg_, MAX_STEERING_DEG_), -MAX_STEERING_DEG_);

    try {
      uint16_t raw_pos = 0;
      uint8_t  dxl_error = 0;
      int      dxl_comm_result = packetHandler_->read2ByteTxRx(
        portHandler_.get(), DXL_ID_, 36, &raw_pos, &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS) {
        throw std::runtime_error(
          packetHandler_->getTxRxResult(dxl_comm_result));
      }
      if (dxl_error != 0 && debug_) {
        RCLCPP_WARN(this->get_logger(), "[WARNING] -- DXL read status error: %s",
          packetHandler_->getRxPacketError(dxl_error));
      }

      curr_steering_angle_deg_ =
        -180.0 / M_PI * pos2psi(static_cast<double>(raw_pos));

      int target_pos = set_dir_deg(target_steering_angle_deg_);

      if (debug_ && debug_) {
        RCLCPP_DEBUG(this->get_logger(), "[DEBUG] -- DXL position : %d", target_pos);
      }

      dxl_comm_result = packetHandler_->write2ByteTxRx(
        portHandler_.get(), DXL_ID_, 30,
        static_cast<uint16_t>(target_pos), &dxl_error);

      if (dxl_comm_result != COMM_SUCCESS) {
        throw std::runtime_error(
          packetHandler_->getTxRxResult(dxl_comm_result));
      }
      if (dxl_error != 0 && debug_) {
        RCLCPP_WARN(this->get_logger(), "[WARNING] -- DXL write status error: %s",
          packetHandler_->getRxPacketError(dxl_error));
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(this->get_logger(), "[WARNING] -- DYNAMIXEL PROBLEM");
      RCLCPP_WARN(this->get_logger(), "[WARNING] -- %s", e.what());
    }
  }

  // ── Subscription callback ─────────────────────────────────────────────────
  void cmd_callback(const std_msgs::msg::Float32::SharedPtr data)
  {
    if (ms_) {
      target_steering_angle_deg_ = -data->data;
    } else {
      target_steering_angle_deg_ = -data->data * MAX_STEERING_DEG_;
    }
    last_command_time_ = this->now();
  }

  // ── Members ───────────────────────────────────────────────────────────────
  bool   debug_;
  bool   ms_;
  int    DXL_ID_;
  double MAX_STEERING_DEG_;
  double target_steering_angle_deg_;
  double curr_steering_angle_deg_;

  rclcpp::Time last_command_time_;

  std::unique_ptr<dynamixel::PortHandler>   portHandler_;
  std::unique_ptr<dynamixel::PacketHandler> packetHandler_;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

// ─── main ────────────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<CommandDirection>());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
      "Error in Command Direction: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}     