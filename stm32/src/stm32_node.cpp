#ifndef STM32_BRIDGE__STM32_NODE_HPP_
#define STM32_BRIDGE__STM32_NODE_HPP_

#include <functional>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
#include <stdexcept>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "bolide_interfaces/msg/fork_speed.hpp"
#include "bolide_interfaces/msg/multiple_range.hpp"

using std::placeholders::_1;

namespace stm32_bridge
{

class Stm32Node : public rclcpp::Node
{
public:
  explicit Stm32Node(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("stm32_node", options)
  {
    // ── Parameters ──────────────────────────────────────────────────────────
    this->declare_parameter("debug", false);
    debug_ = this->get_parameter("debug").as_bool();
    if (debug_)
      this->get_logger().set_level(rclcpp::Logger::Level::Debug);

    this->declare_parameter("device", "/dev/spidev0.1");
    std::string device = this->get_parameter("device").as_string();

    // ── SPI setup ────────────────────────────────────────────────────────────
    // Python uses spidev writebytes() then readbytes() — two separate half-duplex
    // transfers, NOT simultaneous xfer2. We replicate that with two ioctl calls.
    spi_fd_ = open(device.c_str(), O_RDWR);
    if (spi_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to open SPI device: %s", device.c_str());
      throw std::runtime_error("SPI device open failed");
    }

    uint8_t  mode  = SPI_MODE_0;
    uint32_t speed = BAUDRATE;
    uint8_t  bits  = 8;
    ioctl(spi_fd_, SPI_IOC_WR_MODE,          &mode);
    ioctl(spi_fd_, SPI_IOC_WR_MAX_SPEED_HZ,  &speed);
    ioctl(spi_fd_, SPI_IOC_WR_BITS_PER_WORD, &bits);

    // ── Publishers ───────────────────────────────────────────────────────────
    stm_pub_    = create_publisher<std_msgs::msg::Float32MultiArray>("/stm32_sensors", 10);
    speed_pub_  = create_publisher<bolide_interfaces::msg::ForkSpeed>("/raw_fork_data", 10);
    ranges_pub_ = create_publisher<bolide_interfaces::msg::MultipleRange>("/raw_rear_range_data", 10);
    imu_pub_    = create_publisher<sensor_msgs::msg::Imu>("/raw_imu_data", 10);

    // ── Subscriber ───────────────────────────────────────────────────────────
    get_cmd_ = create_subscription<std_msgs::msg::Int16>(
      "/stm32_data", 10, std::bind(&Stm32Node::get_command, this, _1));

    // ── Initial values ───────────────────────────────────────────────────────
    sensor_data_.data.assign(7, 0.0f);

    // TX: Python initialises as [0]*8, i.e. 8 bytes, no padding to 20 here.
    // Padding to 20 happens inside get_command() after appending the CRC.
    tx_buffer_.assign(8, 0);
    rx_buffer_.assign(20, 0);

    // IMU covariances
    imu_data_.angular_velocity_covariance   = {0,    0, 0, 0, 0, 0, 0, 0, 1e-2};
    imu_data_.linear_acceleration_covariance = {1e-1, 0, 0, 0, 0, 0, 0, 0, 0};
    imu_data_.orientation_covariance        = {0,    0, 0, 0, 0, 0, 0, 0, 1e-2};
    imu_data_.header.frame_id = "base_link";

    sensors_init();

    // ── Timer: 200 Hz (5 ms) — matching Python's create_rate(200) ────────────
    timer_ = create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&Stm32Node::receiveSensorData, this));
  }

  ~Stm32Node()
  {
    if (spi_fd_ >= 0)
      close(spi_fd_);
  }

private:
  // ── Constants ──────────────────────────────────────────────────────────────
  static constexpr uint32_t BAUDRATE   = 112500;
  static constexpr float    YAW_SCALE  = 900.0f;
  static constexpr float    IR_A       = 15.38f;
  static constexpr float    IR_B       = 0.42f;
  static constexpr float    SPEED_SCALE = 0.002f;
  static constexpr float    IR_MIN     = 0.06f;
  static constexpr float    IR_MAX     = 0.30f;

  // ── CRC-32/MPEG-2 ──────────────────────────────────────────────────────────
  // Matches Python: crc ^= val << 24; then 8-bit loop with poly 0x104c11db7
  // Note: Python masks implicitly to 32 bits via & in the shift; we do the same.
  static uint32_t crc32mpeg2(const uint8_t * data, size_t length, uint32_t crc = 0xFFFFFFFF)
  {
    for (size_t i = 0; i < length; ++i) {
      crc ^= static_cast<uint32_t>(data[i]) << 24;
      for (int j = 0; j < 8; ++j)
        crc = (crc & 0x80000000U) ? ((crc << 1) ^ 0x04C11DB7U) : (crc << 1);
    }
    return crc;
  }

  // ── SPI helpers (half-duplex, matching Python spidev behaviour) ─────────────
  bool spi_write(const std::vector<uint8_t> & buf)
  {
    spi_ioc_transfer tr{};
    tr.tx_buf        = reinterpret_cast<unsigned long>(buf.data());
    tr.rx_buf        = 0;          // write-only
    tr.len           = buf.size();
    tr.speed_hz      = BAUDRATE;
    tr.bits_per_word = 8;
    return ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr) >= 0;
  }

  bool spi_read(std::vector<uint8_t> & buf, size_t len)
  {
    buf.assign(len, 0);
    spi_ioc_transfer tr{};
    tr.tx_buf        = 0;          // read-only (MOSI held low)
    tr.rx_buf        = reinterpret_cast<unsigned long>(buf.data());
    tr.len           = len;
    tr.speed_hz      = BAUDRATE;
    tr.bits_per_word = 8;
    return ioctl(spi_fd_, SPI_IOC_MESSAGE(1), &tr) >= 0;
  }

  // ── Subscription callback ──────────────────────────────────────────────────
  void get_command(const std_msgs::msg::Int16::SharedPtr msg)
  {
    // Matches Python exactly:
    //   command = [high_byte, low_byte]
    //   crc = crc32mpeg2(command)          ← CRC over 2 bytes only
    //   command += [crc bytes]             ← 6 bytes total
    //   tx_buffer = command + [0]*2        ← 8 bytes total
    uint16_t val = static_cast<uint16_t>(msg->data);
    std::vector<uint8_t> command = {
      static_cast<uint8_t>((val >> 8) & 0xFF),
      static_cast<uint8_t>( val       & 0xFF)
    };

    uint32_t crc = crc32mpeg2(command.data(), command.size());

    command.push_back((crc >> 24) & 0xFF);
    command.push_back((crc >> 16) & 0xFF);
    command.push_back((crc >>  8) & 0xFF);
    command.push_back( crc        & 0xFF);

    // Pad to 8 bytes with zeros (matching Python's `+ [0]*2`)
    command.resize(8, 0);
    tx_buffer_ = command;
  }

  // ── Sensor initialisation ──────────────────────────────────────────────────
  void sensors_init()
  {
    multi_range_frame_.ir_rear_left.header.frame_id  = "rear_ir_range_frame";
    multi_range_frame_.ir_rear_left.radiation_type   = sensor_msgs::msg::Range::INFRARED;
    multi_range_frame_.ir_rear_left.min_range        = IR_MIN;
    multi_range_frame_.ir_rear_left.max_range        = IR_MAX;

    multi_range_frame_.ir_rear_right.header.frame_id = "rear_ir_range_frame";
    multi_range_frame_.ir_rear_right.radiation_type  = sensor_msgs::msg::Range::INFRARED;
    multi_range_frame_.ir_rear_right.min_range       = IR_MIN;
    multi_range_frame_.ir_rear_right.max_range       = IR_MAX;
  }

  // ── IR range conversion (matches Python) ───────────────────────────────────
  float calculateIRRange(uint16_t raw)
  {
    float val = raw / 1000.0f;          // mV → V
    if (val > 0.0f)
      val = (IR_A / val - IR_B) / 100.0f;   // V → m
    else
      val = IR_MAX;
    return std::clamp(val, 0.0f, IR_MAX);
  }

  // ── Main sensor loop ───────────────────────────────────────────────────────
  void receiveSensorData()
  {
    // Half-duplex: write tx, then read rx — mirrors Python writebytes/readbytes
    if (!spi_write(tx_buffer_)) {
      RCLCPP_ERROR(get_logger(), "SPI write failed");
      return;
    }
    if (!spi_read(rx_buffer_, 20)) {
      RCLCPP_ERROR(get_logger(), "SPI read failed");
      return;
    }

    if (debug_)
      RCLCPP_DEBUG(get_logger(), "[DEBUG] raw SPI RX: %02X %02X %02X %02X ...",
        rx_buffer_[0], rx_buffer_[1], rx_buffer_[2], rx_buffer_[3]);

    // CRC check: Python does crc32mpeg2(data) over all 20 bytes.
    // A valid frame returns 0 (residue property of CRC-32/MPEG-2).
    uint32_t crc_residue = crc32mpeg2(rx_buffer_.data(), rx_buffer_.size());

    if (crc_residue != 0) {
      if (debug_)
        RCLCPP_DEBUG(get_logger(), "CRC mismatch (residue=0x%08X) — frame dropped", crc_residue);
      // Still publish stale data outside the if-block, matching Python behaviour
    } else {
      if (debug_)
        RCLCPP_DEBUG(get_logger(), "CRC OK — parsing frame");

      // Parse — all big-endian 16-bit words, signed where noted
      auto u16 = [&](int i) -> uint16_t {
        return (static_cast<uint16_t>(rx_buffer_[i]) << 8) | rx_buffer_[i + 1];
      };
      auto s16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(u16(i));
      };

      vbat_        =  static_cast<float>(u16(0));
      yaw_         =  static_cast<float>(s16(2)) / -YAW_SCALE;
      ir_gauche_   =  static_cast<float>(u16(4));
      ir_droit_    =  static_cast<float>(u16(6));
      speed_       =  SPEED_SCALE * static_cast<float>(u16(8));
      distance_us_ =  0.01f * static_cast<float>(u16(10));
      acc_x_       =  0.01f * static_cast<float>(s16(12));
      yaw_rate_    =  static_cast<float>(s16(14)) / YAW_SCALE;

      sensor_data_.data = {yaw_, speed_, ir_gauche_, ir_droit_, distance_us_, acc_x_, yaw_rate_};

      // Fork
      fork_data_.speed = speed_;

      // Ranges
      multi_range_frame_.ir_rear_left.range         = calculateIRRange(static_cast<uint16_t>(ir_gauche_));
      multi_range_frame_.ir_rear_left.header.stamp  = get_clock()->now();
      multi_range_frame_.ir_rear_right.range        = calculateIRRange(static_cast<uint16_t>(ir_droit_));
      multi_range_frame_.ir_rear_right.header.stamp = get_clock()->now();

      // IMU
      imu_data_.orientation.z      = std::sin(yaw_ * 0.5f);
      imu_data_.orientation.w      = std::cos(yaw_ * 0.5f);
      imu_data_.angular_velocity.z = yaw_rate_;

      auto stamp = get_clock()->now();
      fork_data_.header.stamp = stamp;
      imu_data_.header.stamp  = stamp;

      // Publish sensor array only on valid frames (Python publishes inside the if)
      if (debug_)
        RCLCPP_DEBUG(get_logger(), "[DEBUG] yaw=%.3f spd=%.3f ir_l=%.0f ir_r=%.0f",
          yaw_, speed_, ir_gauche_, ir_droit_);

      stm_pub_->publish(sensor_data_);
    }

    // These three are published unconditionally in Python (outside the if-block)
    speed_pub_->publish(fork_data_);
    ranges_pub_->publish(multi_range_frame_);
    imu_pub_->publish(imu_data_);
  }

  // ── Members ────────────────────────────────────────────────────────────────
  bool   debug_    = false;
  int    spi_fd_   = -1;

  float  vbat_        = 0.0f;
  float  ir_gauche_   = 0.0f;
  float  ir_droit_    = 0.0f;
  float  speed_       = 0.0f;
  float  distance_us_ = 0.0f;
  float  yaw_         = 0.0f;
  float  acc_x_       = 0.0f;
  float  yaw_rate_    = 0.0f;

  std::vector<uint8_t> tx_buffer_;
  std::vector<uint8_t> rx_buffer_;

  std_msgs::msg::Float32MultiArray          sensor_data_;
  bolide_interfaces::msg::ForkSpeed         fork_data_;
  bolide_interfaces::msg::MultipleRange     multi_range_frame_;
  sensor_msgs::msg::Imu                     imu_data_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr          stm_pub_;
  rclcpp::Publisher<bolide_interfaces::msg::ForkSpeed>::SharedPtr         speed_pub_;
  rclcpp::Publisher<bolide_interfaces::msg::MultipleRange>::SharedPtr     ranges_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr                     imu_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr                   get_cmd_;
  rclcpp::TimerBase::SharedPtr                                            timer_;
};

}  // namespace stm32_bridge

#endif  // STM32_BRIDGE__STM32_NODE_HPP_

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stm32_bridge::Stm32Node>());
  rclcpp::shutdown();
  return 0;
}