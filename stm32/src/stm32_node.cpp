#ifndef STM32_BRIDGE__STM32_NODE_HPP_
#define STM32_BRIDGE__STM32_NODE_HPP_
#include <functional>
#include <vector>
#include <cmath>
#include <algorithm>
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
  private:
    bool debug;
    int BAUDRATE;
    int spi_fd;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr stm_pub;
    rclcpp::Publisher<bolide_interfaces::msg::ForkSpeed>::SharedPtr speed_pub;
    rclcpp::Publisher<bolide_interfaces::msg::MultipleRange>::SharedPtr ranges_pub;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr get_cmd;
    rclcpp::TimerBase::SharedPtr timer_;

    // Values
    std_msgs::msg::Float32MultiArray sensor_data;
    float vbat;
    float ir_min_range;
    float ir_max_range;
    float ir_gauche;
    float ir_droit;
    float speed;
    bolide_interfaces::msg::ForkSpeed fork_data;
    float distance_US;
    float yaw;
    float acc_x;
    float yaw_rate;
    sensor_msgs::msg::Imu imu_data;
    bolide_interfaces::msg::MultipleRange multi_range_frame;
    std::vector<uint8_t> tx_buffer;

    std::vector<uint8_t> rx_buffer;
    bool data_updated;

    static constexpr float YAW_SCALE = 900.0f;
  static constexpr float IR_A = 15.38f;
  static constexpr float IR_B = 0.42f;
  static constexpr float SPEED_SCALE = 0.002f;

  public:
    // Use explicit to prevent unintended type conversions
    explicit Stm32Node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("stm32_node", options)
    {
      this->declare_parameter("debug", false);
      this->debug = this->get_parameter("debug").get_value<bool>();
      if (this->debug)
      {
        this->get_logger().set_level(rclcpp::Logger::Level::Debug);
      }
      this->BAUDRATE = 112500;

      // SPI setup
      const char *device = "/dev/spidev0.1";
      this->spi_fd = open(device, O_RDWR);
      if (this->spi_fd < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open SPI device");
        throw std::runtime_error("SPI device open failed");
      }
      uint8_t mode = SPI_MODE_0;
      ioctl(this->spi_fd, SPI_IOC_WR_MODE, &mode);
      uint32_t speed = this->BAUDRATE;
      ioctl(this->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

      stm_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/stm32_sensors", 10);
      speed_pub = this->create_publisher<bolide_interfaces::msg::ForkSpeed>("/raw_fork_data", 10);
      ranges_pub = this->create_publisher<bolide_interfaces::msg::MultipleRange>("/raw_rear_range_data", 10);
      imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/raw_imu_data", 10);

      get_cmd = this->create_subscription<std_msgs::msg::Int16>(
          "/stm32_data", 10, std::bind(&Stm32Node::get_command, this, _1));

      // Initialize values
      sensor_data.data = std::vector<float>(7, 0.0f);
      vbat = 0.0f;
      ir_min_range = 0.06f;
      ir_max_range = 0.3f;
      ir_gauche = 0.0f;
      ir_droit = 0.0f;
      speed = 0.0f;
      distance_US = 0.0f;
      yaw = 0.0f;
      acc_x = 0.0f;
      yaw_rate = 0.0f;
      tx_buffer = std::vector<uint8_t>(8, 0);
      rx_buffer.resize(20, 0);

      // IMU covariances
      imu_data.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 1e-2};
      imu_data.linear_acceleration_covariance = {1e-1, 0, 0, 0, 0, 0, 0, 0, 0};
      imu_data.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 1e-2};
      imu_data.header.frame_id = "base_link";

      sensors_init();

      // Timer for 200Hz loop
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(5), std::bind(&Stm32Node::receiveSensorData, this));
    }

    ~Stm32Node()
    {
      if (spi_fd >= 0)
      {
        close(spi_fd);
      }
    }

  private:
    void get_command(const std_msgs::msg::Int16::SharedPtr msg)
    {
      std::vector<uint8_t> command;
      uint16_t cmded_bytes = msg->data;
      command.push_back((cmded_bytes >> 8) & 0xFF);
      command.push_back(cmded_bytes & 0xFF);

      uint32_t crc = crc32mpeg2(command);

      command.push_back((crc >> 24) & 0xFF);
      command.push_back((crc >> 16) & 0xFF);
      command.push_back((crc >> 8) & 0xFF);
      command.push_back(crc & 0xFF);

      tx_buffer = command;
      tx_buffer.resize(8, 0);
    }

    uint32_t crc32mpeg2(const std::vector<uint8_t> &buf, uint32_t crc = 0xffffffff) const
    {
      for (uint8_t val : buf)
      {
        crc ^= val << 24;
        for (int i = 0; i < 8; ++i)
        {
          crc = (crc & 0x80000000) == 0 ? crc << 1 : (crc << 1) ^ 0x104c11db7;
        }
      }
      return crc;
    }

    void sensors_init()
    {
      multi_range_frame.ir_rear_left.header.frame_id = "rear_ir_range_frame";
      multi_range_frame.ir_rear_left.radiation_type = sensor_msgs::msg::Range::INFRARED;
      multi_range_frame.ir_rear_left.min_range = ir_min_range;
      multi_range_frame.ir_rear_left.max_range = ir_max_range;

      multi_range_frame.ir_rear_right.header.frame_id = "rear_ir_range_frame";
      multi_range_frame.ir_rear_right.radiation_type = sensor_msgs::msg::Range::INFRARED;
      multi_range_frame.ir_rear_right.min_range = ir_min_range;
      multi_range_frame.ir_rear_right.max_range = ir_max_range;
    }

    template <typename T>
    T bytesToInteger(const uint8_t *buffer) const
    {
      T result = 0;
      for (size_t i = 0; i < sizeof(T); ++i)
      {
        // Shift left, then OR with the next byte
        result = (result << 8) | static_cast<T>(buffer[i]);
      }
      return result;
    }

    float calculateIRRange(uint16_t raw_value)
    {
      float val = raw_value / 1000.0f;

      if (val > 0)
      {
        val = (IR_A / val - IR_B) / 100.0f;
      }
      else
      {
        val = ir_max_range;
      }

      // std::clamp(value, min, max)
      return std::clamp(val, 0.0f, ir_max_range);
    }

    void receiveSensorData()
    {
      struct spi_ioc_transfer tr = {};
      tr.tx_buf = (unsigned long)tx_buffer.data();
      tr.rx_buf = (unsigned long)rx_buffer.data();
      tr.len = 20;
      tr.speed_hz = BAUDRATE;
      tr.bits_per_word = 8;

      if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "SPI transfer failed");
        return;
      }

      if (debug)
      {
        RCLCPP_DEBUG(this->get_logger(), "raw SPI RX: %s", std::to_string(rx_buffer[0]).c_str()); // Simplified debug
      }

      if (crc32mpeg2(rx_buffer) == 0)
      {
        vbat = bytesToInteger<uint16_t>(&rx_buffer[0]);
        yaw = static_cast<float>(static_cast<int16_t>(bytesToInteger<uint16_t>(&rx_buffer[2]))) / -YAW_SCALE;
        ir_gauche = bytesToInteger<uint16_t>(&rx_buffer[4]);
        ir_droit = bytesToInteger<uint16_t>(&rx_buffer[6]);
        speed = SPEED_SCALE * bytesToInteger<uint16_t>(&rx_buffer[8]);
        distance_US = 0.01f * bytesToInteger<uint16_t>(&rx_buffer[10]);
        acc_x = 0.01f * static_cast<int16_t>(bytesToInteger<uint16_t>(&rx_buffer[12]));
        yaw_rate = static_cast<float>(static_cast<int16_t>(bytesToInteger<uint16_t>(&rx_buffer[14]))) / YAW_SCALE;

        sensor_data.data = {yaw, speed, ir_gauche, ir_droit, distance_US, acc_x, yaw_rate};

        fork_data.speed = speed;
        fork_data.header.stamp = this->get_clock()->now();

        multi_range_frame.ir_rear_left.range = calculateIRRange(ir_gauche);
        multi_range_frame.ir_rear_left.header.stamp = this->get_clock()->now();
        multi_range_frame.ir_rear_right.range = calculateIRRange(ir_droit);
        multi_range_frame.ir_rear_right.header.stamp = this->get_clock()->now();

        // IMU
        imu_data.orientation.z = std::sin(yaw * 0.5f);
        imu_data.orientation.w = std::cos(yaw * 0.5f);
        imu_data.angular_velocity.z = yaw_rate;

        builtin_interfaces::msg::Time stamp = this->get_clock()->now();
        fork_data.header.stamp = stamp;
        imu_data.header.stamp = stamp;
        data_updated = true;

        stm_pub->publish(sensor_data);
        if (debug)
        {
          RCLCPP_DEBUG(this->get_logger(), "stm32 sensors value: %f", sensor_data.data[0]); // Simplified
        }
      }
      if (data_updated)
      {
        speed_pub->publish(fork_data);
        ranges_pub->publish(multi_range_frame);
        imu_pub->publish(imu_data);
      }
    }
  };
} // namespace stm32_bridge

#endif // STM32_BRIDGE__STM32_NODE_HPP_

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<stm32_bridge::Stm32Node>());
  rclcpp::shutdown();
  return 0;
}
