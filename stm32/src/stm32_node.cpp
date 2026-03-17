#ifndef STM32_BRIDGE__STM32_NODE_HPP_
#define STM32_BRIDGE__STM32_NODE_HPP_
#include <functional>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>
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
// Maybe add Header to MultipleRange interface later
namespace stm32_bridge
{
  class Stm32Node : public rclcpp::Node
  {
  private:
    bool debug;
    int BAUDRATE = 112500;
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
    // precomputed values
    static const uint32_t crc32_mpeg_table[256];

  public:
    explicit Stm32Node(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("stm32_node", options)
    {
      // Parameter set ups

      this->declare_parameter("debug", false);
      this->debug = this->get_parameter("debug").as_bool();
      if (this->debug)
      {
        this->get_logger().set_level(rclcpp::Logger::Level::Debug);
      }

      this->declare_parameter("esp32", "/dev/spidev0.1");
      std::string device = this->get_parameter("esp32").as_string();

      // SPI setup
      // const char \*device = "/dev/spidev0.1";
      this->spi_fd = open(device.c_str(), O_RDWR);
      if (this->spi_fd < 0)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open SPI device");
        throw std::runtime_error("SPI device open failed");
      }
      uint8_t mode = SPI_MODE_0;
      ioctl(this->spi_fd, SPI_IOC_WR_MODE, &mode);
      uint32_t speed = this->BAUDRATE;
      ioctl(this->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

      // publishers
      stm_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>("/stm32_sensors", 10);
      speed_pub = this->create_publisher<bolide_interfaces::msg::ForkSpeed>("/raw_fork_data", 10);
      ranges_pub = this->create_publisher<bolide_interfaces::msg::MultipleRange>("/raw_rear_range_data", 10);
      imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/raw_imu_data", 10);

      // subscribers
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
      tx_buffer = std::vector<uint8_t>(20, 0);
      rx_buffer.resize(20, 0);

      // IMU covariances
      imu_data.angular_velocity_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 1e-2};
      imu_data.linear_acceleration_covariance = {1e-1, 0, 0, 0, 0, 0, 0, 0, 0};
      imu_data.orientation_covariance = {0, 0, 0, 0, 0, 0, 0, 0, 1e-2};
      imu_data.header.frame_id = "base_link";

      sensors_init();

      // Timer for 12hz I chose this to be the same as the LiDar
      timer_ = this->create_wall_timer(
          std::chrono::milliseconds(1/12), std::bind(&Stm32Node::receiveSensorData, this));
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

      uint32_t crc = crc32mpeg2(command.data(), command.size());

      command.push_back((crc >> 24) & 0xFF);
      command.push_back((crc >> 16) & 0xFF);
      command.push_back((crc >> 8) & 0xFF);
      command.push_back(crc & 0xFF);

      // CHANGE THIS: Match the 20-byte rx_buffer length
      tx_buffer = command;
      tx_buffer.resize(20, 0);
    }

    static uint32_t crc32mpeg2(const uint8_t *data, size_t length, uint32_t crc = 0xFFFFFFFF)
    {
      for (size_t i = 0; i < length; ++i)
      {
        crc = (crc << 8) ^ crc32_mpeg_table[((crc >> 24) ^ data[i]) & 0xFF];
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

      if (crc32mpeg2(rx_buffer.data(), rx_buffer.size()) == 0)
      {

        // vbat is never published for now
        vbat = bytesToInteger<uint16_t>(&rx_buffer[0]);
        yaw = static_cast<float>(static_cast<int16_t>(bytesToInteger<uint16_t>(&rx_buffer[2]))) / -YAW_SCALE;
        ir_gauche = bytesToInteger<uint16_t>(&rx_buffer[4]);
        ir_droit = bytesToInteger<uint16_t>(&rx_buffer[6]);
        speed = SPEED_SCALE * bytesToInteger<uint16_t>(&rx_buffer[8]);

        // I have no idea what ditance_US and acc_x are
        distance_US = 0.01f * bytesToInteger<uint16_t>(&rx_buffer[10]);
        acc_x = 0.01f * static_cast<int16_t>(bytesToInteger<uint16_t>(&rx_buffer[12]));
        yaw_rate = static_cast<float>(static_cast<int16_t>(bytesToInteger<uint16_t>(&rx_buffer[14]))) / YAW_SCALE;

        sensor_data.data = {yaw, speed, ir_gauche, ir_droit, distance_US, acc_x, yaw_rate};

        fork_data.speed = speed;
        // fork_data.header.stamp = this->get_clock()->now();

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

        // stm_pub is not used for now
        // stm_pub->publish(sensor_data);
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
    // Table initialization (polynomial: 0x04C11DB7, reflected: false)
    const uint32_t Stm32Node::crc32_mpeg_table[256] = {
        0x00000000, 0x04C11DB7, 0x09823B6E, 0x0D4326D9, 0x130476DC, 0x17C56B6B,
        0x1A864DB2, 0x1E475005, 0x2608EDB8, 0x22C9F00F, 0x2F8AD6D6, 0x2B4BCB61,
        0x350C9B64, 0x31CD86D3, 0x3C8EA00A, 0x384FBDBD, 0x4C11DB70, 0x48D0C6C7,
        0x4593E01E, 0x4152FDA9, 0x5F15ADAC, 0x5BD4B01B, 0x569796C2, 0x52568B75,
        0x6A1936C8, 0x6ED82B7F, 0x639B0DA6, 0x675A1011, 0x791D4014, 0x7DDC5DA3,
        0x709F7B7A, 0x745E66CD, 0x9823B6E0, 0x9CE2AB57, 0x91A18D8E, 0x95609039,
        0x8B27C03C, 0x8FE6DD8B, 0x82A5FB52, 0x8664E6E5, 0xBE2B5B58, 0xBAEA46EF,
        0xB7A96036, 0xB3687D81, 0xAD2F2D84, 0xA9EE3033, 0xA4AD16EA, 0xA06C0B5D,
        0xD4326D90, 0xD0F37027, 0xDDB056FE, 0xD9714B49, 0xC7361B4C, 0xC3F706FB,
        0xCEB42022, 0xCA753D95, 0xF23A8028, 0xF6FB9D9F, 0xFBB8BB46, 0xFF79A6F1,
        0xE13EF6F4, 0xE5FFEB43, 0xE8BCCD9A, 0xEC7DD02D, 0x34867077, 0x30476DC0,
        0x3D044B19, 0x39C556AE, 0x278206AB, 0x23431B1C, 0x2E003DC5, 0x2AC12072,
        0x128E9DCF, 0x164F8078, 0x1B0CA6A1, 0x1FCDBB16, 0x018AEB13, 0x054BF6A4,
        0x0808D07D, 0x0CC9CDCA, 0x7897AB07, 0x7C56B6B0, 0x71159069, 0x75D48DDE,
        0x6B93DDDB, 0x6F52C06C, 0x6211E6B5, 0x66D0FB02, 0x5E9F46BF, 0x5A5E5B08,
        0x571D7DD1, 0x53DC6066, 0x4D9B3063, 0x495A2DD4, 0x44190B0D, 0x40D816BA,
        0xACA5C697, 0xA864DB20, 0xA527FDF9, 0xA1E6E04E, 0xBFA1B04B, 0xBB60ADFC,
        0xB6238B25, 0xB2E29692, 0x8AAD2B2F, 0x8E6C3698, 0x832F1041, 0x87EE0DF6,
        0x99A95DF3, 0x9D684044, 0x902B669D, 0x94EA7B2A, 0xE0B41DE7, 0xE4750050,
        0xE9362689, 0xEDF73B3E, 0xF3B06B3B, 0xF771768C, 0xFA325055, 0xFEF34DE2,
        0xC6BCF05F, 0xC27DEDE8, 0xCF3ECB31, 0xCBFFD686, 0xD5B88683, 0xD1799B34,
        0xDC3ABDED, 0xD8FBA05A, 0x690CE0EE, 0x6DCDFD59, 0x608EDB80, 0x644FC637,
        0x7A089632, 0x7EC98B85, 0x738AAD5C, 0x774BB0EB, 0x4F040D56, 0x4BC510E1,
        0x46863638, 0x42472B8F, 0x5C007B8A, 0x58C1663D, 0x558240E4, 0x51435D53,
        0x251D3B9E, 0x21DC2629, 0x2C9F00F0, 0x285E1D47, 0x36194D42, 0x32D850F5,
        0x3F9B762C, 0x3B5A6B9B, 0x0315D626, 0x07D4CB91, 0x0A97ED48, 0x0E56F0FF,
        0x1011A0FA, 0x14D0BD4D, 0x19939B94, 0x1D528623, 0xF12F560E, 0xF5EE4BB9,
        0xF8AD6D60, 0xFC6C70D7, 0xE22B20D2, 0xE6EA3D65, 0xEBA91BBC, 0xEF68060B,
        0xD727BBB6, 0xD3E6A601, 0xDEA580D8, 0xDA649D6F, 0xC423CD6A, 0xC0E2D0DD,
        0xCDA1F604, 0xC960EBB3, 0xBD3E8D7E, 0xB9FF90C9, 0xB4BCB610, 0xB07DABA7,
        0xAE3AFBA2, 0xAAFBE615, 0xA7B8C0CC, 0xA379DD7B, 0x9B3660C6, 0x9FF77D71,
        0x92B45BA8, 0x9675461F, 0x8832161A, 0x8CF30BAD, 0x81B02D74, 0x857130C3,
        0x5D8A9099, 0x594B8D2E, 0x5408ABF7, 0x50C9B640, 0x4E8EE645, 0x4A4FFBF2,
        0x470CDD2B, 0x43CDC09C, 0x7B827D21, 0x7F436096, 0x7200464F, 0x76C15BF8,
        0x68860BFD, 0x6C47164A, 0x61043093, 0x65C52D24, 0x119B4BE9, 0x155A565E,
        0x18197087, 0x1CD86D30, 0x029F3D35, 0x065E2082, 0x0B1D065B, 0x0FDC1BEC,
        0x3793A651, 0x3352BBE6, 0x3E119D3F, 0x3AD08088, 0x2497D08D, 0x2056CD3A,
        0x2D15EBE3, 0x29D4F654, 0xC5A92679, 0xC1683BCE, 0xCC2B1D17, 0xC8EA00A0,
        0xD6AD50A5, 0xD26C4D12, 0xDF2F6BCB, 0xDBEE767C, 0xE3A1CBC1, 0xE760D676,
        0xEA23F0AF, 0xEEE2ED18, 0xF0A5BD1D, 0xF464A0AA, 0xF9278673, 0xFDE69BC4,
        0x89B8FD09, 0x8D79E0BE, 0x803AC667, 0x84FBDBD0, 0x9ABC8BD5, 0x9E7D9662,
        0x933EB0BB, 0x97FFAD0C, 0xAFB010B1, 0xAB710D06, 0xA6322BDF, 0xA2F33668,
        0xBCB4666D, 0xB8757BDA, 0xB5365D03, 0xB1F740B4};
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