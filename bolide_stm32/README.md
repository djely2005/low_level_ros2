# Bolide STM32

This ROS2 package manages communication with the STM32 microcontroller and the associated electronic components for the Bolide autonomous car.

## Description

The package provides the interface between ROS2 and the STM32 which controls:
- Navigation sensors (IMU, rear infrared sensors, optical speed sensor)
- Propulsion motor via ESC (Electronic Speed Controller)
- Bidirectional SPI communication

## Architecture

The package includes three main nodes:

### stm32_node
Main communication node with the STM32 via SPI. It:
- Receives sensor data (IMU, IR, fork)
- Converts and publishes data on appropriate topics
- Transmits PWM commands to the STM32

### cmd_vel_node
Speed control node that:
- Receives speed commands on `/cmd_vel`
- Converts normalized values (-1.0 to 1.0) to PWM signals
- Handles safety with emergency stop timeout
- Implements direction change logic

### esc_setup
ESC calibration and testing tool for Tamiya TBLE-04S.

## Managed Sensors

### IMU (Inertial Measurement Unit)
- **Acceleration**: X-axis (m/s²)
- **Angular velocity**: around Z-axis (rad/s)
- **Orientation**: yaw in radians

### Rear Infrared Sensors
- **Range**: 6cm to 30cm
- **Position**: left and right of the vehicle
- **Usage**: rear obstacle detection

### Optical Speed Sensor (Fork)
- **Measurement**: linear wheel speed (m/s)
- **Direction**: rotation direction (forward/backward)

## Speed Commands

The system accepts normalized commands:
- `0.0`: neutral point (freewheel)
- `0.01` to `1.0`: forward (increasing speed)
- `-0.01` to `-1.0`: reverse (increasing speed)
- `2.0`: emergency brake (not implemented in cmd_vel_node)

**Warning**: Avoid `1.0` and `-1.0` values which may cause Raspberry Pi voltage drops.

## Topics

### Publishers (stm32_node)
- `/raw_fork_data` (bolide_interfaces/ForkSpeed): wheel speed
- `/raw_imu_data` (sensor_msgs/Imu): IMU data
- `/raw_rear_range_data` (bolide_interfaces/MultipleRange): rear IR sensors
- `/stm32_sensors` (std_msgs/Float32MultiArray): raw data array

### Subscribers (stm32_node)
- `/stm32_data` (std_msgs/Int16): PWM commands for ESC

### Subscribers (cmd_vel_node)
- `/cmd_vel` (std_msgs/Float32): normalized speed commands

## Parameters

### stm32_node
- `debug`: enables debug logs (default: False)

### cmd_vel_node
- `debug`: enables debug logs (default: False)
- `minimal_speed`: minimum PWM speed for ESC (default: 8.3)

## Launch

```bash
# Main STM32 node
ros2 run bolide_stm32 stm32_node

# Speed control node
ros2 run bolide_stm32 cmd_vel_node

# ESC calibration tool
ros2 run bolide_stm32 esc_setup
```

## ESC Calibration

The ESC must be calibrated before first use:

1. Launch `esc_setup` and choose option 1 (calibration)
2. Power on the ESC
3. Follow instructions to record MAX, MIN and NEUTRAL
4. Test with option 2

If this code doesn't work properly, use the teleop to calibrate the ESC : 
- Launch the teleop launch
- Enter the ESC in calibration mode (make sure it's on and press the SET button until it start the sequence green-orange-red and release when it's red. It should start blinking red)
- Press 25 times the UP arrow to send the max throttle
- Press the SET button again to go to the next step (it should blink red twice now)
- Press 25 times the DOWN arrow to send the min throttle
- Press the SET button again to go to the next step (the light should stop blinking and stay off)

Your ESC is now recalibrated.

## Safety

- **Safety timeout**: automatic stop after 0.5s without command
- **Direction change**: requires prior stop to avoid jerks
- **Voltage limits**: avoid extreme values to preserve Raspberry Pi

## Troubleshooting

### SPI Communication Issues
- Check physical connections (SPI bus 0, device 1)
- Check SPI speed (112500 Hz)

### ESC Not Responding
- Recalibrate ESC with `esc_setup`
- Check ESC power supply
- Test PWM values with option 3 of `esc_setup`

### Faulty Sensors
- Check connections on STM32
- Check debug logs (`debug: true`)
- IR sensors may require manual calibration

## Hardware Architecture

The STM32 communicates via SPI with the Raspberry Pi and controls:
- 9DOF IMU (accelerometer, gyroscope, magnetometer)
- 2 Sharp GP2Y0A21YK IR sensors (rear)
- Optical speed sensor (fork)
- Tamiya TBLE-04S ESC (propulsion motor)