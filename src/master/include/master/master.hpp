#ifndef MASTER_HPP
#define MASTER_HPP

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "ds4_driver_msgs/msg/status.hpp"
#include "ros2_utils/help_logger.hpp"
#include "ros2_interface/msg/stm32_from_pc.hpp"
#include "ros2_interface/msg/stm32_to_pc.hpp"

class Master : public rclcpp::Node
{
public:
    rclcpp::TimerBase::SharedPtr tim_routine;
    rclcpp::Publisher<ros2_interface::msg::Stm32FromPc>::SharedPtr pub_stm32_from_pc;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
    rclcpp::Subscription<ds4_driver_msgs::msg::Status>::SharedPtr sub_ds4_to_pc;

    HelpLogger logger;

    float motor_meter_to_pulse = 1000.0;  // 1000 pulse per meter
    float motor_radian_to_pulse = 159.15; // 159.15 pulse per radian

    Master();
    ~Master();

    ds4_driver_msgs::msg::Status ds4_now, ds4_old;

    float vx_nav2 = 0.0;
    float vy_nav2 = 0.0;
    float vtheta_nav2 = 0.0;

    float vx_joy = 0.0;
    float vy_joy = 0.0;
    float vtheta_joy = 0.0;

    float vx_to_stm32 = 0.0;
    float vy_to_stm32 = 0.0;
    float vtheta_to_stm32 = 0.0;
    uint8_t stm32_mode = 0; // 0 free wheel, 1 control active

    void callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg);
    void callback_ds4_to_pc(const ds4_driver_msgs::msg::Status::SharedPtr msg);

    void callback_routine();

    void manual_motion(float vx, float vy, float vtheta);

    void process_transmitter();
};

#endif // MASTER_HPP