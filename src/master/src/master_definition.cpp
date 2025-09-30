#include "master/master.hpp"

void Master::manual_motion(float vx, float vy, float vtheta)
{
    static rclcpp::Time _time = this->now();

    vx_to_stm32 = vx_to_stm32 * 0.25 + vx * 0.75;
    vy_to_stm32 = vy_to_stm32 * 0.25 + vy * 0.75;
    vtheta_to_stm32 = vtheta_to_stm32 * 0.1 + vtheta * 0.9;

    if (fabsf(vx_to_stm32) > 0.01 || fabsf(vy_to_stm32) > 0.01 || fabsf(vtheta_to_stm32) > 0.01)
    {
        _time = this->now();
        stm32_mode = 1;
    }
    else if (this->now() - _time > rclcpp::Duration::from_seconds(5.0))
    {
        stm32_mode = 0;
    }
}

void Master::process_transmitter()
{
    ros2_interface::msg::Stm32FromPc msg_stm32_from_pc;
    msg_stm32_from_pc.dx = vx_to_stm32;
    msg_stm32_from_pc.dy = vy_to_stm32;
    msg_stm32_from_pc.dtheta = vtheta_to_stm32;
    msg_stm32_from_pc.mode = stm32_mode;
    pub_stm32_from_pc->publish(msg_stm32_from_pc);
}