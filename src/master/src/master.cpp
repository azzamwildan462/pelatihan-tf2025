#include "master/master.hpp"

Master::Master()
    : Node("master")
{
    if (!logger.init())
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize logger");
        rclcpp::shutdown();
    }

    this->declare_parameter("motor_meter_to_pulse", motor_meter_to_pulse);
    this->get_parameter("motor_meter_to_pulse", motor_meter_to_pulse);
    this->declare_parameter("motor_radian_to_pulse", motor_radian_to_pulse);
    this->get_parameter("motor_radian_to_pulse", motor_radian_to_pulse);

    pub_stm32_from_pc = this->create_publisher<ros2_interface::msg::Stm32FromPc>("stm32/from_pc", 10);
    sub_ds4_to_pc = this->create_subscription<ds4_driver_msgs::msg::Status>(
        "/ds4/to_pc", 10, std::bind(&Master::callback_ds4_to_pc, this, std::placeholders::_1));
    sub_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&Master::callback_cmd_vel, this, std::placeholders::_1));

    tim_routine = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&Master::callback_routine, this));

    logger.info("Master node initialized");
}

Master::~Master()
{
}

void Master::callback_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
{
    vx_nav2 = msg->linear.x;
    vy_nav2 = msg->linear.y;
    vtheta_nav2 = msg->angular.z;

    logger.info("Received cmd_vel: vx=%.2f, vy=%.2f, vtheta=%.2f", vx_nav2, vy_nav2, vtheta_nav2);
}

void Master::callback_ds4_to_pc(const ds4_driver_msgs::msg::Status::SharedPtr msg)
{
    ds4_now = *msg;

    vx_joy = ds4_now.axis_left_y * 1.00;      // Max 1.00 m/s
    vy_joy = ds4_now.axis_left_x * 0.25;      // Max 0.25 m/s
    vtheta_joy = ds4_now.axis_right_x * 0.79; // Max 0.79 rad/s
}

void Master::callback_routine()
{

    // if r1 pressed
    if (ds4_now.button_r1 == 1)
    {
        manual_motion(vx_nav2 * motor_meter_to_pulse, vy_nav2 * motor_meter_to_pulse, vtheta_nav2 * motor_radian_to_pulse);
    }
    else
    {
        manual_motion(vx_joy * motor_meter_to_pulse, vy_joy * motor_meter_to_pulse, vtheta_joy * motor_radian_to_pulse);
    }

    ds4_old = ds4_now;

    process_transmitter();
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_master = std::make_shared<Master>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_master);
    executor.spin();

    return 0;
}