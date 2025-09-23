#include "ros2_utils/help_logger.hpp"
#include "ros2_utils/help_udp.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

struct EIGEN_ALIGN16 MyPointXYZI
{
    PCL_ADD_POINT4D;
    float intensity;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointXYZI,                  // MyPointXYZI
                                  (float, x, x)                 // x
                                  (float, y, y)                 // y
                                  (float, z, z)                 // z
                                  (float, intensity, intensity) // intensity
)

class IOLSLIDARN301 : public rclcpp::Node
{
public:
    //----Parameter
    int port_msop;
    int port_difop;
    std::string frame_id;
    float distance_min;
    float distance_max;
    float azimuth_start;
    float azimuth_stop;
    float azimuth_step;
    //----Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_xyz;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_xyzi;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laserscan;
    //----Help
    HelpLogger logger;
    HelpUDP udp_msop;
    HelpUDP udp_difop;
    //----Thread
    std::thread thr_main;

    // Socket connection
    // -----------------
    uint8_t rx_buffer_msop[1206];
    uint8_t rx_buffer_difop[1206];

    // Lidar data
    // ----------
    float azimuth_cos[36000];
    float azimuth_sin[36000];
    pcl::PointCloud<pcl::PointXYZ> pointcloud_xyz;
    pcl::PointCloud<MyPointXYZI> pointcloud_xyzi;
    sensor_msgs::msg::LaserScan msg_laserscan;

    typedef struct
    {
        uint8_t distance[2];
        uint8_t intensity;
    } msop_data_t;

    typedef struct
    {
        uint8_t flag[2];
        uint8_t azimuth[2];
        msop_data_t data[32];
    } msop_block_t;

    typedef struct
    {
        msop_block_t block[12];
        uint8_t timestamp[4];
        uint8_t factory[2];
    } msop_packet_t;

    IOLSLIDARN301() : Node("io_lslidar_n301")
    {
        //----Parameter
        this->declare_parameter("port_msop", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("port_difop", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
        this->declare_parameter("distance_min", 0.1);
        this->declare_parameter("distance_max", 10.0);
        this->declare_parameter("azimuth_start", 0.0);
        this->declare_parameter("azimuth_stop", 360.0);
        this->declare_parameter("azimuth_step", 1.0);
        this->get_parameter("port_msop", port_msop);
        this->get_parameter("port_difop", port_difop);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("distance_min", distance_min);
        this->get_parameter("distance_max", distance_max);
        this->get_parameter("azimuth_start", azimuth_start);
        this->get_parameter("azimuth_stop", azimuth_stop);
        this->get_parameter("azimuth_step", azimuth_step);
        //----Publisher
        pub_pointcloud_xyz = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_xyz", 10);
        pub_pointcloud_xyzi = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_xyzi", 10);
        pub_laserscan = this->create_publisher<sensor_msgs::msg::LaserScan>("laserscan", 10);
        //----Help
        if (!logger.init())
        {
            rclcpp::shutdown();
        }
        if (!udp_msop.init_as_server(port_msop))
        {
            rclcpp::shutdown();
        }
        if (!udp_difop.init_as_server(port_difop))
        {
            rclcpp::shutdown();
        }
        //----Thread
        thr_main = std::thread(&IOLSLIDARN301::main, this);
    }

    ~IOLSLIDARN301()
    {
        if (thr_main.joinable())
        {
            thr_main.join();
        }
    }

    void
    main()
    {
        logger.info("port_msop: %d", port_msop);
        logger.info("port_difop: %d", port_difop);
        logger.info("frame_id: %s", frame_id.c_str());
        logger.info("distance_min: %f", distance_min);
        logger.info("distance_max: %f", distance_max);
        logger.info("azimuth_start: %f", azimuth_start);
        logger.info("azimuth_stop: %f", azimuth_stop);
        logger.info("azimuth_step: %f", azimuth_step);

        for (int i = 0; i < 36000; i++)
        {
            azimuth_cos[i] = cos(i * M_PI / 18000);
            azimuth_sin[i] = sin(i * M_PI / 18000);
        }

        msg_laserscan.angle_min = azimuth_start * M_PI / 180;
        msg_laserscan.angle_max = azimuth_stop * M_PI / 180;
        msg_laserscan.angle_increment = azimuth_step * M_PI / 180;
        msg_laserscan.range_min = distance_min;
        msg_laserscan.range_max = distance_max;
        int _n = azimuth_stop > azimuth_start ? (azimuth_stop - azimuth_start) / azimuth_step + 1
                                              : (360 - azimuth_start + azimuth_stop) / azimuth_step + 1;
        msg_laserscan.ranges.assign(_n, 0);
        msg_laserscan.intensities.resize(_n, 0);

        while (rclcpp::ok())
        {
            if (udp_msop.recv(rx_buffer_msop, 1206) == 1206)
            {
                parse_msop();
            }
            if (udp_difop.recv(rx_buffer_difop, 1206) == 1206)
            {
                parse_difop();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void
    parse_msop()
    {
        msop_packet_t *packet = (msop_packet_t *)rx_buffer_msop;

        for (int i_block = 0; i_block < 12; i_block++)
        {
            // Check if packet header is correct
            static const uint8_t flag[2] = {0xFF, 0xEE};

            if (memcmp(packet->block[i_block].flag, flag, 2) != 0)
            {
                continue;
            }

            // Calculate azimuth difference between current and next/previous block
            static float azimuth_difference;
            if (i_block > 0)
            {
                uint16_t azimuth0 = *(uint16_t *)packet->block[i_block - 1].azimuth;
                uint16_t azimuth1 = *(uint16_t *)packet->block[i_block].azimuth;
                azimuth_difference = (float)((azimuth1 - azimuth0 + 36000) % 36000) / 100;
            }
            else
            {
                uint16_t azimuth0 = *(uint16_t *)packet->block[i_block].azimuth;
                uint16_t azimuth1 = *(uint16_t *)packet->block[i_block + 1].azimuth;
                azimuth_difference = (float)((azimuth1 - azimuth0 + 36000) % 36000) / 100;
            }

            // Calculate azimuth of current block
            static float azimuth_block;
            static float azimuth_block_old;
            azimuth_block_old = azimuth_block;
            azimuth_block = (float)(*(uint16_t *)packet->block[i_block].azimuth) / 100;

            for (int i_laser = 0; i_laser < 2; i_laser++)
            {
                for (int i_data = 0; i_data < 15; i_data++)
                {
                    // Calculate azimuth of current data
                    static float azimuth_data;
                    azimuth_data = azimuth_block + (azimuth_difference / 30) * (i_laser * 15 + i_data);
                    azimuth_data = azimuth_data >= 360 ? azimuth_data - 360 : azimuth_data;

                    // Calculate distance and intensity of current data
                    uint16_t distance_int = *(uint16_t *)packet->block[i_block].data[i_laser * 16 + i_data].distance;
                    uint8_t intensity_int = packet->block[i_block].data[i_laser * 16 + i_data].intensity;
                    float distance_float = (float)distance_int * 0.004;
                    float intensity_float = (float)intensity_int / 255;

                    // Check if data is valid for publishing
                    if (distance_float < distance_min || distance_float > distance_max)
                    {
                        continue;
                    }
                    else if (azimuth_stop > azimuth_start && (azimuth_data < azimuth_start || azimuth_data > azimuth_stop))
                    {
                        continue;
                    }
                    else if (azimuth_stop < azimuth_start && (azimuth_data < azimuth_start && azimuth_data > azimuth_stop))
                    {
                        continue;
                    }

                    // Filling point cloud data
                    int _i = azimuth_data * 100;
                    pcl::PointXYZ point_xyz;
                    MyPointXYZI point_xyzi;
                    point_xyzi.x = point_xyz.x = distance_float * azimuth_cos[_i];
                    point_xyzi.y = point_xyz.y = distance_float * azimuth_sin[_i];
                    point_xyzi.z = point_xyz.z = 0;
                    point_xyzi.intensity = intensity_float;
                    pointcloud_xyz.push_back(point_xyz);
                    pointcloud_xyzi.push_back(point_xyzi);

                    // Filling laser scan data
                    int _j = (azimuth_data - azimuth_start) / azimuth_step;
                    msg_laserscan.ranges[_j] = fmax(msg_laserscan.ranges[_j], distance_float);
                    msg_laserscan.intensities[_j] = fmax(msg_laserscan.intensities[_j], intensity_float);
                }
            }

            if (azimuth_block < azimuth_block_old)
            {
                sensor_msgs::msg::PointCloud2 msg_pointcloud_xyz;
                sensor_msgs::msg::PointCloud2 msg_pointcloud_xyzi;
                pcl::toROSMsg(pointcloud_xyz, msg_pointcloud_xyz);
                pcl::toROSMsg(pointcloud_xyzi, msg_pointcloud_xyzi);
                msg_pointcloud_xyzi.header.frame_id = msg_pointcloud_xyz.header.frame_id = frame_id;
                msg_pointcloud_xyzi.header.stamp = msg_pointcloud_xyz.header.stamp = rclcpp::Clock().now();
                pub_pointcloud_xyz->publish(msg_pointcloud_xyz);
                pub_pointcloud_xyzi->publish(msg_pointcloud_xyzi);

                pointcloud_xyz.clear();
                pointcloud_xyzi.clear();

                msg_laserscan.header.frame_id = frame_id;
                msg_laserscan.header.stamp = rclcpp::Clock().now();
                pub_laserscan->publish(msg_laserscan);

                msg_laserscan.ranges.assign(msg_laserscan.ranges.size(), 0);
                msg_laserscan.intensities.assign(msg_laserscan.intensities.size(), 0);
            }
        }
    }

    void
    parse_difop() {}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node_io_lslidar_n301 = std::make_shared<IOLSLIDARN301>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_io_lslidar_n301);
    executor.spin();

    return 0;
}
