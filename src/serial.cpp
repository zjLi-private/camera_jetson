#include "serial.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_bridge");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string port;
    int baud = 115200; //波特率设置
    pnh.param<std::string>("port", port, "/dev/camera");
    pnh.param<int>("baudrate", baud, 115200);

    try {
        ser.setPort(port);
        ser.setBaudrate(baud);
        serial::Timeout to(serial::Timeout::max(), 1000, 0, 1000, 0);
        ser.setTimeout(to);
        ser.open();
    } catch (const std::exception &e) {
        ROS_FATAL_STREAM("Failed to open serial port: " << e.what());
        return -1;
    }

    if (!ser.isOpen()) {
        ROS_FATAL("Serial port not open after open()");
        return -1;
    }
    ROS_INFO("Serial port opened.");

    // 订阅 k4a → 串口
    ros::Subscriber sub_k4a = nh.subscribe("/k4a/target_info", 10, k4aCallback);

    // 发布 串口 → ROS
    ros::Publisher pub_rx = nh.advertise<std_msgs::String>("/serial_rx", 10);

    ros::spin();

    if (ser.isOpen()) ser.close();
    return 0;
}
