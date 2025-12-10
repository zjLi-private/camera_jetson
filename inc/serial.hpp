#include <std_msgs/String.h>
#include <thread>
#include <sstream>
#include <vector>
#include <cstring>   // memcpy
#include <iomanip>   // setw setfill
#include <limits>
#include <exception>
#include <ros/ros.h>
#include <serial/serial.h>



serial::Serial ser;

// 包头包尾（binary）
const uint8_t K4A_HEAD[2] = {0xFF, 0xFE};
const uint8_t K4A_TAIL[2] = {0xAA, 0xDD};
// 解析 k4a bbox 的字符串并生成串口二进制 payload
// 输入示例: "1,12.3,45.6,78.9,0.123"  --> 5 个字段
// 输出 payload: [ID(1B)] [x(4B float)] [y(4B float)] [z(4B float)] [yaw(4B float)]
bool parseBBoxString(const std::string &msg, std::vector<uint8_t> &payload)
{
    std::stringstream ss(msg);
    std::string token;
    std::vector<std::string> tokens;

    while (std::getline(ss, token, ','))
        tokens.push_back(token);

    //  5 个字段： id,x,y,z,yaw
    if (tokens.size() != 5)
        return false;

    int cls = 0;
    float x = 0.f, y = 0.f, z = 0.f, yaw = 0.f;
    try {
        cls = std::stoi(tokens[0]);
        x = std::stof(tokens[1]);
        y = std::stof(tokens[2]);
        z = std::stof(tokens[3]);
        yaw = std::stof(tokens[4]);
    } catch (const std::exception &e) {
        ROS_WARN_STREAM("[serial_bridge] parse error: " << e.what() << " input: " << msg);
        return false;
    }

    // cls 允许范围 1..3
    // R1,R2_Fake,R2Real
    if (!(cls >= 1 && cls <= 3))
        return false;
    // 如果 x == 0 则视为无效
    if (x == 0.0f)
        return false;

    // 组装二进制 payload
    payload.clear();
    payload.push_back(static_cast<uint8_t>(cls)); // ID

    auto append_float = [&](float value) {
        uint8_t bytes[4];
        std::memcpy(bytes, &value, sizeof(float)); // 小端序的原生 float bytes
        payload.insert(payload.end(), bytes, bytes + 4);
    };

    append_float(x);
    append_float(y);
    append_float(z);
    append_float(yaw);

    return true;
}

// 回调 1：来自 k4a 的发送请求（需要加包头包尾）
void k4aCallback(const std_msgs::String::ConstPtr &msg)
{
    if (!ser.isOpen()) {
        ROS_WARN_THROTTLE(5.0, "[serial_bridge] serial port not open");
        return;
    }

    std::vector<uint8_t> payload;
    if (!parseBBoxString(msg->data, payload)) {
        ROS_WARN_STREAM("[serial_bridge] Invalid k4a bbox: " << msg->data);
        return;
    }

    // 构建帧： HEAD + payload + TAIL  
    std::vector<uint8_t> packet;
    packet.insert(packet.end(), K4A_HEAD, K4A_HEAD + 2);
    packet.insert(packet.end(), payload.begin(), payload.end());
    packet.insert(packet.end(), K4A_TAIL, K4A_TAIL + 2);

    // 发送二进制数据（使用 const uint8_t*）
    try {
        ser.write(packet.data(), packet.size());
    } catch (const std::exception &e) {
        ROS_ERROR_STREAM("[serial_bridge] serial write failed: " << e.what());
        return;
    }

}
