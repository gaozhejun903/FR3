#ifndef EPG50_SERIAL_H
#define EPG50_SERIAL_H

#include <iostream>
#include <vector>
#include <cstdint>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <mutex> // 添加互斥锁头文件

class EPG50_Serial {
private:
    int serial_port;
    uint8_t slave_id               = 0x09;   // 默认从站ID
    const uint16_t WRITE_REG_START = 0x03E8; // 写寄存器首地址
    const uint16_t READ_REG_START  = 0x07D0; // 读寄存器首地址
    std::mutex serial_mutex;                 // 添加串口访问互斥锁

    // CRC16计算（Modbus RTU）
    uint16_t crc16(const uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; ++i) {
            crc ^= data[i];
            for (int j = 0; j < 8; ++j) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc >>= 1;
                }
            }
        }
        return crc;
    }

    // 发送Modbus命令并接收响应
    bool send_command(const std::vector<uint8_t>& command, std::vector<uint8_t>& response) {
        // 获取互斥锁，确保一次只有一个线程可以访问串口
        std::lock_guard<std::mutex> lock(serial_mutex);

        if (debug) {
            std::cout << "发送命令: ";
            for (auto byte: command) {
                std::cout << std::hex << static_cast<int>(byte) << " ";
            }
            std::cout << std::dec << std::endl;
        }
        // 清空响应缓冲区
        response.clear();

        // 先清空接收缓冲区中可能残留的数据
        tcflush(serial_port, TCIFLUSH);

        if (write(serial_port, command.data(), command.size()) < 0) {
            if (debug)
                std::cerr << "写入失败: " << strerror(errno) << std::endl;
            return false;
        }

        // 等待响应（示例超时500ms）
        auto start = std::chrono::steady_clock::now();
        uint8_t buffer[256];
        response.clear();

        // 可能需要多次读取才能获取完整响应
        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
            if (debug) {
                std::cout << "等待响应..." << std::endl;
            }

            ssize_t len = read(serial_port, buffer, sizeof(buffer));
            if (len > 0) {
                // 添加到响应缓冲区
                response.insert(response.end(), buffer, buffer + len);

                // 检查是否收到完整响应 (基于Modbus协议可以做更精确判断)
                if (is_response_complete(response)) {
                    if (debug) {
                        std::cout << "接收到完整响应" << std::endl;
                        std::cout << "响应数据: ";
                        for (auto byte: response) {
                            std::cout << std::hex << static_cast<int>(byte) << " ";
                        }
                        std::cout << std::dec << std::endl;
                    }
                    return true;
                }
            } else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                // 真正的错误，不是由于非阻塞引起的
                if (debug)
                    std::cerr << "读取错误: " << strerror(errno) << std::endl;
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (debug)
            std::cerr << "响应超时" << std::endl;
        return !response.empty(); // 如果收到任何数据，可以决定返回true
    }

    // 添加这个辅助函数来判断响应是否完整
    bool is_response_complete(const std::vector<uint8_t>& response) {
        // 基本实现：检查是否有足够的数据(根据Modbus格式)
        if (response.size() < 4)
            return false;

        // 根据Modbus协议格式判断
        uint8_t function_code = response[1];

        // 读寄存器响应
        if (function_code == 0x03) {
            if (response.size() < 3)
                return false;
            uint8_t byte_count = response[2];
            return static_cast<int>(response.size())
                >= byte_count + 5; // 地址(1) + 功能码(1) + 字节数(1) + 数据(n) + CRC(2)
        }
        // 写寄存器响应
        else if (function_code == 0x10) {
            return response.size() >= 8; // 标准写响应长度为8字节
        }

        return false;
    }

public:
    bool debug = false; // Debug flag
    EPG50_Serial(const std::string& port = "/dev/ttyACM0", const uint8_t slave_id = 0x09) {
        this->slave_id = slave_id;
        serial_port    = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (serial_port < 0) {
            throw std::runtime_error("Failed to open serial port");
        }

        struct termios tty;
        memset(&tty, 0, sizeof(tty));
        if (tcgetattr(serial_port, &tty) != 0) {
            close(serial_port);
            throw std::runtime_error("Error getting termios attributes");
        }

        // 配置串口参数（115200 8N1）
        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag &= ~PARENB; // 无奇偶校验
        tty.c_cflag &= ~CSTOPB; // 1位停止位
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;            // 8位数据位
        tty.c_cflag &= ~CRTSCTS;       // 无硬件流控
        tty.c_cflag |= CREAD | CLOCAL; // 开启接收功能，忽略调制解调器线路状态

        // 设置为原始模式
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
        tty.c_oflag &= ~OPOST;

        // 设置读取超时和最小字符数
        tty.c_cc[VMIN]  = 0; // 不需要最小字符数
        tty.c_cc[VTIME] = 5; // 0.5秒超时 (单位是十分之一秒)

        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            close(serial_port);
            throw std::runtime_error("Error setting termios attributes");
        }
    }

    // 获取当前从站ID
    uint8_t get_slave_id() const {
        return slave_id;
    }

    // 设置从站ID
    void set_slave_id(uint8_t id) {
        slave_id = id;
        if (debug) {
            std::cout << "从站ID已设置为: 0x" << std::hex << static_cast<int>(slave_id) << std::dec << std::endl;
        }
    }

    bool rename_gripper(uint8_t current_id, uint8_t target_id) {
        std::vector<uint8_t> cmd = {
            current_id,
            0x10,                 // FC16
            0x13,       0x8D,     // 寄存器地址
            0x00,       0x01,     // 寄存器数量
            0x02,                 // 数据字节数
            0x00,       target_id // 数据
        };
        // 添加CRC校验
        uint16_t crc = crc16(cmd.data(), cmd.size());
        cmd.push_back(crc & 0xFF);
        cmd.push_back(crc >> 8);
        std::vector<uint8_t> response;
        if (!send_command(cmd, response)) {
            std::cerr << "Error: Failed to send rename command." << std::endl;
            return false;
        } else if (response.size() < 8) {
            std::cerr << "Error: Response size is less than 8 bytes." << std::endl;
            return false;
        }
        std::vector<uint8_t> expected_response = { current_id, 0x10, 0x13, 0x8D, // 寄存器地址
                                                   0x00,       0x01 };
        crc                                    = crc16(expected_response.data(), expected_response.size());
        expected_response.push_back(crc & 0xFF);
        expected_response.push_back(crc >> 8);
        if (response != expected_response) {
            std::cerr << "Error: Response does not match expected value." << std::endl;
            return false;
        }
        return true;
    }

    bool enable() {
        // 09 10 03 E8 00 01 02 00 01 24 78
        std::vector<uint8_t> cmd = {
            slave_id,
            0x10,           // FC16
            0x03,     0xE8, // 寄存器地址
            0x00,     0x01, // 寄存器数量
            0x02,           // 数据字节数
            0x00,     0x01  // 数据
        };
        uint16_t crc = crc16(cmd.data(), cmd.size());
        cmd.push_back(crc & 0xFF);
        cmd.push_back(crc >> 8);
        std::vector<uint8_t> response;
        bool success             = send_command(cmd, response);
        std::vector<uint8_t> ret = {
            slave_id, 0x10, 0x03, 0xE8, // 寄存器地址
            0x00,     0x01              // 寄存器数量
        };
        crc = crc16(ret.data(), ret.size());
        ret.push_back(crc & 0xFF);
        ret.push_back(crc >> 8);
        if (response.size() < 8) {
            std::cerr << "Error: Response size is less than 8 bytes." << std::endl;
            return false;
        }
        if (response != ret) {
            std::cerr << "Error: Response does not match expected value." << std::endl;
            return false;
        }
        return success;
    }

    // 指定从站ID的enable方法
    bool enable_with_id(uint8_t id) {
        uint8_t old_id = slave_id;
        set_slave_id(id);
        bool result = enable();
        set_slave_id(old_id); // 恢复原来的从站ID
        return result;
    }

    bool disable() {
        std::vector<uint8_t> cmd = {
            slave_id,
            0x10,           // FC16
            0x03,     0xE8, // 寄存器地址
            0x00,     0x01, // 寄存器数量
            0x02,           // 数据字节数
            0x00,     0x00  // 数据
        };
        uint16_t crc = crc16(cmd.data(), cmd.size());
        cmd.push_back(crc & 0xFF);
        cmd.push_back(crc >> 8);
        std::vector<uint8_t> response;
        return send_command(cmd, response);
    }

    // 指定从站ID的disable方法
    bool disable_with_id(uint8_t id) {
        uint8_t old_id = slave_id;
        set_slave_id(id);
        bool result = disable();
        set_slave_id(old_id); // 恢复原来的从站ID
        return result;
    }

    ~EPG50_Serial() {
        close(serial_port);
    }

    // 设置夹爪参数（功能码FC16）
    bool set_parameters(uint8_t position, uint8_t speed, uint8_t torque) {
        std::vector<uint8_t> cmd = { slave_id,
                                     0x10, // FC16
                                     static_cast<uint8_t>(WRITE_REG_START >> 8),
                                     static_cast<uint8_t>(WRITE_REG_START & 0xFF),
                                     0x00,
                                     0x03, // 设置3个寄存器（位置、速度、力矩）
                                     0x06, // 数据字节数
                                     static_cast<uint8_t>(0x00),
                                     static_cast<uint8_t>(0x09),
                                     static_cast<uint8_t>(position),
                                     static_cast<uint8_t>(0x00),
                                     static_cast<uint8_t>(speed),
                                     static_cast<uint8_t>(torque) };

        // 添加CRC校验
        uint16_t crc = crc16(cmd.data(), cmd.size());
        cmd.push_back(crc & 0xFF);
        cmd.push_back(crc >> 8);

        std::vector<uint8_t> response;
        if (!send_command(cmd, response))
            return false;

        // 验证响应（示例简单验证）
        return (response.size() >= 8 && response[0] == slave_id && response[1] == 0x10);
    }

    // 指定从站ID的set_parameters方法
    bool set_parameters_with_id(uint8_t id, uint8_t position, uint8_t speed, uint8_t torque) {
        uint8_t old_id = slave_id;
        set_slave_id(id);
        bool result = set_parameters(position, speed, torque);
        set_slave_id(old_id); // 恢复原来的从站ID
        return result;
    }

    // 读取夹爪状态（功能码FC03）
    std::vector<uint16_t> read_status() {
        std::vector<uint8_t> cmd = {
            slave_id,
            0x03, // FC03
            static_cast<uint8_t>(READ_REG_START >> 8),
            static_cast<uint8_t>(READ_REG_START & 0xFF),
            0x00,
            0x04 // 读取4个寄存器
        };

        uint16_t crc = crc16(cmd.data(), cmd.size());
        cmd.push_back(crc & 0xFF);
        cmd.push_back(crc >> 8);

        std::vector<uint8_t> response;
        if (!send_command(cmd, response) || response.size() < 13) {
            return {};
        }

        // 解析返回数据 - 根据更新的寄存器映射
        std::vector<uint16_t> status(8, 0); // 初始化8个元素的数组，用于存储8个状态值

        // 检查响应长度是否足够
        if (response.size() >= 3 + (8 * 1) + 2) { // 头部3字节 + 数据字节 + CRC 2字节
            // 0x07D0: 低字节-电动夹爪状态寄存器，高字节-留空
            status[0] = response[4]; // 低字节 - 夹爪状态
            status[1] = response[3]; // 高字节 - 留空

            // 0x07D1: 低字节-故障错误状态寄存器，高字节-位置状态寄存器
            status[2] = response[6]; // 低字节 - 错误状态
            status[3] = response[5]; // 高字节 - 位置状态

            // 0x07D2: 低字节-速度状态寄存器，高字节-力状态(即时电流)寄存器
            status[4] = response[8]; // 低字节 - 速度状态
            status[5] = response[7]; // 高字节 - 力状态(即时电流)

            // 0x07D3: 低字节-母线电压寄存器，高字节-环境温度寄存器
            status[6] = response[10]; // 低字节 - 母线电压
            status[7] = response[9];  // 高字节 - 环境温度
        }

        return status; // 返回所有状态值
    }

    // 指定从站ID的read_status方法
    std::vector<uint16_t> read_status_with_id(uint8_t id) {
        uint8_t old_id = slave_id;
        set_slave_id(id);
        auto result = read_status();
        set_slave_id(old_id); // 恢复原来的从站ID
        return result;
    }

    // 故障检测（基于状态字节）
    std::string check_errors(uint8_t error_status) {
        if (error_status & 0x01)
            return "通讯异常";
        if (error_status & 0x02)
            return "控制指令错误";
        if (error_status & 0x04)
            return "过温故障";
        if (error_status & 0x08)
            return "电压异常";
        if (error_status & 0x10)
            return "过流故障";
        return "正常";
    }

    // 新增：解析状态位
    struct GripperStatusBits {
        bool gact;    // bit0: 电动夹爪的使能状态
        bool gmod;    // bit2: 工作模式
        bool ggto;    // bit3: 电动夹爪动作状态
        uint8_t gsta; // bit4-5: 电动夹爪当前状态
        uint8_t gobj; // bit6-7: 目标检测状态
    };

    // 从状态字节中解析出各个位的含义
    GripperStatusBits parse_status_bits(uint8_t status_byte) {
        GripperStatusBits bits;
        bits.gact = (status_byte & 0x01) != 0; // bit 0
        bits.gmod = (status_byte & 0x04) != 0; // bit 2
        bits.ggto = (status_byte & 0x08) != 0; // bit 3
        bits.gsta = (status_byte & 0x30) >> 4; // bits 4-5
        bits.gobj = (status_byte & 0xC0) >> 6; // bits 6-7
        return bits;
    }

    // 获取物体抓取状态描述
    std::string get_object_status(const GripperStatusBits& bits) {
        // if (!bits.ggto) {
        //     return "夹爪未运动，物体检测状态无效";
        // }

        switch (bits.gobj) {
            case 0:
                return "手指正向指定位置移动";
            case 1:
                return "手指张开过程中接触到物体并停止";
            case 2:
                return "手指闭合过程中接触到物体并停止";
            case 3:
                return "手指已到达指定位置，但未检测到物体或物体已脱落";
            default:
                return "未知状态";
        }
    }

    // 获取夹爪工作状态描述
    std::string get_gripper_status(const GripperStatusBits& bits) {
        std::string status = bits.gact ? "已使能" : "未使能/复位中";
        status += ", ";

        status += bits.gmod ? "无输入参数控制模式" : "参数控制模式";
        status += ", ";

        status += bits.ggto ? "前往目标位置" : "停止/执行激活或巡检";
        status += ", ";

        switch (bits.gsta) {
            case 0:
                status += "复位或巡检状态";
                break;
            case 1:
                status += "正在激活";
                break;
            case 2:
                status += "未使用状态";
                break;
            case 3:
                status += "激活完成";
                break;
            default:
                status += "未知状态";
        }

        return status;
    }

    bool full_open() {
        // 完全打开夹爪
        return this->set_parameters(0x00, 0xFF, 0xFF);
    }

    // 指定从站ID的full_open方法
    bool full_open_with_id(uint8_t id) {
        uint8_t old_id = slave_id;
        set_slave_id(id);
        bool result = full_open();
        set_slave_id(old_id); // 恢复原来的从站ID
        return result;
    }
};

#endif // EPG50_SERIAL_H