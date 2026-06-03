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
#include <mutex>

class EPG50_Serial {
private:
    int serial_port;
    uint8_t slave_id               = 0x09;   // 默认从站ID
    const uint16_t WRITE_REG_START = 0x03E8; // 写寄存器首地址
    const uint16_t READ_REG_START  = 0x07D0; // 读寄存器首地址
    std::mutex serial_mutex;

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
        std::lock_guard<std::mutex> lock(serial_mutex);

        if (debug) {
            std::cout << "发送命令: ";
            for (auto byte: command) {
                std::cout << std::hex << static_cast<int>(byte) << " ";
            }
            std::cout << std::dec << std::endl;
        }
        response.clear();

        tcflush(serial_port, TCIFLUSH);

        if (write(serial_port, command.data(), command.size()) < 0) {
            if (debug)
                std::cerr << "写入失败: " << strerror(errno) << std::endl;
            return false;
        }

        auto start = std::chrono::steady_clock::now();
        uint8_t buffer[256];
        response.clear();

        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500)) {
            if (debug) {
                std::cout << "等待响应..." << std::endl;
            }

            ssize_t len = read(serial_port, buffer, sizeof(buffer));
            if (len > 0) {
                response.insert(response.end(), buffer, buffer + len);

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
                if (debug)
                    std::cerr << "读取错误: " << strerror(errno) << std::endl;
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        if (debug)
            std::cerr << "响应超时" << std::endl;
        return !response.empty();
    }

    bool is_response_complete(const std::vector<uint8_t>& response) {
        if (response.size() < 4)
            return false;

        uint8_t function_code = response[1];

        if (function_code == 0x03) {
            if (response.size() < 3)
                return false;
            uint8_t byte_count = response[2];
            return static_cast<int>(response.size())
                >= byte_count + 5;
        }
        else if (function_code == 0x10) {
            return response.size() >= 8;
        }

        return false;
    }

public:
    bool debug = false;
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

        cfsetospeed(&tty, B115200);
        cfsetispeed(&tty, B115200);
        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cflag |= CREAD | CLOCAL;

        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 5;

        if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
            close(serial_port);
            throw std::runtime_error("Error setting termios attributes");
        }
    }

    uint8_t get_slave_id() const {
        return slave_id;
    }

    void set_slave_id(uint8_t id) {
        slave_id = id;
    }

    bool rename_gripper(uint8_t current_id, uint8_t target_id) {
        std::vector<uint8_t> cmd = {
            current_id,
            0x10,
            0x13,       0x8D,
            0x00,       0x01,
            0x02,
            0x00,       target_id
        };
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
        std::vector<uint8_t> expected_response = { current_id, 0x10, 0x13, 0x8D, 0x00, 0x01 };
        crc = crc16(expected_response.data(), expected_response.size());
        expected_response.push_back(crc & 0xFF);
        expected_response.push_back(crc >> 8);
        if (response != expected_response) {
            std::cerr << "Error: Response does not match expected value." << std::endl;
            return false;
        }
        return true;
    }

    bool enable() {
        std::vector<uint8_t> cmd = {
            slave_id,
            0x10,
            0x03,     0xE8,
            0x00,     0x01,
            0x02,
            0x00,     0x01
        };
        uint16_t crc = crc16(cmd.data(), cmd.size());
        cmd.push_back(crc & 0xFF);
        cmd.push_back(crc >> 8);
        std::vector<uint8_t> response;
        bool success = send_command(cmd, response);
        std::vector<uint8_t> ret = {
            slave_id, 0x10, 0x03, 0xE8,
            0x00,     0x01
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

    bool enable_with_id(uint8_t id) {
        uint8_t old_id = slave_id;
        set_slave_id(id);
        bool result = enable();
        set_slave_id(old_id);
        return result;
    }

    bool disable() {
        std::vector<uint8_t> cmd = {
            slave_id,
            0x10,
            0x03,     0xE8,
            0x00,     0x01,
            0x02,
            0x00,     0x00
        };
        uint16_t crc = crc16(cmd.data(), cmd.size());
        cmd.push_back(crc & 0xFF);
        cmd.push_back(crc >> 8);
        std::vector<uint8_t> response;
        return send_command(cmd, response);
    }

    bool disable_with_id(uint8_t id) {
        uint8_t old_id = slave_id;
        set_slave_id(id);
        bool result = disable();
        set_slave_id(old_id);
        return result;
    }

    ~EPG50_Serial() {
        close(serial_port);
    }

    bool set_parameters(uint8_t position, uint8_t speed, uint8_t torque) {
        std::vector<uint8_t> cmd = { slave_id,
                                     0x10,
                                     static_cast<uint8_t>(WRITE_REG_START >> 8),
                                     static_cast<uint8_t>(WRITE_REG_START & 0xFF),
                                     0x00,
                                     0x03,
                                     0x06,
                                     static_cast<uint8_t>(0x00),
                                     static_cast<uint8_t>(0x09),
                                     static_cast<uint8_t>(position),
                                     static_cast<uint8_t>(0x00),
                                     static_cast<uint8_t>(speed),
                                     static_cast<uint8_t>(torque) };

        uint16_t crc = crc16(cmd.data(), cmd.size());
        cmd.push_back(crc & 0xFF);
        cmd.push_back(crc >> 8);

        std::vector<uint8_t> response;
        if (!send_command(cmd, response))
            return false;

        return (response.size() >= 8 && response[0] == slave_id && response[1] == 0x10);
    }

    bool set_parameters_with_id(uint8_t id, uint8_t position, uint8_t speed, uint8_t torque) {
        uint8_t old_id = slave_id;
        set_slave_id(id);
        bool result = set_parameters(position, speed, torque);
        set_slave_id(old_id);
        return result;
    }

    std::vector<uint16_t> read_status() {
        std::vector<uint8_t> cmd = {
            slave_id,
            0x03,
            static_cast<uint8_t>(READ_REG_START >> 8),
            static_cast<uint8_t>(READ_REG_START & 0xFF),
            0x00,
            0x04
        };

        uint16_t crc = crc16(cmd.data(), cmd.size());
        cmd.push_back(crc & 0xFF);
        cmd.push_back(crc >> 8);

        std::vector<uint8_t> response;
        if (!send_command(cmd, response) || response.size() < 13) {
            return {};
        }

        std::vector<uint16_t> status(8, 0);

        if (response.size() >= 3 + (8 * 1) + 2) {
            status[0] = response[4];
            status[1] = response[3];
            status[2] = response[6];
            status[3] = response[5];
            status[4] = response[8];
            status[5] = response[7];
            status[6] = response[10];
            status[7] = response[9];
        }

        return status;
    }

    std::vector<uint16_t> read_status_with_id(uint8_t id) {
        uint8_t old_id = slave_id;
        set_slave_id(id);
        auto result = read_status();
        set_slave_id(old_id);
        return result;
    }

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

    struct GripperStatusBits {
        bool gact;
        bool gmod;
        bool ggto;
        uint8_t gsta;
        uint8_t gobj;
    };

    GripperStatusBits parse_status_bits(uint8_t status_byte) {
        GripperStatusBits bits;
        bits.gact = (status_byte & 0x01) != 0;
        bits.gmod = (status_byte & 0x04) != 0;
        bits.ggto = (status_byte & 0x08) != 0;
        bits.gsta = (status_byte & 0x30) >> 4;
        bits.gobj = (status_byte & 0xC0) >> 6;
        return bits;
    }

    std::string get_object_status(const GripperStatusBits& bits) {
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
        return this->set_parameters(0x00, 0xFF, 0xFF);
    }

    bool full_open_with_id(uint8_t id) {
        uint8_t old_id = slave_id;
        set_slave_id(id);
        bool result = full_open();
        set_slave_id(old_id);
        return result;
    }
};

#endif // EPG50_SERIAL_H