#ifndef FAIRINO_HARDWARE_INTERFACE_HPP
#define FAIRINO_HARDWARE_INTERFACE_HPP

#include <array>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"

namespace robo_ctrl
{

class FairinoHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(FairinoHardwareInterface)

  FairinoHardwareInterface() = default;
  ~FairinoHardwareInterface() override;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  // libfairino robot object (forward declaration, included in .cpp)
  void * robot_ = nullptr;

  // Parameters
  std::string robot_ip_ = "192.168.58.2";

  // ServoJ parameters
  float servo_acc_ = 50.0f;
  float servo_vel_ = 30.0f;
  float servo_cmd_time_ = 0.008f;
  float servo_filter_time_ = 0.003f;
  float servo_gain_ = 0.0f;

  // State
  bool connected_ = false;
  bool servo_started_ = false;
  size_t num_joints_ = 6;

  // Joint buffers (radians)
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  // ---------------------------------------------------------------
  //  Background communication thread
  //  ServoJ / GetActualJointPosDegree 均在该线程中执行,
  //  避免阻塞 controller_manager 的 RT 控制循环。
  // ---------------------------------------------------------------
  std::thread comm_thread_;
  std::mutex comm_mutex_;
  std::atomic<bool> comm_running_{false};

  // 在 RT 线程 (read/write) 与后台通信线程之间共享的缓冲区
  std::array<double, 6> shared_cmd_;   // rad, RT→后台
  std::array<double, 6> shared_pos_;   // rad, 后台→RT

  void commThreadLoop();
};

}  // namespace robo_ctrl

#endif  // FAIRINO_HARDWARE_INTERFACE_HPP
