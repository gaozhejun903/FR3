#include "robo_ctrl/fairino_hardware_interface.hpp"

#include <cmath>
#include <string>
#include <thread>
#include <chrono>
#include <mutex>

#include "rclcpp/logging.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

// libfairino 头文件
#include "libfairino/robot.h"
#include "libfairino/robot_error.h"

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("FairinoHardwareInterface");
}
}  // anonymous namespace

namespace robo_ctrl
{

FairinoHardwareInterface::~FairinoHardwareInterface()
{
  // Stop background thread first (safety net if on_deactivate wasn't called)
  if (comm_running_) {
    comm_running_ = false;
    if (comm_thread_.joinable()) {
      comm_thread_.join();
    }
  }

  if (servo_started_) {
    FRRobot * r = static_cast<FRRobot *>(robot_);
    r->ServoMoveEnd();
    servo_started_ = false;
  }
  if (connected_) {
    FRRobot * r = static_cast<FRRobot *>(robot_);
    r->CloseRPC();
    connected_ = false;
  }
  if (robot_) {
    delete static_cast<FRRobot *>(robot_);
    robot_ = nullptr;
  }
}

// ----------------------------------------------------------------
// on_init — 读取 URDF 中配置的参数
// ----------------------------------------------------------------
hardware_interface::SystemInterface::CallbackReturn
FairinoHardwareInterface::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  num_joints_ = info_.joints.size();
  hw_commands_.resize(num_joints_, 0.0);
  hw_positions_.resize(num_joints_, 0.0);
  hw_velocities_.resize(num_joints_, 0.0);

  // URDF 中可选的硬件参数
  auto it = info_.hardware_parameters.find("robot_ip");
  if (it != info_.hardware_parameters.end()) {
    robot_ip_ = it->second;
  }

  it = info_.hardware_parameters.find("servo_acc");
  if (it != info_.hardware_parameters.end()) {
    servo_acc_ = std::stof(it->second);
  }

  it = info_.hardware_parameters.find("servo_vel");
  if (it != info_.hardware_parameters.end()) {
    servo_vel_ = std::stof(it->second);
  }

  it = info_.hardware_parameters.find("servo_cmd_time");
  if (it != info_.hardware_parameters.end()) {
    servo_cmd_time_ = std::stof(it->second);
  }

  it = info_.hardware_parameters.find("servo_filter_time");
  if (it != info_.hardware_parameters.end()) {
    servo_filter_time_ = std::stof(it->second);
  }

  it = info_.hardware_parameters.find("servo_gain");
  if (it != info_.hardware_parameters.end()) {
    servo_gain_ = std::stof(it->second);
  }

  // 创建 FRRobot 实例
  robot_ = new FRRobot();

  return CallbackReturn::SUCCESS;
}

// ----------------------------------------------------------------
// 导出状态 / 命令接口
// ----------------------------------------------------------------
std::vector<hardware_interface::StateInterface>
FairinoHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < num_joints_; ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FairinoHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < num_joints_; ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }
  return command_interfaces;
}

// ----------------------------------------------------------------
// on_activate — 连接真实机器人 + 启动 Servo 模式
// ----------------------------------------------------------------
hardware_interface::SystemInterface::CallbackReturn
FairinoHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  FRRobot * r = static_cast<FRRobot *>(robot_);

  // 1. RPC 连接
  int ret = r->RPC(robot_ip_.c_str());
  if (ret != 0) {
    RCLCPP_FATAL(
      logger(),
      "连接机器人失败 (IP=%s), 错误码=%d", robot_ip_.c_str(), ret);
    return CallbackReturn::ERROR;
  }
  connected_ = true;
  RCLCPP_INFO(
    logger(),
    "已连接到机器人 %s", robot_ip_.c_str());

  // 2. 读取当前关节位置作为初始值
  JointPos jpos;
  bool got_initial_positions = false;
  ret = r->GetActualJointPosDegree(0, &jpos);
  if (ret == 0) {
    // 检查是否全零 (可能机器人未上电)
    bool all_zero = true;
    for (size_t i = 0; i < num_joints_ && i < 6; ++i) {
      if (std::fabs(jpos.jPos[i]) > 0.001) {
        all_zero = false;
        break;
      }
    }
    if (!all_zero) {
      for (size_t i = 0; i < num_joints_ && i < 6; ++i) {
        double rad = jpos.jPos[i] * M_PI / 180.0;
        hw_positions_[i] = rad;
        hw_commands_[i] = rad;
      }
      RCLCPP_INFO(
        logger(),
        "初始关节位置 (来自机器人): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] deg",
        jpos.jPos[0], jpos.jPos[1], jpos.jPos[2],
        jpos.jPos[3], jpos.jPos[4], jpos.jPos[5]);
      got_initial_positions = true;
    } else {
      RCLCPP_WARN(
        logger(),
        "读取到全零关节位置 — 机器人可能未上电");
    }
  } else {
    RCLCPP_WARN(
      logger(),
      "获取初始关节位置失败 (%d)", ret);
  }

  if (!got_initial_positions) {
    // 回退到 URDF 中配置的 initial_value
    for (size_t i = 0; i < num_joints_; ++i) {
      double init_val = 0.0;
      if (!info_.joints[i].state_interfaces.empty()) {
        init_val = std::stod(info_.joints[i].state_interfaces[0].initial_value);
      }
      hw_positions_[i] = init_val;
      hw_commands_[i] = init_val;
    }
    RCLCPP_INFO(
      logger(),
      "初始关节位置 (来自 URDF), 使用零位");
  }

  // 3. 检查并清除错误, 确保机器人处于可 Servo 状态
  {
    int main_code = 0, sub_code = 0;
    ret = r->GetRobotErrorCode(&main_code, &sub_code);
    if (ret == 0) {
      if (main_code != 0) {
        RCLCPP_WARN(logger(), "检测到机器人错误: main=%d sub=%d, 尝试清除", main_code, sub_code);
        ret = r->ResetAllError();
        if (ret != 0) {
          RCLCPP_ERROR(logger(), "ResetAllError 失败, 错误码=%d", ret);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      } else {
        RCLCPP_INFO(logger(), "机器人无活动错误");
      }
    } else {
      RCLCPP_WARN(logger(), "获取错误码失败 (%d)", ret);
    }
  }

  // 4. 切换到自动模式 (ServoJ 需要自动模式)
  {
    RCLCPP_INFO(logger(), "设置机器人为自动模式...");
    ret = r->Mode(0);
    if (ret != 0) {
      RCLCPP_WARN(logger(), "Mode(0) 返回错误码=%d (可忽略)", ret);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }

  // 5. 使能机器人
  {
    int enable_retries = 3;
    bool enabled = false;
    for (int i = 0; i < enable_retries; ++i) {
      RCLCPP_INFO(logger(), "尝试使能机器人 (第 %d 次)...", i + 1);
      ret = r->RobotEnable(1);
      if (ret != 0) {
        RCLCPP_WARN(logger(), "RobotEnable 返回错误码=%d", ret);
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));

      // 使能后重新读取关节位置, 验证是否成功
      JointPos check_jpos;
      ret = r->GetActualJointPosDegree(0, &check_jpos);
      if (ret == 0) {
        bool still_zero = true;
        for (size_t j = 0; j < num_joints_ && j < 6; ++j) {
          if (std::fabs(check_jpos.jPos[j]) > 0.001) {
            still_zero = false;
            break;
          }
        }
        if (!still_zero) {
          RCLCPP_INFO(logger(), "机器人使能成功, 关节位置: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f] deg",
            check_jpos.jPos[0], check_jpos.jPos[1], check_jpos.jPos[2],
            check_jpos.jPos[3], check_jpos.jPos[4], check_jpos.jPos[5]);
          enabled = true;
          break;
        }
        RCLCPP_WARN(logger(), "使能后关节位置仍为零, 等待重试...");
      }
    }

    // 如果使能不成功, 也要继续尝试 ServoMoveStart, 因为某些情况下
    // RobotEnable 不改变关节读取值但确实有效
    if (!enabled) {
      RCLCPP_WARN(logger(),
        "机器人驱动器可能未上电。请确认示教器上机器人处于使能/运行状态。"
        "将继续尝试启动 Servo 模式...");
    }
  }

  // 5. 清理残留 Servo 状态
  r->ServoMoveEnd();
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // 6. 启动 Servo 模式 (带重试)
  {
    int servo_retries = 3;
    bool servo_started = false;
    for (int i = 0; i < servo_retries; ++i) {
      // 每次重试前再做一次清理
      if (i > 0) {
        r->ServoMoveEnd();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
      }

      ret = r->ServoMoveStart();
      if (ret == 0) {
        servo_started = true;
        break;
      }
      RCLCPP_WARN(logger(), "ServoMoveStart 尝试 %d/%d 失败, 错误码=%d",
        i + 1, servo_retries, ret);
    }

    if (!servo_started) {
      RCLCPP_FATAL(logger(),
        "ServoMoveStart 失败 (错误码=%d)。请确认: "
        "1) 示教器上机器人已使能(servo on) "
        "2) 无活动错误报警 "
        "3) 急停按钮已释放", ret);
      r->CloseRPC();
      connected_ = false;
      return CallbackReturn::ERROR;
    }
  }

  servo_started_ = true;
  RCLCPP_INFO(
    logger(),
    "Servo 模式已启动 (acc=%.1f vel=%.1f cmdT=%.4f)",
    servo_acc_, servo_vel_, servo_cmd_time_);

  // ServoMoveStart 后等待片刻, 确保机器人就绪
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  RCLCPP_INFO(
    logger(),
    "硬件激活完成 (acc=%.1f vel=%.1f cmdT=%.4f)",
    servo_acc_, servo_vel_, servo_cmd_time_);

  return CallbackReturn::SUCCESS;
}

// ----------------------------------------------------------------
// on_deactivate — 退出 Servo 模式 + 断开连接
// ----------------------------------------------------------------
hardware_interface::SystemInterface::CallbackReturn
FairinoHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  FRRobot * r = static_cast<FRRobot *>(robot_);

  // 1. 停止后台通信线程 (必须先停止, 避免线程还在调用 ServoJ 时执行 ServoMoveEnd)
  if (comm_running_) {
    comm_running_ = false;
    if (comm_thread_.joinable()) {
      comm_thread_.join();
    }
  }

  if (servo_started_) {
    r->ServoMoveEnd();
    servo_started_ = false;
    RCLCPP_INFO(
      logger(), "Servo 模式已退出");
  }

  if (connected_) {
    r->CloseRPC();
    connected_ = false;
    RCLCPP_INFO(
      logger(), "RPC 已断开");
  }

  return CallbackReturn::SUCCESS;
}

// ----------------------------------------------------------------
// read — 从机器人读取当前关节位置 (deg → rad)
// ----------------------------------------------------------------
hardware_interface::return_type
FairinoHardwareInterface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!connected_) {
    return hardware_interface::return_type::ERROR;
  }

  FRRobot * r = static_cast<FRRobot *>(robot_);

  JointPos jpos;
  int ret = r->GetActualJointPosDegree(0, &jpos);
  if (ret != 0) {
    // 通信错误时不返回 ERROR (会导致控制器无法加载),
    // 保持上次读取的位置不变, 仅记录日志
    RCLCPP_DEBUG(
      logger(),
      "读取关节位置失败, 错误码=%d", ret);
    return hardware_interface::return_type::OK;
  }

  for (size_t i = 0; i < num_joints_ && i < 6; ++i) {
    hw_positions_[i] = jpos.jPos[i] * M_PI / 180.0;
  }

  return hardware_interface::return_type::OK;
}

// ----------------------------------------------------------------
// write — 将命令位置通过 ServoJ 发送到机器人 (rad → deg)
// ----------------------------------------------------------------
hardware_interface::return_type
FairinoHardwareInterface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!connected_ || !servo_started_) {
    return hardware_interface::return_type::ERROR;
  }

  FRRobot * r = static_cast<FRRobot *>(robot_);

  // rad → deg
  JointPos jpos;
  for (size_t i = 0; i < num_joints_ && i < 6; ++i) {
    jpos.jPos[i] = hw_commands_[i] * 180.0 / M_PI;
  }

  // 首次失败时打印详细日志
  {
    static std::once_flag flag;
    std::call_once(flag, [&]() {
      RCLCPP_INFO(
        logger(),
        "首次 ServoJ 命令: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f] deg",
        jpos.jPos[0], jpos.jPos[1], jpos.jPos[2],
        jpos.jPos[3], jpos.jPos[4], jpos.jPos[5]);
    });
  }

  int ret = 0;
  try {
    ret = r->ServoJ(
      &jpos,
      servo_acc_,
      servo_vel_,
      servo_cmd_time_,
      servo_filter_time_,
      servo_gain_);
  } catch (...) {
    RCLCPP_DEBUG(
      logger(),
      "ServoJ 异常 (可忽略)");
  }

  if (ret != 0) {
    RCLCPP_WARN(
      logger(),
      "ServoJ 发送失败, 错误码=%d", ret);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace robo_ctrl

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robo_ctrl::FairinoHardwareInterface, hardware_interface::SystemInterface)
