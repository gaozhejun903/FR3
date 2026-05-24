# 夹爪状态消息
uint8 slave_id       # 从站ID
uint16 status        # 状态

# 状态位详细说明 (0x07D0 低字节)
bool gact            # bit0: 电动夹爪的使能状态 (0:复位中, 1:使能状态)
bool gmod            # bit2: 工作模式 (0:参数控制模式, 1:无输入参数控制模式)
bool ggto            # bit3: 电动夹爪动作状态 (0:停止, 1:前往目标位置)
uint8 gsta           # bit4-5: 电动夹爪当前状态 (0:复位/巡检状态, 1:正在激活, 3:激活完成)
uint8 gobj           # bit6-7: 目标检测状态 (0:移动中, 1:内撑模式已接触物体, 2:外夹模式已接触物体, 3:已到达位置但无物体)

uint16 mode          # 模式
uint16 error         # 错误代码
uint16 position      # 当前位置
uint16 speed         # 当前速度
uint16 force         # 当前力
uint16 voltage       # 母线电压
uint16 temperature   # 环境温度
string error_message # 错误信息
string object_status # 物体抓取状态描述