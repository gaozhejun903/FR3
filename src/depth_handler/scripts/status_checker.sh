#!/bin/bash
# AI-Deep: 系统状态检查脚本
# 延迟15秒等待所有节点启动，然后检查各组件状态

sleep 15

# ── 颜色定义 ──
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BOLD='\033[1m'
NC='\033[0m' # No Color
CHECK_MARK="${GREEN}✓${NC}"
CROSS_MARK="${RED}✗${NC}"

echo ""
echo -e "${BOLD}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║              系统启动状态检查                            ║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════════════════════════╝${NC}"
echo ""

# ── 获取当前运行节点和话题列表 ──
NODE_LIST=$(ros2 node list 2>/dev/null)
TOPIC_LIST=$(ros2 topic list 2>/dev/null)

if [ -z "$NODE_LIST" ]; then
    echo -e "${RED}错误: 无法获取ROS2节点列表，请检查ROS2环境是否正常${NC}"
    exit 1
fi

# ── 状态统计 ──
PASS=0
FAIL=0

check_node() {
    local node_name=$1
    if echo "$NODE_LIST" | grep -q "$node_name"; then
        echo -e "  ${CHECK_MARK} 节点 ${BOLD}$node_name${NC} 运行中"
        return 0
    else
        echo -e "  ${CROSS_MARK} 节点 ${BOLD}$node_name${NC} ${RED}未找到${NC}"
        return 1
    fi
}

check_topic() {
    local topic_name=$1
    if echo "$TOPIC_LIST" | grep -q "$topic_name"; then
        echo -e "  ${CHECK_MARK} 话题 ${BOLD}$topic_name${NC} 存在"
        return 0
    else
        echo -e "  ${CROSS_MARK} 话题 ${BOLD}$topic_name${NC} ${RED}未找到${NC}"
        return 1
    fi
}

print_section() {
    echo ""
    echo -e "${BOLD}━━━ $1 ━━━${NC}"
}

# ═══════════════════════════════════════════════════════════
# 1) 左臂 (L)
# ═══════════════════════════════════════════════════════════
print_section "左臂机械臂 (192.168.58.2)"
LEFT_OK=true
check_node "Lrobo_ctrl" || LEFT_OK=false
check_node "Lhigh_level" || LEFT_OK=false
check_topic "/L/joint_states" || LEFT_OK=false
if $LEFT_OK; then
    echo -e "  → 左臂状态: ${GREEN}✅ 启动成功${NC}"
    ((PASS++))
else
    echo -e "  → 左臂状态: ${RED}❌ 启动失败${NC}"
    ((FAIL++))
fi

# ═══════════════════════════════════════════════════════════
# 2) 右臂 (R)
# ═══════════════════════════════════════════════════════════
print_section "右臂机械臂 (192.168.58.3)"
RIGHT_OK=true
check_node "Rrobo_ctrl" || RIGHT_OK=false
check_node "Rhigh_level" || RIGHT_OK=false
check_topic "/R/joint_states" || RIGHT_OK=false
if $RIGHT_OK; then
    echo -e "  → 右臂状态: ${GREEN}✅ 启动成功${NC}"
    ((PASS++))
else
    echo -e "  → 右臂状态: ${RED}❌ 启动失败${NC}"
    ((FAIL++))
fi

# ═══════════════════════════════════════════════════════════
# 3) 左夹爪
# ═══════════════════════════════════════════════════════════
print_section "左夹爪"
LEFT_GRIP_OK=true
check_node "L_gripper_node" || LEFT_GRIP_OK=false
check_topic "/L_gripper_node/status_stream" || LEFT_GRIP_OK=false
if $LEFT_GRIP_OK; then
    echo -e "  → 左夹爪状态: ${GREEN}✅ 启动成功${NC}"
    ((PASS++))
else
    echo -e "  → 左夹爪状态: ${RED}❌ 启动失败${NC}"
    ((FAIL++))
fi

# ═══════════════════════════════════════════════════════════
# 4) 右夹爪
# ═══════════════════════════════════════════════════════════
print_section "右夹爪"
RIGHT_GRIP_OK=true
check_node "R_gripper_node" || RIGHT_GRIP_OK=false
check_topic "/R_gripper_node/status_stream" || RIGHT_GRIP_OK=false
if $RIGHT_GRIP_OK; then
    echo -e "  → 右夹爪状态: ${GREEN}✅ 启动成功${NC}"
    ((PASS++))
else
    echo -e "  → 右夹爪状态: ${RED}❌ 启动失败${NC}"
    ((FAIL++))
fi

# ═══════════════════════════════════════════════════════════
# 5) 目标检测
# ═══════════════════════════════════════════════════════════
print_section "目标检测 (YOLO)"
DETECT_OK=true
check_node "detector_node" || DETECT_OK=false
check_topic "/detector/detections" || DETECT_OK=false
if $DETECT_OK; then
    echo -e "  → 目标检测状态: ${GREEN}✅ 启动成功${NC}"
    ((PASS++))
else
    echo -e "  → 目标检测状态: ${RED}❌ 启动失败${NC}"
    ((FAIL++))
fi

# ═══════════════════════════════════════════════════════════
# 6) 深度处理
# ═══════════════════════════════════════════════════════════
print_section "深度处理"
DEPTH_OK=true
check_node "depth_handler_node" || DEPTH_OK=false
check_topic "/depth_handler/bbox3d" || DEPTH_OK=false
check_topic "/depth_handler/pointcloud" || DEPTH_OK=false
if $DEPTH_OK; then
    echo -e "  → 深度处理状态: ${GREEN}✅ 启动成功${NC}"
    ((PASS++))
else
    echo -e "  → 深度处理状态: ${RED}❌ 启动失败${NC}"
    ((FAIL++))
fi

# ═══════════════════════════════════════════════════════════
# 7) TF发布
# ═══════════════════════════════════════════════════════════
print_section "TF发布"
TF_OK=true
check_node "camera_static_tf_publisher" || TF_OK=false
check_node "gripper_static_tf_publisher" || TF_OK=false
# AI-Deep: 检查虚假夹爪TF发布节点
check_node "Lfake_gripper_tf_publisher" || TF_OK=false
check_node "Rfake_gripper_tf_publisher" || TF_OK=false
if $TF_OK; then
    echo -e "  → TF发布状态: ${GREEN}✅ 启动成功${NC}"
    ((PASS++))
else
    echo -e "  → TF发布状态: ${RED}❌ 启动失败${NC}"
    ((FAIL++))
fi

# ═══════════════════════════════════════════════════════════
# 8) 相机 (额外检查)
# ═══════════════════════════════════════════════════════════
print_section "相机 (Orbbec Gemini 330)"
CAM_OK=true
check_node "ob_camera_node" || check_node "camera_container" || CAM_OK=false
check_topic "/camera/color/image_raw" || CAM_OK=false
check_topic "/camera/depth/image_raw" || CAM_OK=false
if $CAM_OK; then
    echo -e "  → 相机状态: ${GREEN}✅ 启动成功${NC}"
    ((PASS++))
else
    echo -e "  → 相机状态: ${RED}❌ 启动失败${NC}"
    ((FAIL++))
fi

# ═══════════════════════════════════════════════════════════
# 汇总
# ═══════════════════════════════════════════════════════════
TOTAL=$((PASS + FAIL))
echo ""
echo -e "${BOLD}╔══════════════════════════════════════════════════════════╗${NC}"
echo -e "${BOLD}║              最终检查结果                                ║${NC}"
echo -e "${BOLD}╚══════════════════════════════════════════════════════════╝${NC}"
echo -e "  检查项目: ${BOLD}${TOTAL}${NC} 项"
echo -e "  通过: ${GREEN}${BOLD}${PASS}${NC} 项"
if [ $FAIL -gt 0 ]; then
    echo -e "  失败: ${RED}${BOLD}${FAIL}${NC} 项"
fi
echo ""

if [ $FAIL -eq 0 ]; then
    echo -e "${GREEN}${BOLD}  ✅ 所有组件启动成功!${NC}"
else
    echo -e "${RED}${BOLD}  ❌ 有 ${FAIL} 个组件启动失败，请检查日志${NC}"
fi
echo ""
