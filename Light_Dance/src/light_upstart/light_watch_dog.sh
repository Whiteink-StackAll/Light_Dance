#!/bin/bash
# light_watch_dog.sh

TIMEOUT=10  # 设定超时时间为10秒
NAMESPACE=""  # 命名空间 例如 "/infantry_3" 注意要有"/"
NODE_NAMES=("light_solver" "light_serial" "light_camera")  # 监控的节点名称列表
USER="$(whoami)"  # 获取当前用户名
HOME_DIR=$(eval echo ~$USER)  # 用户主目录
WORKING_DIR="$HOME_DIR/LightDance3"  # 代码工作目录
LAUNCH_FILE="light_bringup bringup.launch.py"  # launch文件路径
OUTPUT_FILE="$WORKING_DIR/screen.output"  # 终端输出记录文件

rmw="rmw_fastrtps_cpp"  # RMW实现选择
export RMW_IMPLEMENTATION="$rmw"  # 设置RMW环境变量

export ROS_HOSTNAME=$(hostname)
export ROS_HOME=${ROS_HOME:=$HOME_DIR/.ros}
export ROS_LOG_DIR="/tmp"

# 加载ROS环境
source /opt/ros/humble/setup.bash
source "$WORKING_DIR/install/setup.bash" || {
  echo "ERROR: 无法加载工作空间，检查路径是否正确: $WORKING_DIR"
  exit 1
}

# RMW配置处理
rmw_config=""
if [[ "$rmw" == "rmw_fastrtps_cpp" && -n "$rmw_config" ]]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$rmw_config"
elif [[ "$rmw" == "rmw_cyclonedds_cpp" && -n "$rmw_config" ]]; then
  export CYCLONEDDS_URI="$rmw_config"
fi

# 启动所有节点
function bringup() {
  echo "[$(date)] 启动所有节点..."
  source /opt/ros/humble/setup.bash
  source "$WORKING_DIR/install/setup.bash"
  
  # 清理残留进程
  pkill -f "ros2 launch $LAUNCH_FILE" >/dev/null 2>&1
  for node in "${NODE_NAMES[@]}"; do
    pkill -f "$node" >/dev/null 2>&1
  done
  ros2 daemon stop >/dev/null 2>&1
  
  # 启动节点
  nohup ros2 launch $LAUNCH_FILE > "$OUTPUT_FILE" 2>&1 &
  sleep 15  # 等待节点启动完成
}

# 重启所有节点
function restart() {
  echo "[$(date)] 重启所有节点..."
  # 彻底清理进程
  pkill -f "ros" >/dev/null 2>&1
  ros2 daemon stop >/dev/null 2>&1
  sleep 2  # 等待进程终止
  bringup
}

# 初始启动节点
bringup
# 等待节点完成初始化
sleep $TIMEOUT
sleep $TIMEOUT

# 监控循环
while true; do
  for node in "${NODE_NAMES[@]}"; do
    # 构建完整话题名称
    if [ -z "$NAMESPACE" ]; then
      topic="/$node/heartbeat"
    else
      topic="$NAMESPACE/$node/heartbeat"
    fi
    
    echo "[$(date)] 检查节点: $node"
    # 检查话题是否存在
    if ros2 topic list 2>/dev/null | grep -q "^${topic}$"; then
      # 尝试获取心跳数据
      data_value=$(timeout 5 ros2 topic echo "$topic" --once 2>/dev/null | grep -o "data: [0-9]*" | awk '{print $2}')
      if [ -n "$data_value" ]; then
        echo "[$(date)]   $node 正常，心跳计数: $data_value"
      else
        echo "[$(date)]   $node 心跳数据丢失，重启中..."
        restart
        break  # 跳出节点检查循环，等待重启完成
      fi
    else
      echo "[$(date)]   $node 心跳话题 $topic 不存在，重启中..."
      restart
      break  # 跳出节点检查循环，等待重启完成
    fi
  done
  sleep $TIMEOUT
done
    