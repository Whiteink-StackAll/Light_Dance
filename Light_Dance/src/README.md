# 2025光电赛 Vision Project

本项目为福建师范大学光之舞战队2025全国大学生光电设计竞赛赛题1视觉主项目框架。

## 项目结构

```
.
│
├── light_bringup (启动及参数文件)
│
├── light_interfaces (自定义msg、srv)
│
├── light_camera (相机驱动)(海康相机SDK)
│   
├── light_serial (串口驱动)
│
├── light_detector (识别部分)
│
├── light_solver (解算部分，包括一欧元滤波)
│
├── light_utils (工具包，包括心跳节点) 
│   
└── light_upstart (自启动配置)
```

## 环境配置

### ROS2

  采用鱼香肉丝一键安装，在此感谢。

  ```bash
  wget http://fishros.com/install -O fishros && . fishros
  ```

### opencv

  本代码同时需要C++与python版本的OpenCV

  ```bash
  sudo apt update && sudo apt upgrade -y

  # 基础依赖
  sudo apt install -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev

  # 可选依赖（增强功能）
  sudo apt install -y python3-dev python3-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libjasper-dev libdc1394-22-dev

  # 确保pip已安装
  sudo apt install -y python3-pip

  # 安装OpenCV Python包
  pip3 install opencv-python
  pip3 install opencv-contrib-python  # 包含额外功能
  ```

  C++版本的OpenCV需要从源码编译

  ```bash
  # 创建工作目录并克隆源码
  mkdir -p ~/opencv_build && cd ~/opencv_build
  git clone https://github.com/opencv/opencv.git
  git clone https://github.com/opencv/opencv_contrib.git

  # 创建编译目录
  cd opencv
  mkdir build && cd build

  # 配置编译选项
  cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -D INSTALL_C_EXAMPLES=ON \
        -D INSTALL_PYTHON_EXAMPLES=ON \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -D OPENCV_EXTRA_MODULES_PATH=~/opencv_build/opencv_contrib/modules \
        -D BUILD_EXAMPLES=ON ..

  # 编译
  make -j2

  # 安装
  sudo make install

  # 更新动态链接库
  sudo ldconfig
  ```
  
  验证安装

  ```bash
  # 验证Python版本
  python3 -c "import cv2; print('Python OpenCV版本:', cv2.__version__)"

  # 验证C++版本
  pkg-config --modversion opencv4
  ```

### YOLO
  
  使用CPU进行推理

  ```bash
  # 更新系统包
  sudo apt update && sudo apt upgrade -y

  # 安装基础依赖
  sudo apt install -y build-essential cmake git libopencv-dev python3 python3-pip

  # 安装Python依赖（CPU版本）
  pip3 install --upgrade pip
  pip3 install numpy opencv-python

  # 安装CPU版本的PyTorch
  pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

  # 安装YOLO库和ROS 2依赖
  pip3 install ultralytics
  pip3 install rclpy rosidl_default_generators

  # 验证安装
  python3 -c "import ultralytics; print('YOLO安装成功')"
  python3 -c "import torch; print('PyTorch CPU版本可用:', torch.__version__)"

  # 测试YOLO基础功能（CPU模式）
  yolo predict model=yolov8n.pt source=0 device=cpu
  ```

## 编译与运行

  ```bash
  rm -rf build/ install/ log/

  colcon build --symlink-install --parallel-workers 2

  source install/setup.bash

  ros2 launch light_bringup bringup.launch.py
  ```

## 开机自启动

  请确保代码可正常编译并运行后，执行以下操作
  注意检查线程是否竞争

  ```bash
  cd src/light_upstart

  source /home/pikachu/LightDance3/install/setup.bash

  chmod +x register_service.sh light_watch_dog.sh light_clean_up.sh

  sudo ./register_service.sh

  systemctl status light    # 监测状态

  systemctl start light     # 开启服务

  systemctl stop light      # 关闭服务

  systemctl enable light    # 开启自启动

  systemctl disable light   # 取消自启动
  ```

## 调试

  采用Foxgolve接收并读取话题进行调试

  ```bash

  source install/setup.bash

  ros2 launch foxglove_bridge foxglove_bridge_launch.xml

  ```

  此时打开foxglove：打开连接->ws://localhost:xxxx

## 串口协议

电控发视觉 16位

帧头 1位 0xFF
float32 4位 云台yaw
float32 4位 云台pitch
float32 4位 云台roll
uint8_t 1位 模式
0 未瞄准
1 正在瞄准
2 已瞄准
uint8_t 1位 是否需要后退
0 不需要后退
1 需要后退
中间全部填0
帧尾 1位 0x0D


## 维护者及开源许可证

> 比赛结束后开源

Maintainer : PKA Vision Group

```
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```