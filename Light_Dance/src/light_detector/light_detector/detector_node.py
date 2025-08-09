#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import os
import time
import threading
import signal
from ultralytics import YOLO
import traceback

# 导入自定义消息类型
from light_interfaces.msg import Armor
from light_interfaces.srv import GetAimState

class DetectorNode(Node):
    def __init__(self):
        super().__init__('light_detector')
        
        # 设置信号处理
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        
        # 线程锁，确保共享变量操作安全
        self.lock = threading.Lock()
        
        # 声明参数并设置默认值
        self.declare_parameters(
            namespace='',
            parameters=[
                ('input_source', 'video'),
                ('video_path', './test_video.mp4'),
                ('camera_topic', '/image_raw'),
                ('model_path', 'best.pt'),
                ('device', 'cpu'),
                ('confidence_threshold', 0.5),
                ('nms_threshold', 0.4),
                ('publish_result', True),
                ('show_image', True),
                ('enable_debug', True),
                ('ignore_camera', False),
                ('camera_qos_reliability', 'best_effort'),
                ('frame_publish_limit', 10),
                ('video_loop', True),  # 视频是否循环播放
                # 新增参数
                ('result_topic', 'result_img'),
                ('armor_topic', 'armor_detection'),
                ('max_det', 3),
                # 目标大小参数
                ('max_target_ratio', 0.3),  # 目标最大比例（占图像总面积）
                ('min_target_ratio', 0.05),  # 目标最小比例（占图像总面积）
                # 二值化相关参数
                ('enable_thresholding', False),  # 二值化功能开关
                ('threshold_value', 127),  # 二值化阈值
                ('threshold_max_value', 255),  # 二值化最大值
                ('threshold_topic', 'binary_image'),  # 二值化图像发布话题
                ('threshold_type', 0)  # 二值化类型: 0=THRESH_BINARY, 1=THRESH_BINARY_INV
            ]
        )
        
        # 先获取调试开关参数，以便后续日志使用
        self.enable_debug = self.get_parameter('enable_debug').value
        
        # 获取并验证其他参数
        self.ignore_camera = self.get_parameter('ignore_camera').value
        self.input_source = self.get_parameter('input_source').value
        self.video_path = self.get_parameter('video_path').value
        self.video_loop = self.get_parameter('video_loop').value  # 获取视频循环参数
        self.camera_topic = self.get_parameter('camera_topic').value
        self.model_path = self.get_parameter('model_path').value
        
        # 目标大小参数
        self.max_target_ratio = self.get_parameter('max_target_ratio').value
        self.min_target_ratio = self.get_parameter('min_target_ratio').value
        
        # 二值化相关参数
        self.enable_thresholding = self.get_parameter('enable_thresholding').value
        self.threshold_value = max(0, min(255, self.get_parameter('threshold_value').value))
        self.threshold_max_value = max(0, min(255, self.get_parameter('threshold_max_value').value))
        self.threshold_topic = self.get_parameter('threshold_topic').value
        self.threshold_type = self.get_parameter('threshold_type').value
        
        # 验证二值化类型
        if self.threshold_type not in [0, 1]:
            self.get_logger().warning(f"无效的二值化类型 {self.threshold_type}，使用默认值0 (THRESH_BINARY)")
            self.threshold_type = 0
        
        # 映射到OpenCV的二值化类型
        self.threshold_opencv_type = cv2.THRESH_BINARY if self.threshold_type == 0 else cv2.THRESH_BINARY_INV
        
        # 验证设备参数
        self.device = self.get_parameter('device').value
        if self.device not in ['cpu', 'cuda'] and not self.device.startswith('cuda:'):
            self.get_logger().warning(f"无效设备 {self.device}，自动切换为cpu")
            self.device = 'cpu'
            
        # 验证阈值参数
        self.confidence_threshold = max(0.01, min(1.0, self.get_parameter('confidence_threshold').value))
        self.nms_threshold = max(0.01, min(1.0, self.get_parameter('nms_threshold').value))
        
        self.publish_result = self.get_parameter('publish_result').value
        self.show_image = self.get_parameter('show_image').value
        self.camera_qos_reliability = self.get_parameter('camera_qos_reliability').value
        
        # 新增参数
        self.result_topic = self.get_parameter('result_topic').value
        self.armor_topic = self.get_parameter('armor_topic').value
        self.max_det = self.get_parameter('max_det').value
        
        # 验证帧率限制参数
        self.frame_publish_limit = max(1, self.get_parameter('frame_publish_limit').value)
        
        # 状态变量
        self.last_published_time = 0.0
        self.frame_interval = 1.0 / self.frame_publish_limit
        self.image_receive_count = 0
        self.result_publish_count = 0
        self.binary_image_publish_count = 0  # 二值化图像发布计数
        self.last_image_time = time.time()
        self.last_result_published = time.time()
        self.result_failure_count = 0
        self.node_running = True
        self.aim_state = "NOT_AIMING"  # 初始瞄准状态
        self.cap = None  # 视频捕获设备引用，用于安全释放
        self.video_ended = False  # 视频是否已结束的标志
        
        # 初始化信息只在调试模式下输出
        if self.enable_debug:
            self.get_logger().info(f"===== 检测器节点初始化 =====")
            self.get_logger().info(f"输入源: {self.input_source}")
            self.get_logger().info(f"相机话题: {self.camera_topic}")
            self.get_logger().info(f"模型路径: {self.model_path}")
            self.get_logger().info(f"运行设备: {self.device}")
            self.get_logger().info(f"相机QoS可靠性: {self.camera_qos_reliability}")
            self.get_logger().info(f"帧率限制: {self.frame_publish_limit} FPS")
            self.get_logger().info(f"视频循环: {self.video_loop}")
            self.get_logger().info(f"目标大小参数: 最大比例={self.max_target_ratio}, 最小比例={self.min_target_ratio}")
            self.get_logger().info(f"二值化配置: 启用={self.enable_thresholding}, 阈值={self.threshold_value}, "
                                  f"最大值={self.threshold_max_value}, 类型={self.threshold_opencv_type}, "
                                  f"话题={self.threshold_topic}")
            self.get_logger().info(f"发布话题: {self.result_topic}")
            self.get_logger().info(f"OpenCV版本: {cv2.__version__}")
            self.get_logger().info(f"RMW实现: {os.environ.get('RMW_IMPLEMENTATION', '默认')}")
        
        # 检查RTPS错误环境变量
        if 'FASTRTPS_DEFAULT_PROFILES_FILE' in os.environ:
            self.get_logger().info(f"使用FastDDS配置文件: {os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']}")
        
        # 初始化ROS通信
        self.bridge = CvBridge()
        
        # 创建带持久性的发布器
        result_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        self.result_pub = self.create_publisher(Image, self.result_topic, result_qos)
        self.armor_pub = self.create_publisher(Armor, self.armor_topic, 10)
        
        # 创建二值化图像发布器
        self.binary_pub = self.create_publisher(Image, self.threshold_topic, result_qos)
        
        # 配置相机订阅的QoS
        self.camera_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 根据参数设置可靠性策略
        if self.camera_qos_reliability.lower() == 'reliable':
            self.camera_qos.reliability = QoSReliabilityPolicy.RELIABLE
        else:
            self.camera_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        
        # 调试QoS配置
        if self.enable_debug:
            self.get_logger().debug(
                f"结果发布QoS: 可靠性={result_qos.reliability}, "
                f"持久性={result_qos.durability}, 话题={self.result_topic}"
            )
        
        # 初始化模型
        self.model = self.init_model()
        if self.model is None:
            self.get_logger().fatal("模型初始化失败，节点无法启动")
            return
        
        # 创建服务客户端获取瞄准状态
        self.aim_state_client = self.create_client(GetAimState, 'get_aim_state')
        
        # 创建服务检查定时器并保存句柄
        self.service_check_timer = self.create_timer(1.0, self.check_service_availability)
        
        # 添加定时器定期获取状态（每0.1秒）
        self.create_timer(0.1, self.fetch_aim_state)
        
        # 其他定时器
        self.create_timer(2.0, self.check_image_topic)
        self.create_timer(3.0, self.check_image_reception)
        self.create_timer(10.0, self.log_status)
        
        # 视频/相机处理
        self.video_thread = None
        self.camera_sub = None
        self.keep_alive_thread = None
        
        # 帧率计算
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.fps = 0.0
        
        # 新增：追踪是否检测到目标
        self.detected_target = False
        self.last_detection_time = time.time()
        
        # 初始化输入源
        self.init_input_source()
        
        # 发布一个初始测试帧
        self.publish_test_frame("节点初始化完成")
        
        # 添加定期检查节点健康状态的定时器
        self.health_check_timer = self.create_timer(5.0, self.check_health)

    def signal_handler(self, sig, frame):
        """处理外部终止信号"""
        self.get_logger().info(f"接收到信号 {sig}，准备关闭节点")
        self.node_running = False
        # 给节点一些时间清理资源
        time.sleep(1)
        self.destroy_node()
        rclpy.shutdown()

    def check_health(self):
        """定期检查节点健康状态"""
        current_time = time.time()
        
        # 检查是否长时间未发布结果
        with self.lock:
            if current_time - self.last_result_published > 10.0:
                self.get_logger().warning(
                    f"长时间未发布结果帧！({current_time - self.last_result_published:.1f}秒)"
                )
                # 尝试发布测试帧恢复
                self.publish_test_frame("检测到长时间未发布结果，尝试恢复")
        
        # 检查服务连接状态
        if not self.aim_state_client.service_is_ready():
            self.get_logger().warning("get_aim_state服务已断开连接，尝试重新连接")
            # 重新创建服务客户端
            self.aim_state_client.destroy()
            self.aim_state_client = self.create_client(GetAimState, 'get_aim_state')
            # 重启服务检查定时器
            if self.service_check_timer.is_canceled():
                self.service_check_timer = self.create_timer(1.0, self.check_service_availability)

    def check_service_availability(self):
        """检查服务是否可用，避免阻塞节点启动"""
        if not self.aim_state_client.service_is_ready():
            if self.enable_debug:
                self.get_logger().info('get_aim_state服务未就绪，等待中...')
        else:
            if self.enable_debug:
                self.get_logger().info('get_aim_state服务已就绪')
            # 服务就绪后取消这个定时器（使用保存的定时器句柄）
            self.destroy_timer(self.service_check_timer)

    def init_model(self):
        """初始化YOLO模型，带更完善的错误处理"""
        try:
            if not os.path.exists(self.model_path):
                raise FileNotFoundError(f"模型文件不存在: {self.model_path}")
            
            if self.enable_debug:
                self.get_logger().info(f"加载YOLO模型: {self.model_path}")
            model = YOLO(self.model_path, verbose=False)
            model.to(self.device)
            if self.enable_debug:
                self.get_logger().info("模型加载成功")
            return model
        except FileNotFoundError as e:
            self.get_logger().error(f"模型文件未找到: {str(e)}")
            return None
        except RuntimeError as e:
            self.get_logger().error(f"模型加载到设备失败: {str(e)}")
            if 'cuda' in str(e).lower() and self.device != 'cpu':
                self.get_logger().warning("尝试使用CPU加载模型...")
                try:
                    model = YOLO(self.model_path, verbose=False)
                    model.to('cpu')
                    self.device = 'cpu'  # 更新设备为CPU
                    return model
                except Exception as e:
                    self.get_logger().error(f"CPU加载模型也失败: {str(e)}")
            return None
        except Exception as e:
            self.get_logger().error(f"模型加载失败: {str(e)}")
            if self.enable_debug:
                self.get_logger().error(traceback.format_exc())
            return None

    def init_input_source(self):
        if self.ignore_camera:
            self.keep_alive_thread = threading.Thread(target=self.keep_alive, daemon=True)
            self.keep_alive_thread.start()
            if self.enable_debug:
                self.get_logger().info("已启用忽略相机模式")
            return
            
        if self.input_source == 'camera':
            if self.enable_debug:
                self.get_logger().info(f"订阅相机话题: {self.camera_topic}")
            self.camera_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.safe_camera_callback,
                self.camera_qos
            )
        else:
            # 增强视频路径验证
            video_path = os.path.abspath(self.video_path)
            if not os.path.exists(video_path) and self.video_path != '0':
                self.get_logger().error(f"视频文件不存在: {video_path}")
                self.keep_alive_thread = threading.Thread(target=self.keep_alive, daemon=True)
                self.keep_alive_thread.start()
                return
                
            if os.path.exists(video_path) and not os.access(video_path, os.R_OK):
                self.get_logger().error(f"无权限读取视频文件: {video_path}")
                self.keep_alive_thread = threading.Thread(target=self.keep_alive, daemon=True)
                self.keep_alive_thread.start()
                return
                
            self.video_thread = threading.Thread(target=self.video_processing, daemon=True)
            self.video_thread.start()
            if self.enable_debug:
                self.get_logger().info("视频处理线程已启动")

    def keep_alive(self):
        while rclpy.ok() and self.node_running:
            time.sleep(1.0)

    def safe_camera_callback(self, msg):
        """带错误处理的相机回调函数"""
        try:
            current_time = time.time()
            with self.lock:  # 线程安全检查
                if current_time - self.last_published_time < self.frame_interval:
                    return
                self.last_published_time = current_time
                
            self.camera_callback(msg)
        except Exception as e:
            self.get_logger().error(f"相机回调错误: {e}")
            if self.enable_debug:
                self.get_logger().error(traceback.format_exc())

    def camera_callback(self, msg):
        """实际的相机回调处理"""
        with self.lock:  # 线程安全更新计数
            self.image_receive_count += 1
            self.last_image_time = time.time()
        
        # 每50帧输出一次信息，减少日志量
        if self.enable_debug and self.image_receive_count % 50 == 0:
            self.get_logger().debug(f"已接收 {self.image_receive_count} 帧图像")
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # 如果启用二值化，处理并发布二值化图像
            if self.enable_thresholding:
                self.process_and_publish_binary_image(cv_image)
                
            self.process_frame(cv_image)
        except CvBridgeError as e:
            self.get_logger().error(f"图像转换错误: {e}")
        except Exception as e:
            self.get_logger().error(f"处理相机图像错误: {e}")

    def process_and_publish_binary_image(self, cv_image):
        """处理并发布二值化图像"""
        try:
            # 将图像转为灰度图
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 应用二值化
            ret, binary = cv2.threshold(
                gray, 
                self.threshold_value, 
                self.threshold_max_value, 
                self.threshold_opencv_type
            )
            
            # 发布二值化图像
            binary_msg = self.bridge.cv2_to_imgmsg(binary, 'mono8')
            self.binary_pub.publish(binary_msg)
            
            with self.lock:
                self.binary_image_publish_count += 1
                
            if self.enable_debug and (self.binary_image_publish_count <= 10 or 
                                     self.binary_image_publish_count % 50 == 0):
                self.get_logger().debug(
                    f"已发布 {self.binary_image_publish_count} 帧二值化图像到 {self.threshold_topic}"
                )
                
        except Exception as e:
            self.get_logger().error(f"处理或发布二值化图像时出错: {e}")
            if self.enable_debug:
                self.get_logger().error(traceback.format_exc())

    def video_processing(self):
        try:
            # 打开视频源
            if self.video_path.isdigit():
                self.cap = cv2.VideoCapture(int(self.video_path))
                source_type = "摄像头"
            else:
                video_path = os.path.abspath(self.video_path)
                self.cap = cv2.VideoCapture(video_path)
                source_type = "视频文件"
                
            if not self.cap.isOpened():
                self.get_logger().error(f"无法打开{source_type}: {self.video_path}")
                self.cap = None
                return
            
            fps = self.cap.get(cv2.CAP_PROP_FPS)
            delay = 1.0 / fps if fps > 0 else 0.033
            if self.enable_debug:
                self.get_logger().info(f"{source_type}FPS: {fps:.1f}, 延迟: {delay:.3f}秒")
            
            # 视频总帧数，用于循环播放
            frame_count = self.cap.get(cv2.CAP_PROP_FRAME_COUNT) if source_type == "视频文件" else 0
            
            while rclpy.ok() and self.node_running and self.cap is not None:
                # 为避免无限阻塞，添加超时机制
                start_time = time.time()
                ret, frame = self.cap.read()
                
                # 检查是否超时或读取失败
                if not ret or (time.time() - start_time) > 1.0:
                    if self.enable_debug:
                        self.get_logger().info(f"{source_type}读取帧失败")
                    
                    # 如果是视频文件且需要循环播放
                    if source_type == "视频文件" and self.video_loop:
                        if self.enable_debug:
                            self.get_logger().info("视频播放完毕，重新开始")
                        # 重置视频到开头
                        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        # 继续下一次循环
                        continue
                    else:
                        # 对于摄像头或不需要循环的视频，退出循环
                        if self.enable_debug:
                            self.get_logger().info(f"{source_type}处理完成")
                        break
                
                current_time = time.time()
                with self.lock:  # 线程安全检查，允许一定时间误差
                    if current_time - self.last_published_time >= self.frame_interval - 0.1:
                        self.last_published_time = current_time
                        
                        # 如果启用二值化，处理并发布二值化图像
                        if self.enable_thresholding:
                            self.process_and_publish_binary_image(frame)
                            
                        self.process_frame(frame)
                
                # 控制读取速度
                time.sleep(delay)
                
            # 释放资源
            self.cap.release()
            self.cap = None
            
            # 如果是视频文件且不循环，保持节点运行
            if source_type == "视频文件" and not self.video_loop:
                self.get_logger().info("视频播放完毕，保持节点运行")
                self.keep_alive()
                
        except Exception as e:
            self.get_logger().error(f"视频处理错误: {e}")
            if self.enable_debug:
                self.get_logger().error(traceback.format_exc())
        finally:
            # 确保资源释放
            if self.cap is not None:
                self.cap.release()
                self.cap = None

    def process_frame(self, frame):
        if self.model is None:
            self.get_logger().error("模型未初始化，无法处理图像")
            return
            
        try:
            # 帧率计算（线程安全）
            with self.lock:
                self.frame_count += 1
                current_time = time.time()
                elapsed = current_time - self.last_fps_time
                
                if elapsed >= 1.0:
                    self.fps = self.frame_count / elapsed
                    self.frame_count = 0
                    self.last_fps_time = current_time
            
            # YOLO推理（捕获特定异常）
            try:
                results = self.model(frame, 
                                   conf=self.confidence_threshold, 
                                   iou=self.nms_threshold,
                                   max_det=self.max_det,  # 使用max_det参数
                                   verbose=False)
            except RuntimeError as e:
                self.get_logger().error(f"模型推理失败: {e}")
                with self.lock:
                    self.result_failure_count += 1
                return
            
            # 图像处理和绘制
            h, w = frame.shape[:2]
            image_area = w * h  # 计算图像总面积
            image_center = (w // 2, h // 2)
            cv2.circle(frame, image_center, 10, (0, 0, 255), 2)
            
            # 筛选出置信度最高的目标（限制为1个）
            best_box = None
            best_confidence = 0.0
            best_area_ratio = 0.0
            
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # 获取边界框和置信度
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    confidence = box.conf[0].item()
                    
                    # 计算目标区域和占比
                    target_area = (x2 - x1) * (y2 - y1)
                    area_ratio = target_area / image_area
                    
                    # 保留置信度最高的目标
                    if confidence > best_confidence:
                        best_confidence = confidence
                        best_box = (x1, y1, x2, y2)
                        best_area_ratio = area_ratio
            
            # 更新目标检测状态
            with self.lock:
                if best_box is not None:
                    self.detected_target = True
                    self.last_detection_time = current_time
                else:
                    self.detected_target = False
            
            # 处理最佳目标
            if best_box is not None:
                x1, y1, x2, y2 = best_box
                
                # 计算中心点
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0
                width = x2 - x1
                height = y2 - y1
                
                # 创建并发布Armor消息
                armor_msg = Armor()
                armor_msg.x = center_x
                armor_msg.y = center_y
                armor_msg.width = float(width)
                armor_msg.height = float(height)
                self.armor_pub.publish(armor_msg)
                
                # 绘制边界框和中心点
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                armor_center = (int(center_x), int(center_y))
                cv2.circle(frame, armor_center, 5, (0, 0, 255), 2)
                
                # 显示目标大小比例
                ratio_text = f"Ratio: {best_area_ratio:.2f}"
                cv2.putText(frame, ratio_text, (x1, y1 - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
                if self.enable_debug:
                    self.get_logger().debug(
                        f"检测到目标 - 中心: ({center_x:.2f}, {center_y:.2f}), "
                        f"大小: {width}x{height}, 占比: {best_area_ratio:.2f}"
                    )
            else:
                # 没有检测到目标时发布全零Armor消息
                armor_msg = Armor()
                armor_msg.x = 0.0
                armor_msg.y = 0.0
                armor_msg.width = 0.0
                armor_msg.height = 0.0
                self.armor_pub.publish(armor_msg)
                
                if self.enable_debug:
                    self.get_logger().debug("未检测到目标，发布全零Armor消息")
            
            # 绘制信息条
            bar_height = 30
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (w, bar_height), (0, 0, 0), -1)
            alpha = 0.5
            cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
            
            # 显示FPS
            fps_text = f"FPS: {self.fps:.1f}"
            cv2.putText(frame, fps_text, (10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 显示状态（右上角）
            status_text = f"State: {self.aim_state}"
            # 计算文本宽度以右对齐显示
            text_size, _ = cv2.getTextSize(status_text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)
            cv2.putText(frame, status_text, (w - text_size[0] - 10, 20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # 发布result_img
            if self.publish_result:
                try:
                    if frame is None or frame.size == 0:
                        raise ValueError("无效的图像帧")
                        
                    result_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                    self.result_pub.publish(result_msg)
                    
                    with self.lock:  # 线程安全更新状态
                        self.last_result_published = current_time
                        self.result_failure_count = 0
                        self.result_publish_count += 1
                    
                    # 优化日志输出：前10帧每帧输出，之后每50帧输出
                    if self.enable_debug:
                        if self.result_publish_count <= 10 or self.result_publish_count % 50 == 0:
                            self.get_logger().info(
                                f"已发布 {self.result_publish_count} 帧到{self.result_topic}"
                            )
                        
                except CvBridgeError as e:
                    with self.lock:
                        self.result_failure_count += 1
                    self.get_logger().error(f"图像转换错误: {e}")
                except Exception as e:
                    with self.lock:
                        self.result_failure_count += 1
                    self.get_logger().error(f"发布结果图像错误: {e}")
            
            # 显示图像（仅在show_image=True时执行）
            if self.show_image:
                try:
                    cv2.imshow('YOLO Detection', frame)
                    # 检查按键事件，设置较短超时避免阻塞
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        if self.enable_debug:
                            self.get_logger().info("用户请求退出")
                        rclpy.shutdown()
                except Exception as e:
                    self.get_logger().warning(f"无法显示图像: {e}，禁用显示功能")
                    self.show_image = False
        except Exception as e:
            self.get_logger().error(f"帧处理错误: {e}")
            if self.enable_debug:
                self.get_logger().error(traceback.format_exc())
            with self.lock:
                self.result_failure_count += 1

    def publish_test_frame(self, message):
        """发布测试帧，确保话题正常工作"""
        try:
            test_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(test_frame, message, (50, 240),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            result_msg = self.bridge.cv2_to_imgmsg(test_frame, 'bgr8')
            self.result_pub.publish(result_msg)
            
            # 如果启用二值化，也发布二值化测试帧
            if self.enable_thresholding:
                binary_test = np.zeros((480, 640), dtype=np.uint8)
                cv2.putText(binary_test, "Binary Test", (50, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.0, 255, 2)
                binary_msg = self.bridge.cv2_to_imgmsg(binary_test, 'mono8')
                self.binary_pub.publish(binary_msg)
            
            with self.lock:
                self.last_result_published = time.time()
                
            if self.enable_debug:
                self.get_logger().info(f"发布测试帧到话题 {self.result_topic}: {message}")
        except Exception as e:
            self.get_logger().error(f"发布测试帧失败: {e}")

    def check_image_reception(self):
        """检查是否收到图像数据"""
        current_time = time.time()
        with self.lock:
            elapsed_since_last_image = current_time - self.last_image_time
            image_count = self.image_receive_count
        
        if elapsed_since_last_image > 5.0 and self.input_source == 'camera':
            self.get_logger().warning(
                f"长时间未收到图像数据！({elapsed_since_last_image:.1f}秒)，"
                f"已接收总数: {image_count}"
            )

    def check_image_topic(self):
        """检查图像话题是否存在和有发布者"""
        if self.input_source != 'camera':
            return
            
        try:
            topic_names_and_types = self.get_topic_names_and_types()
            topic_exists = any(topic[0] == self.camera_topic for topic in topic_names_and_types)
            
            if not topic_exists:
                self.get_logger().error(f"相机话题 {self.camera_topic} 不存在！请检查相机节点")
                return
                
            publishers = self.get_publishers_info_by_topic(self.camera_topic)
            if not publishers:
                self.get_logger().warning(f"相机话题 {self.camera_topic} 没有发布者")
            else:
                if self.enable_debug:
                    self.get_logger().debug(f"相机话题 {self.camera_topic} 有 {len(publishers)} 个发布者")
                
        except Exception as e:
            self.get_logger().error(f"检查图像话题时出错: {e}")

    def log_status(self):
        """定期记录节点状态，仅在调试模式下输出"""
        if self.enable_debug:
            with self.lock:
                img_count = self.image_receive_count
                pub_count = self.result_publish_count
                bin_count = self.binary_image_publish_count
                current_fps = self.fps
                target_status = "检测到目标" if self.detected_target else "未检测到目标"
                
            self.get_logger().info(
                f"节点状态 - 接收图像: {img_count}, "
                f"发布结果: {pub_count}, "
                f"发布二值化图像: {bin_count}, "
                f"FPS: {current_fps:.1f}, "
                f"目标状态: {target_status}"
            )

    def fetch_aim_state(self):
        """定期调用服务获取当前瞄准状态"""
        try:
            # 检查服务是否就绪
            if not self.aim_state_client.service_is_ready():
                return
                
            request = GetAimState.Request()
            future = self.aim_state_client.call_async(request)
            future.add_done_callback(self.handle_aim_state_response)
        except Exception as e:
            self.get_logger().error(f"调用状态服务失败: {e}")

    def handle_aim_state_response(self, future):
        """处理服务响应，更新状态"""
        try:
            # 检查是否超时或被取消
            if not future.done():
                self.get_logger().warning("服务调用超时")
                return
                
            response = future.result()
            # 根据solver_node中的状态枚举转换为字符串
            state_mapping = {
                0: "NOT_AIMING",
                1: "AIMING",
                2: "AIMED"
            }
            self.aim_state = state_mapping.get(response.state, f"未知状态({response.state})")
            if self.enable_debug:
                self.get_logger().debug(f"更新瞄准状态: {self.aim_state}")
        except Exception as e:
            self.get_logger().error(f"处理状态响应失败: {e}")
            # 检查是否是因为solver节点未启动导致的错误
            if "Failed to find service" in str(e):
                self.get_logger().warning("未找到get_aim_state服务，可能solver节点未启动")

    def destroy_node(self):
        # 线程安全地设置退出标志
        with self.lock:
            self.node_running = False
            
        if self.enable_debug:
            self.get_logger().info("正在关闭节点...")
        
        # 释放OpenCV资源
        if self.show_image:
            cv2.destroyAllWindows()
            
        # 确保视频捕获设备释放
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception as e:
                self.get_logger().warning(f"释放视频设备失败: {e}")
        
        # 等待所有子线程退出（最多等待1秒）
        if self.video_thread and self.video_thread.is_alive():
            self.video_thread.join(timeout=1.0)
        if self.keep_alive_thread and self.keep_alive_thread.is_alive():
            self.keep_alive_thread.join(timeout=1.0)
        
        super().destroy_node()

def main(args=None):
    try:
        # 初始化前检查RMW实现
        rmw_impl = os.environ.get('RMW_IMPLEMENTATION', '默认')
        print(f"使用RMW实现: {rmw_impl}")
        
        rclpy.init(args=args)
        node = DetectorNode()
        
        if node.model is None:
            node.get_logger().error("模型加载失败，节点无法启动")
            node.destroy_node()
            rclpy.shutdown()
            return
            
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        if 'node' in locals() and node.enable_debug:
            node.get_logger().info("用户中断程序")
    except Exception as e:
        if 'node' in locals():
            node.get_logger().error(f"节点运行错误: {str(e)}")
            if node.enable_debug:
                node.get_logger().error(traceback.format_exc())
        else:
            print(f"节点初始化错误: {str(e)}")
            traceback.print_exc()
    finally:
        # 确保所有资源释放后再彻底关闭ROS
        cv2.destroyAllWindows()
        if 'node' in locals():
            node.destroy_node()
        # 强制等待ROS完成shutdown
        if rclpy.ok():
            rclpy.shutdown()
        # 给DDS足够时间清理资源
        time.sleep(0.5)

if __name__ == '__main__':
    main()