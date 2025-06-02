#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
语音识别模块 (基于Vosk实现)
===========================

此模块实现了机器人的语音识别功能，基于开源的Vosk语音识别引擎。
系统能够通过麦克风阵列采集声音，进行中文语音识别，并将结果发布到ROS话题。

主要功能：
1. 自动加载中文语音模型(支持多种模型大小)
2. 自动检测和配置麦克风设备
3. 通过两种方式触发语音识别：
   - 用户按下Enter键(调试模式)
   - ROS话题消息触发(正常使用模式)
4. 动态调整的语音活动检测
5. 结果通过ROS话题发布，供其他节点订阅处理

技术细节：
- 使用Vosk作为语音识别引擎(离线工作)
- 使用PyAudio处理音频流
- 采用自适应的语音活动检测算法

使用方法：
1. 直接运行此脚本: python3 voice_recognition.py
2. 通过ROS节点启动: rosrun language voice_recognition.py
3. 或者通过launch文件启动

触发方式：
- 调试模式: 按下Enter键开始录音和识别
- 正常模式: 向'/mic/awake/angle'话题发送消息触发

输出：
- 识别结果发布到'chatter'话题(std_msgs/String类型)

开发历史：
- 初始版本：基本的语音识别功能
- 改进版本：添加ROS集成和环境自适应功能
- 当前版本：解决了并发访问导致的崩溃问题，采用标志位设计模式

作者: Educational Robot团队
维护人员: [填写您的名字]
"""

import os
import sys
import json
import wave
import time
import pyaudio
import rospy  # ROS系统的Python客户端库
from std_msgs.msg import String, Int32, Int8  # ROS标准消息类型
import numpy as np
import zipfile
from vosk import Model, KaldiRecognizer

# ====================================================================
# 全局配置参数 - 系统性能和行为调优
# ====================================================================
"""
全局参数配置说明
---------------

以下参数控制语音识别系统的行为和性能。根据具体硬件和使用场景，
这些参数可能需要调整以获得最佳效果。

模型配置：
- MODEL_SEARCH_PATHS: 按优先级排列的模型搜索路径列表
- MODEL_ZIP_FILES: 备选的模型ZIP文件，按优先级排序
- EXTERNAL_MODEL_PATH: 指定的外部模型路径

音频配置：
- SAMPLE_RATE: 音频采样率，建议保持为16000Hz (Vosk优化值)
- CHANNELS: 通道数，通常为1(单声道)
- FRAME_SIZE: 每次读取的音频帧大小，影响处理延迟

识别配置：
- MAX_RECOGNITION_SECONDS: 最大录音时间，防止无限录制
- SILENCE_THRESHOLD: 静默检测阈值，低于此值视为静默
- MIN_BUFFER_SIZE: 最小音频缓冲区大小，确保有足够数据识别
- SPEECH_TIMEOUT: 停止说话后多久结束录音
- BUFFER_PROCESS_INTERVAL: 处理缓冲区的间隔时间
- FORCED_END_SECONDS: 连续说话超过此时间将强制结束录音
- MIN_SPEECH_SECONDS: 最少需要的语音时间，过滤短噪音
- VOLUME_RISE_THRESHOLD: 音量突然上升的倍数阈值，用于检测说话开始
- CALIBRATION_TIME_SECONDS: 环境噪音校准时间

调试配置：
- DEBUG_FILE: 调试音频文件名，用于保存识别失败的音频

调优指南：
- 如果识别总是过早结束：增大SPEECH_TIMEOUT值
- 如果响应速度慢：减小CALIBRATION_TIME_SECONDS和FRAME_SIZE
- 如果经常误触发：增大SILENCE_THRESHOLD和VOLUME_RISE_THRESHOLD
- 如果环境噪音大：增加CALIBRATION_TIME_SECONDS来获得更准确的噪音基线
"""

# 模型配置
MODEL_SEARCH_PATHS = [
    # 优先使用中间大小的模型(平衡速度和准确性)
    "vosk-model-cn-kaldi-multicn-0.15",
    os.path.join(os.getcwd(), "vosk-model-cn-kaldi-multicn-0.15"),
    
    # 其次使用大模型(更精确但较慢)
    "vosk-model-cn-0.15",
    os.path.join(os.getcwd(), "vosk-model-cn-0.15"),
    
    # 最后尝试小模型(速度快)
    "vosk-model-small-cn-0.22",
    os.path.join(os.getcwd(), "vosk-model-small-cn-0.22"),
    
    # 其他可能的模型路径
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "model"),
    os.path.join(os.getcwd(), "model"),
    "model",
]

MODEL_ZIP_FILES = [
    # 优先解压中型模型
    "vosk-model-cn-kaldi-multicn-0.15.zip",  # 中型模型
    "vosk-model-cn-0.15.zip",  # 大模型，约1.7GB
    "vosk-model-small-cn-0.22.zip",  # 小模型，约44MB
]

# 音频配置
SAMPLE_RATE = 16000  # 采样率
CHANNELS = 1         # 通道数
FRAME_SIZE = 2048    # 读取音频的帧大小

# 识别配置
MAX_RECOGNITION_SECONDS = 20       # 最大识别时间(秒)
SILENCE_THRESHOLD = 1500.0         # 静默阈值，用于检测语音结束
MIN_BUFFER_SIZE = SAMPLE_RATE * 1  # 最小缓冲区大小(1秒音频)
SPEECH_TIMEOUT = 6              # 停止说话多少秒后结束识别
BUFFER_PROCESS_INTERVAL = 1.5      # 缓冲区处理间隔(秒)
FORCED_END_SECONDS = 5             # 检测到持续语音超过此时间将强制结束
MIN_SPEECH_SECONDS = 5             # 最少需要录制的语音时间(秒)
VOLUME_RISE_THRESHOLD = 1.5          # 音量突然上升倍数，用于检测说话开始
CALIBRATION_TIME_SECONDS = 1.0     # 环境校准时间(秒)

# 调试配置
DEBUG_FILE = "debug_audio.raw"     # 调试音频文件名

# 添加一个新的配置项，指定外部已解压的模型路径
EXTERNAL_MODEL_PATH = "/home/ros/xiazai/mox"  # 用户已解压的模型所在路径

class VoskSpeechRecognition:
    """
    基于Vosk的语音识别类，集成了ROS功能
    
    该类是语音识别系统的核心，负责:
    1. 加载和管理Vosk语音识别模型
    2. 初始化和管理麦克风设备
    3. 实现语音活动检测(VAD)算法
    4. 处理和识别音频数据
    5. 通过ROS与其他节点交互
    
    系统设计特点:
    -------------
    1. 双触发模式: 
       - 手动触发(按Enter键)用于调试
       - ROS话题触发用于正常工作模式
    2. 标志位同步设计:
       - 使用ros_triggered标志位避免并发访问问题
       - 所有语音处理都在主循环中进行，避免回调函数直接处理音频
    3. 自适应语音检测:
       - 实时校准环境噪音
       - 动态调整静默阈值
    4. 错误恢复机制:
       - 自动尝试不同的音频配置
       - 失败时重新初始化设备
    
    使用方法:
    --------
    1. 创建实例: node = VoskSpeechRecognition()
    2. 启动主循环: node.run()
    3. 通过向'/mic/awake/angle'话题发送消息触发语音识别
    """
    def __init__(self):
        """
        初始化语音识别系统
        
        完成以下任务:
        1. 初始化ROS节点和发布/订阅关系
        2. 设置各种状态标志位
        3. 加载Vosk语音识别模型
        4. 初始化麦克风设备
        
        注意:
        - 模型加载可能需要较长时间，取决于模型大小
        - 麦克风初始化会自动搜索合适的设备
        """
        # ROS节点初始化
        rospy.init_node('voiceRecognition', anonymous=True)
        
        # 发布器：语音识别结果
        self.voice_pub = rospy.Publisher('chatter', String, queue_size=10)
        
        # 订阅器：麦克风唤醒角度
        rospy.Subscriber('/mic/awake/angle', Int32, self.wakeup_callback)
        
        # 变量初始化
        self.wakeup_flag = False    # 唤醒标志
        self.result_flag = False    # 结果标志
        self.recognizer = None      # Vosk识别器
        self.audio = None           # PyAudio对象
        self.stream = None          # 音频流
        self.device_index = None    # 麦克风设备索引
        self.model_loaded = False   # 模型加载标志
        self.mic_initialized = False # 麦克风初始化标志
        self.is_listening = False   # 是否正在监听
        self.ros_triggered = False  # ROS话题触发标志
        
        # 显示日志
        rospy.loginfo("语音识别初始化完成")
        print("语音识别初始化完成")
        
        # 在初始化时就加载Vosk模型
        self.load_vosk_model()
        
        # 麦克风初始化
        self.init_microphone()
    
    """
    解压ZIP文件
    
    参数:
        zip_path (str): ZIP文件路径
        extract_dir (str): 解压目标目录
        
    返回:
        bool: 解压是否成功
    """
    def extract_zip(self, zip_path, extract_dir):

        try:
            with zipfile.ZipFile(zip_path, 'r') as zip_ref:
                zip_ref.extractall(extract_dir)
            print(f"成功解压模型文件: {zip_path}")
            return True
        except Exception as e:
            print(f"解压模型文件失败: {e}")
            return False
    """
    加载Vosk语音识别模型
    
    按以下顺序尝试加载模型:
    1. 从外部指定路径加载模型
    2. 查找已解压的模型目录
    3. 解压模型ZIP文件
    
    如果所有尝试都失败，则退出程序
    """
    def load_vosk_model(self):
  
        # 首先尝试从外部指定路径加载模型
        if EXTERNAL_MODEL_PATH and os.path.exists(EXTERNAL_MODEL_PATH):
            print(f"检查外部模型目录: {EXTERNAL_MODEL_PATH}")
            # 按优先级顺序检查模型
            model_names = [
                "vosk-model-cn-kaldi-multicn-0.15",  # 中型模型
                "vosk-model-cn-0.15",                # 大模型
                "vosk-model-small-cn-0.22"           # 小型模型
            ]
            
            for model_name in model_names:
                ext_model_path = os.path.join(EXTERNAL_MODEL_PATH, model_name)
                if os.path.exists(ext_model_path) and os.path.isdir(ext_model_path):
                    print(f"找到外部模型: {ext_model_path}")
                    
                    # 检查模型目录结构
                    dir_contents = os.listdir(ext_model_path)
                    if "am" in dir_contents or "conf" in dir_contents:
                        print("使用已解压的外部模型")
                        model_path = ext_model_path
                        
                        # 加载模型
                        try:
                            print(f"正在加载外部模型: {model_path}")
                            # 关键：加载模型
                            model = Model(model_path)
                            self.recognizer = KaldiRecognizer(model, SAMPLE_RATE)
                            self.recognizer.SetWords(True)  # 启用词识别
                            print(f"Vosk模型加载成功: {model_path}")
                            self.model_loaded = True
                            return  # 成功加载，直接返回
                        except Exception as e:
                            print(f"加载外部模型失败: {e}")
                            # 继续尝试其他方式
        
        # 如果外部模型加载失败，使用原来的方法
        # 检查常见模型路径，优先使用中型模型
        model_paths = MODEL_SEARCH_PATHS
        
        model_path = None
        # 查找有效模型路径
        for path in model_paths:
            if os.path.exists(path) and os.path.isdir(path):
                model_path = path
                print(f"找到Vosk模型目录: {model_path}")
                # 检查模型文件夹内容
                print(f"模型目录内容: {os.listdir(path)}")
                if "am" in os.listdir(path) or "conf" in os.listdir(path):
                    print("模型目录结构正确")
                break
        
        # 如果没找到已解压模型，尝试解压ZIP文件
        if not model_path:
            model_zips = MODEL_ZIP_FILES
            
            for zip_file in model_zips:
                zip_path = os.path.join(os.getcwd(), zip_file)
                if os.path.exists(zip_path):
                    print(f"找到模型压缩文件: {zip_path}")
                    # 创建解压目录
                    extract_dir = os.path.splitext(zip_file)[0]
                    if not os.path.exists(extract_dir):
                        os.makedirs(extract_dir)
                        print(f"创建解压目录: {extract_dir}")
                    
                    # 解压模型文件
                    print(f"开始解压模型文件 {zip_file}，这可能需要几分钟...")
                    if self.extract_zip(zip_path, os.getcwd()):
                        model_path = extract_dir
                        print(f"模型已解压到: {model_path}")
                        break
        
        # 如果仍然没有找到模型，退出程序
        if not model_path:
            print("错误: 找不到Vosk模型，请确保模型已下载到当前目录")
            print("可以从 https://alphacephei.com/vosk/models 下载中文模型")
            print("推荐下载: vosk-model-cn-0.15.zip (大模型，识别效果好)")
            sys.exit(1)
        
        # 加载模型
        try:
            print(f"正在加载模型: {model_path}")
            # 大模型加载可能需要更多内存和时间
            print("模型加载中，请耐心等待...")
            
            model = Model(model_path)
            self.recognizer = KaldiRecognizer(model, SAMPLE_RATE)
            self.recognizer.SetWords(True)  # 启用词识别
            print(f"Vosk模型加载成功: {model_path}")
            self.model_loaded = True
        except Exception as e:
            print(f"错误: 加载Vosk模型失败: {e}")
            import traceback
            traceback.print_exc()
            sys.exit(1)
    """
    查找可用的麦克风设备
        
    优先选择USB音频设备，如果没有找到，则使用默认设备
        
    返回:
        int: 麦克风设备索引，失败返回None
    """
    def find_microphone_device(self):
        p = pyaudio.PyAudio()
        device_index = None
        
        # 列出所有音频设备
        print("正在查找麦克风设备...")
        try:
            # 首先尝试查找USB音频设备
            for i in range(p.get_device_count()):
                try:
                    dev_info = p.get_device_info_by_index(i)
                    name = dev_info.get('name', '')
                    if dev_info.get('maxInputChannels', 0) > 0:
                        print(f"发现麦克风设备: {name} (index: {i})")
                        # 优先选择带有 "usb" 和 "audio" 的设备（可能是Yahboom麦克风阵列）
                        if "usb" in name.lower() and "audio" in name.lower():
                            device_index = i
                            print(f"选择USB设备: {name} (index: {i})")
                            break
                except Exception as e:
                    print(f"获取设备 {i} 信息出错: {e}")
            
            # 如果没找到USB设备，使用默认设备
            if device_index is None:
                try:
                    # 通常情况下，0是默认设备
                    device_index = 0
                    dev_info = p.get_device_info_by_index(device_index)
                    print(f"使用默认设备: {dev_info.get('name')} (index: {device_index})")
                except:
                    # 如果还是失败，尝试使用系统默认设备
                    try:
                        device_index = p.get_default_input_device_info().get('index')
                        dev_info = p.get_device_info_by_index(device_index)
                        print(f"使用系统默认麦克风: {dev_info.get('name')} (index: {device_index})")
                    except Exception as e:
                        print(f"获取默认设备信息失败: {e}")
        except Exception as e:
            print(f"查找麦克风设备出错: {e}")
        
        p.terminate()
        return device_index
    
    """
        初始化麦克风设备
        
        查找并保存麦克风设备索引，初始化PyAudio
        
        返回:
            bool: 初始化是否成功
    """
    def init_microphone(self):
        try:
            # 查找设备
            self.device_index = self.find_microphone_device()
            if self.device_index is None:
                print("错误: 未找到可用的麦克风设备")
                return False
            
            # 初始化PyAudio
            self.audio = pyaudio.PyAudio()
            
            # 尝试获取麦克风信息
            try:
                dev_info = self.audio.get_device_info_by_index(self.device_index)
                print(f"使用麦克风设备: {dev_info.get('name')}")
                print(f"通道数: {dev_info.get('maxInputChannels')}")
                print(f"采样率: {SAMPLE_RATE} Hz")
            except:
                print("无法获取设备详细信息，但将继续尝试打开麦克风")
            
            self.mic_initialized = True
            return True
        except Exception as e:
            print(f"错误: 初始化麦克风失败: {e}")
            return False
    """
        打开麦克风并开始录音
        
        尝试使用不同的通道数和采样率配置，确保麦克风能够成功打开
        
        返回:
            bool: 是否成功打开麦克风
    """       
    def open_microphone(self):
        
        # 确保先关闭之前的麦克风实例
        self.close_microphone()
        
        # 尝试用不同通道和设置打开麦克风
        for channels in [CHANNELS, 2]:  # 先尝试配置的通道数，然后是2通道
            for rate in [SAMPLE_RATE, 44100]:  # 先尝试配置的采样率，然后是44100
                try:
                    print(f"尝试使用 {channels} 个通道, {rate}Hz 打开麦克风...")
                    
                    if self.audio is None:
                        self.audio = pyaudio.PyAudio()
                    
                    self.stream = self.audio.open(
                        format=pyaudio.paInt16,
                        channels=channels,
                        rate=rate,
                        input=True,
                        input_device_index=self.device_index,
                        frames_per_buffer=FRAME_SIZE
                    )
                    
                    print(f"成功打开麦克风: {channels}通道, {rate}Hz")
                    return True
                except Exception as e:
                    print(f"尝试配置失败 ({channels}通道, {rate}Hz): {e}")
        
        # 都失败了，尝试使用默认配置（不指定设备索引）
        try:
            print("尝试使用系统默认音频配置...")
            self.audio = pyaudio.PyAudio()
            self.stream = self.audio.open(
                format=pyaudio.paInt16,
                channels=CHANNELS,
                rate=SAMPLE_RATE,
                input=True,
                # 不指定设备索引，让PyAudio使用系统默认设备
                frames_per_buffer=FRAME_SIZE
            )
            print("成功使用系统默认配置打开麦克风")
            return True
        except Exception as e:
            print(f"所有尝试均失败: {e}")
            return False
    
    def close_microphone(self):
        """
        关闭麦克风
        
        安全地关闭音频流和PyAudio实例
        """
        try:
            if self.stream:
                try:
                    self.stream.stop_stream()
                    self.stream.close()
                except Exception as e:
                    print(f"关闭音频流出错: {e}")
                self.stream = None
                print("麦克风流已关闭")
            
            if self.audio:
                try:
                    self.audio.terminate()
                except Exception as e:
                    print(f"终止PyAudio出错: {e}")
                self.audio = None
                print("PyAudio已终止")
        except Exception as e:
            print(f"关闭麦克风时出错: {e}")
    """
        录音并进行语音识别 - 采用一次性处理方式
        
        这是系统的核心函数，实现了以下功能:
        1. 环境噪音校准
        2. 语音活动检测(VAD)
        3. 语音数据收集
        4. 语音识别处理
        
        算法流程:
        ---------
        1. 环境校准阶段:
           - 收集环境噪音样本
           - 计算环境噪音基线
           - 设置自适应静默阈值
        
        2. 语音检测阶段:
           - 等待音量明显高于基线(说话开始)
           - 动态更新背景噪音基线
        
        3. 录音阶段:
           - 累积音频数据
           - 监测音量变化
           - 检测静默(说话结束)
        
        4. 结束条件判断:
           - 检测到足够长的静默
           - 或达到最大录音时间
           - 或持续语音超过强制结束时间
        
        5. 语音处理阶段:
           - 将收集的音频数据发送给Vosk识别器
           - 获取并解析识别结果
        
        关键算法:
        ---------
        1. 自适应静默阈值:
           静默阈值 = max(预设阈值, 环境噪音 * 1.2)
           
        2. 语音开始检测:
           当前音量 > 基线音量 * VOLUME_RISE_THRESHOLD
           
        3. 语音结束检测:
           连续静默帧数 > 10 且 静默时间 > SPEECH_TIMEOUT
        
        4. 强制结束机制:
           连续语音时间 > FORCED_END_SECONDS 且 检测到稳定背景噪音
        
        参数:
            max_seconds (int): 最大录音时间(秒)
            
        返回:
            str: 识别结果文本，失败返回空字符串
            
        调试技巧:
        ---------
        - 识别失败时，会将音频保存到DEBUG_FILE文件
        - 可以通过分析该文件来调试识别问题
    """    
    def recognize_speech(self, max_seconds=MAX_RECOGNITION_SECONDS):

        # 不再每次都重新打开麦克风，假定麦克风已经打开
        if not self.stream:
            print("麦克风未打开，尝试打开麦克风...")
            if not self.open_microphone():
                print("无法打开麦克风，放弃识别")
                return None
            else:
                print("麦克风打开成功")
        
        print("开始语音识别...")
        start_time = time.time()
        result_text = ""
        
        # 重置识别器状态
        # 防止上一次的数据，影响这一次的识别
        try:
            if hasattr(self, 'recognizer') and self.recognizer:
                self.recognizer.Reset()
        except Exception as e:
            print(f"重置识别器失败: {e}")
        
        # 用于累积音频数据的缓冲区
        audio_buffer = b''
        min_buffer_size = MIN_BUFFER_SIZE
        
        try:
            # 校准环境噪音
            print("校准环境噪音中，请保持安静...")
            calibration_start = time.time()
            env_volume_samples = []
            calibration_frames = 0
            
            # 收集环境音量样本
            while time.time() - calibration_start < CALIBRATION_TIME_SECONDS:
                # 不断循环获取时间，然后求平均值
                # 就是计算当前不说话是的平均音量
                try:
                    data = self.stream.read(FRAME_SIZE, exception_on_overflow=False)
                    calibration_frames += 1
                    
                    # 计算当前帧的音量
                    audio_data = np.frombuffer(data, dtype=np.int16)
                    volume_norm = np.linalg.norm(audio_data) / 100
                    env_volume_samples.append(volume_norm)
                    
                    # 显示校准进度
                    if calibration_frames % 5 == 0:
                        print(f"校准中: {time.time() - calibration_start:.1f}秒, 音量: {volume_norm:.2f}", end='\r')
                except Exception as e:
                    print(f"校准过程出错: {e}")
            
            # 计算环境噪音基线
            # 通过比较固定阈值，来获取相对于的动态阈值
            if env_volume_samples:
                env_noise_level = sum(env_volume_samples) / len(env_volume_samples)
                # 设置静默阈值为环境噪音的1.2倍
                adaptive_silence_threshold = max(SILENCE_THRESHOLD, env_noise_level * 1.2)
                print(f"\n环境噪音基线: {env_noise_level:.2f}, 设置静默阈值为: {adaptive_silence_threshold:.2f}")
            else:
                adaptive_silence_threshold = SILENCE_THRESHOLD
                print(f"\n无法校准环境噪音，使用默认静默阈值: {adaptive_silence_threshold:.2f}")
            
            # 添加进度指示
            print("请开始说话...")

            # 标志位清零
            dots = 0
            last_progress_time = time.time()
            last_voice_time = time.time()  # 上次检测到声音的时间
            speech_start_time = None      # 语音开始时间
            waiting_for_speech = True     # 是否正在等待语音输入
            speech_detected = False       # 是否检测到语音
            speech_ended = False          # 语音是否结束
            
            # 音量历史和检测器
            max_volume = 0
            volume_history = []  # 用于存储最近的音量值
            baseline_volume = env_noise_level if env_volume_samples else 0
            
            # 语音活动检测计数器
            speech_frames = 0
            silence_frames = 0
            continuous_speech_seconds = 0  # 连续说话时间
            speech_duration = 0            # 实际语音持续时间
            stable_noise_counter = 0       # 稳定噪音计数器
            """
            语音活动检测(VAD)循环状态机说明
            ================================================================================
            状态标志：
            waiting_for_speech : bool  初始True - 等待语音开始阶段
            speech_detected    : bool  初始False - 已检测到有效语音
            speech_ended       : bool  初始False - 应结束录音标志
            计数器说明：
            speech_frames      : int  连续语音帧计数 (音量>阈值时累加，否则清零)
            silence_frames     : int  连续静默帧计数 (音量<静默阈值时累加)
            stable_noise_counter : int 稳定背景噪音计数 (音量接近基线时累加)
            时间记录点：
            start_time         : float 录音开始绝对时间
            last_voice_time    : float 最后检测到语音的时间 (每次非静默帧更新)
            speech_start_time  : float 语音开始绝对时间 (首次检测到语音时设置)
            处理流程：
            ┌──────────────────────┐
            │      开始循环         │
            └──────────┬───────────┘
                        ▼
            ┌──────────────────────┐
            │ 读取音频帧 → 计算音量  │
            └──────────┬───────────┘
            │ 等待语音阶段? (waiting_for_speech) 
            ├─是→更新背景噪音基线 → 检查语音开始条件 → 达标则切换状态
            │
            └─否→将音频存入buffer → 检查静默阈值
                ├─低于阈值 → silence_frames+1 → 检查静默超时
                └─高于阈值 → 重置静默计数 → 更新last_voice_time
                        
            结束条件检查（任一满足即结束）：
            1. 静默超时: silence_frames > 10 且 (当前时间 - last_voice_time) > SPEECH_TIMEOUT
            2. 持续说话超时: (当前时间 - speech_start_time) > MAX_SPEECH_DURATION
            3. 总时长超限: (当前时间 - start_time) > MAX_RECORD_SECONDS
            数据流向：
            麦克风 → PyAudio流 → 分帧读取 → 音量计算 → 状态判断 → 有效语音存入audio_buffer
            ================================================================================
            """
            # 循环检测音频
            while time.time() - start_time < max_seconds and not speech_ended:
                # 每0.5秒打印一次进度指示
                if time.time() - last_progress_time > 0.5:
                    dots = (dots + 1) % 6
                    status = "等待说话" if waiting_for_speech else "正在录音"
                    print(f"{status}{'.' * dots}", end='\r')
                    last_progress_time = time.time()
                
                try:
                    # 读取音频数据
                    data = self.stream.read(FRAME_SIZE, exception_on_overflow=False)
                    data_len = len(data)
                    if data_len == 0:
                        print("警告: 读取到空音频数据")
                        continue
                    
                    # 计算当前帧的音量
                    audio_data = np.frombuffer(data, dtype=np.int16)
                    volume_norm = np.linalg.norm(audio_data) / 100
                    
                    # 更新音量历史
                    volume_history.append(volume_norm)
                    if len(volume_history) > 10:  # 保留最近10帧的音量历史
                        volume_history.pop(0)
                    
                    # 更新最大音量
                    if volume_norm > max_volume:
                        max_volume = volume_norm
                    
                    # 平滑音量值（取最近帧的平均值）
                    avg_volume = sum(volume_history) / len(volume_history) if volume_history else volume_norm
                    
                    # 如果没检测到语音，检查是否有音量突然上升(说话开始)
                    if waiting_for_speech:
                        # 比较当前音量与基线音量
                        if baseline_volume > 0 and avg_volume > (baseline_volume * VOLUME_RISE_THRESHOLD):
                            speech_frames += 1
                            if speech_frames > 3:  # 连续3帧都有明显音量上升
                                print(f"\n检测到音量突然上升，音量: {avg_volume:.2f}，基线: {baseline_volume:.2f}")
                                waiting_for_speech = False
                                speech_detected = True
                                speech_start_time = time.time()
                        else:
                            # 更新背景噪音基线(缓慢适应环境)
                            baseline_volume = baseline_volume * 0.95 + avg_volume * 0.05
                            speech_frames = 0
                    else:
                        # 已经检测到语音，判断是否为静默
                        if avg_volume < adaptive_silence_threshold:
                            silence_frames += 1
                            # 如果音量稳定在噪音水平一段时间，可能是环境噪音
                            if abs(avg_volume - baseline_volume) < 10:
                                stable_noise_counter += 1
                            else:
                                stable_noise_counter = 0
                        else:
                            silence_frames = 0
                            stable_noise_counter = 0
                            last_voice_time = time.time()
                    
                    # 累积所有音频数据，无论是否有声音
                    if speech_detected:
                        audio_buffer += data
                        
                        # 计算连续说话时间
                        if speech_start_time:
                            continuous_speech_seconds = time.time() - speech_start_time
                            
                        # 强制结束条件1：如果已经录音超过FORCED_END_SECONDS秒，且处于稳定背景噪音
                        if (continuous_speech_seconds > FORCED_END_SECONDS and 
                            stable_noise_counter > 10):
                            print(f"\n录音时间已达 {continuous_speech_seconds:.1f} 秒，检测到稳定背景噪音，结束录音")
                            speech_ended = True
                            break
                    
                    # 语音结束条件：检测到语音后，静默超过SPEECH_TIMEOUT秒
                    if (speech_detected and 
                        not waiting_for_speech and 
                        silence_frames > 10 and
                        time.time() - last_voice_time > SPEECH_TIMEOUT):
                        print(f"\n检测到语音结束 (静默: {time.time() - last_voice_time:.1f}秒)")
                        speech_ended = True
                        break
                    
                    # 输出调试信息（降低频率，只在录音过程中每秒输出一次）
                    current_buffer_seconds = len(audio_buffer)/SAMPLE_RATE if speech_detected else 0
                    if dots == 1:
                        if speech_detected and not waiting_for_speech:
                            print(f"音频数据: {current_buffer_seconds:.1f}秒, 音量: {avg_volume:.2f}/{max_volume:.2f}, 静默帧: {silence_frames}, 阈值: {adaptive_silence_threshold:.2f}")
                        else:
                            print(f"等待音量上升, 当前: {avg_volume:.2f}, 基线: {baseline_volume:.2f}, 需要: {baseline_volume * VOLUME_RISE_THRESHOLD:.2f}", end='\r')
                
                except Exception as e:
                    print(f"读取音频数据出错: {e}")
                    time.sleep(0.1)
                    continue
            
            # 退出录音循环原因
            if time.time() - start_time >= max_seconds:
                print(f"\n达到最大录音时间限制 ({max_seconds}秒)，强制结束")
                return
            
            # 检查是否有足够的音频数据
            if len(audio_buffer) > min_buffer_size:
                buffer_seconds = len(audio_buffer) / SAMPLE_RATE
                print(f"\n处理 {buffer_seconds:.1f} 秒的音频...")
                
                try:
                    # 一次性处理整个音频缓冲区
                    print("开始识别...")
                    process_start = time.time()
                    self.recognizer.AcceptWaveform(audio_buffer)
                    result_json = self.recognizer.FinalResult()
                    process_time = time.time() - process_start
                    print(f"识别处理用时: {process_time:.2f}秒")
                    
                    result = json.loads(result_json)
                    
                    if 'text' in result and result['text'].strip():
                        result_text = result['text']
                        print(f"识别结果: 【{result_text}】")
                    else:
                        print("未能识别出任何内容")
                        # 保存音频文件以便调试
                        try:
                            debug_file = DEBUG_FILE
                            with open(debug_file, "wb") as f:
                                f.write(audio_buffer)
                            print(f"已保存原始音频数据到 {debug_file} 供调试")
                        except Exception as e:
                            print(f"保存调试音频失败: {e}")
                except Exception as e:
                    print(f"处理语音识别结果出错: {e}")
            else:
                print("\n未检测到足够的语音数据")
        
        except Exception as e:
            print(f"错误: 语音识别过程出错: {e}")
            import traceback
            traceback.print_exc()
        finally:
            elapsed_time = time.time() - start_time
            print(f"识别过程结束，用时: {elapsed_time:.2f}秒")
            # 不再关闭麦克风，保持打开状态
            # self.close_microphone()
        
        return result_text
    
    def wakeup_callback(self, msg):
        """
        麦克风唤醒角度回调函数
        
        当接收到唤醒角度消息时，设置标志位
        
        设计说明:
        ---------
        注意这个函数采用了"标志位"设计模式，而不是直接在回调函数中处理语音。
        这种设计有以下几个关键优势:
        
        1. 避免并发问题:
           ROS回调可能发生在任何时间，如果直接在回调中操作麦克风等共享资源，
           可能与主循环产生竞争条件，导致系统崩溃。
           
        2. 简化错误处理:
           将实际的语音处理逻辑集中在主循环中，可以使用统一的错误处理流程，
           而不需要在每个回调中重复实现。
           
        3. 提高系统稳定性:
           即使语音处理过程出错，也不会影响ROS回调系统的正常运行。
        
        实现逻辑:
        ---------
        接收到唤醒信号后，只是简单地设置ros_triggered标志位为True，
        然后由主循环检测到这个标志位并执行实际的语音识别流程。
        
        参数:
            msg (Int32): ROS消息，包含麦克风唤醒角度
        """
        angle = msg.data
        rospy.loginfo(f"麦克风唤醒，角度: {angle}")
        print(f"麦克风唤醒，角度: {angle}")
        
        # 仅设置标志位，不执行任何音频操作
        self.ros_triggered = True
    
    def start_recognition(self):
        """
        开始语音识别流程
        
        执行语音识别并发布结果到ROS话题
        """
        if not self.wakeup_flag:
            return
        
        # 确保麦克风已准备就绪
        if not self.stream:
            print("麦克风已关闭，重新打开...")
            if not self.open_microphone():
                print("无法打开麦克风，放弃识别")
                return
        
        # 重置识别器状态
        try:
            if hasattr(self, 'recognizer') and self.recognizer:
                self.recognizer.Reset()
        except Exception as e:
            print(f"重置识别器失败: {e}")
        
        # 执行语音识别
        result_text = self.recognize_speech()
        
        # 如果有识别结果，发布到ROS话题
        if result_text:
            rospy.loginfo(f"识别结果: {result_text}")
            print(f"识别结果: {result_text}")
            msg = String()
            msg.data = result_text
            self.voice_pub.publish(msg)
    
    def test_recognition(self):
        """
        测试语音识别功能
        
        直接运行主循环进行测试
        """
        print("=" * 50)
        print("语音识别测试")
        print("=" * 50)
        
        # 直接开始运行主循环
        self.run()
    
    def run(self):
        """
        主循环 - 系统的核心运行逻辑
        
        该方法实现了系统的主要运行逻辑，包括:
        1. 处理用户手动触发(按Enter键)
        2. 处理ROS话题触发(通过标志位)
        3. 维护麦克风状态
        4. 执行语音识别
        5. 发布识别结果
        
        执行流程:
        ---------
        1. 初始化:
           - 打开麦克风
           - 显示欢迎信息
        
        2. 主循环:
           a) 检查ROS触发标志位:
              - 如果被设置，执行语音识别
              - 处理完成后重置标志位
              
           b) 处理用户输入:
              - 检查是否有键盘输入
              - 如果是Enter键，执行语音识别
              - 如果是'q'，退出程序
        
        3. 语音识别流程(无论手动还是ROS触发):
           - 重置音频流
           - 识别语音
           - 发布结果到ROS话题
        
        并发处理设计:
        -------------
        整个系统采用"标志位"模式处理并发:
        1. ROS回调只负责设置标志位
        2. 主循环负责检查标志位并执行实际操作
        3. 所有对共享资源的访问都在主循环中进行
        
        这种设计避免了由于并发访问共享资源(如麦克风)导致的崩溃问题。
        
        错误处理:
        ---------
        - 主循环捕获所有异常，确保程序不会因单次操作失败而退出
        - 麦克风出错时会自动尝试重新打开
        - 使用定期重置音频流的方式避免资源泄露
        
        退出条件:
        ---------
        1. 用户输入'q'
        2. ROS节点关闭(rospy.is_shutdown()返回True)
        3. 键盘中断(Ctrl+C)
        """
        rate = rospy.Rate(10)  # 10Hz
        
        try:
            print("\n" + "="*50)
            print("语音识别系统已启动并预热完成")
            print("麦克风已开启，模型已加载")
            print("按Enter键开始录音识别，输入q退出")
            print("="*50)
            
            # 预先打开麦克风以加快响应速度
            if not self.mic_initialized or not self.stream:
                print("首次打开麦克风...")
                self.open_microphone()
            
            running = True
            # 初始打印提示信息
            print("\n准备好了吗？按Enter开始录音，输入'q'退出: ")
            
            while running and not rospy.is_shutdown():
                # 检测ROS触发标志
                if self.ros_triggered:
                    print("\nROS话题触发语音识别...")
                    self.ros_triggered = False  # 重置标志位
                    
                    # 清空缓冲区数据
                    if self.stream:
                        try:
                            # 丢弃所有待处理的音频数据
                            self.stream.stop_stream()
                            self.stream.start_stream()
                        except Exception as e:
                            print(f"重置音频流失败: {e}")
                            # 如果失败，尝试重新打开麦克风
                            self.close_microphone()
                            self.open_microphone()
                    else:
                        # 如果流未打开，尝试打开麦克风
                        print("麦克风未打开，尝试打开麦克风...")
                        self.open_microphone()
                    
                    # 重置识别器状态
                    try:
                        if hasattr(self, 'recognizer') and self.recognizer:
                            self.recognizer.Reset()
                    except Exception as e:
                        print(f"重置识别器失败: {e}")
                    
                    # 执行语音识别
                    result = self.recognize_speech()
                    
                    if result:
                        print(f"\n识别结果: 【{result}】\n")
                        # 发布到ROS话题
                        msg = String()
                        msg.data = result
                        self.voice_pub.publish(msg)
                    else:
                        print("\n未能识别语音内容\n")
                    
                    # 识别完成后再次打印提示
                    print("\n准备好了吗？按Enter开始录音，输入'q'退出: ")
                
                # 清空缓冲区数据
                if self.stream:
                    try:
                        # 丢弃所有待处理的音频数据
                        self.stream.stop_stream()
                        self.stream.start_stream()
                    except Exception as e:
                        print(f"重置音频流失败: {e}")
                        # 如果失败，尝试重新打开麦克风
                        self.close_microphone()
                        self.open_microphone()
                
                # 等待用户输入，但不阻塞太久
                import select
                import sys
                
                # 不在每次循环都打印提示
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                
                if ready:
                    user_input = sys.stdin.readline().strip()
                    if user_input.lower() == 'q':
                        print("退出语音识别...")
                        running = False
                    else:
                        # 不需要重新初始化，麦克风已经打开
                        if not self.stream:
                            print("麦克风已关闭，重新打开...")
                            self.open_microphone()
                        else:
                            print("麦克风已准备就绪")
                        
                        # 重置识别器状态
                        try:
                            if hasattr(self, 'recognizer') and self.recognizer:
                                self.recognizer.Reset()
                        except Exception as e:
                            print(f"重置识别器失败: {e}")
                            
                        # 执行语音识别
                        result = self.recognize_speech()
                        
                        if result:
                            print(f"\n识别结果: 【{result}】\n")
                            # 发布到ROS话题
                            msg = String()
                            msg.data = result
                            self.voice_pub.publish(msg)
                        else:
                            print("\n未能识别语音内容\n")
                        
                        # 识别完成后再次打印提示
                        print("\n准备好了吗？按Enter开始录音，输入'q'退出: ")
                
                # 简短延迟避免CPU过度使用
                rate.sleep()
                
        except KeyboardInterrupt:
            print("\n语音识别已停止")
        finally:
            # 最终关闭麦克风
            self.close_microphone()

if __name__ == '__main__':
    """
    程序入口点
    
    执行流程:
    1. 创建VoskSpeechRecognition实例
    2. 运行主循环
    3. 捕获异常并优雅地退出
    
    使用方法:
    $ python3 voice_recognition.py
    
    或者通过ROS启动:
    $ rosrun language voice_recognition.py
    """
    try:
        print("=" * 50)
        print("语音识别系统启动中...")
        print("模型加载可能需要一段时间，请耐心等待")
        print("=" * 50)
        
        node = VoskSpeechRecognition()
        node.run()  # 直接运行主循环，不再使用test_recognition
    except Exception as e:
        print(f"错误: {e}")
    except rospy.ROSInterruptException:
        pass 

"""
系统总结
=======

该语音识别系统是教育机器人的关键组件，提供了离线的中文语音识别能力。

关键特性:
1. 完全离线运行 - 不依赖网络连接
2. 自适应语音检测 - 自动适应不同环境噪音
3. 双触发模式 - 支持手动和ROS消息触发
4. 强健的错误处理 - 能够从各种异常中恢复

系统限制:
1. 语音识别精度受限于Vosk模型能力
2. 对特定领域术语识别可能不准确
3. 在高噪音环境下性能可能下降

扩展建议:
1. 添加关键词激活功能
2. 集成语义理解模块
3. 添加参数自动调优
4. 实现多麦克风阵列波束形成

维护提示:
1. 定期检查ROS话题连接
2. 监控CPU使用率，如过高考虑使用小模型
3. 检查麦克风设备稳定性
4. 如有识别问题，分析debug_audio.raw文件

© 2023 Educational Robot Team
""" 