'''
Author: ${git_name} <${git_email}>
Date: 2025-01-23 10:28:51
LastEditors: ${git_name} <${git_email}>
LastEditTime: 2025-01-23 20:27:14
FilePath: /educational_robot/src/xinhuo/scripts/SparkPy.py
'''
#! /usr/bin/env python3
# coding: utf-8

"""
星火大模型ROS节点
该节点订阅ROS话题接收用户输入，通过星火大模型API生成回答，
并将回答通过另一个ROS话题发布出去，实现机器人的对话功能
"""

import rospy
from std_msgs.msg import String


#虽然不知道为什么,但是这么作了，就能正常使用了，ros就能正常识别了
import os
import sys
# 获取当前工作目录的绝对路径
path = os.path.abspath(".")
# 将自定义模块的路径添加到 Python 搜索路径中
sys.path.insert(0, path+'/src/xinhuo/scripts')

import Sparktext  # 导入处理对话历史的模块
import MyWebSocket  # 导入WebSocket连接模块

# 星火大模型API认证信息
# 这些信息从讯飞开放平台控制台获取: https://console.xfyun.cn/services/bm35
appid = "68a31b42"     # 应用ID
api_secret = "NGFhMDdmOGZhZTE1YjViZGNhNWVmOGIy"   # API密钥secret
api_key = "902b4931840c29d0aa07250d3b518159"    # API密钥
domain = "4.0Ultra"    # 使用的模型版本，4.0Ultra为最新版本
Spark_url = "wss://spark-api.xf-yun.com/v4.0/chat"  # API服务地址

# 处理传入的消息，调用星火大模型API获取回答
def doMsg(msg):
    """
    处理从ROS话题接收到的消息
    
    参数:
        msg (std_msgs.msg.String): 包含用户输入的ROS消息
    """
    # 获取用户输入文本
    user_input = msg.data
    rospy.loginfo(f"用户输入: {user_input}")

    # 将用户输入添加到对话历史并检查长度限制
    question = Sparktext.checklen(Sparktext.getText("user", user_input))
    
    # 调用WebSocket模块发送请求到星火大模型API
    # 回答将通过MyWebSocket中的on_message函数处理并发布到ROS话题
    MyWebSocket.main(appid, api_key, api_secret, Spark_url, domain, question)
    
    # 模拟添加AI回答到对话历史，实际回答将在WebSocket回调中处理
    Sparktext.getText("assistant", "AI's response goes here...")

# 主函数，初始化ROS节点并设置订阅者
if __name__ == '__main__':
    # 初始化ROS节点，名称为"wenta"
    rospy.init_node("wenta")
    
    # 订阅"chatter"话题，接收用户输入消息
    # 收到消息后调用doMsg函数处理
    rospy.Subscriber("chatter", String, doMsg, queue_size=100)
    
    # 进入ROS事件循环，等待消息
    rospy.spin()
