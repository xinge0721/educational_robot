'''
Author: ${git_name} <${git_email}>
Date: 2025-01-23 10:28:51
LastEditors: ${git_name} <${git_email}>
LastEditTime: 2025-01-23 20:27:14
FilePath: /educational_robot/src/xinhuo/scripts/SparkPy.py
'''
#! /usr/bin/env python3
# coding: utf-8
import rospy
from std_msgs.msg import String


#虽然不知道为什么但是这么作了，就能正常使用了，ros就能正常识别了
import os
import sys
# 获取当前工作目录的绝对路径
path = os.path.abspath(".")
# 将自定义模块的路径添加到 Python 搜索路径中
sys.path.insert(0, path+'/src/xinhuo/scripts')

import Sparktext 
import MyWebSocket

# 以下密钥信息从控制台获取 https://console.xfyun.cn/services/bm35
appid = "68a31b42"     # 填写控制台中获取的 APPID 信息
api_secret = "NGFhMDdmOGZhZTE1YjViZGNhNWVmOGIy"   # 填写控制台中获取的 APISecret 信息
api_key = "902b4931840c29d0aa07250d3b518159"    # 填写控制台中获取的 APIKey 信息
domain = "4.0Ultra"       # 4.0Ultra版本
Spark_url = "wss://spark-api.xf-yun.com/v4.0/chat"  # 4.0Ultra服务地址

# 处理传入的消息，调用 SparkApi 获取回答并更新对话历史
def doMsg(msg):
    user_input = msg.data
    rospy.loginfo(f"用户输入: {user_input}")

    question = Sparktext.checklen(Sparktext.getText("user", user_input))
    MyWebSocket.main(appid, api_key, api_secret, Spark_url, domain, question)
    Sparktext.getText("assistant", "AI's response goes here...")

# 初始化 ROS 节点并订阅 chatter 话题
if __name__ == '__main__':
    rospy.init_node("wenta")
    rospy.Subscriber("chatter", String, doMsg, queue_size=100)
    rospy.spin()
