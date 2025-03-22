#!/usr/bin/env python3
# coding: utf-8
import rospy
from std_msgs.msg import String

# 导入路径设置
import os
import sys
# 获取当前工作目录的绝对路径
path = os.path.abspath(".")
# 将自定义模块的路径添加到 Python 搜索路径中
sys.path.insert(0, path+'/src/xinhuo/scripts')
sys.path.insert(0, path+'/src/xinhuo/scripts/http')
sys.path.insert(0, path+'/src/xinhuo/scripts/common')

from common import Sparktext 
import MyHttpApi  # 导入HTTP API处理模块

# 以下密钥信息从控制台获取 https://console.xfyun.cn/services/bm35
appid = "68a31b42"     # 填写控制台中获取的 APPID 信息
api_secret = "NGFhMDdmOGZhZTE1YjViZGNhNWVmOGIy"   # 填写控制台中获取的 APISecret 信息
api_key = "902b4931840c29d0aa07250d3b518159"    # 填写控制台中获取的 APIKey 信息
domain = "4.0Ultra"       # 4.0Ultra版本
Spark_url = "https://spark-api-open.xf-yun.com/v1/chat/completions"  # HTTP API地址

# 处理传入的消息，调用 HTTP API 获取回答并更新对话历史
def doMsg(msg):
    user_input = msg.data
    rospy.loginfo(f"用户输入: {user_input}")

    # 添加用户输入到对话历史
    question = Sparktext.checklen(Sparktext.getText("user", user_input))
    
    # 调用HTTP API获取回答
    # 注意：MyHttpApi.main会直接将回答和"对话已结束"发布到/tts/text话题
    response = MyHttpApi.main(appid, api_key, api_secret, Spark_url, domain, question)
    
    # 如果获取到回答，将其添加到对话历史
    if response:
        Sparktext.getText("assistant", response)
    else:
        rospy.logerr("无法获取星火大模型回答")
        # 添加一个占位回答以保持对话历史完整
        Sparktext.getText("assistant", "无法获取回答，请稍后再试。")

# 主函数，初始化ROS节点并订阅话题
if __name__ == '__main__':
    rospy.init_node("spark_http_node")
    rospy.loginfo("星火大模型HTTP API节点已启动")
    rospy.Subscriber("chatter", String, doMsg, queue_size=100)
    rospy.spin() 