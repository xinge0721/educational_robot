#!/usr/bin/env python3
# coding: utf-8

import rospy
from std_msgs.msg import String
import requests
import json
import hmac
import base64
import hashlib
import datetime
import time
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time

# 全局变量存储完整回答
full_answer = ""
pub_tts = None

# 生成认证URL所需的参数
class Http_Param(object):
    def __init__(self, APPID, APIKey, APISecret, api_url):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.api_url = api_url
        self.host = api_url.split('https://')[1].split('/')[0]
        self.path = '/' + '/'.join(api_url.split('https://')[1].split('/')[1:])

    def create_auth_params(self):
        now = datetime.datetime.now()
        date = format_date_time(time.mktime(now.timetuple()))
        signature_origin = f"host: {self.host}\ndate: {date}\nPOST {self.path} HTTP/1.1"
        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'), digestmod=hashlib.sha256).digest()
        signature_sha_base64 = base64.b64encode(signature_sha).decode('utf-8')
        authorization_origin = f'api_key="{self.APIKey}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature_sha_base64}"'
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8')
        return {
            "authorization": authorization,
            "date": date,
            "host": self.host
        }

# 生成HTTP请求参数
def gen_params(appid, domain, question):
    messages = []
    for item in question:
        messages.append({
            "role": item["role"],
            "content": item["content"]
        })
    
    return {
        "header": {"app_id": appid, "uid": "1234"},
        "parameter": {
            "chat": {
                "domain": domain,
                "temperature": 0.8,
                "max_tokens": 2048,
                "top_k": 5,
                "auditing": "default"
            }
        },
        "payload": {
            "message": {
                "text": messages
            }
        }
    }

# 发送HTTP请求到星火API
def send_http_request(appid, api_key, api_secret, api_url, domain, question):
    global full_answer, pub_tts
    
    # 初始化发布器
    if pub_tts is None:
        pub_tts = rospy.Publisher('/tts/text', String, queue_size=10)
        rospy.sleep(0.5)  # 等待发布器初始化完成
    
    # 创建认证参数
    http_param = Http_Param(appid, api_key, api_secret, api_url)
    auth_params = http_param.create_auth_params()
    
    # 构建请求头
    headers = {
        "Content-Type": "application/json",
        "Authorization": f'Bearer {auth_params["authorization"]}',
        "Date": auth_params["date"],
        "Host": auth_params["host"]
    }
    
    # 构建请求体
    payload = gen_params(appid, domain, question)
    
    try:
        # 发送HTTP POST请求
        response = requests.post(api_url, headers=headers, json=payload)
        
        # 解析响应
        if response.status_code == 200:
            result = response.json()
            
            # 检查是否有错误
            if result["header"]["code"] != 0:
                rospy.logerr(f"API请求错误: {result}")
                return None
            
            # 获取回答内容
            answer = result["payload"]["choices"]["text"][0]["content"]
            full_answer = answer
            
            # 直接发送回答内容
            msg = String()
            msg.data = answer
            pub_tts.publish(msg)
            rospy.loginfo(f"星火回答: {answer}")
            
            # 等待一小段时间确保TTS有时间处理
            rospy.sleep(0.5)
            
            # 发送对话结束标志
            end_msg = String()
            end_msg.data = "对话已结束"
            pub_tts.publish(end_msg)
            rospy.loginfo("已发送对话结束标志")
            
            return full_answer
        else:
            rospy.logerr(f"HTTP请求失败，状态码: {response.status_code}")
            rospy.logerr(f"错误信息: {response.text}")
            return None
    except Exception as e:
        rospy.logerr(f"发送HTTP请求时发生错误: {str(e)}")
        return None

# 主函数，供外部调用
def main(appid, api_key, api_secret, api_url, domain, question):
    return send_http_request(appid, api_key, api_secret, api_url, domain, question) 