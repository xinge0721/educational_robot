#!/usr/bin/env python3
# coding: utf-8

"""
测试星火大模型HTTP API接口
此脚本可以直接运行，不依赖ROS环境
"""

import sys
import os
# 添加当前目录到搜索路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import requests
import json
import hmac
import base64
import hashlib
import datetime
import time
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time

# 配置参数
appid = "68a31b42"
api_secret = "NGFhMDdmOGZhZTE1YjViZGNhNWVmOGIy"
api_key = "902b4931840c29d0aa07250d3b518159"
domain = "4.0Ultra"
api_url = "https://spark-api-open.xf-yun.com/v1/chat/completions"

# HTTP认证参数生成
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

# 生成请求参数
def gen_params(appid, domain, messages):
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

# 发送HTTP请求
def send_request(messages):
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
    payload = gen_params(appid, domain, messages)
    
    try:
        # 发送请求
        print("正在发送请求...")
        response = requests.post(api_url, headers=headers, json=payload)
        
        # 打印响应状态
        print(f"响应状态码: {response.status_code}")
        
        if response.status_code == 200:
            result = response.json()
            
            if result["header"]["code"] != 0:
                print(f"API返回错误: {result}")
                return None
            
            # 获取回答
            answer = result["payload"]["choices"]["text"][0]["content"]
            
            # 模拟ROS节点的行为：直接发送答案
            print(f"\n星火回答: {answer}")
            
            # 模拟发送对话结束标志
            time.sleep(0.5)  # 短暂延迟，模拟实际环境
            print("\n对话已结束")
            
            return answer
        else:
            print(f"请求失败: {response.text}")
            return None
    except Exception as e:
        print(f"发生错误: {str(e)}")
        return None

# 主函数
def main():
    print("星火大模型HTTP API测试")
    print("输入'q'退出")
    
    # 初始化对话历史
    messages = [{"role": "system", "content": "你现在扮演一个老师，你通晓世界万物的知识；接下来请用老师的口吻，最简单的方法，教导用户。"}]
    
    while True:
        # 获取用户输入
        user_input = input("\n请输入问题: ")
        if user_input.lower() == 'q':
            break
        
        # 添加用户输入到对话历史
        messages.append({"role": "user", "content": user_input})
        
        # 发送请求并获取回答
        answer = send_request(messages)
        
        if answer:
            # 答案已在send_request中打印
            # 添加回答到对话历史
            messages.append({"role": "assistant", "content": answer})
            
            # 控制对话历史长度
            while sum(len(m["content"]) for m in messages) > 8000:
                if len(messages) <= 1:
                    break
                messages.pop(1)  # 保留system提示，删除最早的对话
        else:
            print("\n获取回答失败")

if __name__ == "__main__":
    main() 