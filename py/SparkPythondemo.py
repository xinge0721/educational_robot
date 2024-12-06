# coding: utf-8
import SparkApi
import time
#以下密钥信息从控制台获取   https://console.xfyun.cn/services/bm35
appid = "68a31b42"     #填写控制台中获取的 APPID 信息
api_secret = "NGFhMDdmOGZhZTE1YjViZGNhNWVmOGIy"   #填写控制台中获取的 APISecret 信息
api_key ="902b4931840c29d0aa07250d3b518159"    #填写控制台中获取的 APIKey 信息

#domain = "generalv3.5"      # Max版本
domain = "4.0Ultra"       # 4.0Ultra版本
#domain = "generalv3"       # Pro版本
#domain = "lite"         # Lite版本

#Spark_url = "wss://spark-api.xf-yun.com/v3.5/chat"   # Max服务地址
Spark_url = "wss://spark-api.xf-yun.com/v4.0/chat"  # 4.0Ultra服务地址
#Spark_url = "wss://spark-api.xf-yun.com/v3.1/chat"  # Pro服务地址
#Spark_url = "wss://spark-api.xf-yun.com/v1.1/chat"  # Lite服务地址

#初始上下文内容，当前可传system、user、assistant 等角色
text =[
     {"role": "system", "content": "你现在扮演李白，你豪情万丈，狂放不羁；接下来请用李白的口吻和用户对话。"} , # 设置对话背景或者模型角色
    # {"role": "user", "content": "你是谁"},  # 用户的历史问题
    # {"role": "assistant", "content": "....."} , # AI的历史回答结果
    # # ....... 省略的历史对话
    # {"role": "user", "content": "你会做什么"}  # 最新的一条问题，如无需上下文，可只传最新一条问题
]


def getText(role,content):
    jsoncon = {}
    jsoncon["role"] = role
    jsoncon["content"] = content
    text.append(jsoncon)
    return text

def getlength(text):
    length = 0
    for content in text:
        temp = content["content"]
        leng = len(temp)
        length += leng
    return length

def checklen(text):
    while (getlength(text) > 8000):
        del text[0]
    return text
    

import os

def clear_file(file_path):
    """清空文件内容"""
    with open(file_path, 'w', encoding='utf-8') as file:
        file.truncate(0)

if __name__ == '__main__':
    # 设定1.txt文件的绝对路径
    file_path = '../data/问题.txt'
    print("我：")
    while True:
        # 从1.txt文件读取内容
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                Input = file.read().strip()  # 读取并去除多余的空白字符
        except FileNotFoundError:
            print(f"文件 {file_path} 未找到")
            continue

        if not Input:  # 如果1.txt为空，则跳过本次循环
            continue

        # 处理输入的内容
        question = checklen(getText("user", Input))
        SparkApi.answer = ""

        print("星火:", end="")
        SparkApi.main(appid, api_key, api_secret, Spark_url, domain, question)

        # 将生成的回答传递给getText
        getText("assistant", SparkApi.answer)

        # 清空1.txt文件内容
        clear_file(file_path)


