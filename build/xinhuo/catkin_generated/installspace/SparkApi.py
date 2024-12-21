#! /usr/bin/env python3

import _thread as thread
import base64
import datetime
import hashlib
import hmac
import json
import time
from urllib.parse import urlparse
import ssl
from datetime import datetime
from time import mktime
from urllib.parse import urlencode
from wsgiref.handlers import format_date_time

import websocket  # 使用websocket_client库
answer = ""
sid = ''  # 用于存储会话ID

class Ws_Param(object):
    # 初始化 WebSocket 请求的参数
    def __init__(self, APPID, APIKey, APISecret, Spark_url):
        self.APPID = APPID  # 应用ID
        self.APIKey = APIKey  # API密钥
        self.APISecret = APISecret  # API密钥
        self.host = urlparse(Spark_url).netloc  # 解析 Spark API URL 获取主机地址
        self.path = urlparse(Spark_url).path  # 解析路径
        self.Spark_url = Spark_url  # 保存 Spark API 的完整 URL

    # 生成连接的 URL，包含认证信息
    def create_url(self):
        # 生成 RFC1123 格式的时间戳
        now = datetime.now()
        date = format_date_time(mktime(now.timetuple()))  # 当前时间格式化为 HTTP 日期

        # 拼接签名原始字符串，按照协议要求格式化
        signature_origin = "host: " + self.host + "\n"
        signature_origin += "date: " + date + "\n"
        signature_origin += "GET " + self.path + " HTTP/1.1"

        # 使用 HMAC-SHA256 对签名字符串进行加密
        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'),
                                 digestmod=hashlib.sha256).digest()

        # 对签名进行 Base64 编码
        signature_sha_base64 = base64.b64encode(signature_sha).decode(encoding='utf-8')

        # 构建认证头，包含API密钥、加密算法等
        authorization_origin = f'api_key="{self.APIKey}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature_sha_base64}"'

        # 对认证信息进行 Base64 编码
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8')

        # 生成用于请求的认证信息字典
        v = {
            "authorization": authorization,
            "date": date,
            "host": self.host
        }

        # 拼接认证参数，生成最终的 URL
        url = self.Spark_url + '?' + urlencode(v)
        return url  # 返回生成的 URL


# 收到 WebSocket 错误时的处理函数
def on_error(ws, error):
    print("### error:", error)


# 收到 WebSocket 关闭事件时的处理函数
def on_close(ws, one, two):
    print("WebSocket connection closed.")


# 收到 WebSocket 连接建立时的处理函数
def on_open(ws):
    # 连接成功后启动一个新的线程进行数据传输
    thread.start_new_thread(run, (ws,))


def run(ws, *args):
    # 在 WebSocket 连接成功后，准备发送请求
    data = json.dumps(gen_params(appid=ws.appid, domain=ws.domain, question=ws.question))
    ws.send(data)  # 发送请求数据


# 收到 WebSocket 消息时的处理函数
def on_message(ws, message):
    # 解析收到的消息，转换为 JSON 格式
    data = json.loads(message)
    code = data['header']['code']  # 提取返回的错误码

    # 如果返回错误码不为 0，表示请求失败，关闭连接
    if code != 0:
        print(f'请求错误: {code}, {data}')
        ws.close()
    else:
        # 获取会话 ID (sid)
        global sid
        sid = data["header"]["sid"]

        # 提取消息中的内容
        choices = data["payload"]["choices"]
        status = choices["status"]
        content = choices["text"][0]["content"]

        # 打印返回的内容
        print(content, end="")

        # 将 AI 的回答追加到文件中
        file_path = '../data/回答.txt'
        with open(file_path, 'a', encoding='utf-8') as file:  # 以追加模式打开文件
            file.write(content)  # 追加内容到文件

        # 如果 status 为 2，表示回答完成，关闭 WebSocket 连接
        if status == 2:
            with open(file_path, 'a', encoding='utf-8') as file:
                file.write("\nEND_OF_ANSWER\n")  # 追加结束标志
            ws.close()


# 生成请求参数的函数
def gen_params(appid, domain, question):
    """
    通过appid和用户提问生成请求的参数
    """
    data = {
        "header": {
            "app_id": appid,  # 使用的应用ID
            "uid": "1234"  # 用户ID（示例）
        },
        "parameter": {
            "chat": {
                "domain": domain,  # 领域，4.0Ultra版本的应用领域
                "temperature": 0.8,  # 模型生成的多样性，值越高输出越多样
                "max_tokens": 2048,  # 最大token数
                "top_k": 5,  # 输出的前k个候选文本
                "auditing": "default"  # 默认审查级别
            }
        },
        "payload": {
            "message": {
                "text": question  # 用户输入的问题
            }
        }
    }
    return data  # 返回生成的参数


# 主函数，负责创建 WebSocket 连接并发送请求
def main(appid, api_key, api_secret, Spark_url, domain, question):
    # 初始化 WebSocket 参数
    wsParam = Ws_Param(appid, api_key, api_secret, Spark_url)
    websocket.enableTrace(False)  # 禁用 WebSocket 调试信息
    wsUrl = wsParam.create_url()  # 创建 WebSocket 连接 URL
    # 创建 WebSocket 客户端应用，并设置消息处理回调函数
    ws = websocket.WebSocketApp(wsUrl, on_message=on_message, on_error=on_error, on_close=on_close, on_open=on_open)
    ws.appid = appid  # 保存 appid
    ws.question = question  # 保存用户问题
    ws.domain = domain  # 保存领域信息
    # 启动 WebSocket 客户端，开启连接并等待消息
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
