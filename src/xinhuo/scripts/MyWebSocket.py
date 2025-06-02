"""
星火大模型API WebSocket连接模块
该模块实现了与讯飞星火大模型API的WebSocket连接，用于发送问题并接收AI回答
通过ROS话题发布AI回答，实现与其他ROS节点的集成
"""

# 在文件顶部添加ROS依赖
import rospy
from std_msgs.msg import String

import base64
import datetime
import hashlib
import hmac
import json
import time
from urllib.parse import urlparse, urlencode
from wsgiref.handlers import format_date_time
import websocket  # 使用websocket_client库
import _thread as thread
import ssl

# 初始上下文内容，当前可传system、user、assistant等角色

# 生成请求参数的函数
def gen_params(appid, domain, question):
    """
    生成星火大模型API请求参数
    
    参数:
        appid (str): 应用ID
        domain (str): 模型版本/领域，如"4.0Ultra"
        question (list): 对话历史记录，包含角色和内容
        
    返回:
        dict: 格式化的请求参数字典
    """
    return {
        "header": {"app_id": appid, "uid": "1234"},  # 请求头，包含应用ID和用户ID
        "parameter": {
            "chat": {
                "domain": domain,  # 模型版本/领域
                "temperature": 0.8,  # 温度参数，控制生成文本的随机性，越大越随机
                "max_tokens": 2048,  # 生成文本的最大长度
                "top_k": 5,  # 从k个候选中随机选择一个
                "auditing": "default"  # 审核设置
            }
        },
        "payload": {"message": {"text": question}}  # 消息内容，包含对话历史
    }

# WebSocket请求参数
class Ws_Param(object):
    """
    WebSocket连接参数类，用于生成带认证信息的WebSocket URL
    """
    def __init__(self, APPID, APIKey, APISecret, Spark_url):
        """
        初始化WebSocket参数
        
        参数:
            APPID (str): 应用ID
            APIKey (str): API密钥
            APISecret (str): API密钥secret
            Spark_url (str): 星火API WebSocket URL
        """
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.host = urlparse(Spark_url).netloc  # 获取主机名
        self.path = urlparse(Spark_url).path    # 获取路径
        self.Spark_url = Spark_url

    def create_url(self):
        """
        创建带认证信息的WebSocket URL
        
        返回:
            str: 完整的带认证参数的WebSocket URL
        """
        # 生成RFC1123格式的时间戳
        now = datetime.datetime.now()
        date = format_date_time(time.mktime(now.timetuple()))
        
        # 构建签名原文
        signature_origin = f"host: {self.host}\ndate: {date}\nGET {self.path} HTTP/1.1"
        
        # 使用HMAC-SHA256算法结合APISecret对签名原文进行加密
        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'), digestmod=hashlib.sha256).digest()
        signature_sha_base64 = base64.b64encode(signature_sha).decode('utf-8')
        
        # 构建授权原文并进行base64编码
        authorization_origin = f'api_key="{self.APIKey}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature_sha_base64}"'
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8')
        
        # 将认证信息添加到URL参数中
        v = {"authorization": authorization, "date": date, "host": self.host}
        return self.Spark_url + '?' + urlencode(v)


# 初始化全局变量，用于保存完整的AI回答
full_answer = ""
pub_tts = None

# WebSocket消息处理函数
def on_message(ws, message):
    """
    处理从WebSocket接收到的消息
    
    参数:
        ws: WebSocket实例
        message: 接收到的消息内容
    """
    global full_answer, pub_tts # 使用全局变量来保存完整的回答

    # 解析JSON消息
    data = json.loads(message)
    code = data['header']['code']

    # 如果返回错误码，关闭连接
    if code != 0:
        print(f'请求错误: {code}, {data}')
        ws.close()
    else:
        # 获取会话ID (sid)
        global sid
        sid = data["header"]["sid"]
        
        # 从返回数据中提取内容
        choices = data["payload"]["choices"]
        status = choices["status"]  # 状态：0-开始，1-继续，2-结束
        content = choices["text"][0]["content"]  # 实际内容
        
        # 拼接收到的每一部分内容（大模型回答可能分多次返回）
        full_answer += content
        
        # 如果status为2，表示回答完成，发布到ROS话题并关闭连接
        if status == 2:
            # 确保ROS发布器已初始化
            if pub_tts is None:
                pub_tts = rospy.Publisher('/tts/text', String, queue_size=10)
            
            # 构造ROS消息并发布到/tts/text话题
            msg = String()
            msg.data = full_answer
            pub_tts.publish(msg)
            
            # 打印完整回答并重置
            print("星火回答:", full_answer)
            full_answer = ""


# WebSocket错误处理
def on_error(ws, error):
    """
    WebSocket错误处理函数
    
    参数:
        ws: WebSocket实例
        error: 错误信息
    """
    print(f"WebSocket 错误: {error}")

# WebSocket关闭处理
def on_close(ws, one, two):
    """
    WebSocket连接关闭处理函数
    
    参数:
        ws: WebSocket实例
        one, two: 关闭状态参数
    """
    print(" ")

# WebSocket连接建立时处理
def on_open(ws):
    """
    WebSocket连接建立时的处理函数
    
    参数:
        ws: WebSocket实例
    """
    thread.start_new_thread(run, (ws,))

# WebSocket发送数据
def run(ws):
    """
    在新线程中向WebSocket发送数据
    
    参数:
        ws: WebSocket实例
    """
    # 生成请求参数并发送
    data = json.dumps(gen_params(appid=ws.appid, domain=ws.domain, question=ws.question))
    ws.send(data)

# 主函数，负责创建WebSocket连接并发送请求
def main(appid, api_key, api_secret, Spark_url, domain, question):
    """
    创建WebSocket连接并发送请求到星火大模型API
    
    参数:
        appid (str): 应用ID
        api_key (str): API密钥
        api_secret (str): API密钥secret
        Spark_url (str): 星火API WebSocket URL
        domain (str): 模型版本/领域
        question (list): 对话历史
    """
    # 创建WebSocket参数并获取URL
    ws_param = Ws_Param(appid, api_key, api_secret, Spark_url)
    ws_url = ws_param.create_url()
    
    # 创建WebSocket连接
    ws = websocket.WebSocketApp(ws_url, on_message=on_message, on_error=on_error, on_close=on_close, on_open=on_open)
    ws.appid = appid
    ws.question = question
    ws.domain = domain
    
    # 运行WebSocket连接（忽略SSL证书验证）
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
