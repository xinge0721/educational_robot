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
        "payload": {"message": {"text": question}}
    }

# WebSocket请求参数
class Ws_Param(object):
    def __init__(self, APPID, APIKey, APISecret, Spark_url):
        self.APPID = APPID
        self.APIKey = APIKey
        self.APISecret = APISecret
        self.host = urlparse(Spark_url).netloc
        self.path = urlparse(Spark_url).path
        self.Spark_url = Spark_url

    def create_url(self):
        now = datetime.datetime.now()
        date = format_date_time(time.mktime(now.timetuple()))
        signature_origin = f"host: {self.host}\ndate: {date}\nGET {self.path} HTTP/1.1"
        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'), digestmod=hashlib.sha256).digest()
        signature_sha_base64 = base64.b64encode(signature_sha).decode('utf-8')
        authorization_origin = f'api_key="{self.APIKey}", algorithm="hmac-sha256", headers="host date request-line", signature="{signature_sha_base64}"'
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode('utf-8')
        v = {"authorization": authorization, "date": date, "host": self.host}
        return self.Spark_url + '?' + urlencode(v)


# 初始化全局变量，用于保存完整的AI回答
full_answer = ""
pub_tts = None

# WebSocket消息处理函数
def on_message(ws, message):
    """
    处理接收到的消息并打印到终端
    """
    global full_answer, pub_tts # 使用全局变量来保存完整的回答

    # 解析消息
    data = json.loads(message)
    code = data['header']['code']

    # 如果返回错误码，关闭连接
    if code != 0:
        print(f'请求错误: {code}, {data}')
        ws.close()
    else:
        # 获取 session id (sid)
        global sid
        sid = data["header"]["sid"]
        
        # 获取消息中的内容
        choices = data["payload"]["choices"]
        status = choices["status"]
        content = choices["text"][0]["content"]
        
        # 拼接收到的每一部分内容
        full_answer += content
        

        # 如果 status 为 2，表示回答完成，关闭 WebSocket 连接
        if status == 2:
        # 确保发布器初始化
            if pub_tts is None:
                pub_tts = rospy.Publisher('/tts/text', String, queue_size=10)
            
            # 构造ROS消息并发布
            msg = String()
            msg.data = full_answer
            pub_tts.publish(msg)
            
            # 后续原有处理保持不变
            print("星火回答:", full_answer)
            full_answer = ""


# WebSocket错误处理
def on_error(ws, error):
    print(f"WebSocket 错误: {error}")

# WebSocket关闭处理
def on_close(ws, one, two):
    print(" ")

# WebSocket连接建立时处理
def on_open(ws):
    thread.start_new_thread(run, (ws,))

# WebSocket发送数据
def run(ws):
    data = json.dumps(gen_params(appid=ws.appid, domain=ws.domain, question=ws.question))
    ws.send(data)

# 主函数，负责创建 WebSocket 连接并发送请求
def main(appid, api_key, api_secret, Spark_url, domain, question):
    ws_param = Ws_Param(appid, api_key, api_secret, Spark_url)
    ws_url = ws_param.create_url()
    ws = websocket.WebSocketApp(ws_url, on_message=on_message, on_error=on_error, on_close=on_close, on_open=on_open)
    ws.appid = appid
    ws.question = question
    ws.domain = domain
    ws.run_forever(sslopt={"cert_reqs": ssl.CERT_NONE})
