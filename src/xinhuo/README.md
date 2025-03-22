# 星火大模型ROS接口

本模块提供了两种与科大讯飞星火大模型交互的方式：WebSocket接口和HTTP接口。

## 功能介绍

- 语音识别：利用科大讯飞语音识别将语音转为文字
- 大模型交互：将文字发送给星火大模型，获取回复
- 文本转语音：将大模型回复发送给TTS模块
- 对话结束标志：在回答结束时发送"对话已结束"消息，便于后续处理

## 文件组织结构

```
src/xinhuo/
├── scripts/
│   ├── http/              # HTTP API相关文件
│   │   ├── MyHttpApi.py   # HTTP API实现
│   │   ├── SparkHttpAPI.py # 使用HTTP API的ROS节点
│   │   └── test_http_api.py # HTTP API测试工具
│   ├── websocket/         # WebSocket API相关文件
│   │   ├── MyWebSocket.py  # WebSocket API实现
│   │   └── SparkWebSocket.py # 使用WebSocket的ROS节点
│   ├── common/            # 两种API共用的文件
│   │   └── Sparktext.py   # 对话历史管理
│   └── set_permissions.sh # 设置脚本权限
└── launch/
    ├── spark_http.launch     # 启动HTTP版本
    └── spark_websocket.launch # 启动WebSocket版本
```

## 使用方法

### HTTP API (推荐)

新的HTTP API提供了更稳定的连接和更简单的处理逻辑：

1. 启动HTTP版本：
```
roslaunch xinhuo spark_http.launch
```

2. 系统将订阅`chatter`话题，接收语音识别结果
3. 系统将通过HTTP API与星火大模型通信
4. 回复结果会被发布到`/tts/text`话题
5. 对话结束时会发送"对话已结束"消息到同一话题

### WebSocket API

原有的WebSocket API也已更新，支持相同的功能：

1. 启动WebSocket版本：
```
roslaunch xinhuo spark_websocket.launch
```

2. 功能与HTTP版本相同，只是使用不同的通信方式

### 独立测试

可以使用测试脚本独立测试HTTP API功能：

```
cd src/xinhuo/scripts/http
python3 test_http_api.py
```

## 配置说明

在`SparkHttpAPI.py`和`SparkWebSocket.py`中设置您的API密钥：

```python
appid = "您的APPID"
api_secret = "您的APISecret"
api_key = "您的APIKey"
```

## 特性

1. 对话历史管理：自动维护对话历史，确保上下文连贯
2. 动态响应：即时将大模型回答发送给TTS模块
3. 对话结束标志：在对话结束时发送特定标志，便于后续处理
4. 错误处理：完善的错误处理和日志记录

## 注意事项

1. HTTP API使用的是较新的4.0版本接口
2. 请确保您的API密钥具有相应权限
3. 两个版本使用相同的API密钥和对话管理逻辑，可以根据需要切换 