'''
Author: ${git_name} <${git_email}>
Date: 2025-01-23 10:30:39
LastEditors: ${git_name} <${git_email}>
LastEditTime: 2025-01-23 11:12:22
FilePath: /educational_robot/src/xinhuo/scripts/Sparktxtadd.py
'''

#! /usr/bin/env python3
# coding: utf-8

"""
对话历史管理模块
该模块负责管理与星火大模型API交互的对话历史，
包括添加对话内容、控制对话历史长度以及清空内容等功能
"""

# 初始化对话历史，包含系统角色设定
# 系统角色为李白，设定其性格特点
text = [{"role": "system", "content": "你现在扮演李白，你豪情万丈，狂放不羁；接下来请用李白的口吻和用户对话。"}]

def getText(role, content):
    """
    将新的对话内容添加到对话历史中
    
    参数:
        role (str): 对话角色，可以是"system"、"user"或"assistant"
        content (str): 对话内容
        
    返回:
        list: 更新后的完整对话历史
    """
    # 创建符合星火API格式的对话记录
    jsoncon = {"role": role, "content": content}
    # 添加到对话历史列表
    text.append(jsoncon)
    return text

def getlength(text):
    """
    计算当前对话历史的总字符数
    
    参数:
        text (list): 对话历史列表
        
    返回:
        int: 所有对话内容的总字符数
    """
    length = 0
    for content in text:
        length += len(content["content"])
    return length

def checklen(text):
    """
    检查对话历史长度并控制在限制范围内
    当总字符数超过8000时，从头部删除较早的对话记录
    
    参数:
        text (list): 对话历史列表
        
    返回:
        list: 处理后的对话历史列表
    """
    # 当对话历史总长度超过8000字符时，移除最早的对话
    while getlength(text) > 8000:
        del text[0]
    return text

def clear_file(file_path):
    """
    清空指定文件内容
    
    参数:
        file_path (str): 文件路径
    """
    with open(file_path, 'w', encoding='utf-8') as file:
        file.truncate(0)