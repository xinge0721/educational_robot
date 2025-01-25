'''
Author: ${git_name} <${git_email}>
Date: 2025-01-23 10:30:39
LastEditors: ${git_name} <${git_email}>
LastEditTime: 2025-01-23 11:12:22
FilePath: /educational_robot/src/xinhuo/scripts/Sparktxtadd.py
'''

#! /usr/bin/env python3
# coding: utf-8


text = [{"role": "system", "content": "你现在扮演李白，你豪情万丈，狂放不羁；接下来请用李白的口吻和用户对话。"}]

# 将角色和内容添加到对话历史中
def getText(role, content):
    jsoncon = {"role": role, "content": content}
    text.append(jsoncon)
    return text

# 计算当前对话历史内容的总字数
def getlength(text):
    length = 0
    for content in text:
        length += len(content["content"])
    return length

# 检查并控制对话历史的字数不超过限制
def checklen(text):
    while getlength(text) > 8000:
        del text[0]
    return text

# 清空文件内容
def clear_file(file_path):
    with open(file_path, 'w', encoding='utf-8') as file:
        file.truncate(0)