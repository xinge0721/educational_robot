/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/
 
#include <stdlib.h>  // 标准库头文件，包含内存分配、程序控制等功能
#include <stdio.h>   // 标准输入输出头文件
#include <string.h>  // 字符串操作头文件
#include <unistd.h>  // 提供对POSIX操作系统API的访问
#include "qisr.h"    // 语音识别相关的头文件
#include "msp_cmn.h" // 语音识别公共头文件
#include "msp_errors.h" // 语音识别错误码头文件
#include "speech_recognizer.h" // 语音识别器头文件
#include <iconv.h>   // 字符编码转换头文件
 #include <std_msgs/Int32.h>

#include "ros/ros.h" // ROS头文件
#include "std_msgs/String.h" // ROS标准消息类型，字符串消息
 
#define FRAME_LEN   640  // 定义音频帧长度
#define BUFFER_SIZE 4096 // 定义缓冲区大小
 
int wakeupFlag   = 0 ;  // 唤醒标志，1表示唤醒，0表示休眠
int resultFlag   = 0 ;  // 结果标志，1表示有识别结果，0表示无结果
 
/**
 * @brief 显示识别结果
 * @param string 识别结果字符串
 * @param is_over 是否识别结束
 * 功能：打印识别结果，并在识别结束时换行。
 */
static void show_result(char *string, char is_over)
{
    
    printf("\rResult: [ %s ]", string);  // 打印识别结果
    if(is_over)
    {
        resultFlag=1;   // 设置结果标志为1
        putchar('\n');  // 如果识别结束，换行
    }

}
 
static char *g_result = NULL;  // 全局变量，存储识别结果

static unsigned int g_buffersize = BUFFER_SIZE;  // 全局变量，缓冲区大小
 
/**
 * @brief 识别结果回调函数
 * @param result 识别结果
 * @param is_last 是否是最后一次结果
 * 功能：将识别结果拼接到全局变量g_result中，并调用show_result显示结果。
 */
void on_result(const char *result, char is_last)
{
    if (result) {
        size_t left = g_buffersize - 1 - strlen(g_result);  // 计算剩余缓冲区大小
        size_t size = strlen(result);  // 计算当前结果的长度
        if (left < size) {  // 如果剩余空间不足
            g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);  // 重新分配内存
            if (g_result)
                g_buffersize += BUFFER_SIZE;  // 更新缓冲区大小
            else {
                printf("mem alloc failed\n");  // 内存分配失败
                return;
            }
        }
        strncat(g_result, result, size);  // 将结果拼接到g_result中
        show_result(g_result, is_last);  // 显示结果
    }
}
 

/**
 * @brief 语音开始回调函数
 * 功能：初始化g_result缓冲区，准备接收识别结果。
 */
void on_speech_begin()
{
    if (g_result)
    {
        free(g_result);  // 释放之前的缓冲区
    }
    g_result = (char*)malloc(BUFFER_SIZE);  // 分配新的缓冲区
    g_buffersize = BUFFER_SIZE;  // 重置缓冲区大小
    memset(g_result, 0, g_buffersize);  // 清空缓冲区
 
    // printf("Start Listening...\n");  // 打印开始监听提示
}

// 添加全局变量
static int g_speech_end_detected = 0;  // 语音结束检测标志
void (*g_original_on_speech_end)(int reason) = NULL;  // 保存原始回调函数指针

// 添加新的回调函数
void vad_on_speech_end(int reason)
{
    if (reason == END_REASON_VAD_DETECT)
        g_speech_end_detected = 1;
    if (g_original_on_speech_end)
        g_original_on_speech_end(reason);
}

/**
 * @brief 语音结束回调函数
 * @param reason 结束原因
 * 功能：根据结束原因打印提示信息。
 */
void on_speech_end(int reason)
{
    if (reason == END_REASON_VAD_DETECT) {  // 如果是因为语音活动检测结束
        printf("\n检测到语音结束\n");  // 打印说话结束提示
        resultFlag = 1;  // 设置结果标志
    }
    else
        printf("\n识别错误 %d\n", reason);  // 打印识别错误信息
}
 

/**
 * @brief 麦克风录音识别示例
 * @param session_begin_params 会话参数
 * 功能：初始化语音识别器，开始录音并识别，最后停止录音。
 */
static void demo_mic(const char* session_begin_params)
{
    int errcode;
    int i = 0;
    g_speech_end_detected = 0;  // 重置语音结束检测标志
 
    struct speech_rec iat;  // 语音识别器结构体
 
    struct speech_rec_notifier recnotifier = {  // 回调函数结构体
        on_result,
        on_speech_begin,
        on_speech_end
    };
 
    // 保存原始的on_speech_end回调
    g_original_on_speech_end = on_speech_end;
    // 设置新的回调
    recnotifier.on_speech_end = vad_on_speech_end;
 
    errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);  // 初始化语音识别器
    if (errcode) {
        // printf("speech recognizer init failed\n");  // 初始化失败
        return;
    }
    errcode = sr_start_listening(&iat);  // 开始录音
    if (errcode) {
        // printf("start listen failed %d\n", errcode);  // 开始录音失败
    }
    
    // 修改为等待语音结束或超时
    int max_silence_seconds = 3;   // 最大沉默等待时间
    int max_total_seconds = 30;    // 最大总录音时间
    int silence_count = 0;         // 沉默计数器
    int total_count = 0;           // 总时间计数器
    
    // 等待语音结束或超时
    while (total_count++ < max_total_seconds) {
        sleep(1);
        
        if (resultFlag) {  // 有识别结果
            silence_count = 0;  // 重置沉默计数器
        } else {
            silence_count++;  // 增加沉默计数器
        }
        
        // 检测语音结束条件：语音活动检测结束或超过最大沉默时间
        if (g_speech_end_detected || silence_count >= max_silence_seconds) {
            break;
        }
    }
    
    errcode = sr_stop_listening(&iat);  // 停止录音
    if (errcode) {
        // printf("stop listening failed %d\n", errcode);  // 停止录音失败
    }
 
    sr_uninit(&iat);  // 反初始化语音识别器
}
 
 
/* main thread: start/stop record ; query the result of recgonization.
 * record thread: record callback(data write)
 * helper thread: ui(keystroke detection)
 */
 

/**
 * @brief 唤醒回调函数
 * @param msg 唤醒消息
 * 功能：接收到唤醒消息后，设置唤醒标志并延迟700ms。
 */
void WakeUp(const std_msgs::Int32::ConstPtr& msg)
{
    std::cout << "角度为:" <<msg->data << std::endl;
    usleep(700*1000);  // 延迟700ms
    wakeupFlag = !wakeupFlag;  // 设置唤醒标志
}


/**
 * @brief 主函数
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出状态
 * 功能：初始化ROS节点，订阅唤醒消息，发布识别结果，并进入主循环。
 */
int main(int argc, char* argv[])
{
    // 初始化ROS
    ros::init(argc, argv, "voiceRecognition");  // 初始化ROS节点，节点名为"voiceRecognition"
    ros::NodeHandle n;  // 创建节点句柄
    ros::Rate loop_rate(10);  // 设置循环频率为10Hz
 
    // 声明Publisher和Subscriber
    // 订阅唤醒语音识别的信号
    ros::Subscriber wakeUpSub = n.subscribe("/mic/awake/angle", 1000, WakeUp);  // 订阅唤醒话题
    // 发布语音识别结果
    ros::Publisher voiceWordsPub = n.advertise<std_msgs::String>("chatter", 1000);  // 发布识别结果话题
 
    ROS_INFO("等待唤醒...");  // 打印休眠提示
    
    // 初始化科大讯飞SDK
    int ret = MSP_SUCCESS;
    const char* login_params = "appid = 68a31b42, work_dir = .";  // 登录参数
    
    const char* session_begin_params =  // 会话参数
        "sub = iat, domain = iat, language = zh_cn, "
        "accent = mandarin, sample_rate = 16000, "
        "result_type = plain, result_encoding = utf8, "
        "vad_eos = 3000";  // 设置VAD的静音检测时间为3000ms

    // 登录语音识别服务，只需要登录一次
    ret = MSPLogin(NULL, NULL, login_params);
    if(MSP_SUCCESS != ret) {
        printf("MSPLogin失败，错误码 %d.\n", ret);
        return 1;
    }
    
    int processing = 0;  // 添加处理状态标志
    
    while(ros::ok())  // ROS主循环
    {
        // 语音识别唤醒
        if (wakeupFlag && !processing) {  // 如果唤醒且未在处理中
            processing = 1;  // 设置处理状态
            ROS_INFO("已唤醒，开始聆听...");  // 打印唤醒提示
            
            demo_mic(session_begin_params);  // 调用麦克风录音识别示例
 
            // printf("3 sec passed\n");  // 打印提示信息
        
            MSPLogout();  // 登出语音识别服务
        }
 
        // 语音识别完成
        if(resultFlag)  // 如果有识别结果
	{
        wakeupFlag = 0;
            resultFlag=0;  // 重置结果标志
            std_msgs::String msg;  // 定义ROS消息
            msg.data = g_result;  // 设置消息内容为识别结果
            voiceWordsPub.publish(msg);  // 发布识别结果
        }
 
        ros::spinOnce();  // 处理ROS回调
        loop_rate.sleep();  // 休眠以维持循环频率
    }
 
    MSPLogout(); // 登出语音识别服务
 
    return 0;  // 程序正常退出
}