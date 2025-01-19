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
    resultFlag=1;   // 设置结果标志为1
    printf("\rResult: [ %s ]", string);  // 打印识别结果
    if(is_over)
        putchar('\n');  // 如果识别结束，换行
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
 
    printf("Start Listening...\n");  // 打印开始监听提示
}

/**
 * @brief 语音结束回调函数
 * @param reason 结束原因
 * 功能：根据结束原因打印提示信息。
 */
void on_speech_end(int reason)
{
    if (reason == END_REASON_VAD_DETECT)  // 如果是因为语音活动检测结束
        printf("\nSpeaking done \n");  // 打印说话结束提示
    else
        printf("\nRecognizer error %d\n", reason);  // 打印识别错误信息
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
 
    struct speech_rec iat;  // 语音识别器结构体
 
    struct speech_rec_notifier recnotifier = {  // 回调函数结构体
        on_result,
        on_speech_begin,
        on_speech_end
    };
 
    errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);  // 初始化语音识别器
    if (errcode) {
        printf("speech recognizer init failed\n");  // 初始化失败
        return;
    }
    errcode = sr_start_listening(&iat);  // 开始录音
    if (errcode) {
        printf("start listen failed %d\n", errcode);  // 开始录音失败
    }
    /* demo 10 seconds recording */
    while(i++ < 2)  // 模拟录音10秒
        sleep(1);
    errcode = sr_stop_listening(&iat);  // 停止录音
    if (errcode) {
        printf("stop listening failed %d\n", errcode);  // 停止录音失败
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
void WakeUp(const std_msgs::String::ConstPtr& msg)
{
    printf("waking up\r\n");  // 打印唤醒提示
    usleep(700*1000);  // 延迟700ms
    wakeupFlag=1;  // 设置唤醒标志
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
    ros::Subscriber wakeUpSub = n.subscribe("voiceWakeup", 1000, WakeUp);  // 订阅唤醒话题
    // 发布语音识别结果
    ros::Publisher voiceWordsPub = n.advertise<std_msgs::String>("voiceWords", 1000);  // 发布识别结果话题
 
    ROS_INFO("Sleeping...");  // 打印休眠提示
    int count=0;
    while(ros::ok())  // ROS主循环
    {
        // 语音识别唤醒
        if (wakeupFlag){  // 如果唤醒标志为1
            ROS_INFO("Wakeup...");  // 打印唤醒提示
            int ret = MSP_SUCCESS;
            const char* login_params = "appid = 68a31b42, work_dir = .";  // 登录参数
 
            const char* session_begin_params =  // 会话参数
                "sub = iat, domain = iat, language = zh_cn, "
                "accent = mandarin, sample_rate = 16000, "
                "result_type = plain, result_encoding = utf8";
 
            ret = MSPLogin(NULL, NULL, login_params);  // 登录语音识别服务
            if(MSP_SUCCESS != ret){
                MSPLogout();  // 登出
                printf("MSPLogin failed , Error code %d.\n",ret);  // 打印登录失败信息
            }
 
            printf("Demo recognizing the speech from microphone\n");  // 打印提示信息
	
	    printf("Speak in 3 seconds\n");  // 打印提示信息
 	
            demo_mic(session_begin_params);  // 调用麦克风录音识别示例
 
            printf("3 sec passed\n");  // 打印提示信息
        
            wakeupFlag=1;  // 设置唤醒标志
            MSPLogout();  // 登出语音识别服务
        }
 
        // 语音识别完成
        if(resultFlag)  // 如果有识别结果
	{
            resultFlag=0;  // 重置结果标志
            std_msgs::String msg;  // 定义ROS消息
            msg.data = g_result;  // 设置消息内容为识别结果
            voiceWordsPub.publish(msg);  // 发布识别结果
        }
 
        ros::spinOnce();  // 处理ROS回调
        loop_rate.sleep();  // 休眠以维持循环频率
        count++;
    }
 
exit:
    MSPLogout(); // Logout...  // 登出语音识别服务
 
    return 0;  // 程序正常退出
}