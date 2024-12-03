/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "voice/qisr.h"
#include "voice/msp_cmn.h"
#include "voice/msp_errors.h"
#include "voice/speech_recognizer.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096

/* Upload User words */
static int upload_userwords()
{
	char* userwords = NULL; // 用于存储用户词表的字符串指针
	size_t len = 0; // 文件长度变量
	size_t read_len = 0; // 实际读取长度变量
	FILE* fp = NULL; // 文件指针
	int ret = -1; // 返回结果，初始值为 -1 表示失败

	fp = fopen("userwords.txt", "rb"); // 打开用户词表文件
	if (NULL == fp) // 检查文件是否成功打开
	{
		printf("\nopen [userwords.txt] failed! \n"); // 打印错误信息
		goto upload_exit; // 出错跳转到清理和退出部分
	}

	fseek(fp, 0, SEEK_END); // 将文件指针移动到文件末尾
	len = ftell(fp); // 获取文件长度
	fseek(fp, 0, SEEK_SET); // 将文件指针移回文件开头
	
	userwords = (char*)malloc(len + 1); // 分配内存用于存储文件内容
	if (NULL == userwords) // 检查内存分配是否成功
	{
		printf("\nout of memory! \n"); // 打印错误信息
		goto upload_exit; // 出错跳转到清理和退出部分
	}

	read_len = fread((void*)userwords, 1, len, fp); // 从文件读取内容到 userwords
	if (read_len != len) // 检查是否读取到完整文件
	{
		printf("\nread [userwords.txt] failed!\n"); // 打印错误信息
		goto upload_exit; // 出错跳转到清理和退出部分
	}
	userwords[len] = '\0'; // 给字符串添加结尾字符
	
	// 调用 MSPUploadData 上传用户词表
	MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret); 
	if (MSP_SUCCESS != ret) // 检查上传是否成功
	{
		printf("\nMSPUploadData failed ! errorCode: %d \n", ret); // 打印错误信息
		goto upload_exit; // 出错跳转到清理和退出部分
	}
	
upload_exit:
	if (NULL != fp) // 如果文件指针非空
	{
		fclose(fp); // 关闭文件
		fp = NULL; // 置空指针
	}	
	if (NULL != userwords) // 如果 userwords 非空
	{
		free(userwords); // 释放内存
		userwords = NULL; // 置空指针
	}
	
	return ret; // 返回结果
}

/* 显示识别结果 */
static void show_result(char *string, char is_over)
{
	printf("\rResult: [ %s ]", string); // 打印识别结果
	if(is_over) // 如果识别完成
		putchar('\n'); // 换行
}

/* 全局变量，用于存储识别结果 */
static char *g_result = NULL; // 指向识别结果的字符串
static unsigned int g_buffersize = BUFFER_SIZE; // 初始缓冲区大小

/* 识别结果回调函数 */
void on_result(const char *result, char is_last)
{
	if (result) { // 如果结果不为空
		size_t left = g_buffersize - 1 - strlen(g_result); // 计算剩余缓冲区空间
		size_t size = strlen(result); // 获取结果的长度
		if (left < size) { // 如果剩余空间不足
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE); // 重新分配更大的缓冲区
			if (g_result)
				g_buffersize += BUFFER_SIZE; // 更新缓冲区大小
			else {
				printf("mem alloc failed\n"); // 打印内存分配失败信息
				return; // 退出函数
			}
		}
		strncat(g_result, result, size); // 将结果拼接到 g_result
		show_result(g_result, is_last); // 显示结果
	}
}

/* 语音开始回调函数 */
void on_speech_begin()
{
	if (g_result) // 如果 g_result 已分配内存
	{
		free(g_result); // 释放内存
	}
	g_result = (char*)malloc(BUFFER_SIZE); // 分配初始缓冲区
	g_buffersize = BUFFER_SIZE; // 设置缓冲区大小
	memset(g_result, 0, g_buffersize); // 清空缓冲区内容

	printf("Start Listening...\n"); // 打印监听开始信息
}

/* 语音结束回调函数 */
void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT) // 如果因为静音检测结束
		printf("\nSpeaking done \n"); // 打印完成信息
	else
		printf("\nRecognizer error %d\n", reason); // 打印错误信息
}

/* 演示从文件发送音频数据 */
static void demo_file(const char* audio_file, const char* session_begin_params)
{
	int	errcode = 0; // 错误码
	FILE* f_pcm = NULL; // PCM 文件指针
	char* p_pcm = NULL; // 存储 PCM 数据的指针
	unsigned long pcm_count = 0; // 已处理的 PCM 数据计数
	unsigned long pcm_size = 0; // PCM 文件的总大小
	unsigned long read_size = 0; // 实际读取的数据大小
	struct speech_rec iat; // 语音识别结构体
	struct speech_rec_notifier recnotifier = { // 识别回调函数结构体
		on_result,
		on_speech_begin,
		on_speech_end
	};

	if (NULL == audio_file) // 检查文件路径是否为空
		goto iat_exit;

	f_pcm = fopen(audio_file, "rb"); // 打开 PCM 文件
	if (NULL == f_pcm) // 检查文件是否成功打开
	{
		printf("\nopen [%s] failed! \n", audio_file); // 打印错误信息
		goto iat_exit; // 出错跳转到清理和退出部分
	}

	fseek(f_pcm, 0, SEEK_END); // 将文件指针移到文件末尾
	pcm_size = ftell(f_pcm); // 获取文件长度
	fseek(f_pcm, 0, SEEK_SET); // 将文件指针移回文件开头

	p_pcm = (char *)malloc(pcm_size); // 分配内存存储 PCM 数据
	if (NULL == p_pcm) // 检查内存分配是否成功
	{
		printf("\nout of memory! \n"); // 打印错误信息
		goto iat_exit; // 出错跳转到清理和退出部分
	}

	read_size = fread((void *)p_pcm, 1, pcm_size, f_pcm); // 读取 PCM 数据
	if (read_size != pcm_size) // 检查是否读取完整文件
	{
		printf("\nread [%s] error!\n", audio_file); // 打印错误信息
		goto iat_exit; // 出错跳转到清理和退出部分
	}

	errcode = sr_init(&iat, session_begin_params, SR_USER, &recnotifier); // 初始化语音识别
	if (errcode) {
		printf("speech recognizer init failed : %d\n", errcode); // 打印初始化失败信息
		goto iat_exit; // 出错跳转到清理和退出部分
	}

	errcode = sr_start_listening(&iat); // 开始监听
	if (errcode) {
		printf("\nsr_start_listening failed! error code:%d\n", errcode); // 打印监听失败信息
		goto iat_exit; // 出错跳转到清理和退出部分
	}

	while (1) // 循环发送音频数据
	{
		unsigned int len = 10 * FRAME_LEN; /* 每次发送 200ms 的音频数据 */
		int ret = 0;

		if (pcm_size < 2 * len) // 如果剩余数据不足两倍的 len
			len = pcm_size; // 发送剩余数据
		if (len <= 0) // 如果没有剩余数据
			break; // 退出循环

		ret = sr_write_audio_data(&iat, &p_pcm[pcm_count], len); // 写入音频数据

		if (0 != ret) // 检查写入是否成功
		{
			printf("\nwrite audio data failed! error code:%d\n", ret); // 打印错误信息
			goto iat_exit; // 出错跳转到清理和退出部分
		}

		pcm_count += (long)len; // 更新已处理数据计数
		pcm_size -= (long)len; // 更新剩余数据大小		
	}

	errcode = sr_stop_listening(&iat); // 停止监听
	if (errcode) {
		printf("\nsr_stop_listening failed! error code:%d \n", errcode); // 打印停止失败信息
		goto iat_exit; // 出错跳转到清理和退出部分
	}

iat_exit:
	if (NULL != f_pcm) // 如果文件指针非空
	{
		fclose(f_pcm); // 关闭文件
		f_pcm = NULL; // 置空指针
	}
	if (NULL != p_pcm) // 如果 PCM 数据指针非空
	{
		free(p_pcm); // 释放内存
		p_pcm = NULL; // 置空指针
	}

	sr_stop_listening(&iat); // 确保监听已停止
	sr_uninit(&iat); // 释放语音识别资源
}

/* 从麦克风录音并识别的示例函数 */
static void demo_mic(const char* session_begin_params)
{
	int errcode; // 错误码变量
	int i = 0; // 用于计时的变量

	struct speech_rec iat; // 语音识别结构体

	struct speech_rec_notifier recnotifier = { // 回调函数结构体
		on_result, // 识别结果回调
		on_speech_begin, // 语音开始回调
		on_speech_end // 语音结束回调
	};

	// 初始化语音识别，指定从麦克风录入音频
	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) { // 检查初始化是否成功
		printf("speech recognizer init failed\n"); // 打印错误信息
		return; // 退出函数
	}
	// 开始监听麦克风输入
	errcode = sr_start_listening(&iat);
	if (errcode) { // 检查是否成功开始监听
		printf("start listen failed %d\n", errcode); // 打印错误信息
	}

	/* 录音演示：录制 15 秒音频 */
	while(i++ < 15)
		sleep(1); // 每秒循环一次，持续 15 秒

	// 停止监听
	errcode = sr_stop_listening(&iat);
	if (errcode) { // 检查是否成功停止监听
		printf("stop listening failed %d\n", errcode); // 打印错误信息
	}

	sr_uninit(&iat); // 释放语音识别资源
}

/* 主函数：程序入口 */
int main(int argc, char* argv[])
{
	ros::init(argc,argv,"voice_main");
	int ret = MSP_SUCCESS; // 登录状态标志，初始化为成功
	/* 登录参数：包括应用 ID 和工作目录 */
	const char* login_params = "appid = 68a31b42, work_dir = .";
	/*
	 * 语音识别会话参数，定义语音识别的配置
	 * 包括子服务类型、领域、语言、口音、采样率、结果格式等
	 */
	const char* session_begin_params =
		"sub = iat, domain = iat, language = zh_cn, "
		"accent = mandarin, sample_rate = 16000, "
		"result_type = plain, result_encoding = utf8";

	/* 登录到讯飞语音云平台
	 * 第一个参数是用户名，第二个参数是密码
	 * 这里设置为 NULL，因为一般不需要认证
	 * 第三个参数是登录配置
	 */
	ret = MSPLogin(NULL, NULL, login_params);
	if (MSP_SUCCESS != ret)	{ // 检查登录是否成功
		printf("MSPLogin failed , Error code %d.\n",ret); // 打印错误信息
		goto exit; // 登录失败，跳转到退出部分
	}
		demo_mic(session_begin_params); // 调用麦克风录音和识别的演示函数

		printf("15 sec passed\n"); // 提示录音结束


exit:
	MSPLogout(); // 注销，释放所有资源

	return 0; // 返回 0 表示程序正常结束
}
