/*
* 语音听写(iFly Auto Transform)技术能够实时地将语音转换成对应的文字。
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "qisr.h"
#include "msp_cmn.h"
#include "msp_errors.h"
#include "speech_recognizer.h"

#define FRAME_LEN	640 
#define	BUFFER_SIZE	4096

/* 上传用户词表 */
static int upload_userwords()
{
	char*			userwords	=	NULL; // 用户词表的指针
	size_t			len			=	0;    // 用户词表的长度
	size_t			read_len	=	0;    // 实际读取的长度
	FILE*			fp			=	NULL; // 文件指针
	int				ret			=	-1;   // 返回码

	// 打开用户词表文件 "userwords.txt"
	fp = fopen("userwords.txt", "rb");
	if (NULL == fp)										
	{
		printf("\nopen [userwords.txt] failed! \n");
		goto upload_exit; // 文件打开失败，跳转至退出处理
	}

	// 获取文件大小
	fseek(fp, 0, SEEK_END);
	len = ftell(fp); 
	fseek(fp, 0, SEEK_SET);  					
	
	// 为用户词表分配内存
	userwords = (char*)malloc(len + 1);
	if (NULL == userwords)
	{
		printf("\nout of memory! \n");
		goto upload_exit; // 内存分配失败，跳转至退出处理
	}

	// 读取文件内容到内存
	read_len = fread((void*)userwords, 1, len, fp); 
	if (read_len != len)
	{
		printf("\nread [userwords.txt] failed!\n");
		goto upload_exit; // 文件读取失败，跳转至退出处理
	}
	userwords[len] = '\0'; // 确保字符串以空字符结尾
	
	// 上传用户词表
	MSPUploadData("userwords", userwords, len, "sub = uup, dtt = userword", &ret); 
	if (MSP_SUCCESS != ret)
	{
		printf("\nMSPUploadData failed ! errorCode: %d \n", ret);
		goto upload_exit; // 上传失败，跳转至退出处理
	}
	
upload_exit:
	if (NULL != fp)
	{
		fclose(fp); // 关闭文件
		fp = NULL;
	}	
	if (NULL != userwords)
	{
		free(userwords); // 释放内存
		userwords = NULL;
	}
	
	return ret; // 返回上传结果
}

/* 显示识别结果 */
static void show_result(char *string, char is_over)
{
	printf("\rResult: [ %s ]", string); // 打印结果
	if(is_over)
		putchar('\n'); // 如果是最后一条结果，换行
}

static char *g_result = NULL;  // 用于存放识别结果的缓冲区
static unsigned int g_buffersize = BUFFER_SIZE;  // 缓冲区大小

/* 结果回调函数 */
void on_result(const char *result, char is_last)
{
	if (result) {
		// 检查剩余空间
		size_t left = g_buffersize - 1 - strlen(g_result);
		size_t size = strlen(result);
		// 如果剩余空间不够，重新分配更大的内存
		if (left < size) {
			g_result = (char*)realloc(g_result, g_buffersize + BUFFER_SIZE);
			if (g_result)
				g_buffersize += BUFFER_SIZE;
			else {
				printf("mem alloc failed\n");
				return;
			}
		}
		// 将识别结果拼接到现有结果后
		strncat(g_result, result, size);
		show_result(g_result, is_last); // 显示结果
	}
}

/* 语音开始的回调函数 */
void on_speech_begin()
{
	if (g_result)
	{
		free(g_result);  // 释放之前的结果缓冲区
	}
	g_result = (char*)malloc(BUFFER_SIZE);  // 重新分配缓冲区
	g_buffersize = BUFFER_SIZE;  // 初始化缓冲区大小
	memset(g_result, 0, g_buffersize);  // 清空缓冲区

	printf("Start Listening...\n");
}

/* 语音结束的回调函数 */
void on_speech_end(int reason)
{
	if (reason == END_REASON_VAD_DETECT)
		printf("\nSpeaking done \n");  // 如果是由于语音检测到停止说话，输出提示
	else
		printf("\nRecognizer error %d\n", reason); // 否则输出错误信息
}

/* 从文件中发送音频数据进行识别 */
static void demo_file(const char* audio_file, const char* session_begin_params)
{
	int	errcode = 0;
	FILE*	f_pcm = NULL;
	char*	p_pcm = NULL;
	unsigned long	pcm_count = 0;
	unsigned long	pcm_size = 0;
	unsigned long	read_size = 0;
	struct speech_rec iat;
	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	// 打开音频文件
	if (NULL == audio_file)
		goto iat_exit;

	f_pcm = fopen(audio_file, "rb");
	if (NULL == f_pcm)
	{
		printf("\nopen [%s] failed! \n", audio_file);
		goto iat_exit;  // 打开文件失败，跳转至退出处理
	}

	// 获取文件大小并分配内存读取文件内容
	fseek(f_pcm, 0, SEEK_END);
	pcm_size = ftell(f_pcm);
	fseek(f_pcm, 0, SEEK_SET);

	p_pcm = (char *)malloc(pcm_size);
	if (NULL == p_pcm)
	{
		printf("\nout of memory! \n");
		goto iat_exit;  // 内存分配失败，跳转至退出处理
	}

	read_size = fread((void *)p_pcm, 1, pcm_size, f_pcm);
	if (read_size != pcm_size)
	{
		printf("\nread [%s] error!\n", audio_file);
		goto iat_exit;  // 读取文件失败，跳转至退出处理
	}

	// 初始化语音识别
	errcode = sr_init(&iat, session_begin_params, SR_USER, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed : %d\n", errcode);
		goto iat_exit;  // 初始化失败，跳转至退出处理
	}

	// 开始语音识别
	errcode = sr_start_listening(&iat);
	if (errcode) {
		printf("\nsr_start_listening failed! error code:%d\n", errcode);
		goto iat_exit;  // 启动识别失败，跳转至退出处理
	}

	// 循环读取音频数据并进行识别
	while (1)
	{
		unsigned int len = 10 * FRAME_LEN;  // 每次发送200ms的音频数据
		int ret = 0;

		if (pcm_size < 2 * len)
			len = pcm_size;
		if (len <= 0)
			break;

		ret = sr_write_audio_data(&iat, &p_pcm[pcm_count], len);

		if (0 != ret)
		{
			printf("\nwrite audio data failed! error code:%d\n", ret);
			goto iat_exit;  // 写入音频数据失败，跳转至退出处理
		}

		pcm_count += (long)len;
		pcm_size -= (long)len;		
	}

	// 停止识别
	errcode = sr_stop_listening(&iat);
	if (errcode) {
		printf("\nsr_stop_listening failed! error code:%d \n", errcode);
		goto iat_exit;  // 停止识别失败，跳转至退出处理
	}

iat_exit:
	// 资源释放
	if (NULL != f_pcm)
	{
		fclose(f_pcm);  // 关闭文件
		f_pcm = NULL;
	}
	if (NULL != p_pcm)
	{
		free(p_pcm);  // 释放内存
		p_pcm = NULL;
	}

	sr_stop_listening(&iat);  // 停止识别
	sr_uninit(&iat);  // 反初始化识别器
}

/* 从麦克风录音并识别 */
static void demo_mic(const char* session_begin_params)
{
	int errcode;
	int i = 0;

	struct speech_rec iat;

	struct speech_rec_notifier recnotifier = {
		on_result,
		on_speech_begin,
		on_speech_end
	};

	// 初始化语音识别
	errcode = sr_init(&iat, session_begin_params, SR_MIC, &recnotifier);
	if (errcode) {
		printf("speech recognizer init failed\n");
		return;
	}

	// 开始语音识别
	errcode = sr_start_listening(&iat
