/*******************************************************
 This contents of this file may be used by anyone
 for any reason without any conditions and may be
 used as a starting point for your own applications
 which use HIDAPI.
********************************************************/
#include <user_interface.h>  // 用户界面相关的头文件
#include <string>            // 字符串操作相关的头文件
#include <locale>            // 本地化相关的头文件
#include <codecvt>           // 字符编码转换相关的头文件
#include <ctime>             // 时间操作相关的头文件
#include <joint.h>           // 关节操作相关的头文件
#include <record.h>          // 录音操作相关的头文件
#include <ros/ros.h>         // ROS相关的头文件
#include <std_msgs/String.h> // ROS标准消息类型，字符串消息
#include <xf_mic_asr_offline_line/Get_Offline_Result_srv.h> // 离线识别服务相关的头文件

#include <std_msgs/Int8.h>   // ROS标准消息类型，8位整数
#include <std_msgs/Int32.h>  // ROS标准消息类型，32位整数
#include <sys/stat.h>        // 文件状态相关的头文件

ros::Publisher voice_words_pub;  // 发布语音识别结果的ROS发布者
ros::Publisher awake_flag_pub;   // 发布唤醒标志的ROS发布者
ros::Publisher voice_flag_pub;   // 发布语音标志的ROS发布者

std::string voice_words = "voice_words";  // 语音识别结果的ROS话题名称

std::string voice_flag = "voice_flag";    // 语音标志的ROS话题名称

std::string awake_flag = "awake_flag";    // 唤醒标志的ROS话题名称

int offline_recognise_switch = 1; // 离线识别默认开关

using namespace std;

extern UserData asr_data;  // 外部定义的语音识别数据结构
extern int whether_finised;  // 外部定义的识别是否完成的标志
extern char *whole_result;   // 外部定义的完整识别结果
int set_led_id;              // 设置LED的ID

extern int init_rec;         // 外部定义的初始化录音标志
extern int init_success;     // 外部定义的初始化成功标志
extern int write_first_data; // 外部定义的写入第一次数据的标志

unsigned char* record_data;  // 录音数据

/**
 * @brief 将std::string转换为std::wstring
 * @param str 输入的std::string字符串
 * @return 转换后的std::wstring字符串
 */
std::wstring s2ws(const std::string &str)
{
	using convert_typeX = std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.from_bytes(str);
}

/**
 * @brief 将std::wstring转换为std::string
 * @param wstr 输入的std::wstring字符串
 * @return 转换后的std::string字符串
 */
std::string ws2s(const std::wstring &wstr)
{
	using convert_typeX = std::codecvt_utf8<wchar_t>;
	std::wstring_convert<convert_typeX, wchar_t> converterX;

	return converterX.to_bytes(wstr);
}

/**
 * @brief 用于送入音频进行识别 (line)
 * @param record 录音数据的指针
 * @return 无返回值
 * 功能：将录音数据送入语音识别引擎进行处理
 */
int business_data(unsigned char* record)
{
    record_data = record;  // 将录音数据赋值给全局变量
    if (!init_success && init_rec)  // 如果初始化未成功且正在录音
    {
        int len = 3*PCM_MSG_LEN;  // 计算音频数据的长度
        char *pcm_buffer=(char *)malloc(len);  // 分配内存用于存储音频数据
        if (NULL == pcm_buffer)  // 如果内存分配失败
            {
                printf(">>>>>buffer is null\n");  // 打印错误信息
                exit (1);  // 退出程序
            }
        memcpy(pcm_buffer, record_data, len);  // 将录音数据复制到缓冲区

        if (write_first_data++ == 0)  // 如果是第一次写入数据
        {
    #if whether_print_log  // 如果定义了打印日志
            printf("***************write the first voice**********\n");  // 打印日志信息
    #endif
            demo_xf_mic(pcm_buffer, len, 1);  // 调用语音识别函数，1表示第一次写入
        }

        else  // 如果不是第一次写入数据
        {
    #if whether_print_log  // 如果定义了打印日志
            printf("***************write the middle voice**********\n");  // 打印日志信息
    #endif
            demo_xf_mic(pcm_buffer, len, 2);  // 调用语音识别函数，2表示中间写入
        }
        if (whether_finised)  // 如果识别完成
        {
            record_finish = 1;  // 设置录音完成标志
            whether_finised = 0;  // 重置识别完成标志
        }
    }   
}

/**
 * @brief 用于显示离线命令词识别结果
 * @param string 输入的识别结果字符串
 * @return Effective_Result 结构体，包含识别结果和置信度
 * 功能：解析识别结果字符串，提取关键字和置信度
 */
Effective_Result show_result(char *string) //
{
	Effective_Result current;  // 定义当前识别结果
	if (strlen(string) > 250)  // 如果字符串长度大于250
	{
		char asr_result[32];	// 识别到的关键字的结果
		char asr_confidence[3]; // 识别到的关键字的置信度
		char *p1 = strstr(string, "<rawtext>");  // 查找<rawtext>标签
		char *p2 = strstr(string, "</rawtext>"); // 查找</rawtext>标签
		int n1 = p1 - string + 1;  // 计算<rawtext>标签的位置
		int n2 = p2 - string + 1;  // 计算</rawtext>标签的位置

		char *p3 = strstr(string, "<confidence>");  // 查找<confidence>标签
		char *p4 = strstr(string, "</confidence>"); // 查找</confidence>标签
		int n3 = p3 - string + 1;  // 计算<confidence>标签的位置
		int n4 = p4 - string + 1;  // 计算</confidence>标签的位置
		for (int i = 0; i < 32; i++)  // 初始化识别结果数组
		{
			asr_result[i] = '\0';
		}

		strncpy(asr_confidence, string + n3 + strlen("<confidence>") - 1, n4 - n3 - strlen("<confidence>"));  // 复制置信度数据
		asr_confidence[n4 - n3 - strlen("<confidence>")] = '\0';  // 添加字符串结束符
		int confidence_int = 0;  // 定义置信度整数
		confidence_int = atoi(asr_confidence);  // 将置信度字符串转换为整数
		if (confidence_int >= confidence)  // 如果置信度大于阈值
		{
			strncpy(asr_result, string + n1 + strlen("<rawtext>") - 1, n2 - n1 - strlen("<rawtext>"));  // 复制识别结果
			asr_result[n2 - n1 - strlen("<rawtext>")] = '\0'; // 加上字符串结束符。
		}
		else  // 如果置信度小于阈值
		{
			strncpy(asr_result, "", 0);  // 清空识别结果
		}

		current.effective_confidence = confidence_int;  // 设置当前识别结果的置信度
		strcpy(current.effective_word, asr_result);  // 设置当前识别结果的关键字
		return current;  // 返回当前识别结果
	}
	else  // 如果字符串长度小于等于250
	{
		current.effective_confidence = 0;  // 设置置信度为0
		strcpy(current.effective_word, " ");  // 清空识别结果
		return current;  // 返回当前识别结果
	}
}

/**
 * @brief 获取离线命令词识别结果
 * @param req 服务请求，包含离线识别开关
 * @param res 服务响应，包含识别结果、失败原因和识别文本
 * @return bool 表示服务是否成功执行
 * 功能：处理离线语音识别请求，返回识别结果
 */
bool Get_Offline_Recognise_Result(xf_mic_asr_offline_line::Get_Offline_Result_srv::Request &req,
								  xf_mic_asr_offline_line::Get_Offline_Result_srv::Response &res)
{
	char *denoise_sound_path = join(source_path, DENOISE_SOUND_PATH);  // 获取去噪音频路径
	offline_recognise_switch = req.offline_recognise_start;  // 设置离线识别开关
	if (offline_recognise_switch == 1) // 如果是离线识别模式
	{
		whether_finised = 0;  // 重置识别完成标志
		record_finish = 0;  // 重置录音完成标志
		int ret = 0;  // 定义返回值
		ret = create_asr_engine(&asr_data);  // 创建语音识别引擎
		if (MSP_SUCCESS != ret)  // 如果创建失败
		{
#if whether_print_log  // 如果定义了打印日志
			printf("[01]创建语音识别引擎失败！\n");  // 打印错误信息
#endif
		}

		printf(">>>>>开始一次语音识别！\n");  // 打印日志信息
		get_the_record_sound(denoise_sound_path);  // 获取录音数据

		if (whole_result!="")  // 如果有识别结果
		{
			//printf(">>>>>全部返回结果:　[ %s ]\n", whole_result);
			Effective_Result effective_ans = show_result(whole_result);  // 解析识别结果
			if (effective_ans.effective_confidence >= confidence) // 如果置信度大于阈值
			{
				printf(">>>>>是否识别成功:　 [ %s ]\n", "是");  // 打印日志信息
				printf(">>>>>关键字的置信度: [ %d ]\n", effective_ans.effective_confidence);  // 打印置信度
				printf(">>>>>关键字识别结果: [ %s ]\n", effective_ans.effective_word);  // 打印识别结果
				/*发布结果*/
				//control_jetbot(effective_ans.effective_word);  // 控制机器人（注释掉的部分）
				res.result = "ok";  // 设置服务响应结果为成功
				res.fail_reason = "";  // 清空失败原因
				std::wstring wtxt = s2ws(effective_ans.effective_word);  // 将识别结果转换为宽字符
				std::string txt_uft8 = ws2s(wtxt);  // 将宽字符转换为UTF-8字符串
				res.text = txt_uft8;  // 设置服务响应的识别文本
				
				std_msgs::String msg;  // 定义ROS消息
				msg.data = effective_ans.effective_word;  // 设置消息内容为识别结果
				voice_words_pub.publish(msg);  // 发布识别结果
				
			}
			else  // 如果置信度小于阈值
			{
				printf(">>>>>是否识别成功:　[ %s ]\n", "否");  // 打印日志信息
				printf(">>>>>关键字的置信度: [ %d ]\n", effective_ans.effective_confidence);  // 打印置信度
				printf(">>>>>关键字置信度较低，文本不予显示\n");  // 打印日志信息
				res.result = "fail";  // 设置服务响应结果为失败
				res.fail_reason = "low_confidence error or 11212_license_expired_error";  // 设置失败原因
				res.text = " ";  // 清空识别文本
			}
		}
		else  // 如果没有识别结果
		{
			res.result = "fail";  // 设置服务响应结果为失败
			res.fail_reason = "no_valid_sound error";  // 设置失败原因
			res.text = " ";  // 清空识别文本
			printf(">>>>>未能检测到有效声音,请重试\n");  // 打印日志信息
		}
		whole_result = "";  // 清空全局识别结果
		/*[1-3]语音识别结束]*/
		delete_asr_engine();  // 删除语音识别引擎
		write_first_data = 0;  // 重置写入第一次数据的标志
		sleep(1.0);  // 等待1秒
		
	}
	printf(" \n");
	printf(" \n");
	//ROS_INFO("close the offline recognise mode ...\n");
	return true;  // 返回服务执行成功
}
/**
 * @brief 程序入口
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出状态
 * 功能：初始化ROS节点，设置参数，启动语音识别服务。
 */
int main(int argc, char *argv[])
{
	ros::init(argc, argv, "voice_control");  // 初始化ROS节点，节点名为"voice_control"
	ros::NodeHandle ndHandle("~");  // 创建节点句柄，用于访问私有参数
	ndHandle.param("/confidence", confidence, 0);  // 从参数服务器获取置信度阈值，默认为0
	ndHandle.param("/seconds_per_order", time_per_order, 3);  // 从参数服务器获取单次录音时长，默认为3秒
	ndHandle.param("source_path", source_path, std::string("/home/wheeltec/wheeltec_robot/src/vvui_ros-master/xf_mic_asr_offline_line"));  // 从参数服务器获取资源路径
	ndHandle.param("/appid", appid, std::string("68a31b42"));  // 从参数服务器获取appid，默认为"68a31b42"

	printf("-----confidence =%d\n",confidence);  // 打印置信度阈值
	printf("-----time_per_order =%d\n",time_per_order);  // 打印单次录音时长

	cout<<"-----source_path="<<source_path<<endl;  // 打印资源路径
	cout<<"-----appid="<<appid<<endl;  // 打印appid

	APPID = &appid[0];  // 将appid赋值给全局变量APPID

	ros::NodeHandle n;  // 创建全局节点句柄

	voice_words_pub = n.advertise<std_msgs::String>(voice_words, 1);  // 初始化发布者，发布语音识别结果

	awake_flag_pub = n.advertise<std_msgs::Int8>(awake_flag, 1);  // 初始化发布者，发布唤醒标志

	voice_flag_pub = n.advertise<std_msgs::Int8>(voice_flag, 1);  // 初始化发布者，发布语音标志

	/*srv　接收请求，返回离线命令词识别结果*/
	ros::ServiceServer service_get_wav_list = ndHandle.advertiseService("get_offline_recognise_result_srv", Get_Offline_Recognise_Result);  // 初始化服务，处理离线识别请求

	std::string begin = "fo|";  // 定义路径前缀
	//std::string quit_begin = source_path;
	char *jet_path = join((begin + source_path), ASR_RES_PATH);  // 拼接ASR资源路径
	char *grammer_path = join(source_path, GRM_BUILD_PATH);  // 拼接语法构建路径
	char *bnf_path = join(source_path, GRM_FILE);  // 拼接语法文件路径
	//IN_PCM = join(source_path, IN_PCM);
	//[1-1] 通用登录及语法构建

	Recognise_Result inital = initial_asr_paramers(jet_path, grammer_path, bnf_path, LEX_NAME);  // 初始化ASR参数

	init_rec = 0;  // 初始化录音标志
	init_success = 0;  // 初始化成功标志
	write_first_data = 0;  // 初始化写入第一次数据的标志
	
	ros::AsyncSpinner spinner(3);  // 创建异步Spinner，用于多线程处理ROS回调
	spinner.start();  // 启动Spinner

	while(ros::ok())  // ROS主循环
	{
	    if (!init_success && init_rec)  // 如果初始化未成功且正在录音
	    {
	        //获取当前时间
	        clock_t start, finish;  // 定义时间变量
	        double total_time;  // 定义总时间变量
	        start = clock();  // 记录开始时间
	        while (!init_success && whether_finised != 1)  // 如果初始化未成功且识别未完成
	        {   
	            finish = clock();  // 记录结束时间
	            total_time = (double)(finish - start) / CLOCKS_PER_SEC/2;  // 计算总时间
	            //printf(">>>>>total_time:　[ %f ]\n", total_time);
	            //printf(">>>>>whether_finised:　[ %d ]\n", whether_finised);
	            if (total_time > time_per_order)  // 如果总时间超过单次录音时长
	            {
	                printf(">>>>>超出离线命令词最长识别时间\n");  // 打印日志信息
	                record_finish = 1;  // 设置录音完成标志
	                break;  // 退出循环
	            }
	        } 
	    }		
		
	} 
	ros::spinOnce();  // 处理ROS回调
	//ros::waitForShutdown();
	return 0;  // 程序正常退出
}