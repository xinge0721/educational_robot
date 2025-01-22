#include <ros/ros.h>  // ROS库，用于ROS节点的创建和管理
#include <time.h>     // 时间库，用于获取当前时间
#include "wav_head.h" // 自定义头文件，可能包含WAV文件头的定义

using namespace std;  // 使用标准命名空间

// 全局变量定义
std::string source_path = "";  // 资源路径，用于指定资源文件的位置
std::string appid = "";        // 应用程序ID，用于语音合成服务的认证
std::string voice_name = "";   // 语音名称，指定合成语音的类型
std::string tts_text = "";     // 待合成的文本
int rdn;                       // 随机数参数，可能用于语音合成的随机化
int volume;                    // 音量参数，控制合成语音的音量
int pitch;                     // 音调参数，控制合成语音的音调
int speed;                     // 语速参数，控制合成语音的语速
int sample_rate;               // 采样率参数，控制合成语音的采样率
const char* params_l;          // 登录参数，用于语音合成服务的登录
const char* params_s;          // 会话参数，用于语音合成服务的会话
const char* params_f;          // 文件参数，用于指定输出文件
const char* params_t;          // 文本参数，用于指定待合成的文本

// 将std::string转换为std::wstring
std::wstring s2ws(const std::string &str)
{
    using convert_typeX = std::codecvt_utf8<wchar_t>;
    std::wstring_convert<convert_typeX, wchar_t> converterX;

    return converterX.from_bytes(str);  // 将UTF-8字符串转换为宽字符字符串
}

// 将std::wstring转换为std::string
std::string ws2s(const std::wstring &wstr)
{
    using convert_typeX = std::codecvt_utf8<wchar_t>;
    std::wstring_convert<convert_typeX, wchar_t> converterX;

    return converterX.to_bytes(wstr);  // 将宽字符字符串转换为UTF-8字符串
}

/* 字符串与字符数组拼接 */
char *join(std::string b, char *s2)
{
    char s1[600] = "";
    try
    {
        strcpy(s1, b.c_str());  // 将std::string转换为C风格字符串
    }
    catch (...)
    {
        cout << ">>>>>join拷贝失败" << endl;  // 捕获异常并输出错误信息
    }
    char *result = (char *)malloc(strlen(s1) + strlen(s2) + 1);  // 分配内存以存储拼接后的字符串
    if (result == NULL)
        exit(1);  // 如果内存分配失败，退出程序

    try
    {
        strcpy(result, s1);  // 将s1复制到result
        strcat(result, s2);  // 将s2追加到result
    }
    catch (...)
    {
        cout << ">>>>>join拷贝失败" << endl;  // 捕获异常并输出错误信息
    }
    return result;  // 返回拼接后的字符串
}

/* 获取当前时间并格式化为字符串 */
std::string current_time()
{
    std::string fmt = ".wav";  // 文件扩展名
    static char t_buf[64];     // 用于存储时间字符串的缓冲区
    time_t now_time = time(NULL);  // 获取当前时间
    struct tm* time = localtime(&now_time);  // 将时间转换为本地时间结构
    strftime(t_buf, 64, "%Y-%m-%d %H:%M:%S", time);  // 格式化时间为字符串
    std::wstring wtxt = s2ws(t_buf);  // 将时间字符串转换为宽字符字符串
    std::string txt_uft8 = ws2s(wtxt);  // 将宽字符字符串转换回UTF-8字符串
    txt_uft8 += fmt;  // 添加文件扩展名
    return txt_uft8;  // 返回格式化后的时间字符串
}

/* 文本合成函数 */
int text_to_speech(const char* src_text, const char* des_path, const char* params)
{
    int          ret          = -1;  // 返回值，初始化为-1
    FILE*        fp           = NULL;  // 文件指针，用于写入WAV文件
    const char*  sessionID    = NULL;  // 会话ID，用于语音合成会话
    unsigned int audio_len    = 0;  // 音频数据长度
    wave_pcm_hdr wav_hdr      = default_wav_hdr;  // WAV文件头
    int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;  // 合成状态

    if (NULL == src_text || NULL == des_path)
    {
        printf("params is error!\n");  // 检查输入参数是否有效
        return ret;
    }
    fp = fopen(des_path, "wb");  // 打开文件以写入二进制数据
    if (NULL == fp)
    {
        printf("open %s error.\n", des_path);  // 检查文件是否成功打开
        return ret;
    }
    /* 开始合成 */
    sessionID = QTTSSessionBegin(params, &ret);  // 开始语音合成会话
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSSessionBegin failed, error code: %d.\n", ret);  // 检查会话是否成功开始
        fclose(fp);
        return ret;
    }
    ret = QTTSTextPut(sessionID, src_text, (unsigned int)strlen(src_text), NULL);  // 将文本放入会话
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSTextPut failed, error code: %d.\n",ret);  // 检查文本是否成功放入
        QTTSSessionEnd(sessionID, "TextPutError");
        fclose(fp);
        return ret;
    }
    printf("正在合成 ...\n");
    fwrite(&wav_hdr, sizeof(wav_hdr) ,1, fp); //添加wav音频头，使用采样率为16000
    while (1) 
    {
        /* 获取合成音频 */
        const void* data = QTTSAudioGet(sessionID, &audio_len, &synth_status, &ret);  // 获取合成的音频数据
        if (MSP_SUCCESS != ret)
            break;
        if (NULL != data)
        {
            fwrite(data, audio_len, 1, fp);  // 将音频数据写入文件
            wav_hdr.data_size += audio_len; //计算data_size大小
        }
        if (MSP_TTS_FLAG_DATA_END == synth_status)
            break;
    }
    printf("\n");
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSAudioGet failed, error code: %d.\n",ret);  // 检查音频数据是否成功获取
        QTTSSessionEnd(sessionID, "AudioGetError");
        fclose(fp);
        return ret;
    }
    /* 修正wav文件头数据的大小 */
    wav_hdr.size_8 += wav_hdr.data_size + (sizeof(wav_hdr) - 8);
    
    /* 将修正过的数据写回文件头部,音频文件为wav格式 */
    fseek(fp, 4, 0);
    fwrite(&wav_hdr.size_8,sizeof(wav_hdr.size_8), 1, fp); //写入size_8的值
    fseek(fp, 40, 0); //将文件指针偏移到存储data_size值的位置
    fwrite(&wav_hdr.data_size,sizeof(wav_hdr.data_size), 1, fp); //写入data_size的值
    fclose(fp);
    fp = NULL;
    /* 合成完毕 */
    ret = QTTSSessionEnd(sessionID, "Normal");  // 结束语音合成会话
    if (MSP_SUCCESS != ret)
    {
        printf("QTTSSessionEnd failed, error code: %d.\n",ret);  // 检查会话是否成功结束
    }

    return ret;
}

/* 初始化语音合成 */
int tts_init()
{
    int         ret                  = MSP_SUCCESS;  // 返回值，初始化为成功
    std::string login_ori		 	 = "appid = ";  // 登录参数字符串
    std::string login_fin     	 	 = login_ori + appid + ", work_dir = .";  // 拼接登录参数
    const char* login_params = login_fin.c_str();//登录参数,appid与msc库绑定,请勿随意改动
    //cout<< ">>>>>>> login_params:"<< login_params<< endl;

    std::string session_ori_1		 = "engine_type = local,voice_name=";  // 会话参数字符串
    std::string session_ori_2		 = ", text_encoding = UTF8, tts_res_path = fo|";
    std::string session_ori_3		 = "/config/bin/msc/res/tts/xiaoyan.jet;fo|";
    std::string session_ori_4		 = "/config/bin/msc/res/tts/common.jet, sample_rate = ";
    std::string session_ori_5	 	 = ", volume = ";
    std::string session_ori_6	 	 = ", pitch = ";
    std::string session_ori_7	 	 = ", rdn = ";
    std::string session_ori_8	 	 = ", speed = ";
    std::string session_fin			 = session_ori_1 + voice_name + session_ori_2 + source_path + session_ori_3 + source_path + session_ori_4 + std::to_string(sample_rate) + session_ori_5 + std::to_string(volume) + session_ori_6 + std::to_string(pitch) + session_ori_7 + std::to_string(rdn) + session_ori_8 + std::to_string(speed);
    const char* session_begin_params = session_fin.c_str();
    //cout<< ">>>>>>> session_begin_params:"<< session_begin_params<< endl;

    std::string filename_fin		 = source_path + "/audio/" + current_time();  // 生成输出文件名
    const char* filename             = filename_fin.c_str(); //合成的语音文件名称
    //cout<< ">>>>>>> filename:"<< filename << endl;
    const char* text                 = tts_text.c_str(); //合成文本
    /* 用户登录 */
    ret = MSPLogin(NULL, NULL, login_params); //第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://www.xfyun.cn注册获取
    if (MSP_SUCCESS != ret)
    {
        printf("MSPLogin failed, error code: %d.\n", ret);
        goto exit ;//登录失败，退出登录
    }

    printf("\n###########################################################################\n");
    printf("## 语音合成（Text To Speech，TTS）技术能够自动将任意文字实时转换为连续的 ##\n");
    printf("## 自然语音，是一种能够在任何时间、任何地点，向任何人提供语音信息服务的  ##\n");
    printf("## 高效便捷手段，非常符合信息时代海量数据、动态更新和个性化查询的需求。  ##\n");
    printf("###########################################################################\n\n");

    /* 文本合成 */
    printf("开始合成 ...\n");
    ret = text_to_speech(text, filename, session_begin_params);
    if (MSP_SUCCESS != ret)
    {
        printf("text_to_speech failed, error code: %d.\n", ret);
    }
    printf("合成完毕\n");

exit:
    MSPLogout(); //退出登录

    return 0;
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "tts_node");    //初始化ROS节点

    ros::NodeHandle node("~");    //创建句柄

    node.param("source_path", source_path, std::string("/home/passoni/catkin_ws/11111"));  // 从ROS参数服务器获取source_path参数
    node.param("/appid", appid, std::string("111111"));  // 从ROS参数服务器获取appid参数
    node.param("/voice_name", voice_name, std::string("111111"));  // 从ROS参数服务器获取voice_name参数
    node.param("tts_text", tts_text, std::string("111111"));  // 从ROS参数服务器获取tts_text参数
    node.param("/rdn", rdn, 0);  // 从ROS参数服务器获取rdn参数
    node.param("/volume", volume, 0);  // 从ROS参数服务器获取volume参数
    node.param("/pitch", pitch, 0);  // 从ROS参数服务器获取pitch参数
    node.param("/speed", speed, 0);  // 从ROS参数服务器获取speed参数
    node.param("/sample_rate", sample_rate, 0);  // 从ROS参数服务器获取sample_rate参数

    cout<< ">>>>>>> "<< source_path << endl;

    tts_init();  // 初始化并启动语音合成

/*	while(ros::ok())
	{
		
	} */

    //ros::spinOnce(); 
    ros::spin();  // 进入ROS事件循环
     
    return 0;
}