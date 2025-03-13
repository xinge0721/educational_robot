#include "tts_init.h"

using namespace std;  // 使用标准命名空间


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


/* 文本合成函数 */
int text_to_speech(const char* src_text, const char* des_path, const char* params)
{
    int          ret          = -1;  // 返回值，初始化为-1
    FILE*        fp           = NULL;  // 文件指针，用于写入WAV文件
    const char*  sessionID    = NULL;  // 会话ID，用于语音合成会话
    unsigned int audio_len    = 0;  // 音频数据长度
    wave_pcm_hdr wav_hdr      = default_wav_hdr;  // WAV文件头
    int          synth_status = MSP_TTS_FLAG_STILL_HAVE_DATA;  // 合成状态
        // 在text_to_speech函数中增加：
        if (synth_status == MSP_TTS_FLAG_DATA_END && audio_len == 0) {
            printf("[WARN] 合成完成但未获取到音频数据\n");
        }

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
    // 在text_to_speech函数中增加：
if (synth_status == MSP_TTS_FLAG_DATA_END && audio_len == 0) {
    printf("[WARN] 合成完成但未获取到音频数据\n");
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

std::string session_fin;
std::string filename_fin;

/* 初始化语音合成 */
void tts_init( )
{
    int         ret                  = MSP_SUCCESS;  // 返回值，初始化为成功
    std::string login_fin     	 	 = "appid = " + appid + ", work_dir = .";  // 拼接登录参数
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
    session_fin			             =  session_ori_1 + voice_name + session_ori_2 + source_path + 
                                        session_ori_3 + source_path + session_ori_4 + std::to_string(sample_rate) + 
                                        session_ori_5 + std::to_string(volume) + session_ori_6 + 
                                        std::to_string(pitch) + session_ori_7 + std::to_string(rdn) + session_ori_8 + std::to_string(speed);

    //cout<< ">>>>>>> session_begin_params:"<< session_begin_params<< endl;

    filename_fin		 = source_path + "/audio/output.wav";  // 生成输出文件名
    // const char* filename             = filename_fin.c_str(); //合成的语音文件名称
    /* 用户登录 */
    ret = MSPLogin(NULL, NULL, login_params); //第一个参数是用户名，第二个参数是密码，第三个参数是登录参数，用户名和密码可在http://www.xfyun.cn注册获取
    if (MSP_SUCCESS != ret) //如果登陆失败，则结束任务
    {
          printf("[ERROR] MSPLogin failed: %d\n", ret);
        printf("MSPLogin failed, error code: %d.\n", ret);
        MSPLogout(); //退出登录
        return;
    }
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "tts_node");    //初始化ROS节点

    ros::NodeHandle node("~");    //创建句柄

    node.param("source_path", source_path, std::string("${workspaceFolder}"));  // 从ROS参数服务器获取source_path参数
    node.param("/appid", appid, std::string("111111"));  // 从ROS参数服务器获取appid参数
    node.param("/voice_name", voice_name, std::string("111111"));  // 从ROS参数服务器获取voice_name参数
    node.param("tts_text", tts_text, std::string("111111"));  // 从ROS参数服务器获取tts_text参数
    node.param("/rdn", rdn, 0);  // 从ROS参数服务器获取rdn参数
    node.param("/volume", volume, 0);  // 从ROS参数服务器获取volume参数
    node.param("/pitch", pitch, 0);  // 从ROS参数服务器获取pitch参数
    node.param("/speed", speed, 0);  // 从ROS参数服务器获取speed参数
    node.param("/sample_rate", sample_rate, 0);  // 从ROS参数服务器获取sample_rate参数

    cout<< ">>>>>>> "<< source_path << endl;

    
    TTS ts;
    tts_init();

  
    /* 文本合成 */
    printf("开始合成 ...\n");
    ts.text                 = "你好小飞"; //合成文本
    ts.ret = text_to_speech(ts.text,filename_fin.c_str(),session_fin.c_str());
    cout << ">>>>>>> Session params: " << session_fin << endl;

    if (MSP_SUCCESS != ts.ret)
    {
        printf("text_to_speech failed, error code: %d.\n", ts.ret);
    }
    printf("合成完毕\n");
/*	while(ros::ok())
	{
		
	} */

    //ros::spinOnce(); 
    // ros::spin();  // 进入ROS事件循环
     
    return 0;
}