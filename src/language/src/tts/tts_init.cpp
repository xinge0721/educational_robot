#include "tts_init.h"
#include <ros/ros.h>
#include <std_msgs/String.h>  // 添加字符串消息头文件
#include <sys/stat.h>
#include <queue>  // 添加队列头文件
#include <mutex>  // 添加互斥锁头文件
#include <chrono>  // 添加时间头文件，用于生成唯一文件名
using namespace std;  // 使用标准命名空间

// 定义队列和互斥锁用于管理TTS请求
std::queue<std::string> tts_queue;  // 语音合成请求队列
std::mutex queue_mutex;  // 队列互斥锁
bool is_processing = false;  // 是否正在处理语音合成请求的标志

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


bool playAndDeleteAudio(const std::string& file_path) {
    // 检查文件是否存在
    struct stat buffer;
    if (stat(file_path.c_str(), &buffer) != 0) {
        ROS_ERROR("Audio file not found: %s", file_path.c_str());
        return false;
    }

    // 验证文件格式
    if (file_path.substr(file_path.find_last_of(".") + 1) != "wav") {
        ROS_ERROR("Invalid file format (only WAV supported): %s", file_path.c_str());
        return false;
    }

    // 构建播放命令
    std::string command = "aplay -q \"" + file_path + "\"";
    int play_result = system(command.c_str());

    // 处理播放结果
    if (play_result == 0) {
        ROS_INFO("Playback completed: %s", file_path.c_str());
        if (std::remove(file_path.c_str()) != 0) {
            ROS_ERROR("Failed to delete audio file: %s", file_path.c_str());
            return false;
        }
        return true;
    }

    // 错误处理
    if (WIFEXITED(play_result)) {
        ROS_ERROR("Playback failed with exit code %d", WEXITSTATUS(play_result));
    } else {
        ROS_ERROR("Playback terminated abnormally");
    }
    
    // 可选：保留失败文件用于调试
    // ROS_WARN("Keeping failed audio file: %s", file_path.c_str());
    return false;
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

    // 确保音频目录存在
    std::string audio_dir = source_path + "/audio";
    struct stat st;
    if (stat(audio_dir.c_str(), &st) != 0) {
        // 目录不存在，创建它
        printf("音频目录不存在，正在创建: %s\n", audio_dir.c_str());
        
        #ifdef _WIN32
        // Windows系统
        std::string cmd = "mkdir \"" + audio_dir + "\"";
        #else
        // Linux/Unix系统
        std::string cmd = "mkdir -p \"" + audio_dir + "\"";
        #endif
        
        int result = system(cmd.c_str());
        if (result != 0) {
            printf("[ERROR] 无法创建音频目录: %s\n", audio_dir.c_str());
        }
    }

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

// 处理队列中的下一个TTS请求
void processNextTtsRequest() {
    std::string text_to_process;
    
    // 获取互斥锁，检查队列中是否有待处理的请求
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        if (tts_queue.empty()) {
            is_processing = false;
            printf("队列处理完毕，等待新的请求\n");
            return;
        }
        
        // 从队列中取出下一个请求
        text_to_process = tts_queue.front();
        tts_queue.pop();
        
        // 设置处理标志为true
        is_processing = true;
    }
    
    printf("正在处理队列中的文本：%s\n", text_to_process.c_str());
    
    // 生成唯一的文件名，避免文件冲突
    std::string unique_filename = source_path + "/audio/output_" + 
                                  std::to_string(std::chrono::system_clock::now().time_since_epoch().count()) + ".wav";
    
    // 执行文本到语音的转换
    int ret = text_to_speech(text_to_process.c_str(), unique_filename.c_str(), session_fin.c_str());
    
    if (MSP_SUCCESS != ret) {
        printf("合成失败，错误码：%d，跳过当前文本\n", ret);
        
        // 即使当前处理失败，也继续处理队列中的下一个请求
        ros::Duration(0.1).sleep();  // 短暂延迟，避免过快失败导致的资源问题
        processNextTtsRequest();
    } else {
        printf("合成完成，音频已保存至：%s\n", unique_filename.c_str());
        
        // 播放并删除音频文件
        bool play_success = playAndDeleteAudio(unique_filename.c_str());
        
        if (!play_success) {
            printf("播放失败或文件删除失败，继续处理下一个\n");
            // 尝试再次删除文件，确保不会留下临时文件
            std::remove(unique_filename.c_str());
        } else {
            printf("播放成功并已删除音频文件\n");
        }
        
        // 无论播放是否成功，继续处理队列中的下一个请求
        ros::Duration(0.1).sleep();  // 短暂延迟，确保系统有时间处理
        processNextTtsRequest();
    }
}

ros::Publisher audio_pub;
// 新建话题回调函数
void ttsCallback(const std_msgs::String::ConstPtr& msg)
{
    printf("收到合成请求，文本内容：%s\n", msg->data.c_str());
    
    // 检查是否是"对话已结束"特殊标记，如果是则直接返回，不进行处理
    if (msg->data == "对话已结束") {
        printf("收到对话结束标记，跳过处理\n");
        return;
    }
    
    // 获取互斥锁，将新的请求添加到队列中
    {
        std::lock_guard<std::mutex> lock(queue_mutex);
        tts_queue.push(msg->data);
        printf("已将文本添加到队列，当前队列长度：%zu\n", tts_queue.size());
    }
    
    // 如果当前没有正在处理的请求，则开始处理队列
    if (!is_processing) {
        processNextTtsRequest();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tts_node");
    ros::NodeHandle node("~");
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

    // 初始化Publisher（注意使用全局命名空间）
    ros::Publisher audio_pub = node.advertise<std_msgs::String>("/tts_audio_path", 1);

    
    TTS ts;
     tts_init();  // 初始化语音合成引擎

    // 创建订阅者（新增）
    ros::Subscriber sub = node.subscribe("/tts/text", 10, ttsCallback);
    
    // 设置循环频率（可选）
    ros::Rate loop_rate(10);
    
    printf("语音合成节点已启动，等待输入...\n");
    
    // 保持节点运行（修改）
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    MSPLogout(); // 退出登录
    return 0;
}