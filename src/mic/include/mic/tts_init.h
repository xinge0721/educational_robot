#include <ros/ros.h>  // ROS库，用于ROS节点的创建和管理
#include <time.h>     // 时间库，用于获取当前时间
#include "wav_head.h" // 自定义头文件，可能包含WAV文件头的定义

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


typedef struct TTSParams {
	int         ret;				// 合成状态
    char* text;                 // 合成文本
     char* filename;       // 输出文件名
    char* session_begin_params; // 会话参数
}TTS;