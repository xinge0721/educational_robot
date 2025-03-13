#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cstdlib> // 用于执行系统命令

class AudioPlayer
{
public:
    AudioPlayer()
    {
        // 订阅音频路径话题
        sub_ = nh_.subscribe("/tts_audio_path", 1, &AudioPlayer::audioCallback, this);
        ROS_INFO("音频播放节点已启动，等待音频文件路径...");
    }

    void audioCallback(const std_msgs::String::ConstPtr& msg)
    {
        std::string cmd;

        // 根据系统类型构造播放命令
#ifdef _WIN32
        cmd = "start " + msg->data;  // Windows系统使用默认播放器
#elif __linux__
        cmd = "aplay " + msg->data;  // Linux系统使用aplay
#elif __APPLE__
        cmd = "afplay " + msg->data; // macOS系统使用afplay
#endif

        ROS_INFO("正在播放：%s", msg->data.c_str());
        int ret = system(cmd.c_str());
        
        if(ret != 0) {
            ROS_ERROR("播放失败！错误码：%d", ret);
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "audio_player");
    AudioPlayer player;
    ros::spin();
    return 0;
}