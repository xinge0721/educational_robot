#include <com_mic.h>  // 麦克风通信相关头文件
#include <ros/ros.h>  // ROS 核心头文件
#include <iostream>   // 输入输出流
#include <string.h>   // 字符串操作函数
#include <record.h>   // 录音相关头文件
#include "jsoncpp/json/json.h"  // JSON 解析库
#include <std_msgs/Int32.h>     // ROS 标准消息类型：32位整数
#include <std_msgs/Int8.h>      // ROS 标准消息类型：8位整数
#include <std_msgs/String.h>    // ROS 标准消息类型：字符串

using namespace std;  // 使用标准命名空间

// 定义 ROS 发布者对象
ros::Publisher awake_flag_pub;       // 发布唤醒标志位
ros::Publisher voice_flag_pub;       // 发布麦克风设备状态标志位
ros::Publisher voice_words_pub;      // 发布识别到的语音内容
ros::Publisher pub_awake_angle;      // 发布唤醒角度

// 定义话题名称
std::string awake_flag = "awake_flag";          // 唤醒标志位话题名称
std::string voice_flag = "voice_flag";          // 麦克风设备状态标志位话题名称
std::string voice_words = "voice_words";        // 语音内容话题名称
std::string awake_angle_topic = "/mic/awake/angle";  // 唤醒角度话题名称
string usart_port_name;  // 串口设备名称

// 定义接收数据的缓冲区
unsigned char Receive_Data[1024] = {0};  // 接收数据的缓冲区
int angle_int = 0;  // 存储解析出的唤醒角度
int if_awake = 0;   // 唤醒标志位

// 定义唤醒词
char awake_words[30] = "你好小微";  // 唤醒词

/**************************************************************************
函数功能：处理接收到的数据
入口参数：buffer - 接收到的单字节数据
返回  值：0 - 成功
**************************************************************************/
int deal_with(unsigned char buffer)
{
    static int count = 0, frame_len = 0, msg_id = 0;  // 计数器、帧长度、消息ID
    Receive_Data[count] = buffer;  // 将接收到的数据存入缓冲区

    // 检查帧头和用户ID
    if (Receive_Data[0] != FRAME_HEADER || (count == 1 && Receive_Data[1] != USER_ID)) {
        count = 0, frame_len = 0, msg_id = 0;  // 如果帧头或用户ID不匹配，重置计数器
    } else {
        count++;  // 计数器加1
    }

    // 解析帧长度和消息ID
    if (count == 7) {
        msg_id = data_trans(Receive_Data[6], Receive_Data[5]);  // 解析消息ID
        frame_len = data_trans(Receive_Data[4], Receive_Data[3]) + 7 + 1;  // 解析帧长度
    }

    // 处理完整帧数据
    if (count == frame_len) {
        char str[1024] = {0};  // 用于存储解析出的字符串
        switch (Receive_Data[2]) {  // 根据消息类型处理数据
            case 0X01:  // 消息类型 0X01
                /*
                if (check_sum(frame_len - 1) == Receive_Data[frame_len - 1]) {
                    for (int i = 0; i < frame_len; i++) {
                        printf("%x ", Receive_Data[i]);
                    }
                    printf("\n");
                } else {
                    printf("check failed !\n");
                }
                */
                break;

            case 0X04:  // 消息类型 0X04
                if (check_sum(frame_len - 1) == Receive_Data[frame_len - 1]) {  // 校验和检查
                    if_awake = 1;  // 设置唤醒标志位
                    for (int i = 0; i < frame_len - 8; i++) {
                        str[i] = Receive_Data[i + 7];  // 提取有效数据
                    }

                    // 解析 JSON 数据
                    Json::Reader reader;
                    Json::Value value;
                    Json::Value value_iwv;

                    if (reader.parse(str, value)) {  // 解析 JSON 字符串
                        Json::Value content = value["content"];  // 获取 content 字段
                        std::string iwv_msg = content["info"].asString();  // 获取 info 字段

                        if (reader.parse(iwv_msg, value_iwv)) {  // 解析 info 字段
                            angle_int = value_iwv["ivw"]["angle"].asInt();  // 获取角度值
                        }
                    } else {
                        cout << "reader json fail!" << endl;  // JSON 解析失败
                    }
                } else {
                    printf("check failed !\n");  // 校验和失败
                }
                break;

            default:
                break;
        }

        // 重置计数器和缓冲区
        count = 0, frame_len = 0, msg_id = 0;
        memset(Receive_Data, 0, 1024);  // 清空接收缓冲区
    }
    return 0;
}

/**************************************************************************
函数功能：计算校验和
入口参数：count_num - 需要计算校验和的数据长度
返回  值：校验和
**************************************************************************/
unsigned char check_sum(int count_num)
{
    unsigned char check_sum = 0;
    for (int i = 0; i < count_num; i++) {
        check_sum = check_sum + Receive_Data[i];  // 累加计算校验和
    }
    return ~check_sum + 1;  // 返回校验和
}

/**************************************************************************
函数功能：将两个字节数据转换为一个16位整数
入口参数：data_high - 高字节数据
         data_low - 低字节数据
返回  值：转换后的16位整数
**************************************************************************/
short data_trans(unsigned char data_high, unsigned char data_low)
{
    short transition_16 = 0;
    transition_16 |= data_high << 8;  // 高字节左移8位
    transition_16 |= data_low;        // 低字节直接赋值
    return transition_16;
}

/**************************************************************************
函数功能：打开串口
入口参数：uartname - 串口设备名称
返回  值：文件描述符
**************************************************************************/
int open_port(const char* uartname)
{
    int fd = open(uartname, O_RDWR | O_NOCTTY | O_NONBLOCK);  // 打开串口设备
    if (-1 == fd) {
        perror("Can't Open Serial Port");  // 打开失败
        return -1;
    }

    // 恢复串口为阻塞状态
    if (fcntl(fd, F_SETFL, 0) < 0) {
        printf("fcntl failed!\n");
    }

    // 测试是否为终端设备
    if (isatty(STDIN_FILENO) == 0) {
        printf("standard input is not a terminal device\n");
    }

    return fd;  // 返回文件描述符
}

/**************************************************************************
函数功能：设置串口参数
入口参数：fd - 文件描述符
         nSpeed - 波特率
         nBits - 数据位
         nEvent - 校验位
         nStop - 停止位
返回  值：0 - 成功
**************************************************************************/
int set_opt(int fd, int nSpeed, int nBits, unsigned char nEvent, int nStop)
{
    struct termios newtio, oldtio;
    if (tcgetattr(fd, &oldtio) != 0) {  // 获取当前串口配置
        perror("SetupSerial 1");
        return -1;
    }

    bzero(&newtio, sizeof(newtio));  // 清空新配置
    newtio.c_cflag |= CLOCAL | CREAD;  // 启用本地连接和接收
    newtio.c_cflag &= ~CSIZE;  // 清除数据位掩码

    // 设置数据位
    switch (nBits) {
        case 7:
            newtio.c_cflag |= CS7;
            break;
        case 8:
            newtio.c_cflag |= CS8;
            break;
    }

    // 设置校验位
    switch (nEvent) {
        case 'O':  // 奇校验
            newtio.c_cflag |= PARENB;
            newtio.c_cflag |= PARODD;
            newtio.c_iflag |= (INPCK | ISTRIP);
            break;
        case 'E':  // 偶校验
            newtio.c_iflag |= (INPCK | ISTRIP);
            newtio.c_cflag |= PARENB;
            newtio.c_cflag &= ~PARODD;
            break;
        case 'N':  // 无校验
            newtio.c_cflag &= ~PARENB;
            break;
    }

    // 设置波特率
    switch (nSpeed) {
        case 2400:
            cfsetispeed(&newtio, B2400);
            cfsetospeed(&newtio, B2400);
            break;
        case 4800:
            cfsetispeed(&newtio, B4800);
            cfsetospeed(&newtio, B4800);
            break;
        case 9600:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
        case 115200:
            cfsetispeed(&newtio, B115200);
            cfsetospeed(&newtio, B115200);
            break;
        case 460800:
            cfsetispeed(&newtio, B460800);
            cfsetospeed(&newtio, B460800);
            break;
        case 921600:
            printf("B921600\n");
            cfsetispeed(&newtio, B921600);
            cfsetospeed(&newtio, B921600);
            break;
        default:
            cfsetispeed(&newtio, B9600);
            cfsetospeed(&newtio, B9600);
            break;
    }

    // 设置停止位
    if (nStop == 1) {
        newtio.c_cflag &= ~CSTOPB;  // 1位停止位
    } else if (nStop == 2) {
        newtio.c_cflag |= CSTOPB;   // 2位停止位
    }

    newtio.c_cc[VTIME] = 0;  // 设置超时时间
    newtio.c_cc[VMIN] = 0;   // 设置最小读取字节数

    tcflush(fd, TCIFLUSH);  // 清空输入缓冲区
    if (tcsetattr(fd, TCSANOW, &newtio) != 0) {  // 应用新配置
        perror("com set error");
        return -1;
    }

    return 0;  // 成功
}

/**************************************************************************
函数功能：主函数
入口参数：argc - 命令行参数个数
         argv - 命令行参数数组
返回  值：0 - 成功
**************************************************************************/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "wheeltec_mic");  // 初始化 ROS 节点
    ros::NodeHandle node;  // 创建 ROS 句柄

    // 创建话题发布者
    awake_flag_pub = node.advertise<std_msgs::Int8>(awake_flag, 1);  // 发布唤醒标志位
    voice_flag_pub = node.advertise<std_msgs::Int8>(voice_flag, 1);  // 发布麦克风设备状态标志位
    voice_words_pub = node.advertise<std_msgs::String>(voice_words, 1);  // 发布语音内容
    pub_awake_angle = node.advertise<std_msgs::Int32>(awake_angle_topic, 1);  // 发布唤醒角度

    ros::NodeHandle private_n("~");  // 创建私有命名空间句柄
    private_n.param<std::string>("usart_port_name", usart_port_name, "/dev/wheeltec_mic");  // 获取串口设备名称

    int fd = 1, read_num = 0;  // 文件描述符和读取字节数
    unsigned char buffer[1];  // 接收缓冲区
    memset(buffer, 0, 1);  // 清空缓冲区
    const char* uartname = usart_port_name.c_str();  // 获取串口设备名称

    // 打开串口设备
    if ((fd = open_port(uartname)) < 0) {
        printf("open %s is failed\n", uartname);
        printf(">>>>>无法打开麦克风设备，尝试重新连接进行测试\n");
        return 0;
    } else {
        set_opt(fd, 115200, 8, 'N', 1);  // 设置串口参数
        printf(">>>>>成功打开麦克风设备\n");
        printf(">>>>>唤醒词为:\"%s!\"\n", awake_words);

        // 发布麦克风设备状态标志位
        for (int i = 0; i < 3; ++i) {
            std_msgs::Int8 voice_flag_msg;
            voice_flag_msg.data = 1;
            voice_flag_pub.publish(voice_flag_msg);
            sleep(1.0);
        }
    }

    // 主循环
    while (ros::ok()) {
        memset(buffer, 0, 1);  // 清空接收缓冲区
        read_num = read(fd, buffer, 1);  // 读取串口数据

        if (read_num > 0) {
            deal_with(buffer[0]);  // 处理接收到的数据
        }

        // 如果唤醒标志位为1，发布相关话题
        if (if_awake) {
            printf(">>>>>唤醒角度为:%d\n", angle_int);  // 打印唤醒角度

            std_msgs::Int32 awake_angle;
            awake_angle.data = angle_int;
            pub_awake_angle.publish(awake_angle);  // 发布唤醒角度

            std_msgs::Int8 awake_flag_msg;
            awake_flag_msg.data = 1;
            awake_flag_pub.publish(awake_flag_msg);  // 发布唤醒标志位

            std_msgs::String msg;
            msg.data = "小车唤醒";
            
            voice_words_pub.publish(msg);  // 发布语音内容

            sleep(0.8);  // 延时
            if_awake = 0;  // 重置唤醒标志位
        }

        ros::spinOnce();  // 处理 ROS 回调
    }

    return 0;  // 程序结束
}