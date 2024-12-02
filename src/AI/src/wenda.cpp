#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <chrono>

// 文件路径
const std::string FILE_1_PATH = "/home/ros/chenxu/jiaoyu/1.txt";    //问题存放路径
const std::string FILE_2_PATH = "/home/ros/chenxu/jiaoyu/2.txt";    //答案存放路径
const std::string END_MARKER = "END_OF_ANSWER"; // 用于标志回答结束

// 函数：检查文件是否存在且有内容
//参数：要检查的文件路径
bool hasContent(const std::string& filePath)
{
    // 尝试以输入模式打开指定路径的文件
    std::ifstream file(filePath);

    // 如果文件无法打开（可能不存在或路径错误），打印警告信息并返回 false
    if (!file.is_open())
    {
        ROS_WARN("文件 %s 无法打开", filePath.c_str());
        return false; // 文件不存在或无法打开，直接返回 false
    }

    // 使用 peek() 函数检查文件的第一个字符，判断文件是否为空
    // 如果 peek() 返回的值不是文件的结束符 (EOF)，说明文件有内容
    return file.peek() != std::ifstream::traits_type::eof();
    // 返回 true 表示文件存在且有内容，返回 false 表示文件为空
}

// 函数：读取文件的全部内容并返回为字符串
// 参数：filePath 是文件路径（std::string类型），指定要读取的文件位置。
// 例如，"/home/ros/chenxu/jiaoyu/2.txt"。
std::string readFromFile(const std::string& filePath)
{
    // 创建一个输入文件流对象，用于读取文件内容
    std::ifstream file(filePath);

    // 使用迭代器读取文件的所有字符，并存储为字符串
    // std::istreambuf_iterator<char> 用于从文件流中逐字节读取数据
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());

    // 关闭文件，释放文件资源
    file.close();

    // 返回读取到的文件内容
    return content;
}

// 函数：将内容追加到文件中
void appendToFile(const std::string& filePath, const std::string& content)
{
    // 参数说明：
    // filePath 是目标文件的路径，类型为 std::string。
    // content 是要写入文件的内容，类型为 std::string。

    // 打开文件，以追加模式写入内容（std::ios::app）。如果文件不存在，会创建新文件。
    std::ofstream file(filePath, std::ios::app); // 追加模式

    // 检查文件是否成功打开
    if (!file.is_open())
    {
        // 如果无法打开文件，打印错误信息并返回
        ROS_ERROR("无法打开文件 %s 进行写入", filePath.c_str());
        return; // 如果无法打开文件，退出函数
    }

    // 将 content 写入文件并在末尾添加换行符
    file << content << std::endl; // 写入内容并换行

    // 关闭文件，保存更改并释放资源
    file.close();
}

// 函数：清空文件内容
void clearFile(const std::string& filePath)
{
    // 参数说明：
    // filePath 是要清空的文件的路径，类型为 std::string。

    // 打开文件，以截断模式打开文件（std::ofstream::trunc）。此模式会清空文件内容。
    std::ofstream file(filePath, std::ofstream::trunc); // trunc 模式清空文件

    // 关闭文件，完成清空操作
    file.close();
}

// 函数：检查回答是否完成（通过检查文件内容中是否包含结束标志）
bool isAnswerComplete(const std::string& filePath)
{
    // 参数说明：
    // filePath：待检查的文件路径，类型为 std::string。表示要检查的文件位置。

    // 尝试以输入模式打开指定路径的文件
    std::ifstream file(filePath);

    // 检查文件是否成功打开
    if (!file.is_open())
    {
        // 如果文件无法打开，打印警告信息并返回 false
        ROS_WARN("文件 %s 无法打开", filePath.c_str());
        return false; // 文件无法打开，表示检查失败
    }

    // 定义一个字符串变量，用于逐行读取文件内容
    std::string line;

    // 逐行读取文件内容
    while (std::getline(file, line))
    {
        // 检查当前行是否包含结束标志（END_MARKER）
        // 如果找到结束标志，表示回答已经完成
        if (line.find(END_MARKER) != std::string::npos) // 检测到结束标志
        {
            // 如果找到结束标志，关闭文件并返回 true
            file.close();
            return true; // 说明回答已完成
        }
    }

    // 如果没有找到结束标志，关闭文件并返回 false
    file.close();
    return false; // 说明回答未完成
}
int main(int argc, char** argv)
{
    // 初始化ROS节点，节点名为 "qa_handler"
    ros::init(argc, argv, "qa_handler");
    ros::NodeHandle nh;
    setlocale(LC_ALL,"");
    // 在程序开始时清空两个文件
    clearFile(FILE_1_PATH);  // 清空问题文件 1.txt
    clearFile(FILE_2_PATH);  // 清空答案文件 2.txt
    // std::cout << "已清空文件 1.txt 和 2.txt，确保启动时文件为空。" << std::endl;

    // 标志位，控制是否允许存入问题
    bool canAsk = true;

    // 用于存储用户输入的字符串
    std::string user_input;

    // ROS主循环，保持节点在运行状态
    while (ros::ok())
    {
        // 如果可以提问（canAsk 为 true），则允许输入问题
        if (canAsk)
        {
            // 提示用户输入问题
            std::cout << "我: ";
            std::getline(std::cin, user_input);  // 从终端读取一行输入

            // 如果用户输入不为空，则将问题保存到 1.txt 文件中
            if (!user_input.empty())
            {
                appendToFile(FILE_1_PATH, user_input);  // 将问题追加到 1.txt

                // 禁止输入新问题，等待答案
                canAsk = false;
            }
            else
            {
                // 如果输入为空，打印警告信息
                ROS_WARN("请说出你的问题");
            }
        }
        else
        {
            // 检查 2.txt 文件是否有回答
            if (hasContent(FILE_2_PATH))
            {
                std::string content;
                bool complete = false;

                // 循环检查 2.txt 是否包含完整回答
                while (!complete)
                {
                    // 每500ms检查一次是否有完整回答
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 延时500ms

                    // 检查文件内容是否包含结束标志，表示回答完成
                    complete = isAnswerComplete(FILE_2_PATH);
                }
                // 读取 2.txt 文件中的内容
                content = readFromFile(FILE_2_PATH);
                // 定义 END_OF_ANSWER 的长度
                size_t markerLength = END_MARKER.length();

                // 如果字符串末尾有足够的字符，删除末尾的 END_OF_ANSWER
               if (content.length() > markerLength) {
                    content.erase(content.length() - markerLength -1);  // 删除末尾的 END_OF_ANSWER 部分
                    }


                // 打印完整的回答
                std::cout << "星火: " << content << std::endl;

                // 清空 2.txt 文件中的内容
                clearFile(FILE_2_PATH);
                // 清空 1.txt 文件中的内容，准备下一个问题
                clearFile(FILE_1_PATH);


                // 允许再次输入新问题
                canAsk = true;
            }
            else
            {
                // 如果 2.txt 中没有答案，输出等待信息
                ROS_INFO("等待回答...");
            }
        }

        // 延时1秒，避免频繁轮询，占用过多CPU资源
        ros::Duration(1.0).sleep();
    }

    // 程序结束时返回
    return 0;
}