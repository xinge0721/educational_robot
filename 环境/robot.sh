#########################################################################
# File Name: robot.sh
# Author: Louis Ma
# mail: lynn_mg_study@163.com
# Created Time:2025年06月04日 星期三 09时07分48秒
#########################################################################
#!/bin/bash

# 完整环境安装脚本 (智能模型下载版)
# 使用方法: sudo ./install_script.sh

# 检查sudo权限
if [ "$EUID" -ne 0 ]; then
    echo "错误: 请使用sudo运行此脚本"
    exit 1
fi

# 获取当前用户名和主目录
CURRENT_USER=$(logname)
HOME_DIR="/home/$CURRENT_USER"
echo "检测到当前用户: $CURRENT_USER"
echo "主目录: $HOME_DIR"

# 主目录
MAIN_DIR=$(pwd)

# 创建必要目录
echo "=> 创建目录结构"
mkdir -p moxi chenxu
echo "  创建: moxi/"
echo "  创建: chenxu/"
echo "----------------------------------------"

# 安装函数 - 带进度提示
function install_step {
    local cmd="$1"
    local desc="$2"
    
    echo "=> 开始: $desc"
    echo "----------------------------------------"
    
    # 执行命令并显示输出
    eval $cmd
    
    # 检查结果
    if [ $? -eq 0 ]; then
        echo "[成功] $desc"
    else
        echo "[失败] $desc (错误代码: $?)"
    fi
    
    echo "========================================"
    echo ""
}

# 下载函数
function download_model {
    local url="$1"
    local file="$2"
    local desc="$3"
    
    echo "=> 下载: $desc"
    echo "----------------------------------------"
    wget --show-progress -O "$file" "$url"
    if [ $? -eq 0 ]; then
        echo "[成功] 下载完成"
        return 0
    else
        echo "[失败] 下载失败"
        return 1
    fi
}

# 主安装流程
echo ""
echo "=== 开始安装流程 ==="
echo "========================================"
echo "注意: 安装过程中请勿关闭终端"
echo "========================================"
echo ""

# 系统更新与包安装
install_step "apt update" "更新软件包列表"
install_step "apt install -y vim terminator" "安装 vim 和 terminator"
install_step "apt install -y libasound2-dev sox libsox-fmt-all python3-pip unzip nfs-common git cmake libportaudio2 portaudio19-dev python3-dev" "安装系统依赖包"

# Python包安装
install_step "pip3 install websocket-client vosk sounddevice" "安装 Python 依赖包"

# 处理模型文件
echo "=> 检查模型文件"
echo "----------------------------------------"
cd moxi

# 检查关键模型是否存在
KEY_MODEL="vosk-model-cn-kaldi-multicn-0.15"
KEY_MODEL_ZIP="${KEY_MODEL}.zip"
KEY_MODEL_URL="https://alphacephei.com/vosk/models/${KEY_MODEL}.zip"

# 检查模型文件是否存在
if [ ! -d "$KEY_MODEL" ]; then
    echo "关键模型 '$KEY_MODEL' 未找到"
    
    # 检查是否有压缩包
    if [ ! -f "$KEY_MODEL_ZIP" ]; then
        echo "尝试下载关键模型..."
        download_model "$KEY_MODEL_URL" "$KEY_MODEL_ZIP" "Vosk 关键模型 ($KEY_MODEL)"
        if [ $? -ne 0 ]; then
            echo "⚠️ 警告: 关键模型下载失败，请手动下载并放置到 moxi/ 目录"
        fi
    fi
    
    # 如果有压缩包但未解压
    if [ -f "$KEY_MODEL_ZIP" ]; then
        echo "尝试解压关键模型..."
        unzip -q "$KEY_MODEL_ZIP"
        if [ $? -eq 0 ]; then
            echo "[成功] 关键模型解压完成"
        else
            echo "[失败] 关键模型解压失败"
        fi
    fi
else
    echo "关键模型 '$KEY_MODEL' 已存在"
fi

# 解压所有模型文件并修复嵌套目录问题
for model_zip in *.zip; do
    if [ -f "$model_zip" ]; then
        # 获取基础名称（不带.zip）
        base_name="${model_zip%.zip}"
        
        # 如果目录已存在则跳过
        if [ -d "$base_name" ]; then
            echo "跳过: $base_name/ 已存在"
            continue
        fi
        
        # 创建临时解压目录
        temp_dir="temp_${base_name}"
        mkdir -p "$temp_dir"
        
        echo "解压: $model_zip"
        unzip -q "$model_zip" -d "$temp_dir"
        
        # 查找实际模型目录
        actual_model_dir=$(find "$temp_dir" -maxdepth 2 -type d -name "${base_name}*" -print -quit)
        
        if [ -d "$actual_model_dir" ]; then
            # 移动模型到正确位置
            mv "$actual_model_dir" "$base_name"
            echo "  成功: 模型移动到 $base_name/"
            chmod -R a+r "$base_name"
        else
            echo "  警告: 未找到模型目录，尝试直接解压"
            unzip -q "$model_zip" -d "$base_name"
            if [ -d "$base_name" ]; then
                echo "  成功: 直接解压到 $base_name/"
            else
                echo "  失败: 无法解压 $model_zip"
            fi
        fi
        
        # 清理临时目录
        rm -rf "$temp_dir"
    fi
done

# 返回主目录
cd "$MAIN_DIR"
echo "========================================"
echo ""

# 克隆代码仓库
echo "=> 克隆代码仓库到 chenxu/"
echo "----------------------------------------"
git clone https://github.com/xinge0721/educational_robot.git chenxu
if [ $? -eq 0 ]; then
    echo "[成功] 克隆代码仓库"
else
    echo "[失败] 克隆代码仓库"
fi
echo "========================================"
echo ""

# 处理库文件
echo "=> 处理库文件"
echo "----------------------------------------"

# 检测系统架构
ARCH=$(uname -m)
echo "检测到系统架构: $ARCH"

if [ "$ARCH" = "x86_64" ]; then
    LIB_DIR="x64"
elif [ "$ARCH" = "aarch64" ] || [ "$ARCH" = "armv7l" ]; then
    LIB_DIR="arm64"
else
    LIB_DIR=""
    echo "[警告] 未知架构，跳过库文件安装"
fi

if [ -n "$LIB_DIR" ]; then
    # 复制库文件
    if [ -d "libs/$LIB_DIR" ]; then
        echo "复制 $LIB_DIR 库文件到 /usr/lib"
        sudo cp libs/$LIB_DIR/* /usr/lib/
        echo "  复制完成"
    else
        echo "[错误] libs/$LIB_DIR 目录不存在"
    fi
fi
echo "========================================"
echo ""

# 运行CH9102脚本
if [ -f "ch9102_udev.sh" ]; then
    echo "=> 设置CH9102设备权限"
    echo "----------------------------------------"
    chmod +x ch9102_udev.sh
    ./ch9102_udev.sh
    echo "========================================"
    echo ""
else
    echo "[警告] ch9102_udev.sh 文件不存在，跳过"
fi

# 安装cJSON库 (安装到用户主目录)
if [ -d "cJSON" ]; then
    echo "=> 安装cJSON库到 $HOME_DIR"
    echo "----------------------------------------"
    
    # 复制到用户主目录
    echo "复制cJSON到 $HOME_DIR"
    cp -r cJSON "$HOME_DIR/"
    
    # 编译安装
    cd "$HOME_DIR/cJSON"
    if [ -d "build" ]; then
        echo "清理旧构建目录"
        rm -rf build
    fi
    
    mkdir build
    cd build
    cmake .. > /dev/null
    make > /dev/null
    sudo make install > /dev/null
    
    if [ $? -eq 0 ]; then
        echo "[成功] 安装cJSON"
    else
        echo "[失败] 安装cJSON"
    fi
    
    # 返回原目录
    cd "$MAIN_DIR"
    echo "========================================"
    echo ""
else
    echo "[警告] cJSON目录不存在，跳过安装"
fi

# 配置动态链接库
echo "=> 配置动态链接库"
echo "----------------------------------------"
# 检查是否已存在配置
grep -q "/usr/local/lib" /etc/ld.so.conf || echo "/usr/local/lib" | sudo tee -a /etc/ld.so.conf
sudo /sbin/ldconfig
echo "  更新完成"
echo "========================================"
echo ""

# 创建测试脚本
echo "=> 创建Vosk测试脚本"
cat > test_vosk.py << 'EOL'
import sys
import json
import os
from vosk import Model, KaldiRecognizer
import wave

def test_model(model_path, audio_file):
    """测试单个Vosk模型的功能性"""
    print(f"\n{'='*60}")
    print(f"测试模型: {os.path.basename(model_path)}")
    print(f"{'='*60}")
    
    try:
        # 尝试加载模型
        model = Model(model_path)
        print(f"✅ 模型加载成功")
        
        # 尝试打开音频文件
        wf = wave.open(audio_file, "rb")
        
        # 创建识别器
        rec = KaldiRecognizer(model, wf.getframerate())
        
        # 尝试读取一些音频数据
        data = wf.readframes(4000)
        if len(data) == 0:
            print("⚠️ 警告: 音频文件为空")
            return False
        
        # 尝试处理音频
        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            print("识别片段结果:", result.get('text', ''))
        
        # 尝试获取最终结果
        final_result = json.loads(rec.FinalResult())
        full_text = final_result.get('text', '')
        
        if full_text:
            print("\n识别结果:")
            print("-"*60)
            print(full_text)
            print("-"*60)
            print("✅ 功能测试通过: 模型成功处理音频并返回文本")
            return True
        else:
            print("⚠️ 警告: 模型未返回识别文本")
            return False
            
    except Exception as e:
        print(f"❌ 测试失败: {str(e)}")
        return False

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("用法: python3 test_vosk.py <模型目录> <音频文件>")
        sys.exit(1)
    
    model_dir = sys.argv[1]
    audio_file = sys.argv[2]
    
    # 测试所有模型
    success_count = 0
    total_count = 0
    
    print("测试策略: 验证模型加载和基本识别功能")
    print("注意: 文本结果可能因音频内容而异，主要验证功能完整性")
    
    for model_name in os.listdir(model_dir):
        model_path = os.path.join(model_dir, model_name)
        if os.path.isdir(model_path):
            # 检查是否是有效的Vosk模型
            if os.path.exists(os.path.join(model_path, "am/final.mdl")):
                total_count += 1
                if test_model(model_path, audio_file):
                    success_count += 1
            else:
                print(f"\n{'='*60}")
                print(f"跳过无效模型: {model_name}")
                print("原因: 缺少模型文件 (am/final.mdl 不存在)")
                print(f"{'='*60}")
    
    print("\n" + "="*60)
    print(f"功能测试总结: {success_count}/{total_count} 个模型功能正常")
    print("="*60)
    
    # 检查关键模型
    key_model = "vosk-model-cn-kaldi-multicn-0.15"
    key_model_path = os.path.join(model_dir, key_model)
    if os.path.exists(key_model_path) and os.path.exists(os.path.join(key_model_path, "am/final.mdl")):
        print(f"\n检查关键模型: {key_model}")
        if test_model(key_model_path, audio_file):
            print("✅ 关键模型功能正常")
        else:
            print("❌ 关键模型功能异常 - 需要手动检查")
    else:
        print(f"⚠️ 警告: 关键模型 {key_model} 未找到或无效")
EOL

echo "测试脚本已创建: test_vosk.py"
echo "========================================"
echo ""

# 模型测试
echo "=> 测试Vosk模型"
echo "----------------------------------------"
echo "测试策略: 验证模型加载和基本识别功能"
echo "注意: 文本结果可能因音频内容而异，主要验证功能完整性"

# 检查moxi目录
if [ -d "moxi" ]; then
    echo "模型目录: $(pwd)/moxi"
    
    # 检查是否有解压的模型
    model_count=$(find moxi -maxdepth 1 -type d | grep -v "^moxi$" | wc -l)
    if [ "$model_count" -gt 0 ]; then
        # 检查测试音频
        if [ -f "test.wav" ]; then
            echo "发现测试音频文件: test.wav"
            echo "开始语音识别功能测试..."
            python3 test_vosk.py moxi test.wav
        else
            echo "[警告] test.wav 文件不存在，跳过语音识别测试"
            echo "您可以使用以下命令手动测试:"
            echo "  python3 test_vosk.py moxi /path/to/your/audio.wav"
            echo "音频格式要求: 16kHz, 16bit, 单声道WAV"
        fi
    else
        echo "[警告] moxi目录中没有解压的模型"
        echo "请确保模型文件已解压到moxi/目录"
    fi
else
    echo "[错误] moxi目录不存在"
fi

# 最终提示
echo ""
echo "========================================"
echo "安装完成!"
echo "重要路径:"
echo "  模型位置: $(pwd)/moxi"
echo "  代码位置: $(pwd)/chenxu"
echo "  测试脚本: $(pwd)/test_vosk.py"
echo "  cJSON位置: $HOME_DIR/cJSON"
echo "========================================"
echo "下一步建议:"
echo "1. 重启系统使所有更改生效"
echo "2. 检查chenxu目录中的项目"
echo "3. 在terminator中测试程序"
echo "4. 如果关键模型不存在或测试失败:"
echo "   a. 手动下载模型: https://alphacephei.com/vosk/models"
echo "   b. 将模型zip文件放入 moxi/ 目录"
echo "   c. 重新运行此脚本"
echo "5. 音频格式要求: 16kHz, 16bit, 单声道WAV"
echo "========================================"
