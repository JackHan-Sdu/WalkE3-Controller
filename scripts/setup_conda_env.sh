#!/bin/bash
# Conda 环境一键配置脚本
# 用于创建和配置人形机器人控制器的 conda 环境

set -e

# 获取脚本所在目录的绝对路径（项目根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "  人形机器人控制器 - Conda 环境配置"
echo "=========================================="
echo "项目根目录: $PROJECT_ROOT"
echo ""

# 检查 conda 是否安装
if ! command -v conda &> /dev/null; then
    echo "错误: 未找到 conda 命令"
    echo "请先安装 Anaconda 或 Miniconda"
    echo "下载地址: https://www.anaconda.com/products/distribution"
    exit 1
fi

# 初始化 conda
# 优先尝试从常见位置直接 source conda.sh（更可靠）
if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/anaconda3/etc/profile.d/conda.sh"
elif [ -f "$HOME/miniconda/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniconda/etc/profile.d/conda.sh"
elif [ -f "$HOME/anaconda/etc/profile.d/conda.sh" ]; then
    source "$HOME/anaconda/etc/profile.d/conda.sh"
elif [ -f "/opt/conda/etc/profile.d/conda.sh" ]; then
    source "/opt/conda/etc/profile.d/conda.sh"
else
    # 如果找不到 conda.sh，使用 shell hook 方法
    if command -v conda &> /dev/null; then
        eval "$(conda shell.bash hook)"
    fi
fi

# 检查 conda 版本并自动升级（如果版本过旧）
CONDA_VERSION=$(conda --version 2>/dev/null | awk '{print $2}' || echo "unknown")
echo "检测到 conda 版本: $CONDA_VERSION"

# 版本比较函数：检查版本是否低于指定版本
version_lt() {
    local ver1="$1"
    local ver2="$2"
    
    # 提取主版本号、次版本号、修订号
    local v1_major=$(echo "$ver1" | cut -d. -f1)
    local v1_minor=$(echo "$ver1" | cut -d. -f2 | sed 's/[^0-9].*//')
    local v1_patch=$(echo "$ver1" | cut -d. -f3 | sed 's/[^0-9].*//')
    
    local v2_major=$(echo "$ver2" | cut -d. -f1)
    local v2_minor=$(echo "$ver2" | cut -d. -f2 | sed 's/[^0-9].*//')
    local v2_patch=$(echo "$ver2" | cut -d. -f3 | sed 's/[^0-9].*//')
    
    # 默认值为0如果为空
    v1_minor=${v1_minor:-0}
    v1_patch=${v1_patch:-0}
    v2_minor=${v2_minor:-0}
    v2_patch=${v2_patch:-0}
    
    if [ "$v1_major" -lt "$v2_major" ]; then
        return 0  # 版本1 < 版本2
    elif [ "$v1_major" -eq "$v2_major" ]; then
        if [ "$v1_minor" -lt "$v2_minor" ]; then
            return 0
        elif [ "$v1_minor" -eq "$v2_minor" ]; then
            [ "$v1_patch" -lt "$v2_patch" ]
        else
            return 1
        fi
    else
        return 1
    fi
}

# 检查版本是否过旧（低于 4.0.0）
MIN_REQUIRED_VERSION="4.0.0"
if version_lt "$CONDA_VERSION" "$MIN_REQUIRED_VERSION"; then
    echo ""
    echo "⚠️  检测到 conda 版本过旧（当前: $CONDA_VERSION，最低要求: $MIN_REQUIRED_VERSION）"
    echo "必须升级 conda 才能继续配置环境"
    echo ""
    echo "正在自动升级 conda（这可能需要几分钟）..."
    conda update -n base -c defaults conda -y 2>&1 | \
        grep -vE "(^==> WARNING: A newer version|^  current version:|^  latest version:)" | \
        grep -vE "(^Collecting package metadata|^Solving environment)" || true
    
    # 重新获取版本号
    CONDA_VERSION=$(conda --version 2>/dev/null | awk '{print $2}' || echo "unknown")
    echo ""
    
    # 验证升级是否成功
    if version_lt "$CONDA_VERSION" "$MIN_REQUIRED_VERSION"; then
        echo "⚠️  警告: conda 升级后版本仍低于要求（当前: $CONDA_VERSION）"
        echo "请手动运行以下命令升级 conda:"
        echo "  conda update -n base -c defaults conda"
        exit 1
    else
        echo "✓ conda 已升级到版本: $CONDA_VERSION"
        echo ""
        
        # 重新初始化 conda（升级后需要重新加载）
        eval "$(conda shell.bash hook)"
    fi
else
    echo "✓ conda 版本正常（≥ $MIN_REQUIRED_VERSION）"
    echo ""
fi

# 提示用户输入环境名
read -p "请输入 conda 环境名称 [默认: humanoid_controller]: " ENV_NAME
ENV_NAME=${ENV_NAME:-humanoid_controller}

echo ""
echo "环境名称: $ENV_NAME"
echo ""

# 辅助函数：初始化 conda
_init_conda() {
    if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
        source "$HOME/miniconda3/etc/profile.d/conda.sh"
    elif [ -f "$HOME/anaconda3/etc/profile.d/conda.sh" ]; then
        source "$HOME/anaconda3/etc/profile.d/conda.sh"
    elif [ -n "$CONDA_EXE" ]; then
        eval "$("$CONDA_EXE" shell.bash hook)"
    elif command -v conda &> /dev/null; then
        eval "$(conda shell.bash hook)"
    fi
}

# 检查环境是否存在
if conda env list | grep -q "^${ENV_NAME}\s"; then
    echo "环境 '$ENV_NAME' 已存在，将使用现有环境"
    echo "正在激活环境..."
    _init_conda
    conda activate "$ENV_NAME" || {
        echo "警告: conda activate 失败，尝试使用备用方法..."
        _init_conda
        source activate "$ENV_NAME" 2>/dev/null || {
            echo "错误: 无法激活环境，请手动运行: conda activate $ENV_NAME"
            exit 1
        }
    }
    echo "环境已激活"
else
    echo "环境 '$ENV_NAME' 不存在，正在创建新环境..."
    echo "正在创建 conda 环境（这可能需要几分钟，请耐心等待）..."
    echo ""
    
    # 创建 conda 环境（Python 3.8，兼容性较好）
    # 过滤掉 conda 版本警告和元数据收集信息，让输出更清晰
    {
        conda create -n "$ENV_NAME" python=3.8 -y 2>&1 | \
        grep -vE "(^==> WARNING: A newer version|^  current version:|^  latest version:)" | \
        grep -vE "(^Collecting package metadata|^Solving environment)" | \
        grep -v "^$" || true
    }
    
    # 验证环境是否创建成功
    echo ""
    if conda env list | grep -q "^${ENV_NAME}\s"; then
        echo "✓ 环境创建成功"
    else
        echo "✗ 错误: 环境创建失败，请检查 conda 配置"
        exit 1
    fi
    
    # 激活环境
    echo "正在激活环境..."
    # 确保 conda 已初始化（创建环境后可能需要重新初始化）
    _init_conda
    conda activate "$ENV_NAME" || {
        echo "警告: conda activate 失败，尝试使用备用方法..."
        _init_conda
        source activate "$ENV_NAME" 2>/dev/null || {
            echo "错误: 无法激活环境，请手动运行: conda activate $ENV_NAME"
            exit 1
        }
    }
    echo "环境已创建并激活"
fi

echo ""
echo "=========================================="
echo "  安装系统依赖"
echo "=========================================="
echo ""

# 检查是否为 Ubuntu/Debian 系统
if command -v apt-get &> /dev/null; then
    echo "检测到 Ubuntu/Debian 系统，安装系统依赖..."
    
    # 检查是否需要 sudo
    if [ "$EUID" -eq 0 ]; then
        APT_CMD="apt-get"
    else
        APT_CMD="sudo apt-get"
    fi
    
    echo "正在更新软件包列表..."
    # 更新软件包列表，捕获输出以检查是否有 PPA 错误
    UPDATE_OUTPUT=$($APT_CMD update 2>&1) || true
    UPDATE_STATUS=$?
    
    # 显示更新输出，但过滤掉 PPA 相关的警告信息，让输出更清晰
    echo "$UPDATE_OUTPUT" | grep -vE "(没有 Release 文件|无法安全地用该源进行更新|apt-secure)" | grep -vE "^N: " | grep -v "^$" || true
    
    # 检查是否有 PPA 错误（这些错误不影响主要仓库的更新）
    if echo "$UPDATE_OUTPUT" | grep -q "没有 Release 文件"; then
        echo ""
        echo "提示: 检测到某些 PPA 仓库配置问题（不影响主要功能，可忽略）"
        echo "如需修复，可以运行: sudo add-apt-repository --remove ppa:<问题仓库>"
    fi
    
    echo ""
    echo "正在安装系统依赖包..."
    $APT_CMD install -y liblcm-dev libeigen3-dev joystick python3-tk
    
    echo ""
    echo "✓ 系统依赖安装完成"
    
    echo ""
    echo "配置手柄设备权限（/dev/input/js*），便于运行 calibrate_gamepad.py..."
    UDEV_RULE='KERNEL=="js*", MODE="0666"'
    UDEV_RULE_PATH="/etc/udev/rules.d/99-joystick.rules"
    if grep -q 'KERNEL=="js*"' "$UDEV_RULE_PATH" 2>/dev/null; then
        echo "  已存在规则，跳过"
    else
        if [ "$EUID" -eq 0 ]; then
            echo "$UDEV_RULE" > "$UDEV_RULE_PATH"
            udevadm control --reload-rules
            udevadm trigger --subsystem-match=input || true
            echo "  已写入规则并重载 udev（可能需要重新插拔设备）"
        else
            echo "$UDEV_RULE" | sudo tee "$UDEV_RULE_PATH" >/dev/null
            sudo udevadm control --reload-rules
            sudo udevadm trigger --subsystem-match=input || true
            echo "  已通过 sudo 写入规则并重载 udev（可能需要重新插拔设备）"
        fi
    fi
else
    echo "警告: 未检测到 apt-get，请手动安装以下依赖："
    echo "  - liblcm-dev"
    echo "  - libeigen3-dev"
    echo "  - joystick（可选，用于 jstest）"
    echo "  - python3-tk（用于配置文件编辑器 GUI）"
    echo "手柄设备权限示例规则：/etc/udev/rules.d/99-joystick.rules 内容: KERNEL==\"js*\", MODE=\"0666\""
fi

echo ""
echo "=========================================="
echo "  安装 Python 依赖"
echo "=========================================="
echo ""

# 安装 Python 依赖
echo "正在安装 Python 包..."
pip install --upgrade pip

# 安装指定版本的包
echo "正在安装指定版本的 Python 包..."
pip install numpy==1.24.4 mujoco==3.2.3 pyyaml onnx==1.17.0 pillow

# 安装 ONNX Runtime（用于加载 ONNX 模型）
echo "正在安装 ONNX Runtime 1.19.2..."
# 先尝试 pip 安装
pip install onnxruntime==1.19.2 2>&1 | \
    grep -vE "(^==> WARNING: A newer version|^  current version:|^  latest version:)" | \
    grep -vE "(^Collecting onnxruntime|^Using cached|^Already satisfied)" || true

# 验证安装是否成功
if python -c "import onnxruntime; v=onnxruntime.__version__; print('1.19.2' in str(v))" 2>/dev/null | grep -q "True"; then
    echo "✓ ONNX Runtime 1.19.2 安装成功（通过 pip）"
elif python -c "import onnxruntime" 2>/dev/null; then
    VERSION=$(python -c "import onnxruntime; print(onnxruntime.__version__)" 2>/dev/null)
    echo "✓ ONNX Runtime 安装成功（版本: $VERSION，通过 pip）"
else
    echo "pip 安装失败，尝试通过 conda-forge 安装..."
    conda install -c conda-forge onnxruntime=1.19.2 -y 2>&1 | \
        grep -vE "(^==> WARNING: A newer version|^  current version:|^  latest version:)" | \
        grep -vE "(^Collecting package metadata|^Solving environment)" || true
    
    if python -c "import onnxruntime" 2>/dev/null; then
        echo "✓ ONNX Runtime 安装成功（通过 conda-forge）"
    else
        echo "⚠ 警告: 无法自动安装 ONNX Runtime 1.19.2"
        echo "请手动安装: pip install onnxruntime==1.19.2"
    fi
fi

# 安装 PyTorch（CPU 版，供 Algorithm_Template_For_Developer 使用）
echo "正在安装 PyTorch（CPU 版，供算法模板使用）..."
if ! python -c "import torch; print(torch.__version__)" 2>/dev/null | grep -q "2.1.2"; then
    # 使用官方 CPU 源，避免 CUDA 依赖
    pip install torch==2.1.2 --index-url https://download.pytorch.org/whl/cpu 2>&1 | \
        grep -vE "(^==> WARNING: A newer version|^  current version:|^  latest version:)" | \
        grep -vE "(^Collecting torch|^Using cached|^Already satisfied)" || true
    if python -c "import torch; import platform; print(torch.__version__)" 2>/dev/null | grep -q "2.1.2"; then
        echo "✓ PyTorch 2.1.2 (CPU) 安装成功"
    else
        echo "⚠ 警告: PyTorch 未成功安装，请根据平台手动安装适配版本"
        echo "示例: pip install torch==2.1.2 --index-url https://download.pytorch.org/whl/cpu"
    fi
else
    echo "✓ PyTorch 已存在，跳过安装"
fi

# 安装 Python LCM 绑定
echo "正在安装 Python LCM 绑定..."

LCM_INSTALLED=false

# 方法1: 尝试通过 pip 安装 python-lcm
echo "  方法1: 尝试通过 pip 安装 python-lcm..."
if pip install python-lcm 2>&1 | \
    grep -vE "(^==> WARNING: A newer version|^  current version:|^  latest version:)" | \
    grep -vE "(^Collecting python-lcm|^Using cached|^Already satisfied|^ERROR)" | \
    grep -v "^$" | head -5; then
    # 检查是否安装成功
    if python -c "import lcm" 2>/dev/null; then
        echo "  ✓ Python LCM 安装成功（通过 pip python-lcm）"
        LCM_INSTALLED=true
    fi
fi

# 方法2: 如果方法1失败，尝试安装 lcm
if [ "$LCM_INSTALLED" = false ]; then
    echo "  方法2: 尝试通过 pip 安装 lcm..."
    if pip install lcm 2>&1 | \
        grep -vE "(^==> WARNING: A newer version|^  current version:|^  latest version:)" | \
        grep -vE "(^Collecting lcm|^Using cached|^Already satisfied|^ERROR)" | \
        grep -v "^$" | head -5; then
        if python -c "import lcm" 2>/dev/null; then
            echo "  ✓ Python LCM 安装成功（通过 pip lcm）"
            LCM_INSTALLED=true
        fi
    fi
fi

# 方法3: 尝试从系统 LCM 安装位置查找并链接
if [ "$LCM_INSTALLED" = false ]; then
    echo "  方法3: 查找系统 LCM Python 绑定..."
    LCM_PYTHON_PATHS=(
        "/usr/local/lib/python3*/site-packages/lcm"
        "/usr/lib/python3*/dist-packages/lcm"
    )
    
    FOUND_LCM_PATH=""
    for path_pattern in "${LCM_PYTHON_PATHS[@]}"; do
        for path in $path_pattern; do
            if [ -d "$path" ] && [ -f "$path/__init__.py" ]; then
                FOUND_LCM_PATH="$path"
                break
            fi
        done
        [ -n "$FOUND_LCM_PATH" ] && break
    done
    
    if [ -n "$FOUND_LCM_PATH" ]; then
        # 获取 conda 环境的 site-packages 目录
        CONDA_SITE_PACKAGES=$(python -c "import site; print(site.getsitepackages()[0])" 2>/dev/null)
        
        if [ -n "$CONDA_SITE_PACKAGES" ] && [ -d "$CONDA_SITE_PACKAGES" ]; then
            LCM_LINK="$CONDA_SITE_PACKAGES/lcm"
            if [ ! -e "$LCM_LINK" ]; then
                echo "  创建符号链接: $LCM_LINK -> $FOUND_LCM_PATH"
                ln -sf "$FOUND_LCM_PATH" "$LCM_LINK" 2>/dev/null || true
            fi
            
            # 验证安装
            if python -c "import lcm" 2>/dev/null; then
                echo "  ✓ Python LCM 安装成功（通过系统 LCM 链接）"
                LCM_INSTALLED=true
            fi
        fi
    fi
fi

# 如果所有自动方法都失败
if [ "$LCM_INSTALLED" = false ]; then
    echo ""
    echo "⚠  警告: 无法自动安装 Python LCM 绑定"
    echo ""
    echo "LCM Python 绑定无法通过 pip 或 conda 自动安装。"
    echo "可以使用以下方法之一手动安装："
    echo ""
    echo "方法A: 运行专用安装脚本（推荐）"
    echo "  ./scripts/install_python_lcm.sh"
    echo ""
    echo "方法B: 从源码编译安装"
    echo "  1. cd /tmp && git clone https://github.com/lcm-proj/lcm.git"
    echo "  2. cd lcm && mkdir build && cd build"
    echo "  3. cmake .. -DCMAKE_INSTALL_PREFIX=\$CONDA_PREFIX -DLCM_ENABLE_PYTHON=ON"
    echo "  4. make && make install"
    echo ""
    echo "注意: 仿真模式可以在没有 LCM 的情况下运行（功能受限）"
    echo ""
fi

# 安装 GUI 工具依赖（配置文件编辑器等）
echo ""
echo "正在安装 GUI 工具依赖..."
GUI_PACKAGES_INSTALLED=true

# Pillow 用于配置文件编辑器的图片处理
echo "  检查 Pillow（图片处理库）..."
if ! python -c "from PIL import Image, ImageTk, ImageEnhance" 2>/dev/null; then
    echo "  正在安装 Pillow..."
    if pip install pillow 2>&1 | \
        grep -vE "(^==> WARNING: A newer version|^  current version:|^  latest version:)" | \
        grep -vE "(^Collecting pillow|^Using cached|^Already satisfied)" | \
        grep -v "^$" | head -5; then
        if python -c "from PIL import Image, ImageTk, ImageEnhance" 2>/dev/null; then
            echo "  ✓ Pillow 安装成功"
        else
            echo "  ⚠ 警告: Pillow 安装失败（配置文件编辑器的背景图片功能可能不可用）"
            GUI_PACKAGES_INSTALLED=false
        fi
    else
        echo "  ⚠ 警告: Pillow 安装失败（配置文件编辑器的背景图片功能可能不可用）"
        GUI_PACKAGES_INSTALLED=false
    fi
else
    echo "  ✓ Pillow 已安装"
fi

# 检查 tkinter（Python GUI 库，通常为标准库）
echo "  检查 tkinter（GUI 库）..."
if ! python -c "import tkinter" 2>/dev/null; then
    echo "  ⚠ 警告: tkinter 未安装（配置文件编辑器无法运行）"
    echo "  在 Ubuntu/Debian 系统上，请运行: sudo apt-get install python3-tk"
    GUI_PACKAGES_INSTALLED=false
else
    echo "  ✓ tkinter 已安装"
fi

if [ "$GUI_PACKAGES_INSTALLED" = true ]; then
    echo ""
    echo "✓ GUI 工具依赖安装完成"
    echo "  配置文件编辑器可用: ./scripts/config_editor.py"
fi

echo ""
echo "=========================================="
echo "  配置库路径"
echo "=========================================="
echo ""

# 设置库路径
LIB_PATH="$PROJECT_ROOT/lib"
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$LIB_PATH

# 创建激活脚本，用于在 conda 环境中自动设置库路径
ACTIVATE_SCRIPT="$CONDA_PREFIX/etc/conda/activate.d/humanoid_controller.sh"
DEACTIVATE_SCRIPT="$CONDA_PREFIX/etc/conda/deactivate.d/humanoid_controller.sh"

mkdir -p "$CONDA_PREFIX/etc/conda/activate.d"
mkdir -p "$CONDA_PREFIX/etc/conda/deactivate.d"

# 创建激活脚本
cat > "$ACTIVATE_SCRIPT" << EOF
#!/bin/bash
# 自动设置库路径
export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:$LIB_PATH
EOF

# 创建停用脚本
cat > "$DEACTIVATE_SCRIPT" << EOF
#!/bin/bash
# 移除库路径
export LD_LIBRARY_PATH=\$(echo \$LD_LIBRARY_PATH | sed "s|:$LIB_PATH||g" | sed "s|$LIB_PATH:||g" | sed "s|$LIB_PATH||g")
EOF

chmod +x "$ACTIVATE_SCRIPT"
chmod +x "$DEACTIVATE_SCRIPT"

echo "库路径已配置: $LIB_PATH"
echo "已创建自动激活/停用脚本"

echo ""
echo "=========================================="
echo "  配置完成！"
echo "=========================================="
echo ""
echo "环境名称: $ENV_NAME"
echo "项目根目录: $PROJECT_ROOT"
echo ""
echo "使用方法:"
echo "  1. 激活环境: conda activate $ENV_NAME"
echo "  2. 启动仿真环境: cd $PROJECT_ROOT && python mujoco_sim/mujoco_simulator.py"
echo "  3. 启动算法（在另一个终端）: cd $PROJECT_ROOT && bash scripts/run_controller.sh"
echo ""
echo "注意:"
echo "  - 每次使用前请先激活 conda 环境: conda activate $ENV_NAME"
echo "  - 库路径会在激活环境时自动设置"
echo "  - ONNX Runtime 库文件已包含在 lib/ 目录中，无需额外安装"
echo "  - 配置文件编辑器需要 tkinter 和 Pillow（已自动安装）"
echo ""

