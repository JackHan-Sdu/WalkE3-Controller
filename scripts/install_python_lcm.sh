#!/bin/bash
# 安装 Python LCM 绑定脚本
# 用于在 conda 环境中安装 LCM Python 绑定

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "=========================================="
echo "  安装 Python LCM 绑定"
echo "=========================================="
echo ""

# 检查是否在 conda 环境中
if [ -z "$CONDA_DEFAULT_ENV" ]; then
    echo -e "${YELLOW}警告: 未检测到激活的 conda 环境${NC}"
    echo "请先激活 conda 环境: conda activate <环境名>"
    exit 1
fi

echo -e "${BLUE}当前 conda 环境: $CONDA_DEFAULT_ENV${NC}"
echo -e "${BLUE}Python 路径: $(which python)${NC}"
echo ""

# 方法1: 尝试通过 pip 安装
echo "方法1: 尝试通过 pip 安装..."
if pip install python-lcm 2>/dev/null; then
    echo -e "${GREEN}✓ Python LCM 安装成功（通过 pip python-lcm）${NC}"
    exit 0
fi

if pip install lcm 2>/dev/null; then
    echo -e "${GREEN}✓ Python LCM 安装成功（通过 pip lcm）${NC}"
    exit 0
fi

# 方法2: 尝试通过 conda-forge 安装
echo ""
echo "方法2: 尝试通过 conda-forge 安装..."
if conda install -c conda-forge python-lcm -y 2>/dev/null; then
    echo -e "${GREEN}✓ Python LCM 安装成功（通过 conda-forge）${NC}"
    exit 0
fi

# 方法3: 从系统 LCM 安装位置查找并链接
echo ""
echo "方法3: 查找系统 LCM Python 绑定..."
LCM_PYTHON_PATHS=(
    "/usr/local/lib/python3*/site-packages/lcm"
    "/usr/lib/python3*/dist-packages/lcm"
)

FOUND_LCM_PATH=""
for path_pattern in "${LCM_PYTHON_PATHS[@]}"; do
    for path in $path_pattern; do
        if [ -d "$path" ] && [ -f "$path/__init__.py" ]; then
            FOUND_LCM_PATH="$path"
            echo -e "${GREEN}找到系统 LCM Python 绑定: $FOUND_LCM_PATH${NC}"
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
            echo "创建符号链接: $LCM_LINK -> $FOUND_LCM_PATH"
            ln -sf "$FOUND_LCM_PATH" "$LCM_LINK"
            echo -e "${GREEN}✓ 已创建符号链接${NC}"
            
            # 验证安装
            if python -c "import lcm" 2>/dev/null; then
                echo -e "${GREEN}✓ Python LCM 导入成功${NC}"
                exit 0
            else
                echo -e "${YELLOW}警告: 符号链接已创建，但导入测试失败${NC}"
            fi
        else
            echo -e "${YELLOW}符号链接已存在: $LCM_LINK${NC}"
            if python -c "import lcm" 2>/dev/null; then
                echo -e "${GREEN}✓ Python LCM 已可用${NC}"
                exit 0
            fi
        fi
    fi
fi

# 方法4: 从源码编译安装
echo ""
echo "=========================================="
echo -e "${YELLOW}  需要手动安装 Python LCM 绑定${NC}"
echo "=========================================="
echo ""
echo "所有自动安装方法都失败了。请选择以下方法之一："
echo ""
echo "方法A: 从 LCM 源码编译 Python 绑定（推荐）"
echo "  1. cd /tmp"
echo "  2. git clone https://github.com/lcm-proj/lcm.git"
echo "  3. cd lcm && mkdir build && cd build"
echo "  4. cmake .. -DCMAKE_INSTALL_PREFIX=$CONDA_PREFIX -DLCM_ENABLE_PYTHON=ON"
echo "  5. make"
echo "  6. make install"
echo ""
echo "方法B: 安装系统包（如果可用）"
echo "  sudo apt-get install python3-lcm"
echo "  然后运行此脚本再次尝试链接"
echo ""
echo "方法C: 使用系统 Python（不推荐）"
echo "  如果系统 Python 已有 LCM，可以临时使用系统 Python"
echo ""

exit 1

