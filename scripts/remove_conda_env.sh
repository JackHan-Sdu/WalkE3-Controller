#!/bin/bash
# Conda 环境删除脚本
# 用于删除人形机器人控制器的 conda 环境

set -e

echo "=========================================="
echo "  人形机器人控制器 - Conda 环境删除"
echo "=========================================="
echo ""

# 检查 conda 是否安装
if ! command -v conda &> /dev/null; then
    echo "错误: 未找到 conda 命令"
    echo "请先安装 Anaconda 或 Miniconda"
    echo "下载地址: https://www.anaconda.com/products/distribution"
    exit 1
fi

# 初始化 conda（如果尚未初始化）
if [ -z "$CONDA_DEFAULT_ENV" ]; then
    eval "$(conda shell.bash hook)"
fi

# 列出所有环境
echo "当前可用的 conda 环境："
echo ""
conda env list
echo ""

# 提示用户输入环境名
read -p "请输入要删除的 conda 环境名称: " ENV_NAME

if [ -z "$ENV_NAME" ]; then
    echo "错误: 环境名称不能为空"
    exit 1
fi

echo ""
echo "环境名称: $ENV_NAME"
echo ""

# 检查环境是否存在
if ! conda env list | grep -q "^${ENV_NAME}\s"; then
    echo "环境 '$ENV_NAME' 不存在，无需删除"
    exit 0
fi

# 如果当前正在使用该环境，先停用
if [ "$CONDA_DEFAULT_ENV" = "$ENV_NAME" ]; then
    echo "警告: 当前正在使用环境 '$ENV_NAME'"
    echo "正在停用环境..."
    conda deactivate
    echo "环境已停用"
    echo ""
fi

# 确认删除
echo "=========================================="
echo "  确认删除"
echo "=========================================="
echo ""
echo "即将删除环境: $ENV_NAME"
echo ""
read -p "确定要删除吗？(yes/no) [默认: no]: " CONFIRM
CONFIRM=${CONFIRM:-no}

if [ "$CONFIRM" != "yes" ] && [ "$CONFIRM" != "y" ] && [ "$CONFIRM" != "Y" ]; then
    echo "操作已取消"
    exit 0
fi

echo ""
echo "=========================================="
echo "  正在删除环境"
echo "=========================================="
echo ""

# 删除环境
conda env remove -n "$ENV_NAME" -y

echo ""
echo "=========================================="
echo "  删除完成！"
echo "=========================================="
echo ""
echo "环境 '$ENV_NAME' 已成功删除"
echo ""

