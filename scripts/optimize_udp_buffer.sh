#!/bin/bash
#
# UDP 缓冲区优化脚本
# 用于增加 UDP 缓冲区大小以支持高频 LCM 通信
#
# 作者: Han Jiang (jh18954242606@163.com)
# 日期: 2025-12
#

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo "========================================"
echo "  UDP Buffer Optimization Script"
echo "========================================"
echo ""

# 检查是否为 root 用户
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}错误: 此脚本需要 root 权限来修改系统网络参数${NC}"
    echo "请使用: sudo bash $0"
    exit 1
fi

# 默认缓冲区大小（字节）
# 推荐值: 32MB (33554432) 用于高频通信（500Hz LCM消息）
# 可以根据实际需求调整
DEFAULT_RMEM_MAX=33554432    # 32MB 接收缓冲区最大值
DEFAULT_WMEM_MAX=33554432    # 32MB 发送缓冲区最大值
DEFAULT_RMEM_DEFAULT=16777216 # 16MB 接收缓冲区默认值
DEFAULT_WMEM_DEFAULT=16777216 # 16MB 发送缓冲区默认值

# 显示当前UDP缓冲区设置
show_current_settings() {
    echo "当前 UDP 缓冲区设置:"
    echo "----------------------------------------"
    echo "接收缓冲区最大值 (net.core.rmem_max):     $(sysctl -n net.core.rmem_max)"
    echo "发送缓冲区最大值 (net.core.wmem_max):     $(sysctl -n net.core.wmem_max)"
    echo "接收缓冲区默认值 (net.core.rmem_default): $(sysctl -n net.core.rmem_default)"
    echo "发送缓冲区默认值 (net.core.wmem_default): $(sysctl -n net.core.wmem_default)"
    echo ""
}

# 设置UDP缓冲区
set_udp_buffer() {
    local rmem_max=$1
    local wmem_max=$2
    local rmem_default=$3
    local wmem_default=$4
    
    echo -e "${YELLOW}设置 UDP 缓冲区大小...${NC}"
    echo ""
    
    # 设置接收缓冲区
    sysctl -w net.core.rmem_max=$rmem_max >/dev/null 2>&1 && \
        echo -e "${GREEN}✓ 设置 net.core.rmem_max = $rmem_max${NC}" || \
        echo -e "${RED}✗ 设置 net.core.rmem_max 失败${NC}"
    
    sysctl -w net.core.rmem_default=$rmem_default >/dev/null 2>&1 && \
        echo -e "${GREEN}✓ 设置 net.core.rmem_default = $rmem_default${NC}" || \
        echo -e "${RED}✗ 设置 net.core.rmem_default 失败${NC}"
    
    # 设置发送缓冲区
    sysctl -w net.core.wmem_max=$wmem_max >/dev/null 2>&1 && \
        echo -e "${GREEN}✓ 设置 net.core.wmem_max = $wmem_max${NC}" || \
        echo -e "${RED}✗ 设置 net.core.wmem_max 失败${NC}"
    
    sysctl -w net.core.wmem_default=$wmem_default >/dev/null 2>&1 && \
        echo -e "${GREEN}✓ 设置 net.core.wmem_default = $wmem_default${NC}" || \
        echo -e "${RED}✗ 设置 net.core.wmem_default 失败${NC}"
    
    echo ""
}

# 保存配置到 /etc/sysctl.conf 使其永久生效
save_to_sysctl_conf() {
    local rmem_max=$1
    local wmem_max=$2
    local rmem_default=$3
    local wmem_default=$4
    
    local sysctl_file="/etc/sysctl.conf"
    local backup_file="/etc/sysctl.conf.backup.$(date +%Y%m%d_%H%M%S)"
    
    echo -e "${YELLOW}保存配置到 $sysctl_file (永久生效)...${NC}"
    
    # 备份现有配置文件
    if [ -f "$sysctl_file" ]; then
        cp "$sysctl_file" "$backup_file"
        echo "  已备份到: $backup_file"
    fi
    
    # 检查是否已存在相关配置
    if grep -q "^net.core.rmem_max" "$sysctl_file" 2>/dev/null; then
        # 更新现有配置
        sed -i "s/^net.core.rmem_max.*/net.core.rmem_max = $rmem_max/" "$sysctl_file"
        echo "  更新 net.core.rmem_max"
    else
        # 添加新配置
        echo "" >> "$sysctl_file"
        echo "# UDP buffer optimization for LCM high-frequency communication" >> "$sysctl_file"
        echo "# Added by optimize_udp_buffer.sh on $(date)" >> "$sysctl_file"
        echo "net.core.rmem_max = $rmem_max" >> "$sysctl_file"
        echo "  添加 net.core.rmem_max"
    fi
    
    if grep -q "^net.core.rmem_default" "$sysctl_file" 2>/dev/null; then
        sed -i "s/^net.core.rmem_default.*/net.core.rmem_default = $rmem_default/" "$sysctl_file"
        echo "  更新 net.core.rmem_default"
    else
        echo "net.core.rmem_default = $rmem_default" >> "$sysctl_file"
        echo "  添加 net.core.rmem_default"
    fi
    
    if grep -q "^net.core.wmem_max" "$sysctl_file" 2>/dev/null; then
        sed -i "s/^net.core.wmem_max.*/net.core.wmem_max = $wmem_max/" "$sysctl_file"
        echo "  更新 net.core.wmem_max"
    else
        echo "net.core.wmem_max = $wmem_max" >> "$sysctl_file"
        echo "  添加 net.core.wmem_max"
    fi
    
    if grep -q "^net.core.wmem_default" "$sysctl_file" 2>/dev/null; then
        sed -i "s/^net.core.wmem_default.*/net.core.wmem_default = $wmem_default/" "$sysctl_file"
        echo "  更新 net.core.wmem_default"
    else
        echo "net.core.wmem_default = $wmem_default" >> "$sysctl_file"
        echo "  添加 net.core.wmem_default"
    fi
    
    echo ""
    echo -e "${GREEN}配置已保存到 $sysctl_file${NC}"
    echo -e "${YELLOW}注意: 配置将在下次重启后永久生效，或运行 'sudo sysctl -p' 立即生效${NC}"
    echo ""
}

# 主函数
main() {
    # 显示当前设置
    show_current_settings
    
    # 使用推荐值
    rmem_max=$DEFAULT_RMEM_MAX
    wmem_max=$DEFAULT_WMEM_MAX
    rmem_default=$DEFAULT_RMEM_DEFAULT
    wmem_default=$DEFAULT_WMEM_DEFAULT
    
    echo "推荐配置（用于高频LCM通信 500Hz+）:"
    echo "  接收缓冲区最大值: $(printf "%'d" $rmem_max) 字节 ($(($rmem_max / 1024 / 1024)) MB)"
    echo "  发送缓冲区最大值: $(printf "%'d" $wmem_max) 字节 ($(($wmem_max / 1024 / 1024)) MB)"
    echo "  接收缓冲区默认值: $(printf "%'d" $rmem_default) 字节 ($(($rmem_default / 1024 / 1024)) MB)"
    echo "  发送缓冲区默认值: $(printf "%'d" $wmem_default) 字节 ($(($wmem_default / 1024 / 1024)) MB)"
    echo ""
    
    # 询问用户是否使用推荐值
    read -p "是否使用推荐配置? (Y/n): " use_default
    use_default=${use_default:-Y}
    
    if [[ ! "$use_default" =~ ^[Yy] ]]; then
        echo ""
        echo "请输入自定义值（单位: 字节，或输入 'MB' 后缀表示MB，如 '32MB'）:"
        read -p "接收缓冲区最大值 [默认: $(printf "%'d" $rmem_max)]: " custom_rmem_max
        read -p "发送缓冲区最大值 [默认: $(printf "%'d" $wmem_max)]: " custom_wmem_max
        read -p "接收缓冲区默认值 [默认: $(printf "%'d" $rmem_default)]: " custom_rmem_default
        read -p "发送缓冲区默认值 [默认: $(printf "%'d" $wmem_default)]: " custom_wmem_default
        
        # 处理MB后缀
        parse_size() {
            local size=$1
            local default=$2
            if [ -z "$size" ]; then
                echo $default
            elif [[ "$size" =~ MB$ ]]; then
                echo $((${size%MB} * 1024 * 1024))
            elif [[ "$size" =~ ^[0-9]+$ ]]; then
                echo $size
            else
                echo $default
            fi
        }
        
        rmem_max=$(parse_size "$custom_rmem_max" $rmem_max)
        wmem_max=$(parse_size "$custom_wmem_max" $wmem_max)
        rmem_default=$(parse_size "$custom_rmem_default" $rmem_default)
        wmem_default=$(parse_size "$custom_wmem_default" $wmem_default)
    fi
    
    echo ""
    echo "将应用以下配置:"
    echo "  net.core.rmem_max = $rmem_max"
    echo "  net.core.wmem_max = $wmem_max"
    echo "  net.core.rmem_default = $rmem_default"
    echo "  net.core.wmem_default = $wmem_default"
    echo ""
    
    read -p "确认应用配置? (Y/n): " confirm
    confirm=${confirm:-Y}
    
    if [[ ! "$confirm" =~ ^[Yy] ]]; then
        echo "已取消"
        exit 0
    fi
    
    echo ""
    
    # 设置缓冲区
    set_udp_buffer $rmem_max $wmem_max $rmem_default $wmem_default
    
    # 显示新的设置
    echo "新的 UDP 缓冲区设置:"
    echo "----------------------------------------"
    show_current_settings
    
    # 询问是否保存到配置文件
    read -p "是否保存到 /etc/sysctl.conf 使其永久生效? (Y/n): " save_permanent
    save_permanent=${save_permanent:-Y}
    
    if [[ "$save_permanent" =~ ^[Yy] ]]; then
        save_to_sysctl_conf $rmem_max $wmem_max $rmem_default $wmem_default
        
        read -p "是否立即应用配置? (运行 'sysctl -p') (Y/n): " apply_now
        apply_now=${apply_now:-Y}
        
        if [[ "$apply_now" =~ ^[Yy] ]]; then
            echo ""
            echo "应用配置..."
            sysctl -p >/dev/null 2>&1 && \
                echo -e "${GREEN}✓ 配置已应用${NC}" || \
                echo -e "${YELLOW}⚠ 部分配置可能未生效${NC}"
        fi
    fi
    
    echo ""
    echo "========================================"
    echo -e "${GREEN}UDP 缓冲区优化完成!${NC}"
    echo "========================================"
    echo ""
    echo "说明:"
    echo "  - 当前会话的配置已生效"
    if [[ "$save_permanent" =~ ^[Yy] ]]; then
        echo "  - 配置已保存到 /etc/sysctl.conf，重启后仍有效"
    else
        echo "  - 当前配置在重启后会失效"
        echo "  - 如需永久生效，请运行此脚本并选择保存配置"
    fi
    echo ""
    echo "建议:"
    echo "  - 如果仍然遇到消息丢失，可以尝试增加缓冲区大小"
    echo "  - 可以运行 'sysctl -a | grep rmem' 查看所有相关配置"
    echo "  - 可以运行此脚本再次调整配置"
    echo ""
}

# 运行主函数
main

