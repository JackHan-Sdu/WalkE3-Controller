#!/bin/bash
#
# 人形机器人控制器运行脚本
# 用于运行 human_ctrl 控制器程序
#
# 作者: Han Jiang (jh18954242606@163.com)
# 日期: 2025-12
#


# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# 检测是否在打包后的目录结构还是开发环境
# 打包后的结构：脚本在 scripts/ 目录下，且上一级目录有 human_ctrl
# 开发环境：脚本在 scripts/ 目录下，且上一级目录的 build/ 下有 human_ctrl
if [ "$(basename "${SCRIPT_DIR}")" = "scripts" ] && [ -f "${SCRIPT_DIR}/../human_ctrl" ]; then
    # 打包后的结构：脚本在 scripts/ 目录下，human_ctrl 在上一级目录
    PROJECT_ROOT="${SCRIPT_DIR}/.."
    CONTROLLER_EXE="${PROJECT_ROOT}/human_ctrl"
elif [ -f "${SCRIPT_DIR}/human_ctrl" ]; then
    # 兼容旧打包结构：脚本和 human_ctrl 在同一目录（build/ 目录下）
    PROJECT_ROOT="${SCRIPT_DIR}"
    CONTROLLER_EXE="${PROJECT_ROOT}/human_ctrl"
else
    # 开发环境：脚本在 scripts/ 目录下
    PROJECT_ROOT="${SCRIPT_DIR}/.."
    CONTROLLER_EXE="${PROJECT_ROOT}/build/user/MIT_Controller/human_ctrl"
fi

# 检查可执行文件是否存在
if [ ! -f "${CONTROLLER_EXE}" ]; then
    echo -e "${RED}错误: 找不到控制器可执行文件: ${CONTROLLER_EXE}${NC}"
    echo "请先编译项目: cd build && make -j4"
    exit 1
fi

# 检查配置文件是否存在
if [ ! -f "${PROJECT_ROOT}/config.yaml" ]; then
    echo -e "${YELLOW}警告: 找不到配置文件: ${PROJECT_ROOT}/config.yaml${NC}"
    echo "控制器将尝试在当前目录查找配置文件"
fi

# 检查 URDF 文件是否存在
if [ ! -f "${PROJECT_ROOT}/resources/robots/e3/e3.urdf" ]; then
    echo -e "${YELLOW}警告: 找不到 URDF 文件: ${PROJECT_ROOT}/resources/robots/e3/e3.urdf${NC}"
fi

# 检查 ONNX 模型文件是否存在
if [ ! -d "${PROJECT_ROOT}/actor_model" ]; then
    echo -e "${YELLOW}警告: 找不到 ONNX 模型目录: ${PROJECT_ROOT}/actor_model${NC}"
fi

# 设置库路径
if [ "$(basename "${SCRIPT_DIR}")" = "scripts" ] && [ -f "${SCRIPT_DIR}/../human_ctrl" ]; then
    # 打包后的结构：可执行文件在项目根目录，库在 lib/ 下
    export LD_LIBRARY_PATH="${PROJECT_ROOT}/lib:${PROJECT_ROOT}:${LD_LIBRARY_PATH}"
elif [ -f "${SCRIPT_DIR}/human_ctrl" ]; then
    # 兼容旧打包结构：可执行文件和库在同级或 lib/ 目录
    export LD_LIBRARY_PATH="${PROJECT_ROOT}/lib:${PROJECT_ROOT}:${LD_LIBRARY_PATH}"
    # 查找 onnxruntime 库（打包后的结构）
    ONNXRUNTIME_PATHS=(
        "${PROJECT_ROOT}/libonnxruntime.so.1"
        "${PROJECT_ROOT}/libonnxruntime.so"
        "/usr/lib/onnxruntime-linux-x64-1.20.1/lib/libonnxruntime.so.1"
        "/usr/local/lib/libonnxruntime.so.1"
        "/usr/lib/x86_64-linux-gnu/libonnxruntime.so.1"
        "/opt/onnxruntime/lib/libonnxruntime.so.1"
    )
else
    # 开发环境：库文件在各自的子目录下
    export LD_LIBRARY_PATH="${PROJECT_ROOT}/lib:${PROJECT_ROOT}/build/common:${PROJECT_ROOT}/build/robot:${PROJECT_ROOT}/build/third-party/ParamHandler:${PROJECT_ROOT}/build/third-party/lord_imu:${PROJECT_ROOT}/build/third-party/SOEM:${PROJECT_ROOT}/build/third-party/vectornav:${PROJECT_ROOT}/third-party/onnx/lib:${LD_LIBRARY_PATH}"
    # 查找 onnxruntime 库（开发环境）
ONNXRUNTIME_PATHS=(
    "${PROJECT_ROOT}/third-party/onnx/lib/libonnxruntime.so.1"
    "${PROJECT_ROOT}/third-party/onnx/lib/libonnxruntime.so"
    "/usr/lib/onnxruntime-linux-x64-1.20.1/lib/libonnxruntime.so.1"
    "/usr/local/lib/libonnxruntime.so.1"
    "/usr/lib/x86_64-linux-gnu/libonnxruntime.so.1"
    "/opt/onnxruntime/lib/libonnxruntime.so.1"
)
fi

# 查找 onnxruntime 库
ONNXRUNTIME_LIB=""

for path in "${ONNXRUNTIME_PATHS[@]}"; do
    if [ -f "${path}" ]; then
        ONNXRUNTIME_LIB="${path}"
        break
    fi
done

# 如果找到了库，添加到库路径
if [ -n "${ONNXRUNTIME_LIB}" ]; then
    ONNXRUNTIME_DIR=$(dirname "${ONNXRUNTIME_LIB}")
    export LD_LIBRARY_PATH="${ONNXRUNTIME_DIR}:${LD_LIBRARY_PATH}"
else
    echo -e "${YELLOW}警告: 找不到 onnxruntime 库${NC}"
    echo "请确保已安装 onnxruntime 或设置正确的库路径"
fi

# 切换到项目根目录（控制器会在此查找配置文件）
cd "${PROJECT_ROOT}"

# 设置 Python 环境变量（用于算法脚本启动）
# 确保 Python 脚本能够找到项目模块，同时保留系统 Python 包的访问
if [ -z "${PYTHONPATH}" ]; then
    export PYTHONPATH="${PROJECT_ROOT}"
else
    # 如果 PYTHONPATH 已存在，追加项目路径（避免重复）
    if [[ ":${PYTHONPATH}:" != *":${PROJECT_ROOT}:"* ]]; then
        export PYTHONPATH="${PROJECT_ROOT}:${PYTHONPATH}"
    fi
fi


# 显示运行信息
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  人形机器人控制器启动${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "${BLUE}工作目录: ${PROJECT_ROOT}${NC}"
echo ""

# 保存环境变量（用于 sudo 时传递）
SAVED_LD_LIBRARY_PATH="${LD_LIBRARY_PATH}"
SAVED_PYTHONPATH="${PYTHONPATH}"
SAVED_HOME="${HOME}"
SAVED_CONDA_PREFIX="${CONDA_PREFIX}"
SAVED_CONDA_DEFAULT_ENV="${CONDA_DEFAULT_ENV}"
SAVED_PATH="${PATH}"

# 如果通过sudo运行，获取原始用户信息以找到conda环境
if [ "$EUID" -eq 0 ] && [ -n "$SUDO_USER" ]; then
    # 通过sudo运行，需要从原始用户的HOME目录查找conda
    ORIGINAL_USER="$SUDO_USER"
    ORIGINAL_HOME=$(getent passwd "$SUDO_USER" 2>/dev/null | cut -d: -f6)
    
    # 如果getent失败，尝试从/etc/passwd读取
    if [ -z "$ORIGINAL_HOME" ]; then
        ORIGINAL_HOME=$(grep "^$SUDO_USER:" /etc/passwd | cut -d: -f6)
    fi
    
    # 如果还是找不到，使用常见的home目录路径
    if [ -z "$ORIGINAL_HOME" ] || [ ! -d "$ORIGINAL_HOME" ]; then
        ORIGINAL_HOME="/home/$SUDO_USER"
    fi
    
    # 从config.yaml读取conda配置
    CONDA_ENV_NAME=$(python3 -c "import yaml, sys; f=open('${PROJECT_ROOT}/config.yaml'); config=yaml.safe_load(f); print(config.get('algorithm_launcher', {}).get('conda_env_name', 'humanoid_controller'))" 2>/dev/null || echo "humanoid_controller")
    CONDA_SEARCH_PATHS_STR=$(python3 -c "import yaml, sys; f=open('${PROJECT_ROOT}/config.yaml'); config=yaml.safe_load(f); paths=config.get('algorithm_launcher', {}).get('conda_search_paths', []); print(' '.join(paths))" 2>/dev/null || echo "")
    
    # 将读取到的路径字符串转换为数组，并展开$HOME变量
    read -r -a CONDA_SEARCH_PATHS <<< "$CONDA_SEARCH_PATHS_STR"
    for i in "${!CONDA_SEARCH_PATHS[@]}"; do
        CONDA_SEARCH_PATHS[$i]=$(echo "${CONDA_SEARCH_PATHS[$i]}" | sed "s|\$HOME|$ORIGINAL_HOME|g")
    done
    
    # 尝试找到conda环境
    FOUND_CONDA_ENV=""
    for conda_env_path in "${CONDA_SEARCH_PATHS[@]}"; do
        if [ -d "$conda_env_path" ] && [ -f "$conda_env_path/bin/python" ]; then
            FOUND_CONDA_ENV="$conda_env_path"
            export CONDA_PREFIX="$conda_env_path"
            export CONDA_DEFAULT_ENV="$CONDA_ENV_NAME"
            conda_base=$(dirname "$conda_env_path")
            export PATH="$conda_env_path/bin:$conda_base/bin:$PATH"
            echo -e "${GREEN}[RunController] Conda环境: ${FOUND_CONDA_ENV}${NC}"
            break
        fi
    done
    
    # 如果没找到conda环境，尝试使用保存的CONDA_PREFIX（如果存在）
    if [ -z "$FOUND_CONDA_ENV" ] && [ -n "$SAVED_CONDA_PREFIX" ] && [ -f "$SAVED_CONDA_PREFIX/bin/python" ]; then
        export CONDA_PREFIX="$SAVED_CONDA_PREFIX"
        export CONDA_DEFAULT_ENV="$SAVED_CONDA_DEFAULT_ENV"
        export PATH="$SAVED_CONDA_PREFIX/bin:$PATH"
    fi
else
    # 非root运行，使用当前环境
    ORIGINAL_USER="${USER:-$(whoami)}"
    ORIGINAL_HOME="$HOME"
fi

# 运行控制器
echo -e "${GREEN}启动控制器...${NC}"
echo ""

# 捕获 Ctrl+C 信号
trap 'echo -e "\n${YELLOW}正在停止控制器...${NC}"; exit 0' INT TERM

# 检查是否需要 root 权限（硬件模式需要）
# 如果当前不是root，且可以通过sudo获取权限，则使用sudo执行可执行文件
if [ "$EUID" -ne 0 ]; then
    # 检查是否可以通过sudo获取root权限
    if command -v sudo &> /dev/null; then
        # 使用sudo执行可执行文件
        echo -e "${BLUE}[RunController] 需要root权限，使用sudo执行控制器...${NC}"
        echo ""
        
        # 构建sudo命令，确保传递所有必要的环境变量
        SUDO_ENV_ARGS=(
            "LD_LIBRARY_PATH=${LD_LIBRARY_PATH}"
            "PYTHONPATH=${PYTHONPATH}"
            "HOME=${ORIGINAL_HOME:-$HOME}"
        )
        
        # 如果找到了conda环境，也传递conda相关变量
        if [ -n "$CONDA_PREFIX" ]; then
            SUDO_ENV_ARGS+=("CONDA_PREFIX=${CONDA_PREFIX}")
            SUDO_ENV_ARGS+=("CONDA_DEFAULT_ENV=${CONDA_DEFAULT_ENV}")
            SUDO_ENV_ARGS+=("PATH=${PATH}")
        fi
        
        # 使用sudo -E和env确保环境变量被传递
        exec sudo -E env "${SUDO_ENV_ARGS[@]}" "${CONTROLLER_EXE}" "$@"
        exit $?
    else
        # 没有sudo命令，提示用户手动使用sudo
        echo -e "${YELLOW}警告: 当前未以 root 权限运行，且无法使用sudo${NC}"
        echo -e "${YELLOW}硬件模式需要 root 权限（实时调度、硬件访问）${NC}"
        echo -e "${YELLOW}仿真模式通常不需要 root 权限${NC}"
        echo ""
        # 检查是否在交互式终端
        if [ -t 0 ]; then
            read -p "是否继续运行? (y/n) " -n 1 -r
            echo
            if [[ ! $REPLY =~ ^[Yy]$ ]]; then
                echo "已取消"
                exit 0
            fi
        else
            echo -e "${YELLOW}非交互式终端，自动继续运行...${NC}"
        fi
    fi
fi

# 如果以 root 身份运行（通过 sudo），确保库路径和环境变量正确设置
if [ "$EUID" -eq 0 ]; then
    # 如果 LD_LIBRARY_PATH 为空或被重置，使用保存的值
    if [ -z "${LD_LIBRARY_PATH}" ] || [ "${LD_LIBRARY_PATH}" != "${SAVED_LD_LIBRARY_PATH}" ]; then
        export LD_LIBRARY_PATH="${SAVED_LD_LIBRARY_PATH}"
    fi
    # 确保 PYTHONPATH 也被正确设置（sudo 可能会清除环境变量）
    if [ -z "${PYTHONPATH}" ] || [ "${PYTHONPATH}" != "${SAVED_PYTHONPATH}" ]; then
        export PYTHONPATH="${SAVED_PYTHONPATH}"
    fi
    
    # 确保HOME指向原始用户的HOME（某些程序可能需要）
    if [ -n "$ORIGINAL_HOME" ] && [ "$HOME" != "$ORIGINAL_HOME" ]; then
        export HOME="$ORIGINAL_HOME"
    fi
    
    # 如果找到conda环境，打印信息
    if [ -z "$FOUND_CONDA_ENV" ] && [ -z "$CONDA_PREFIX" ]; then
        echo -e "${YELLOW}[RunController] 未找到conda环境，算法启动器将被禁用${NC}"
        echo ""
    fi
fi

# 执行控制器
# 使用 exec 替换当前进程，如果程序退出，脚本也会退出
# 注意：exec 成功后不会返回，如果返回说明 exec 失败（例如文件不存在或权限问题）
exec "${CONTROLLER_EXE}" "$@"

# 如果执行到这里，说明 exec 失败
EXIT_CODE=$?
echo ""
echo -e "${RED}========================================${NC}"
echo -e "${RED}错误: 无法启动控制器程序${NC}"
echo -e "${RED}退出码: ${EXIT_CODE}${NC}"
echo -e "${RED}========================================${NC}"
exit $EXIT_CODE

