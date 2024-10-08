#!/bin/bash

# 设置环境名称
ENV_NAME="rl_env"

# 检查Conda是否已安装
if ! command -v conda &> /dev/null
then
    echo "Conda is not installed. Please install Conda first."
    exit 1
fi

# 检查是否已经有该Conda环境
if conda info --envs | grep -q "$ENV_NAME"; then
    echo "Conda environment '$ENV_NAME' already exists."
else
    # 创建新的Conda环境，指定Python版本
    echo "Creating a new conda environment: $ENV_NAME"
    conda create -n $ENV_NAME python=3.8 -y
fi

# 激活环境
echo "Activating the environment: $ENV_NAME"
source activate $ENV_NAME

# 安装必要的包
echo "Installing necessary packages..."

conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia -y
conda install -c conda-forge gym stable-baselines3 -y
conda install numpy scipy matplotlib -y

# 安装ROS相关的Python包
echo "Installing ROS Python packages..."
pip install rospkg catkin_pkg

echo "All packages have been installed in the '$ENV_NAME' environment."
