# 此sdk为12.15日更新

conda create -n startouch python=3.10
conda activate startouch
<!-- sudo apt-get install pybind11-dev -->

# 更新包列表
sudo apt-get update

# 安装 KDL 库
sudo apt-get install liborocos-kdl-dev

mkdir build
cd build
cmake ..
make

cd ..


#######test#######

python interface_py/test_hardware.py


# lerobot数采部分

## 单臂主从遥操
python single_master_follower.py


## 无遥操双臂数采
### 启动右臂
python interface_py/lerobot_ow_right_arm.py
### 另外开启一个终端，启动左臂
python interface_py/lerobot_ow_left_arm.py



## 双臂推理
python interface_py/lerobot_two_arm_inference.py
