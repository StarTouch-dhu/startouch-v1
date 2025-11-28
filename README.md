# 连接can设备
sudo ip link set can0 up type can bitrate 1000000
ip link show can0


# 创建conda虚拟环境
conda create -n startouch python=3.10
conda activate startouch

# 编译软件sdk
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j&(nproc)

# 测试用例
cd interface_py
python ik.py
