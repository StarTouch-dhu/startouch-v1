#!/bin/bash

CAN_IF=can0
BITRATE=1000000

echo "🔄 正在重启 CAN 接口 $CAN_IF @ ${BITRATE}bps"
sudo ip link set $CAN_IF down
sleep 0.2
sudo ip link set $CAN_IF up type can bitrate $BITRATE
echo "✅ 完成"
