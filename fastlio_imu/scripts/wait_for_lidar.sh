#!/bin/bash
TOPIC="/livox/lidar"
TIMEOUT=30
INTERVAL=1

source /opt/ros/humble/setup.bash
source /home/sunrise/livox_ws/install/setup.bash

echo "Waiting for topic $TOPIC ..."
START_TIME=$(date +%s)
while true; do
    if ros2 topic echo --once $TOPIC > /dev/null 2>&1; then
        echo "Topic $TOPIC is publishing data."
        exit 0
    fi
    CURRENT_TIME=$(date +%s)
    ELAPSED=$((CURRENT_TIME - START_TIME))
    if [ $ELAPSED -ge $TIMEOUT ]; then
        echo "Timeout waiting for $TOPIC" >&2
        exit 1
    fi
    sleep $INTERVAL
done