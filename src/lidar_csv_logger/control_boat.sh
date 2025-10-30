#!/bin/bash

# Simple boat control script
# Usage: ./control_boat.sh [left_thrust] [right_thrust]
# Example: ./control_boat.sh 1.0 1.0  (move forward)
#          ./control_boat.sh 0.5 1.0  (turn left)
#          ./control_boat.sh 1.0 0.5  (turn right)

LEFT_THRUST=${1:-0.0}
RIGHT_THRUST=${2:-0.0}

echo "Controlling boat: Left=$LEFT_THRUST, Right=$RIGHT_THRUST"
echo "Press Ctrl+C to stop"

while true; do
    gz topic -t /model/roboboat/joint/left_thruster_joint/cmd_thrust -m gz.msgs.Double -p "data: $LEFT_THRUST" &
    gz topic -t /model/roboboat/joint/right_thruster_joint/cmd_thrust -m gz.msgs.Double -p "data: $RIGHT_THRUST" &
    sleep 0.1
done
