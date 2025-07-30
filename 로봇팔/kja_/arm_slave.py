#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys, tty, termios
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

import time
import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
from std_msgs.msg import Int32MultiArray

BAUDRATE                    = 1000000
PROTOCOL_VERSION            = 2.0
IDS                         = [0, 1, 2, 3, 4, 5]
DEVICENAME                  = '/dev/ttyACM1'

port_handler = PortHandler(DEVICENAME)
packet_handler = PacketHandler(PROTOCOL_VERSION)

def init_dynamixel():
    if not port_handler.openPort():
        print("slave 포트 열기에 실패했습니다.")
        quit()

    if not port_handler.setBaudRate(BAUDRATE):
        print("slave 통신 속도 설정에 실패했습니다.")
        port_handler.closePort()
        quit()

    for dxl_id in IDS:
        packet_handler.write1ByteTxRx(port_handler, dxl_id, 64, 0)
        packet_handler.write1ByteTxRx(port_handler, dxl_id, 11, 4)
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, 64, 1)
        
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("operator motor ID %d 연결 실패" % dxl_id)

        set_moving_speed(dxl_id, 25, 70)

    print("operator ready") 

def set_moving_speed(dxl_id, acceleration, velocity):
    packet_handler.write4ByteTxRx(port_handler, dxl_id, 108, acceleration) 
    packet_handler.write4ByteTxRx(port_handler, dxl_id, 112, velocity)

def move_to_position(position):
    for i, dxl_id in enumerate(IDS):
        if i < len(position):
            goal  = position[i]
            dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(port_handler, dxl_id, 116, goal)
            
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
                print(f"[!] 이동 실패: ID={dxl_id}, 위치={position}")
            else:
                print(f"[+] 이동 성공: ID={dxl_id}, 위치={position}")

def main():
    rclpy.init()
    node = Node('slave_node')
    # msg는 std_msgs.msg.Int32MultiArray 타입으로, data라는 리스트를 포함
    def callback(msg): # ro_master 토픽에서 1번 다이나믹셀의 위치값 수신 후 목표 위치 설정
        if len(msg.data) > 0:
            goal_positions = msg.data  # data값을 여러개 받아야할 때
            for i, dxl_id in enumerate(IDS):
                if i < len(goal_positions):
                    packet_handler.write4ByteTxRx(port_handler, dxl_id, 116, goal_positions[i])

            gripper_id = IDS[5]
            dxl_current_result, dxl_comm_result, dxl_error = packet_handler.read2ByteTxRx(port_handler, gripper_id, 126)
            if dxl_comm_result != COMM_SUCCESS or dxl_error != 0 or dxl_current_result is None:
                print(f"{gripper_id}: 오류(comm={dxl_comm_result}, err={dxl_error})")
            else:
                current = dxl_current_result if dxl_current_result < 32768 else dxl_current_result - 65536
                print(f"{gripper_id}'<Gripper>': {current:>4} mA")
            print("=" * 30)

    sub = node.create_subscription(Int32MultiArray, 'ro_robot', callback, 10)
    init_dynamixel()
    time.sleep(1)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("종료됨")
    finally:
        port_handler.closePort()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()