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
IDS                         = [1]
DEVICENAME                  = '/dev/ttyACM0'

port_handler = PortHandler(DEVICENAME)
packet_handler = PacketHandler(PROTOCOL_VERSION)
groupSyncRead = GroupSyncRead(port_handler, packet_handler, 132, 4)

# controller_current = [50, 150, 100, 50]

def init_dynamixel(): 
    if not port_handler.openPort():
        print("포트 열기에 실패했습니다.")
        quit()

    if not port_handler.setBaudRate(BAUDRATE):
        print("통신 속도 설정 실패")
        port_handler.closePort()
        quit()

    for dxl_id in IDS:
        packet_handler.write1ByteTxRx(port_handler, dxl_id, 64, 0) # Torque OFF
        packet_handler.write1ByteTxRx(port_handler, dxl_id, 11, 5) # 전류 기반 위치제어 모드
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, dxl_id, 64, 1) # Torque ON
        
        
        if dxl_comm_result != COMM_SUCCESS or dxl_error != 0:
            print("motor ID %d 연결 실패" % dxl_id)
        
        set_moving_speed(dxl_id, 20, 50) 
        packet_handler.write2ByteTxRx(port_handler, dxl_id, 102, 1) # 전류 제한 설정
        packet_handler.write4ByteTxRx(port_handler, dxl_id, 116, 1024) # Goal Position Setting
        groupSyncRead.addParam(dxl_id) # 현재 아이디를 등록, groupSyncRead.txRxPacket()으로 한 번에 현재 위치 읽기 가능

def set_moving_speed(dxl_id, acceleration, velocity): # 속도와 가속도를 설정하는 함수
    packet_handler.write4ByteTxRx(port_handler, dxl_id, 108, acceleration) 
    packet_handler.write4ByteTxRx(port_handler, dxl_id, 112, velocity)
    
def read_position(pub_arm): 
    groupSyncRead.txRxPacket() # 설정된 다이나믹셀의 모든 위치를 읽는 명령을 전송, groupSyncRead.addParam(dxl_id)가 선행 되어야함
    dxl_present_positions = [] # position을 리스트로 저장
    for dxl_id in IDS:
        if groupSyncRead.isAvailable(dxl_id, 132, 4): # 해당 주소에 데이터가 도착했는지 확인
            pos = groupSyncRead.getData(dxl_id, 132, 4) 
            dxl_present_positions.append(pos) # 읽은 위치를 리스트에 추가
        else:
            dxl_present_positions.append(0) 
    
    if any(dxl_present_positions): # 적어도 하나의 유효한 위치값이 있는 경우만 퍼블리시
        arm_data = Int32MultiArray(data=dxl_present_positions) # 다이나믹셀에서 위치값을 읽어 반환
        pub_arm.publish(arm_data) # 퍼블리셔 객체 pub_arm

def main(): 
    rclpy.init()
    node = Node('master_node')

    pub_arm = node.create_publisher(Int32MultiArray, 'ro_robot', 10)   
    init_dynamixel() # 모터와 통신을 시작하기 위한 초기 설정 함수 호출
    time.sleep(1) # 초기 설정이 끝난 후 1초의 대기 시간을 줌

    try:
        while rclpy.ok():
            read_position(pub_arm)
            time.sleep(0.05)

    except KeyboardInterrupt:
        print("종료됨")

    finally:
        port_handler.closePort()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
