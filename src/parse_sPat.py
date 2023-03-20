#!/usr/bin/env python
#-*- coding: utf-8 -*-

import rospy, socket, time

from queue import Queue
from v2x_msgs.msg import IntersectionStateList, IntersectionState, MovementEventList, MovementState, ManeuverAssistList, MovementStateList

class V2XdataParse2ROS:
    def __init__(self):
        rospy.init_node("V2XData_parse_to_ROS", anonymous=False)
        self.v2x_pub = rospy.Publisher("/v2x/sPat", IntersectionStateList, queue_size=10)

        while True:
            self.v2x_data = self.udp_reciever('127.0.0.1', 50115)  #송신부 주소(상대 IP)
            print(self.v2x_data)
            self.spat_ros_msg = self.processing_to_ros_msg(self.v2x_data)
            self.v2x_pub.publish(self.spat_ros_msg)
            time.sleep(0.01)

    def udp_reciever(self, ip, port):
        
        sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

        data_queue = Queue() #FIFO data
        # print(data_queue.qsize())

        if sock <= -1:
            print("소켓 생성 실패!")
        else:
            print("소켓 생성 완료")
 
        sock.bind((ip,port))
        # print("바인드 값 : {}".format(a))
        while True:
            self.data = sock.recv(2048)
            data_queue.put(self.data)
            # print(self.data)
            if bool(self.data) != True:
                break
            # print(data_queue.qsize())
            return data_queue.get()
        
    def processing_to_ros_msg(self, msg):
        msgs = msg.split(',')
        intersectionStateList = IntersectionStateList()
        # print(msgs)
        # print("length of msgs list : {}".format(len(msgs)))
        del msgs[0] #헤더 삭제
        for i in msgs:
            intersectionState = IntersectionState()
            split_msgs = i.split(":")
            # print(split_msgs)     
            # print("length of split_msgs list : {}".format(len(split_msgs)))
            intersection_msgs = split_msgs[0].split(" ")
            # print(intersection_msgs, len(intersection_msgs))
            intersectionState.name = intersection_msgs[1]
            intersectionState.intersections.region = int(intersection_msgs[2])
            intersectionState.intersections.id = int(intersection_msgs[3])
            intersectionState.revision = int(intersection_msgs[4])
            intersectionState.status = intersection_msgs[5]
            intersectionState.minuteOfTheYear = int(intersection_msgs[6])
            intersectionState.timeStamp = int(intersection_msgs[7])
            del split_msgs[0]
            # print(split_msgs)
            movementStateList = MovementStateList()
            for j in split_msgs:
                movementState = MovementState()
                split_states_msg = j.split("-")
                # print(split_states_msg)
                movement_msgs = split_states_msg[0].split(" ")
                # print(movement_msgs)
                movementState.movementName = movement_msgs[1]
                movementState.signalGroup = int(movement_msgs[2])
                # print(split_states_msg)
                stateTimeSpeed_msg = split_states_msg[1].split('sts')
                maneuverAssist_msg = split_states_msg[2].split('mAL')
                stateTimeSpeed_msg = list(filter(bool, stateTimeSpeed_msg))
                maneuverAssist_msg = list(filter(bool, maneuverAssist_msg))
                # print(stateTimeSpeed_msg)
                # print(maneuverAssist_msg)
                for k in stateTimeSpeed_msg:
                    movementEventList = MovementEventList()
                    movementEventList_msg = k.split(" ")
                    # print(movementEventList_msg)
                    movementEventList.eventState = int(movementEventList_msg[1])
                    movementEventList.timingMinEndTime = int(movementEventList_msg[2])
                    movementState.stateTimeSpeed.append(movementEventList)

                for q in maneuverAssist_msg:
                    maneuverAssistList = ManeuverAssistList()
                    maneuverAssistList_msg = q.split(" ")
                    maneuverAssistList.connectionID = int(maneuverAssistList_msg[1])
                    maneuverAssistList.pedBicycleDetect = int(maneuverAssistList_msg[2])
                    movementState.maneuverAssistList.append(maneuverAssistList)
                
                movementStateList.states.append(movementState)

            intersectionState.list.append(movementStateList) #추후 메시지 타입 조정 필요
            intersectionStateList.instersections.append(intersectionState)

        return intersectionStateList

if __name__ == "__main__":
    try:
        vp = V2XdataParse2ROS()
    except rospy.ROSInterruptException:
        print("parse_sPat.py occured Error! check this node.")
        