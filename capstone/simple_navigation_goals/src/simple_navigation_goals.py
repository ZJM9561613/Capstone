#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
import random
import actionlib
import time

from actionlib_msgs.msg import *                                
from geometry_msgs.msg import Pose, Point, Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class PatrolNav():
    def __init__(self):
        
        # 节点名称
        rospy.init_node('patrol_nav_node', anonymous=False)

        # 终端按下Ctrl+C后可以终止节点
        rospy.on_shutdown(self.shutdown)

        # From launch file get parameters
        # 从patrol_nav.launch文件中得到的参数
        self.rest_time     = rospy.get_param("~rest_time", 5)
        self.keep_patrol   = rospy.get_param("~keep_patrol",   False)
        self.random_patrol = rospy.get_param("~random_patrol", False)
        self.patrol_type   = rospy.get_param("~patrol_type", 0)
        self.patrol_loop   = rospy.get_param("~patrol_loop", 2)
        self.patrol_time   = rospy.get_param("~patrol_time", 5)

        # set all navigation target pose
        # 设置所有的导航目标点，其中还包括了最终的朝向
        # 可以使用Rviz中的"2D Nav Goal"功能来获取到地图上坐标点的坐标和朝向：rostopic echo /move_base_simple/goal
        self.locations = dict()  
        self.locations['one']   = Pose(Point(-3.2151082886
, 3.79491506034, 0.000), Quaternion(0.000, 0.000, -0.505139810618,  0.863037526257))
       

        self.locations['two']   = Pose(Point(-2.27892068239
, 2.39615205945, 0.000), Quaternion(0.000, 0.000, -0.50362352698
,  0.863923227534
))
        self.locations['three'] = Pose(Point(-4.01225441711
,  0.97978160941, 0.000), Quaternion(0.000, 0.000, -0.977662033058
,  0.210183132334
))
        self.locations['four']  = Pose(Point( -6.5709122351
,   1.74040295262
, 0.000), Quaternion(0.000, 0.000,  0.290839531671
,0.956771846794))



        self.locations['five']  = Pose(Point(-5.88300475179
,2.18560683872
, 0.000), Quaternion(0.000, 0.000, -0.442922315673, 0.896559993686
))

        self.locations['six']  = Pose(Point(-5.79644760359, 0.642602701841, 0.000), Quaternion(0.000, 0.000, -0.479553639128, 0.877512568114))

        self.locations['seven']  = Pose(Point( -4.39536167967
,-1.66673813632
, 0.000), Quaternion(0.000, 0.000, -0.391608797303
,0.920131811142
))





        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 'SUCCEEDED', 'ABORTED',
                       'REJECTED', 'PREEMPTING', 'RECALLING', 'RECALLED', 'LOST']

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(30))
        rospy.loginfo("Connected to move base server")

        n_successes  = 0
        target_num   = 0
        location   = ""
        start_time = rospy.Time.now()
        locations_cnt = len(self.locations)
        sequeue = ['one', 'two', 'three', 'four', 'five','six','seven']
         
        rospy.loginfo("Starting position navigation ")
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():

            #target_num = random.randint(0, locations_cnt-1)

            # Get the next location in the current sequence
            # 取出坐标点向move_base发送，move_base发送导航坐标点的函数为self.send_goal
            location = sequeue[target_num]

            rospy.loginfo("target_num_value:"+str(target_num)) #打印target_num的值

            rospy.loginfo("Going to: " + str(location))
            time.sleep(5)
            self.send_goal(location)

           

            #我的理解是一次导航给300秒的时间，返回的是此次导航结束的时间，如果finished_within_time=0 说明时间到了导航还没结束，如果finished_within_time！=0说明，导航到了，在300秒以内
            #有个问题就是这段时间机器人是在运动的，程序会停在这里等机器人运动
            #finished_within_time是个bool值，就是说明有没有超时的意思，超时时间为300秒，如果300秒内没有完成任务就是超时了，finished_within_time是个bool值就是false，如果完成任务了就是true
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(120))
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.logerr("ERROR:Timed out achieving goal")
                time.sleep(5)
            else:
                state = self.move_base.get_state()

            target_num += 1
            if target_num > 6:
               target_num = 0;
            

    def send_goal(self, locate):
        self.goal = MoveBaseGoal() 
        self.goal.target_pose.pose = self.locations[locate]
        self.goal.target_pose.header.frame_id = 'map'
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.move_base.send_goal(self.goal) #send goal to move_base


    def shutdown(self):
        rospy.logwarn("Stopping the patrol...")

if __name__ == '__main__':
    try:
        PatrolNav()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logwarn("patrol navigation exception finished.")

