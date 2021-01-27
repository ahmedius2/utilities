#!/usr/bin/env python

import rospy
import threading
from collections import deque
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

class Topic_barrier:
    def __init__(self):

        self.subscribe_to = [
            '/kitti/velo/pointcloud',
            '/kitti/oxts/imu',
            '/kitti/oxts/gps/fix',
            '/kitti/oxts/gps/vel',
            '/kitti/camera_color_right/camera_info',
            '/kitti/camera_color_right/image_raw'
        ]

        self.publish_to = [
            '/points_raw',
            '/imu_raw',
            '/fix',
            '/vel',
            '/camera_info',
            '/image_raw'
        ]

        self.topic_types = [
            PointCloud2,
            Imu,
            NavSatFix,
            TwistStamped,
            CameraInfo,
            Image
        ]

        self.approx_size_bytes = [
            2*1024*1024,
            1024,
            1024,
            1024,
            1024,
            2*1024*1024
        ]

        self.subscribers = []
        self.publishers = []
        self.data_deques = []
        self.publish_at_round = rospy.get_param('~publish_at_round', 1)
        self.cur_round = 1
        self.data_lock = threading.Lock()

        subs_qs=10
        for i in range(len(self.subscribe_to)):
            cb_lambda = lambda data, index: self.barrier_callback(data, index)
            self.subscribers.append(rospy.Subscriber(self.subscribe_to[i], self.topic_types[i], 
                callback=cb_lambda, callback_args=i, queue_size=subs_qs,
                buff_size=self.approx_size_bytes[i]*subs_qs, tcp_nodelay=True))

            self.publishers.append(rospy.Publisher(self.publish_to[i], self.topic_types[i],
                tcp_nodelay=True, latch=False, queue_size=1))

            self.data_deques.append(deque())

    def all_deques_have_elem(self):
        for dq in self.data_deques:
            if not dq: # if empty
                return False
        return True

    def barrier_callback(self, data, topic_index):
       
        #debug
        #print(type(data), topic_index)
        
        with self.data_lock:
            self.data_deques[topic_index].appendleft(data)

            if not self.all_deques_have_elem():
                return

            # When all deques have data, publish one from all simultaneously
            if self.cur_round == self.publish_at_round:
                for pub, dq in zip(self.publishers, self.data_deques):
                    pub.publish(dq.pop())
                self.cur_round=1
            else:
                for dq in self.data_deques:
                    dq.pop()
                self.cur_round += 1

        return

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('topic_barrier', anonymous=True)
        tb_obj = Topic_barrier() 
        tb_obj.run()
    except rospy.ROSInterruptException:
        pass
