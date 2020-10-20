from typing import NamedTuple

import queue
import numpy as np
import rospy
from src.motion.moat_test_car import MoatTestCar
from src.motion.pos_types import Pos

from sensor_msgs.msg import LaserScan
import message_filters

import itertools


LaserScanPara = NamedTuple("LaserScanPara", [('angle_min', float),
                                             ('angle_max', float),
                                             ('angle_increment', float),
                                             ('time_increment', float),
                                             ('scan_time', float),
                                             ('range_min', float),
                                             ('range_max', float)])


class MoatWithLidar(MoatTestCar):

    def __init__(self, config):
        super(MoatWithLidar, self).__init__(config)
        self.tsync = queue.Queue()
        scan_topic = config.rospy_node.replace("/waypoint_node", '/racecar/laser/scan')
        sub_scan = message_filters.Subscriber(scan_topic, LaserScan)
        sub_pose = message_filters.Subscriber(self._sub_positioning.name,
                                              self._sub_positioning.data_class)

        ts = MyApproximateTimeSynchronizer([sub_scan, sub_pose], 10, 0.005)
        ts.registerCallback(self._get_scan_at_pos)

    def reset(self) -> None:
        super().reset()
        if self.tsync:
            tmp = self.tsync
            self.tsync = queue.Queue()
            del tmp

    @staticmethod
    def _quaternion_to_euler(q):
        from scipy.spatial.transform import Rotation
        return Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('zyx')

    def _get_scan_at_pos(self, scan, pos) -> None:
        yaw = self._quaternion_to_euler(pos.pose.orientation)[0]
        position = Pos(np.array([pos.pose.position.x, pos.pose.position.y, pos.pose.position.z, yaw]))

        tmp_list = []
        cur_angle = scan.angle_min
        for distance in scan.ranges:
            cur_angle += scan.angle_increment
            tmp_list.append((distance, cur_angle))

        scan_para = LaserScanPara(angle_min=scan.angle_min,
                                  angle_max=scan.angle_max,
                                  angle_increment=scan.angle_increment,
                                  time_increment=scan.time_increment,
                                  scan_time=scan.scan_time,
                                  range_min=scan.range_min,
                                  range_max=scan.range_max)
        try:
            self.tsync.put_nowait((position, tmp_list, scan_para))
        except queue.Full:
            pass


class MyApproximateTimeSynchronizer(message_filters.ApproximateTimeSynchronizer):
    def add(self, msg, my_queue, my_queue_index=None):
        if not hasattr(msg, 'header') or not hasattr(msg.header, 'stamp'):
            if not self.allow_headerless:
                rospy.logwarn("Cannot use message filters with non-stamped messages. "
                              "Use the 'allow_headerless' constructor option to "
                              "auto-assign ROS time to headerless messages.")
                return
            stamp = rospy.Time.now()
        else:
            stamp = msg.header.stamp

        self.lock.acquire()
        my_queue[stamp] = msg
        while len(my_queue) > self.queue_size:
            del my_queue[min(my_queue)]
        # self.queues = [topic_0 {stamp: msg}, topic_1 {stamp: msg}, ...]
        if my_queue_index is None:
            search_queues = self.queues
        else:
            search_queues = self.queues[:my_queue_index] + \
                self.queues[my_queue_index+1:]
        # sort and leave only reasonable stamps for synchronization
        stamps = []
        for queue in search_queues:
            topic_stamps = []
            for s in queue:
                stamp_delta = abs(s - stamp)
                if stamp_delta > self.slop:
                    continue  # far over the slop
                topic_stamps.append((s, stamp_delta))
            if not topic_stamps:
                self.lock.release()
                return
            topic_stamps = sorted(topic_stamps, key=lambda x: x[1])
            stamps.append(topic_stamps)
        for vv in itertools.product(*[list(zip(*s))[0] for s in stamps]):
            vv = list(vv)
            # insert the new message
            if my_queue_index is not None:
                vv.insert(my_queue_index, stamp)
            qt = list(zip(self.queues, vv))
            if ( ((max(vv) - min(vv)) < self.slop) and
                (len([1 for q,t in qt if t not in q]) == 0) ):
                msgs = [q[t] for q,t in qt]
                self.signalMessage(*msgs)
                for q,t in qt:
                    del q[t]
                break  # fast finish after the synchronization
        self.lock.release()
