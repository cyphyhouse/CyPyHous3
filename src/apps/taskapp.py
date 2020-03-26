import rospy
from std_msgs.msg import String
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose, Vector3
from cym_marker.msg import Marker, Material, Script

from src.config.configs import AgentConfig, MoatConfig
from src.harness.agentThread import AgentThread
from src.motion.deconflict import clear_path
from src.objects.udt import Task
from src.motion.pos_types import Pos
from src.motion.rectobs import RectObs
# from src.motion.pf import ParticleFilter

import numpy as np


class TaskApp(AgentThread):

    tasks = [
        Task(Pos((4*pos[0], 4*pos[1], pos[2])), i, False, None)
        for i, pos in enumerate([
            (1.75, 1.75, 0),
            (0, 2, 0),
            (-2, 2, 0),
            (-2, -2, 0),
            (0.25, -2, 1),
            (2, 0, 0),
            (1.75, -1.75, 0),
            (2, 1.75, 1),
            (0.25, 2, 1.25),
            (-2, 0.25, 1.25),
            (0, 0.25, 1.5),
            (1, -1, 1),
            (0, -2, 0),
            (-2, 0, 0),
            (-1.75, 2, 1.5),
            (-2, -1.75, 1.15),
        ])
    ]

    def __init__(self, agent_config: AgentConfig, moat_config: MoatConfig):
        super(TaskApp, self).__init__(agent_config, moat_config)
        # NOTE: to enable obstacle detection from position system, uncomment line below
        self.counter = 0
        self._obstacle_listener = rospy.Subscriber("/gazebo/model_states", ModelStates, self.updateObstacles)

    def initialize_vars(self):
        self.initialize_lock('pick_route')
        self.agent_gvh.create_aw_var('tasks', list, TaskApp.tasks)
        self.agent_gvh.create_ar_var('route', list, [self.agent_gvh.moat.position])
        self.locals['my_task'] = None
        self.locals['test_route'] = None
        self.locals['doing'] = False
        self.locals['tasks'] = []
        self.locals['obstacles'] = {}
        self.locals['obstaclesUpdated'] = False

        self.pub_marker = rospy.Publisher("/cym_marker", Marker, queue_size=10)
        self.marker_list = []
        # self.pf = ParticleFilter()
        # self.obstalce_prime = Pos()

    def loop_body(self):
        if not self.locals['doing']:
            if sum([int(a.assigned) for a in self.read_from_shared('tasks', None)]) == len(
                    self.read_from_shared('tasks', None)):
                self.trystop()
                return

            if self.lock('pick_route'):
                self.locals['tasks'] = self.read_from_shared('tasks', None)
                print("Agent", self.pid(), "at", self.agent_gvh.moat.position,
                      "has lock. Remaining tasks:",
                      [t.id for t in self.locals['tasks'] if not t.assigned])
                for i in range(len(self.locals['tasks'])):
                    if not self.locals['tasks'][i].assigned:
                        self.locals['my_task'] = self.locals['tasks'][i]
                        print("Obstacles are: ", self.locals['obstacles'])
                        self.locals['test_route'] = self.agent_gvh.moat.planner.find_path(self.agent_gvh.moat.position,
                                                                                          self.locals[
                                                                                              'my_task'].location,
                                                                                          self.locals['obstacles'].values())
                        if clear_path([path for path in
                                       [self.read_from_shared('route', pid) for pid in range(self.num_agents())]],
                                      self.locals['test_route'], self.pid(), tolerance=1.0):
                            self.locals['doing'] = True
                            self.locals['my_task'].assign(self.pid())
                            self.locals['tasks'][i] = self.locals['my_task']
                            self.share_path()
                        else:
                            self.agent_gvh.put('route', [self.agent_gvh.moat.position],
                                               self.pid())
                            self.locals['my_task'] = None
                            self.locals['doing'] = False
                            continue
                        break
                if not self.locals['doing']:
                    print("Agent", self.pid(), "didnt find a clear path")
                self.unlock('pick_route')
                rospy.sleep(0.05)
        else:
            # print(self.locals['obstaclesUpdated'])
            if self.locals['obstaclesUpdated']:
                tooClose = False
                for obs in self.locals['obstacles'].values():
                    if (self.agent_gvh.moat.position - obs.position).magnitude() < 3:
                        tooClose = True 
                if tooClose:
                    self.agent_gvh.moat.stop()
                    self.locals['obstaclesUpdated'] = False
                    self.locals['test_route'] = self.agent_gvh.moat.planner.update_obstacles(self.locals['obstacles'].values(), self.agent_gvh.moat.position, tooClose=True)
                    if self.locals['test_route'] == None:
                        self.agent_gvh.put('route', [self.agent_gvh.moat.position],
                                            self.pid())
                        self.locals['my_task'] = None
                        self.locals['doing'] = False
                        
                    if clear_path([path for path in
                                       [self.read_from_shared('route', pid) for pid in range(self.num_agents())]],
                                      self.locals['test_route'], self.pid(), tolerance=1.0):
                        self.share_path()
                    else:
                        self.agent_gvh.put('route', [self.agent_gvh.moat.position],
                                            self.pid())
                        self.locals['my_task'] = None
                        self.locals['doing'] = False

                else:
                    self.agent_gvh.moat.planner.update_obstacles(self.locals['obstacles'].values(), self.agent_gvh.moat.position, tooClose=False)

            if self.agent_gvh.moat.reached:
                if self.locals['my_task'] is not None:
                    self.locals['my_task'] = None
                self.locals['doing'] = False
                self.change_marker(None, 0, add=False)
                rospy.sleep(1.0)  # Wait at the task for a while
                return

    def share_path(self):
        self.agent_gvh.put('tasks', self.locals['tasks'])
        self.agent_gvh.put('route', self.locals['test_route'], self.pid())
        # print(self.locals['test_route'])
        # for p in self.locals['test_route']:
        #     print(p)
        #     self.change_marker(p, 0)
        self.change_marker(self.locals['test_route'], 1)
        # print("Agent", self.pid(), "is going to task", i, "at", self.locals['my_task'].location)
        self.agent_gvh.moat.follow_path(self.locals['test_route'])


    def updateObstacles(self, data):
        self.counter += 1
        if self.counter % 1000 != 0:
            return
        print("update obs", self.counter)
        for name in data.name:
            if "cube" in name or "box" in name or "actor" in name:
                index = data.name.index(name)

                obstacle = RectObs(Pos(np.array([data.pose[index].position.x, data.pose[index].position.y, 0])), np.array([1,1,1]))
                try:
                    if name not in self.locals['obstacles']:
                        self.locals['obstacles'][name] = obstacle
                    if obstacle not in self.locals['obstacles'].values():
                        print(name, "obstacle has changed to ", obstacle.position)
                        # print(name, "predicted position is ", self.obstalce_prime)
                        # moved_dist = obstacle.position-self.locals['obstacles'][name].position
                        # print(moved_dist)
                        # self.obstalce_prime = pf.run_particle_filter_iter(self.agent_gvh.moat.position, obstacle.position, moved_dist)
                        self.locals['obstaclesUpdated'] = True
                        self.locals['obstacles'][name] = obstacle
                except KeyError:
                    pass

    def change_marker(self, path, i, add=True) -> None:
        if add:
            for pos in path:
                self.marker_list.append(pos)
                self.pub_marker.publish(self.factory_marker(i, pos))
                rospy.sleep(0.1)
        else:
            for p in self.marker_list:
                self.pub_marker.publish(self.factory_marker(i, p, action=Marker.DELETE_MARKER))
                rospy.sleep(0.1)


    def factory_script(self, i: int) -> Script:
        PREDEFINED_SCRIPT = [
        "Gazebo/RedTransparent",
        "Gazebo/GreenTransparent",
        "Gazebo/BlueTransparent",
        "Gazebo/DarkMagentaTransparent",
        "Gazebo/GreyTransparent",
        "Gazebo/BlackTransparent",
        "Gazebo/YellowTransparent",
        ]
        return Script(name=PREDEFINED_SCRIPT[i % len(PREDEFINED_SCRIPT)])


    def factory_pose(self, pos: Pos) -> Pose:
        pose = Pose()
        pose.position.x = pos.x
        pose.position.y = pos.y
        pose.position.z = 0
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        return pose


    def factory_scale(self) -> Vector3:
        return Vector3(x=0.5, y=0.5, z=0.1)


    def factory_marker(self, i: int, pos: Pos, action = Marker.ADD_MODIFY) -> Marker:
        SIMPLE_TYPE = [
            Marker.BOX,
            Marker.CYLINDER,
            Marker.SPHERE,
            Marker.TEXT
        ]

        mat = Material(script=self.factory_script(i))

        marker = Marker()
        marker.header.frame_id = "world"
        marker.action = action
        marker.id = i
        if action != Marker.ADD_MODIFY:
            return marker

        marker.type = SIMPLE_TYPE[i % len(SIMPLE_TYPE)]
        marker.pose = self.factory_pose(pos)
        marker.scale = self.factory_scale() 
        marker.material = mat

        if marker.type == Marker.TEXT:
            marker.text = "Hello world!"

        return marker