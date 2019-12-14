#  Copyright (c) 2019 CyPhyHouse. All Rights Reserved.

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from src.datatypes.robot import BotType
from src.functionality.basicMutexHandler import BasicMutexHandler
from src.motion.automata.moatCar import MoatCar
from src.motion.automata.moatDrone import MoatDrone
from src.motion.planning.rrt_car import RRT as RRT_CAR
from src.motion.planning.simplePlanner import SimplePlanner

planner_dict = dict()
planner_dict['SimplePlanner'] = SimplePlanner
planner_dict['RRT_CAR'] = RRT_CAR

msg_type_dict = dict()
msg_type_dict['PoseStamped'] = PoseStamped
msg_type_dict['String'] = String

bot_type_dict = dict()
bot_type_dict['CAR'] = BotType.CAR
bot_type_dict['QUAD'] = BotType.QUAD

mutex_handler_dict = dict()
mutex_handler_dict['BasicMutexHandler'] = BasicMutexHandler

moat_class_dict = dict()
moat_class_dict['MoatCar'] = MoatCar
moat_class_dict['MoatDrone'] = MoatDrone
