#from geometry_msgs.msg import PoseStamped
#from std_msgs.msg import String

from src.config.configs import BotType
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.motion.demo_planner import DemoPlan
from src.motion.moat_test_car import MoatTestCar
from src.motion.moat_test_drone import MoatTestDrone
from src.motion.rrt_star import RRT
from src.motion.simpleplanner import SimplePlanner

planner_dict = dict()
planner_dict['SimplePlanner'] = SimplePlanner()
planner_dict['RRT'] = RRT
planner_dict['DemoPlan'] = DemoPlan

msg_type_dict = dict()
msg_type_dict['PoseStamped'] = 'PoseStamped'
msg_type_dict['String'] = 'String'

bot_type_dict = dict()
bot_type_dict['CAR'] = BotType.CAR
bot_type_dict['QUAD'] = BotType.QUAD

mutex_handler_dict = dict()
mutex_handler_dict['BaseMutexHandler'] = BaseMutexHandler

moat_class_dict = dict()
moat_class_dict['MoatTestCar'] = MoatTestCar
moat_class_dict['MoatTestDrone'] = MoatTestDrone
