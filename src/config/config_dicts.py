#from geometry_msgs.msg import PoseStamped
#from std_msgs.msg import String

from src.config.configs import BotType
from src.functionality.base_mutex_handler import BaseMutexHandler
from src.motion.demo_planner import DemoPlan

from src.motion.moat_test_car import MoatTestCar
from src.motion.moat_test_drone import MoatTestDrone
from src.motion.rrt_star import RRT as RRT_STAR_CAR
from src.motion.rrt_drone import RRT as RRT_DRONE
from src.motion.rrt_star_drone import RRT as RRT_STAR_DRONE
from src.motion.rrt_star_dubins import RRT_DUBINS as RRT_DUB_CAR
from src.motion.simpleplanner import SimplePlanner

planner_dict = dict()
planner_dict['SimplePlanner'] = SimplePlanner
planner_dict['Reeds_Shepp_Planner'] = Reeds_Shepp_Planner
planner_dict['RRT_STAR_CAR'] = RRT_STAR_CAR
planner_dict['RRT_DRONE'] = RRT_DRONE
planner_dict['RRT_STAR_DRONE'] = RRT_STAR_DRONE
planner_dict['RRT_DUB_CAR'] = RRT_DUB_CAR
planner_dict['DemoPlan'] = DemoPlan

msg_type_dict = dict()

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
msg_type_dict['PoseStamped'] = PoseStamped
msg_type_dict['String'] = String

bot_type_dict = dict()
bot_type_dict['CAR'] = BotType.CAR
bot_type_dict['QUAD'] = BotType.QUAD

mutex_handler_dict = dict()
mutex_handler_dict['BaseMutexHandler'] = BaseMutexHandler

moat_class_dict = dict()
moat_class_dict['MoatTestCar'] = MoatTestCar
moat_class_dict['MoatTestDrone'] = MoatTestDrone
