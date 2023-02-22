## pip install stable-baselines3[extra]

# %%
import numpy as np
from numpy import dot
from numpy.linalg import norm


from gym import Env,spaces
from stable_baselines3 import PPO, DQN
from stable_baselines3.common.evaluation import evaluate_policy
from Control import *
from Lidar import *
from RespawnGoal import *
import torch

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg._model_state import ModelState

from std_srvs.srv import Empty
from std_srvs.srv._empty import Empty_Request


from math import *

from tqdm import trange

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerGuardCondition
from rclpy.utilities import timeout_sec_to_nsec

import time
import random
import os

# init
X_INIT = 0.0
Y_INIT = 0.0
THETA_INIT = 0.0

X_GOAL = [0, 1.7, -1.7, 1.5]
Y_GOAL = [-1.5, -1, 0, 1.1]

LIDAR_ANGLE_STEPS = 10



GETGOALTHRESHOLD = 0.25 # ???????? 25 cm


def pidnode(node):
    # ROS node shutdown 
    node.destroy_node()
    rclpy.shutdown()

def getGoal(get_goal_threshold, robot_coor, goal_coor):
    output = abs(dist(robot_coor,goal_coor))
    if output <= get_goal_threshold:
        return True
    else:
        return False
    
# %%
class Publisher():
    def __init__(self, node: Node):
        self.velPub = node.create_publisher(Twist, 'cmd_vel', 10)
        self.setPosPub = node.create_publisher(ModelState, 'gazebo/set_model_state', 10)
        self.setGoalPub = node.create_publisher(ModelState, 'gazebo/set_goal_model_state', 10)
        self.resetWorld = node.create_client(Empty, '/reset_simulation')


# %%
class RosWorld(Env):
    def __init__(self):
        self.node = Node("RosGym")
        self.publisher = Publisher(self.node)
        self.dist_robot_to_goal = 0
        
        # define action space
        self.action_space = spaces.Box(
            low=-1., high=1., shape=(2,), dtype=np.float32)

        # for checking obervation or state shape format
        obs_lidar_shape = int(360 / LIDAR_ANGLE_STEPS)
        self.observation_space = spaces.Box(
            low=np.NINF, high=np.Inf, shape=(obs_lidar_shape+2,), dtype=np.float32)
        
        self.step_count = 0
        self.crash_count = 0
        self.getgoal_count = 0
        self.timeout_count = 0
        self.dist_robot_to_goal = 0

    def reset(self):
        pidnode(self.node)
        rclpy.init()
        self.node = Node("RosGym")
        self.publisher = Publisher(self.node)
        # reset world
        while True:
            self.publisher.resetWorld.call_async(Empty_Request())
            # get init position -> [x, y]
            _, odomMsg = self.wait_for_message('/odom', Odometry)
            _, msgScan = self.wait_for_message('/scan', LaserScan)
            ( self.x, self.y ) = getPosition(odomMsg)
            self.yaw = getRotation(odomMsg)
            print(self.yaw)
            # print(f'{self.x, self.y}')
            if (abs(self.x - X_INIT) <= 0.01) & (abs(self.y - Y_INIT) <= 0.01):
                break
            else:
                pass   
        
        ( self.lidar, self.angles ) = lidarScan(msgScan)
        self.new_lidar = self.lidar[::LIDAR_ANGLE_STEPS]

        # reset reward
        self.step_per_reset = 0
        self.crash_reward = 0
        self.timeout_reward = 0
        self.goal_reward = 0
        self.prev_reward = 0
        self.cosine_reward = 0
        
        self.done = False
        
        # reset goal
        delete_circle()
        random_idx = random.randint(0, 3)
        self.x_goal = X_GOAL[random_idx]
        self.y_goal = Y_GOAL[random_idx]
        spawn_circle([self.x_goal, self.y_goal])
        
        self.dist_robot_to_goal = dist([self.x, self.y], [self.x_goal, self.y_goal])

        
        self.observation = np.concatenate((np.array([self.dist_robot_to_goal, self.yaw]), self.new_lidar))

        return self.observation

    def step(self, action):
        
        # take action to get next state
        # robotDoAction(self.publisher.velPub, action=action)
        robotContAction(self.publisher.velPub, float(action[0]), float(action[1]))
        
        # collect next state
        ## get msgs
        _, odomMsg = self.wait_for_message('/odom', Odometry)
        _, msgScan = self.wait_for_message('/scan', LaserScan)
        
        ## prepocess msg
        ( self.x, self.y )= getPosition(odomMsg)
        self.yaw = getRotation(odomMsg)
        ( self.lidar, self.angles ) = lidarScan(msgScan)
        self.new_lidar = self.lidar[::LIDAR_ANGLE_STEPS]
        
        self.dist_robot_to_goal = dist([self.x, self.y], [self.x_goal, self.y_goal])
        
        robot_coor = [self.x, self.y]
        goal_coor = [self.x_goal, self.y_goal]
        cosineSim = dot(robot_coor, goal_coor)/(norm(robot_coor)*norm(goal_coor))
        
        # update new observation
        self.observation = np.concatenate((np.array([self.dist_robot_to_goal, self.yaw]), self.new_lidar))
        
        # set reward function
        
        # crash reward
        if checkCrash(self.lidar):
            robotStop(self.publisher.velPub)
            print("CrashHHHHHHHH!!!!")
            self.crash_count += 1
            self.crash_reward = -300
            self.done = True
        else:
            pass

        # get goal reward
        self.goal_reward = 0
        if getGoal(GETGOALTHRESHOLD, robot_coor, goal_coor):
            print("GOALLLLLLLLLL!!!!")
            # delete_circle()
            # self.x_goal = random.uniform(-1.5, 1.5)
            # self.y_goal = random.uniform(-1.5, 1.5)
            # spawn_circle([self.x_goal, self.y_goal])
            # print(f'{self.x_goal, self.y_goal}')
            self.getgoal_count += 1
            self.goal_reward = 1000
            # self.step_per_reset = 0
            self.done = True
        else:
            pass
            
            
        if self.step_per_reset >= 250:
            print('TIMEOUT!!!')
            self.timeout_reward = -400
            self.timeout_count += 1
            self.done = True
        else:
            pass
        
        self.reward = (5 - self.dist_robot_to_goal) + self.cosine_reward + self.goal_reward + self.crash_reward + self.timeout_reward
        
        self.step_count += 1
        self.step_per_reset += 1
        
        self.dist_robot_to_goal_prev = self.dist_robot_to_goal

        info = {}
        
        if (self.step_count % 1 == 0):
            print(f'step_count: {self.step_count}')
            print(f'step_per_reset: {self.step_per_reset}')
            print(f'action: {action}')
            print(f"X: {self.x:.2f} => {self.x_goal:.2f}")
            print(f"Y: {self.y:.2f} => {self.y_goal:.2f}")
            print(f"dist: {self.dist_robot_to_goal:.2f}")
            print(f"cosineSim: {cosineSim:.2}")
            print(f"reward: {self.reward:.2f}")
            print(f"crash_count: {self.crash_count}")
            print(f"getgoal_count: {self.getgoal_count}")
            print(f"timeout_count: {self.timeout_count}")
            print(f'obs: {self.observation}')
            print("###########################################")
        return self.observation, self.reward, self.done, info

    def wait_for_message(
        self,
        topic: str,
        msg_type,
        time_to_wait=-1
    ):
        """
        Wait for the next incoming message.
        :param msg_type: message type
        :param node: node to initialize the subscription on
        :param topic: topic name to wait for message
        :time_to_wait: seconds to wait before returning
        :return (True, msg) if a message was successfully received, (False, ()) if message
            could not be obtained or shutdown was triggered asynchronously on the context.
        """
        context = self.node.context
        wait_set = _rclpy.WaitSet(1, 1, 0, 0, 0, 0, context.handle)
        wait_set.clear_entities()

        sub = self.node.create_subscription(msg_type, topic, lambda _: None, 1)
        wait_set.add_subscription(sub.handle)
        sigint_gc = SignalHandlerGuardCondition(context=context)
        wait_set.add_guard_condition(sigint_gc.handle)

        timeout_nsec = timeout_sec_to_nsec(time_to_wait)
        wait_set.wait(timeout_nsec)

        subs_ready = wait_set.get_ready_entities('subscription')
        guards_ready = wait_set.get_ready_entities('guard_condition')

        if guards_ready:
            if sigint_gc.handle.pointer in guards_ready:
                return (False, None)

        if subs_ready:
            if sub.handle.pointer in subs_ready:
                msg_info = sub.handle.take_message(sub.msg_type, sub.raw)
                return (True, msg_info[0])

        return (False, None)



# %%
if not rclpy.ok():
    rclpy.init()

REV = "PPO50_Adc_enco_0.01_nearreward"

models_dir = f"models/{REV}"
logdir = f"logs"

if not os.path.exists(models_dir):
	os.makedirs(models_dir)

if not os.path.exists(logdir):
	os.makedirs(logdir)

env = RosWorld()
env.reset()

model_path = '/root/models/PPO50_Adc_enco_0.01_nearreward/30000.zip'
model = PPO.load(model_path, env=env)

episodes = 500

for ep in range(episodes):
    obs = env.reset()
    done = False
    while not done:
        action, _state = model.predict(obs)
        obs, rewards, done, info = env.step(action)
        print(rewards)