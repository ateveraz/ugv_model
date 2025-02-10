import numpy as np

from src.ugv import UGV
from src.controller import Controller
from src.simulator import Simulator
from src.path_planning import PathPlanning

class PDController(Controller):
    def __init__(self):
        """
        PDController class constructor
        """
        super().__init__()
        self.kp = 1
        self.kd = 0 #0.1
        self.previous_error = 0

    def compute_control(self, state, desired):
        """
        Computes the control input based on the current state.

        @param state: numpy vector containing the current state of the UGV
        @return: numpy vector containing the computed control input
        """
        #error = desired - state
        error = desired - state[:2]
        distance_error = np.linalg.norm(error)
        angle_to_goal = np.arctan2(error[1], error[0])
        angle_error = angle_to_goal - state[2]

        # Normalize the angle error
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))

        error_vector = np.array([distance_error, angle_error])
        derivative = error_vector - self.previous_error
        self.previous_error = error_vector

        control = self.kp * error_vector + self.kd * derivative
        return control, error

class Regulation(PathPlanning):
    def __init__(self, goal):
        """
        Regulation class constructor
        """
        super().__init__('Regulation')
        self.goal = goal
    
    def desired(self, t):
        """
        Computes the desired state of the UGV at a given time step.

        @param t: float containing the current time step.
        @return: numpy vector containing the desired state of the UGV.
        """
        return self.goal
    


# Instance UGV
params = { 'dt': 0.01 }
initial_state = np.array([-1, 1, 0])
ugv = UGV(params, initial_state)

# Instance Controller
ctrl = PDController()

# Instance Path Planning
goal = np.array([2, 2])
path_planning = Regulation(goal)

# Instance Simulator
sim = Simulator(ugv, ctrl, path_planning)
t, states = sim.run(20, show_animation = False)