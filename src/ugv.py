import numpy as np

class UGV:
    def __init__(self, params, initial_state):
        """
        UGV class constructor.

        @param params: dictionary containing the parameters of the UGV.
        @param initial_state: numpy vector containing the initial state of the UGV.
        """
        self.params = params
        self.state = initial_state

    def update_state(self, control):
        """
        Updates the state of the UGV.

        @param control: numpy vector containing the control input.
        """
        self.state = self.state + self.params['dt'] * self.dynamics(self.state, control)

    def dynamics(self, state, control):
        """
        Computes the dynamics of the UGV.

        @param state: numpy vector containing the state of the UGV.
        @param control: numpy vector containing the control input
        """

        theta = state[2]
        state_matrix = np.array([[np.cos(theta), 0], [np.sin(theta),  0], [0, 1]])

        return state_matrix@control
    
    def get_state(self):
        """
        Returns the current state of the UGV.
        """
        return self.state