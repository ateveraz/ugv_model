import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class Simulator:
    def __init__(self, ugv, controller, path_planning):
        """
        Simulator class constructor.

        @param ugv: instance of the UGV class.
        @param controller: instance of the Controller class.
        @param path_planning: instance of the PathPlanning class.
        """
        self.ugv = ugv
        self.controller = controller
        self.path_planning = path_planning

    def run(self, time, show_animation = False):
        """
        Runs the simulation for a specified number of steps.

        @param num_steps: number of simulation steps to run.
        """
        num_steps = int(time / self.ugv.params['dt'])

        self.states = np.zeros((num_steps, 3))
        self.errors = np.zeros((num_steps, 2))
        self.t = np.zeros(num_steps)
        
        for i in range(num_steps):
            self.t[i] = i * self.ugv.params['dt']
            desired = self.path_planning.desired(self.t[i])
            control_input, error = self.controller.compute_control(self.ugv.state, desired)
            self.errors[i] = error
            self.ugv.update_state(control_input)
            self.states[i] = self.ugv.state

        if show_animation:
            self.animate()
        else:
            self.plot()
        
        return self.t, self.states

    def runCustom(self, time, show_animation = False):
        """
        Run custom simulation for specific applications. 
        """
        raise NotImplementedError("This method should be overridden by subclasses.")

    def animationSetup(self):
        self.fig = plt.figure()

        tol = 0.5 # tolerance for axis limits
        maxs = np.max(self.states, axis=0)
        mins = np.min(self.states, axis=0)
        self.axis = plt.axes(xlim=(mins[0] - tol, maxs[0] + tol), ylim=(mins[1] - tol, maxs[1] + tol))
        self.trajectory, = self.axis.plot([], [], lw = 2)

    def plot(self):
        """
        Plots the states of the UGV.

        @param t: numpy array containing the time steps.
        @param states: numpy array containing the states of the UGV.
        """
        plt.figure()
        # Plot the states
        plt.subplot(2, 1, 1)
        plt.plot(self.t, self.states[:, 0], label='x')
        plt.plot(self.t, self.states[:, 1], label='y')
        plt.xlabel('Time [s]')
        plt.ylabel('States (x, y) [m]')
        plt.title('UGV States (coordinates)')
        plt.grid('minor')
        plt.legend()
        
        # Plot the error norm
        plt.subplot(2, 1, 2)
        error_norm = np.linalg.norm(self.errors, axis=1)
        plt.plot(self.t, error_norm, label='error norm')
        plt.xlabel('Time [s]')
        plt.ylabel('Error norm [m]')
        plt.title('Error norm')
        plt.grid('minor')
        plt.show()
    
    def plotXY(self):
        """
        Plots the states of the UGV in the XY plane.
        """
        plt.figure()
        plt.plot(self.states[:, 0], self.states[:, 1], label='trajectory')
        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.title('UGV Trajectory')
        plt.grid('minor')
        plt.legend()
        plt.show()

    def makeFrame(self, i):
        self.trajectory.set_data(self.states[:i, 0], self.states[:i, 1])
        return self.trajectory,

    def init_animation(self):
        self.trajectory.set_data([], [])
        return self.trajectory,

    def animate(self):
        """
        Animates the trajectory of the UGV.
        """

        self.animationSetup()

        anim = animation.FuncAnimation(self.fig, self.makeFrame, init_func=self.init_animation, frames=len(self.states), interval=20, blit=True, repeat=True)

        if self.path_planning.type == 'Regulation':
            self.axis.plot(self.path_planning.goal[0], self.path_planning.goal[1], 'r*', label='goal')
        self.axis.plot(self.states[0, 0], self.states[0, 1], 'bo', label='start')

        plt.xlabel('x [m]')
        plt.ylabel('y [m]')
        plt.title('UGV Trajectory')
        plt.grid('minor')
        plt.legend()
        plt.show()