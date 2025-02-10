class Controller:
    def __init__(self):
        """
        Controller class constructor.
        This class can be customized to implement different control laws.
        """
        self.control_input = None

    def set_control(self, control):
        """
        Sets the control input for the controller.

        @param control: numpy vector containing the control input.
        """
        self.control_input = control

    def get_control(self, state):
        """
        Returns the current control input.

        @return: numpy vector containing the control input.
        """
        return self.control_input

    def compute_control(self, state, desired):
        """
        Computes the control input based on the current state.
        This method should be overridden by subclasses to implement specific control laws.

        @param state: numpy vector containing the current state of the UGV.
        @param desired: numpy vector containing the desired state of the UGV.
        @return: numpy vector containing the computed control input.
        """
        raise NotImplementedError("This method should be overridden by subclasses.")