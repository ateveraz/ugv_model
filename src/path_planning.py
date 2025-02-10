class PathPlanning:
    def __init__(self, type_):
        """
        PathPlanning class constructor.

        @param type_: string containing the type of path planning algorithm.
        """
        self.type = type_

    def desired(self, t):
        """
        Computes the desired state of the UGV at a given time step.

        @param t: float containing the current time step.
        @return: numpy vector containing the desired state of the UGV.
        """
        raise NotImplementedError("This method should be overridden by subclasses.")
