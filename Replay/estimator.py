class Estimator():
    def __init__(self):
        pass

    def get_orientation(self):
        pass

    def get_position(self):
        pass


# This class is to abstract the EKF and whatever filter with standardized functions




class BasicEstimator(Estimator):
    def __init__(self):
        super().__init__()

        self.state = []


    def initialize_orientation(self, a_b, a_i, m_b, m_i):
        pass

    def initialize_position(self, ecef_x, ecef_y, ecef_z):
        pass