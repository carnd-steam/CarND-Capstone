GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):

    def __init__(self):
        pass

    def control(self, throttle, steering_step):
        # Return throttle, brake, steer
        return throttle, 0., steering_step
