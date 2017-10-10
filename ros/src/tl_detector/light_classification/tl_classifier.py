from styx_msgs.msg import TrafficLight

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        # insert your model
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        """ example code
        if result == 0:
            state = TrafficLight.GREEN
        else:
            state = TrafficLight.RED
        """

        return TrafficLight.UNKNOWN
