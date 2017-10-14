from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import rospy
import os


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier 
        GRAPH_FILE =  'models/frozen_inference_graph_test_site.pb'    
        #GRAPH_FILE =  'models/frozen_inference_graph_simulator.pb'            
        graph_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), GRAPH_FILE)
        
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_path, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        self.sess = tf.Session(graph=self.detection_graph)
        print("Model is loaded...")
        

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
        else
            state = TrafficLight.RED
        """
        with self.detection_graph.as_default():
            boxes, scores, classes, num_detections = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections], feed_dict={self.image_tensor: np.expand_dims(image, axis=0)})
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)
            max_score_idx = np.argmax(scores)
            result = classes[max_score_idx]
            if result == 1:
                print('RED Light')
                return TrafficLight.RED
            elif result == 2:
                print('Yellow Light')
                return TrafficLight.YELLOW
            elif result == 3:
                print('Green Light')
                return TrafficLight.GREEN
        return TrafficLight.UNKNOWN

