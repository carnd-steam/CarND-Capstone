from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import rospy
from PIL import Image
from PIL import ImageDraw
from PIL import ImageColor


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier 
        GRAPH_FILE =  'models/frozen_inference_graph_test_site.pb'    
        # GRAPH_FILE =  'models/frozen_inference_graph_simulator.pb'            
        graph_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), GRAPH_FILE)
        self.graph = load_graph(graph_file)
        self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.graph.get_tensor_by_name('num_detections:0')
        print("Model is loaded...")
        
    
    def load_graph(graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph
        
    def filter_boxes(min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        for i in range(n):
            if scores[i] >= min_score:
                idxs.append(i)
    
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes
        

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
        try:
            with self.graph.as_default():
                boxes, scores, classes = sess.run([detection_boxes, detection_scores, detection_classes], feed_dict={image_tensor: np.expand_dims(image, 0)})
                boxes = np.squeeze(boxes)
                scores = np.squeeze(scores)
                classes = np.squeeze(classes)
            
                confidence_cutoff = 0.8
                boxes, scores, classes = self.filter_boxes(confidence_cutoff, boxes, scores, classes)
                
                label_map = {1: TrafficLight.RED, 2: TrafficLight.GREEN, 3: TrafficLight.YELLOW}
                max_score_idx = np.argmax(scores)
                class_id = classes[max_score_idx]
                state = label_map[class_id]
                
                return state
        except Exception:
            return TrafficLight.UNKNOWN
