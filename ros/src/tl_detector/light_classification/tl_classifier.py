from styx_msgs.msg import TrafficLight
import rospy
import os
import sys
import time
import numpy as np
import tensorflow as tf
from collections import defaultdict
from io import StringIO
from utils import label_map_util

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        NUM_CLASSES = 13
        CKPT = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'models/frozen_inference_graph.pb')
        PATH_TO_LABELS = os.path.join(os.path.dirname(os.path.realpath(__file__)) , 'models/label_map.pbtxt') 
        label_map = label_map_util.load_labelmap(PATH_TO_LABELS)
        categories = label_map_util.convert_label_map_to_categories(label_map, max_num_classes=NUM_CLASSES, use_display_name=True)
        self.category_index = label_map_util.create_category_index(categories)
        self.detection_graph = tf.Graph()        
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
                with tf.gfile.GFile(CKPT, 'rb') as fid:
                    serialized_graph = fid.read()
                    od_graph_def.ParseFromString(serialized_graph)
                    tf.import_graph_def(od_graph_def, name='')
        self.sess = tf.Session(graph=self.detection_graph)
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        
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
        else:
            state = TrafficLight.RED
        """
        image_np_expanded = np.expand_dims(image, axis=0)
        time0 = time.time()
        with self.detection_graph.as_default():
            op = [self.detection_boxes, self.detection_scores, self.detection_classes, self.num_detections]
            (boxes, scores, classes, num) = self.sess.run(op, feed_dict={self.image_tensor: image_np_expanded})
            time1 = time.time()
            print("Time in milliseconds", (time1 - time0) * 1000)
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes).astype(np.int32)
            self.state = TrafficLight.UNKNOWN
            if scores is None:
                return self.state
            else:
                maxscores = 0
                for i in range(boxes.shape[0]):
                    if scores is None or scores[i] > maxscores:
                        maxscores = scores[i]
                        class_name = self.category_index[classes[i]]['name']
                        print('{}'.format(class_name))
                        if class_name == 'Red':
                             self.state = TrafficLight.RED
                        elif class_name == 'Green':
                            self.state = TrafficLight.GREEN
                        elif class_name == 'Yellow':
                            self.state = TrafficLight.YELLOW
                return self.state
