from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import rospy
from collections import defaultdict
import os

GRAPH_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    'frozen_inference_graph_v0.pb')

class TLClassifier(object):
    def __init__(self):
        """Loads a frozen inference graph"""
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(GRAPH_FILE, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')

    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return: boxes with min_score threshold applied"""
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
        light = TrafficLight.UNKNOWN
        with tf.Session(graph=self.detection_graph) as sess:
            image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)
            (boxes, scores, classes) = sess.run(
                [self.detection_boxes, self.detection_scores, self.detection_classes],
                feed_dict={self.image_tensor: image_np})
            boxes = np.squeeze(boxes)
            scores = np.squeeze(scores)
            classes = np.squeeze(classes)
            boxes, scores, classes = self.filter_boxes(0.5, boxes, scores, classes)
            for c in classes:
                class_id = int(c)
                # 1 = green, 2 = red, 3 = yellow, 4 = unknown
                labels = {1: 'green', 2: 'red', 3: 'yellow'}
                if class_id in labels:
                    rospy.loginfo("traffic light label: %s", labels[class_id])
                else:
                    rospy.loginfo("traffic light class not found: %d", class_id)
                if class_id == 2:
                    return TrafficLight.RED
        return TrafficLight.UNKNOWN
