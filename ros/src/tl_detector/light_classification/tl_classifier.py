from styx_msgs.msg import TrafficLight
import numpy as np
import os.path
import rospy
import tensorflow as tf

class TLClassifier(object):
    def __init__(self):
        
        self.detected_light = TrafficLight.UNKNOWN      
                
        # Load the classifier
        pwd = os.path.dirname(os.path.realpath(__file__))     
        PATH_TO_MODEL = pwd + '/frozen_inference_graph.pb'
        self.detection_graph = tf.Graph()
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            
            with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
            
            self.sess = tf.Session(graph=self.detection_graph)
            
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
        self.num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')
        

    def get_classification(self, image):
        
        MIN_SCORE_THRESH = 0.40
        
        # Perform the detection
        with self.detection_graph.as_default():
            # Expand dimension since the model expects image to have shape [1, None, None, 3].
            img_expanded = np.expand_dims(image, axis=0)  
            (boxes, scores, classes, num) = self.sess.run(
                [self.detection_boxes, 
                 self.detection_scores, 
                 self.detection_classes, 
                 self.num_detections],
                feed_dict={self.image_tensor: img_expanded})
    
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes).astype(np.int32) 
        #rospy.loginfo('%d Detections', num)       

        # Save the largest boxes (aka closest) with the highest score as the detection
        largest_box_width = 0
        largest_box_height = 0
        best_score = MIN_SCORE_THRESH
        best_class_idx = 2    
        self.detected_light = TrafficLight.UNKNOWN
        detected = False
        
        # Detections are ranked with highest scores first, only process a handful
        NUM_DETECTIONS_TO_USE = 10   
        
        for idx in range(NUM_DETECTIONS_TO_USE):
            if scores is None or boxes is None or classes is None:
                rospy.loginfo('Odd: %d Detections, missing score, box, or class', num)
            
            elif scores[idx] > best_score:    
                rospy.loginfo('    Possible %d score %.3f', classes[idx], scores[idx])
                box_width = (boxes[idx][3] - boxes[idx][1])
                box_height = (boxes[idx][2] - boxes[idx][0])
            
                if box_width > largest_box_width and box_height > largest_box_height:
                    best_score = scores[idx]
                    largest_box_width = box_width
                    largest_box_height = box_height
                    best_class_idx = classes[idx]
                    detected = True
                    rospy.loginfo('    Best %d score %.3f', classes[idx], scores[idx])          
        
        if detected:
            self.detected_light = self.decode_classification(best_class_idx)
            rospy.loginfo('Detected %d', best_class_idx)
                         
        return self.detected_light
    
    def decode_classification(self, class_index):
        # Constants from the label_map.pbtext used in training
        GREEN_LIGHT_CLASS = 1
        RED_LIGHT_CLASS = 2
        YELLOW_LIGHT_CLASS = 7     
        
        if class_index == RED_LIGHT_CLASS:
            detected_light = TrafficLight.RED
        elif class_index == GREEN_LIGHT_CLASS:
            detected_light = TrafficLight.GREEN
        elif class_index == YELLOW_LIGHT_CLASS:
            detected_light = TrafficLight.YELLOW
        else:
            detected_light = TrafficLight.UNKNOWN
            
        return detected_light
