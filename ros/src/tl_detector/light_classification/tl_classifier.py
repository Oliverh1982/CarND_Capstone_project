from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2
import rospy
import datetime

class TLClassifier(object):
    def __init__(self, is_real_site):
        #TODO load classifier
        if is_real_site:
            PATH_TO_GRAPH = 'light_classification/model/frozen_inference_graph_real.pb'
            rospy.logwarn("real world") 
        else:
            PATH_TO_GRAPH = 'light_classification/model/frozen_inference_graph_sim.pb'
            rospy.logwarn("simulator")
    
        self.is_real_site = is_real_site
        self.gamma = 0.6
        self.correct_gamma = True

        self.graph = tf.Graph()
        self.threshold = .5

        with self.graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(PATH_TO_GRAPH, 'rb') as fid:
                od_graph_def.ParseFromString(fid.read())
                tf.import_graph_def(od_graph_def, name='')
 
            #name = [tensor.name for tensor in tf.get_default_graph().as_graph_def().node]
            #print(name,"\n")

            self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')
            self.boxes = self.graph.get_tensor_by_name('detection_boxes:0')
            self.scores = self.graph.get_tensor_by_name('detection_scores:0')
            self.classes = self.graph.get_tensor_by_name('detection_classes:0')
            self.num_detections = self.graph.get_tensor_by_name(
                'num_detections:0')
        
        self.sess = tf.Session(graph=self.graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if self.is_real_site : #for real site, adjust gamma for better recognition
            if self.correct_gamma:
                if self.gamma == 1.0:
                    self.gamma = 0.6
                elif self.gamma == 0.6:
                    self.gamma = 1.0
            image = self.adjust_gamma(image, self.gamma)
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            detected = False

        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            start = datetime.datetime.now()
            (boxes, scores, classes, num_detections) = self.sess.run(
                [self.boxes, self.scores, self.classes, self.num_detections],
                feed_dict={self.image_tensor: img_expand})
            end = datetime.datetime.now()
            c = end - start
            rospy.loginfo("Light classification took: %.2f", c.total_seconds())

        boxes = np.squeeze(boxes)
        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)

        if self.is_real_site : #for real site
            best_scores = []
            for idx, classID in enumerate(classes):
                if scores[idx] > 0.10:  # confidence level
                    best_scores.append([scores[idx], idx, classID])
                    detected = True

            if detected:
                best_scores.sort(key=lambda tup: tup[0], reverse=True)
                best_score = best_scores[0]
                #rospy.loginfo("number of TL found %d, best score: %f, color: %f", len(best_scores), best_score[0], best_score[2])

                if best_score[2] == 1.0:
                    rospy.loginfo("Real site: Light is GREEN")
                    return TrafficLight.GREEN
                elif best_score[2] == 2.0:
                    rospy.loginfo("Real site: Light is RED")
                    return TrafficLight.RED
                elif best_score[2] == 3.0:
                    rospy.loginfo("Real site:Light is YELLOW")
                    return TrafficLight.YELLOW

            #rospy.loginfo("Light is UNKNOWN")
            return TrafficLight.UNKNOWN
  
        else : #for simulator
            rospy.loginfo("SCORES: %.2f", scores[0])
            rospy.loginfo("CLASSES: %.2f", classes[0])
        
            if scores[0] > self.threshold:
                if classes[0] == 1:
                    rospy.loginfo("Simulator: Light is GREEN")
                    return TrafficLight.GREEN
                elif classes[0] == 2:
                    rospy.loginfo("Simulator: Light is RED")
                    return TrafficLight.RED
                elif classes[0] == 3:
                    rospy.loginfo("Simulator: Light is YELLOW")
                    return TrafficLight.YELLOW
       
        
            return TrafficLight.UNKNOWN
 

    def adjust_gamma(self, bgr, gamma=1.0):
        # build a lookup table mapping the pixel values [0, 255] to
        # their adjusted gamma values
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
                          for i in np.arange(0, 256)]).astype("uint8")
        # apply gamma correction using the lookup table
        return cv2.LUT(bgr, table)

       
