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
            #PATH_TO_GRAPH = 'light_classification/model/frozen_inference_graph_real.pb'
            PATH_TO_GRAPH = 'light_classification/model/ssd_mobilenet_v1_coco_2017_11_17.pb'
            rospy.logwarn("real world") 
        else:
            #PATH_TO_GRAPH = 'light_classification/model/frozen_inference_graph_sim.pb'
            PATH_TO_GRAPH = 'light_classification/model/ssdlite_mobilenet_v2_coco_2018_05_09.pb'
            rospy.logwarn("simulator")


        self.is_real_site = is_real_site
        self.gamma = 0.6
        self.correct_gamma = True

        self.graph = tf.Graph()
        self.threshold = 0.1

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
            self.sess = tf.Session(graph=self.graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        if self.correct_gamma:
            if self.gamma == 1.0:
                self.gamma = 0.6
            elif self.gamma == 0.6:
                self.gamma = 1.0
        #image = self.adjust_gamma(image, self.gamma)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_np = np.asarray(image, dtype="uint8")
        image_np_expanded = np.expand_dims(image_np, axis=0)

        detected = False

        with self.graph.as_default():
            img_expand = np.expand_dims(image, axis=0)
            start = datetime.datetime.now()
            (boxes, scores, classes) = self.sess.run(
                [self.boxes, self.scores, self.classes],
                feed_dict={self.image_tensor: img_expand})
            end = datetime.datetime.now()
            c = end - start
            rospy.loginfo("Light classification took: %.2f", c.total_seconds())

        boxes = np.squeeze(boxes)
        classes = np.squeeze(classes).astype(np.int32)
        scores = np.squeeze(scores)
        best_scores = []

        for idx, classID in enumerate(classes):

            if self.is_real_site: # for real world
                if scores[idx] > self.threshold:  # confidence level
                    best_scores.append([scores[idx], idx, classID])
                    detected = True
            else : # for simulator
                if classID == 10: # 10 is traffic light
                    if scores[idx] > self.threshold : #confidence level
                        best_scores.append([scores[idx], idx, classID])
                        detected = True

        tl_index = TrafficLight.UNKNOWN
        if detected:
            best_scores.sort(key=lambda tup: tup[0], reverse=True)

            best_score = best_scores[0]
            #rospy.loginfo("number of TL found %d, best score: %f, color: %f", len(best_scores), best_score[0], best_score[2])
            nbox = boxes[best_score[1]]

            height = image.shape[0]
            width = image.shape[1]

            box = np.array([nbox[0]*height, nbox[1]*width, nbox[2]*height, nbox[3]*width]).astype(int)

            tl_cropped = image[box[0]:box[2], box[1]:box[3]]         
            
            # Avoids crash of node if tl_cropped is none (this happened with real site test)
            if tl_cropped is not None:
                rospy.loginfo("Dimensions of tl_cropped: %s, %s", tl_cropped.shape[0], tl_cropped.shape[1])   
                tl_color, tl_index = self.get_color(tl_cropped)

                if tl_index == 0:
                    rospy.loginfo("Light is RED")
                    return TrafficLight.RED
                elif tl_index == 1:
                    rospy.loginfo("Light is YELLOW, but return RED")
                    return TrafficLight.RED
                    #return TrafficLight.YELLOW
                elif tl_index == 2:
                    rospy.loginfo("Light is GREEN")
                    return TrafficLight.GREEN
                else :
                    rospy.loginfo("Light is UNKNOWN")
                    return TrafficLight.UNKNOWN
            else:
                rospy.loginfo("tl_cropped is none")

        return tl_index

    def get_color(self, image_rgb):
        image_lab = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2LAB)
        l = image_lab.copy()
        # set a and b channels to 0
        l[:, :, 1] = 0
        l[:, :, 2] = 0

        std_l = self.standardize_input(l)

        red_slice, yellow_slice, green_slice = self.slice_image(std_l)

        y, x, c = red_slice.shape
        px_sums = []
        color = ['RED', 'YELLOW', 'GREEN', 'UNKNOWN']
        px_sums.append(np.sum(red_slice[0:y, 0:x, 0]))
        px_sums.append(np.sum(yellow_slice[0:y, 0:x, 0]))
        px_sums.append(np.sum(green_slice[0:y, 0:x, 0]))

        max_value = max(px_sums)
        max_index = px_sums.index(max_value)

        return color[max_index], max_index

    def crop(self, image):
        row = 2
        col = 6
        cropped_img = image.copy()
        cropped_img = cropped_img[row:-row, col:-col, :]
        return cropped_img

    def standardize_input(self, image):
        standard_img = np.copy(image)
        standard_img = cv2.resize(standard_img, (32, 32))
        standard_img = self.crop(standard_img)
        return standard_img

    def slice_image(self, image):
        img = image.copy()
        shape = img.shape
        slice_height = shape[0] / 3
        upper = img[0:slice_height, :, :]
        middle = img[slice_height:2 * slice_height, :, :]
        lower = img[2 * slice_height:3 * slice_height, :, :]
        return upper, middle, lower


    def adjust_gamma(self, bgr, gamma=1.0):
        # build a lookup table mapping the pixel values [0, 255] to
        # their adjusted gamma values
        invGamma = 1.0 / gamma
        table = np.array([((i / 255.0) ** invGamma) * 255
                          for i in np.arange(0, 256)]).astype("uint8")
        # apply gamma correction using the lookup table
        return cv2.LUT(bgr, table)

       
