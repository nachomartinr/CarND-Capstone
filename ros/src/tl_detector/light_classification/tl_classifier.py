import rospy
import cv2
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
#from PIL import Image
from PIL import ImageDraw
from PIL import ImageColor

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
	SSD_GRAPH_FILE = 'ssd_mobilenet_v1_coco_11_06_2017/frozen_inference_graph.pb'
	self.detection_graph = self.load_graph(SSD_GRAPH_FILE)	
	# The input placeholder for the image.
	# `get_tensor_by_name` returns the Tensor with the associated name in the Graph.
	self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')

	# Each box represents a part of the image where a particular object was detected.
	self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')

	# Each score represent how level of confidence for each of the objects.
	# Score is shown on the result image, together with the class label.
	self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')

	# The classification of the object (integer id).
	self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
	
	config = tf.ConfigProto()
	config.gpu_options.allow_growth = True
	self.sess = tf.Session(graph=self.detection_graph, config=config)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	image_np = np.expand_dims(np.asarray(image, dtype=np.uint8), 0)
	# Actual detection.
	with self.detection_graph.as_default():
    	    (boxes, scores, classes) = self.sess.run([self.detection_boxes, self.detection_scores, self.detection_classes], feed_dict={self.image_tensor: image_np})

    	    # Remove unnecessary dimensions
    	    boxes = np.squeeze(boxes)
    	    scores = np.squeeze(scores)
    	    classes = np.squeeze(classes)

    	    confidence_cutoff = 0.2
    	    # Filter boxes with a confidence score less than `confidence_cutoff`
    	    boxes, scores, classes = self.filter_boxes(confidence_cutoff, boxes, scores, classes)
	    
    	    # The current box coordinates are normalized to a range between 0 and 1.
    	    # This converts the coordinates actual location on the image.
    	    height, width, _= image.shape
    	    box_coords = self.to_image_coords(boxes, height, width)
	    #image = self.draw_boxes(image, box_coords, thickness=4)
	    #image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
	    #bot, left, top, right = box_coords[0, ...]
	    
	    #image = image[int(bot):int(top), int(left):int(right)]
	    state = TrafficLight.GREEN  
	    if len(box_coords)>0:
        	bot, left, top, right = box_coords[0, ...]
        	img = image[int(bot):int(top), int(left):int(right)]
        	hls = cv2.cvtColor(img, cv2.COLOR_RGB2HLS)
        	H = hls[:,:,0]
        	L = hls[:,:,1]
        	S = hls[:,:,2]
        
        	H_r_thresh = (160, 190)
        	H_y_thresh = (10, 40)
        	L_thresh = (100, 255)
        	binary = np.zeros_like(S)

        	binary[ (((H > H_r_thresh[0]) & (H <= H_r_thresh[1])) | ((H > H_y_thresh[0]) & (H <= H_y_thresh[1]))) & ((L > L_thresh[0]) & (L <= L_thresh[1])) ] = 1
		percent = float( sum(sum(binary)) ) / float( binary.shape[0]*binary.shape[1] )
        	if percent>0.02:
		    #cv2.putText(img, 'S',(0,100), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)  
            	    state = TrafficLight.RED
       		else:
		    #cv2.putText(img, 'G',(0,100), cv2.FONT_HERSHEY_SIMPLEX, 1,(255,255,255),2,cv2.LINE_AA)
            	    state = TrafficLight.GREEN   
     	    else:
        	state = TrafficLight.GREEN  
	    #name = str(self.i)+'.jpg'
	    #cv2.imwrite('/home/student/CarND-Capstone/ros/'+name, cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
	    #self.i += 1

        return state

    def filter_boxes(self, min_score, boxes, scores, classes):
        """Return boxes with a confidence >= `min_score`"""
        n = len(classes)
        idxs = []
        max_score = 0
        index = None

        for i in range(n):
            if scores[i] >= min_score and scores[i] > max_score and classes[i]==10 :
	        max_score = scores[i]
                index = i
    
        if index is not None:
	    idxs.append(index)
    
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]
        return filtered_boxes, filtered_scores, filtered_classes

    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].
        
        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width
    
        return box_coords

    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph

    def draw_boxes(self, image, boxes, thickness=4):
        """Draw bounding boxes on the image"""
        for i in range(len(boxes)):
            bot, left, top, right = boxes[i, ...]
            color = 'red'
            cv2.rectangle(image,(left, bot), (right, top), [255,0,0], thickness=4)
	return image
