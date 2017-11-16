#from keras.models import load_model
from styx_msgs.msg import TrafficLight
import numpy as np
import cv2


class ColorClassifier(object):
    def __init__(self):
	#load color classifier
        #self.cls_model = load_model('tl_model_1.h5')
	pass

    def get_color(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): cropped image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)


        # Resize cropped 
        img_resize = cv2.resize(image, (32, 32)) 
        # Convert to four-dimension input as required by Keras
        img_resize = np.expand_dims(img_resize, axis=0).astype('float32')
        # Normalization
        img_resize/=255.
        # Prediction
        predict = self.cls_model.predict(img_resize)
        predict = np.squeeze(predict, axis =0)
        # Get color classification
        tl_color = np.argmax(predict)
	if tl_color==0:
	    state = TrafficLight.RED
	elif tl_color==1:
	    state = TrafficLight.GREEN
	elif tl_color==2:
	    state = TrafficLight.YELLOW
	else:
	    state = TrafficLight.UNKNOWN

	"""
	#classify by color 
        hls = cv2.cvtColor(image, cv2.COLOR_RGB2HLS)
        H = hls[:,:,0]
        L = hls[:,:,1]
        S = hls[:,:,2]
	L_max = np.max(L) 
	L_mean = np.mean(L)        

        r_H_thresh = (160, 190)
	r_binary = np.zeros_like(H)

        y_H_thresh = (0, 30)
	y_binary = np.zeros_like(H)

	g_H_thresh = (60, 90)
	g_binary = np.zeros_like(H)

        L_thresh = (L_mean+(L_max-L_mean)*0.5, 255)        

        r_binary[ ((H > r_H_thresh[0]) & (H <= r_H_thresh[1])) & ((L > L_thresh[0]) & (L <= L_thresh[1])) ] = 1
	r_sum = np.sum(r_binary) 

        y_binary[ ((H > y_H_thresh[0]) & (H <= y_H_thresh[1])) & ((L > L_thresh[0]) & (L <= L_thresh[1])) ] = 1
	y_sum = np.sum(y_binary) 

        g_binary[ ((H > g_H_thresh[0]) & (H <= g_H_thresh[1])) & ((L > L_thresh[0]) & (L <= L_thresh[1])) ] = 1
	g_sum = np.sum(g_binary) 
        
	if ( r_sum>=y_sum and r_sum>g_sum):
            state = TrafficLight.RED
        elif ( y_sum>=r_sum and y_sum>g_sum):
            state = TrafficLight.YELLOW
	elif ( g_sum>y_sum and g_sum>r_sum):
	    state = TrafficLight.GREEN
	else :
	    state = TrafficLight.UNKNOWN
	    #rospy.logwarn('Cant figure out color')

	return state
