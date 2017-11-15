from keras.models import load_model
from styx_msgs.msg import TrafficLight
import numpy as np
import cv2


class ColorClassifier(object):
    def __init__(self):
	#load color classifier
        self.cls_model = load_model('tl_model_new.h5')

    def get_color(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): cropped image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
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
	    """        
        
        return state
