# Import packages
import os
import json
import numpy as np
import cv2
import time 
import tensorflow as tf
from loguru import logger
from threading import Thread
from tensorflow.lite.python.interpreter import Interpreter
from device_control import Module
logger.info("Imported packages successfully")


class VideoStream:
    def __init__(self):
	    # Variable to control when the camera is stopped
        self.video = None
        self.stopped = False 
        self.video_thread = None
        self.camera_available = False

    def start(self,resolution=(640,480)):
        # Initialize the PiCamera and the camera image stream
        self.video = cv2.VideoCapture(0)
        ret = self.video.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        ret = self.video.set(3,resolution[0])
        ret = self.video.set(4,resolution[1])
        
        logger.info("Video capture initialized")
	    # Start the thread that reads frames from the video stream
        self.stopped = False
        self.video_thread = Thread(target=self.update,args=())
        self.video_thread.start()
        if self.video_thread.is_alive():
            logger.info("Video thread started")

    def thread_is_alive(self):
        return self.video_thread.is_alive()
    
    def check_camera_available(self):
        self.video = cv2.VideoCapture(0)
        if self.video.isOpened():
            self.video.release()
            self.camera_available = True
            logger.info("Camera found")
            return True
        else:
            logger.error("Camera not found")
            return False
        
    def check_camera_open(self):
        if self.video is not None:
            self.video.release()
            logger.info("Video capture released")
            time.sleep(1)

    def update(self):
        # Keep looping indefinitely until the thread is stopped
        while True:
            # If the camera is stopped, stop the thread
            if self.stopped:
                # Close camera resources
                self.video.release()
                return
            
            # Otherwise, grab the next frame from the stream
            (self.grabbed, self.frame) = self.video.read()
        
    def read(self):
	# Return the most recent frame
        return self.frame

    def stop(self):
	# Indicate that the camera and thread should be stopped
        self.stopped = True
    
class ObjectDetector:
    
    def __init__(self):
        # Load config file
        config = "config.json"
        logger.info("Loading config file")
        try:
            with open(config, 'r') as config_file:
                self.model_config = json.load(config_file)
            logger.info("Loaded config file successfully")
        except:
            logger.error("Failed to load config file")
            return
        
        # Load model config
        self.current_path = os.getcwd()
        self.config_path = os.path.join("config")
        self.model_path = os.path.join(self.current_path,self.model_config["model_folder"],self.model_config["model_file"])
        self.labels = self.model_config["model_classes"]
        self.min_conf_threshold = self.model_config["config"]["min_conf_threshold"]
        
        # Load parameters config
        self.image_width = self.model_config["config"]["image_width"]
        self.image_height = self.model_config["config"]["image_height"] 
        self.min_conf_threshold = self.model_config["config"]["min_conf_threshold"]
        self.input_mean = self.model_config["config"]["input_mean"]
        self.input_std = self.model_config["config"]["input_std"]
        self.interpreter = Interpreter(model_path=self.model_path)
        self.interpreter.allocate_tensors()
        
        #line and distance line config
        self.leftline = self.model_config["lineconfig"]["leftline"]
        self.rightline = self.model_config["lineconfig"]["rightline"]
        self.distance1 = self.model_config["lineconfig"]["one_meter"]
        self.distance2 = self.model_config["lineconfig"]["two_meter"]
        self.distance3 = self.model_config["lineconfig"]["three_meter"]
        
        #Priority lists 
        self.middle_bottom = []
        self.middle_center = []
        self.middle_top = []
        self.side_bottom = []
        #Can simply use only 4 list, Determine the distance for side list might be unnecessary 
        self.side_center = []
        # self.side_top = []
        
        # Initialize video stream
        self.videostream = VideoStream()
        self.onprocess = False
        
        #Module variable
        self.module = Module()
        self.mode = None
        self.takesnap = False
        self.takesnap_status = False

        # Start Function Here
        self.setup()
        self.thread_start()
        self.run()
        
    def thread_start(self):
        
        self.thread_object_warning = Thread(target=self.object_warning_realtime, args=())
        self.thread_object_warning.daemon = True
        if self.mode == "realtime":
            self.thread_object_warning.start()
        
        #Check mode
        self.thread_mode = Thread(target=self.set_value, args=())
        self.thread_mode.daemon = True
        self.thread_mode.start()
        
    def setup(self):
        logger.info("Setting up object detector")
        # Get model details
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]

        self.floating_model = (self.input_details[0]['dtype'] == np.float32)

        # Check output layer name to determine if this model was created with TF2 or TF1,
        # because outputs are ordered differently for TF2 and TF1 models
        self.outname = self.output_details[0]['name']

        if ('StatefulPartitionedCall' in self.outname): # This is a TF2 model
            self.boxes_idx, self.classes_idx, self.scores_idx = 1, 3, 0
            logger.info("Detected TF2 model")
        else: # This is a TF1 model
            self.boxes_idx, self.classes_idx, self.scores_idx = 0, 1, 2
            logger.info("Detected TF1 model")
        
        # Check setup
        self.module.play_audio("ตรวจสอบอุปกรณ์")
        if self.videostream.check_camera_available() == False:
            self.module.play_audio("กล้องขัดข้อง")
        self.ulrasonic = self.module.check_ultrasonic()
        
        if self.ulrasonic[0] == False:
            self.module.play_audio("เซ็นเซอร์ระยะห่างขัดข้อง")
            self.module.ultrasonic_break_one()
            
        if self.ulrasonic[1] == False:
            self.module.play_audio("เซ็นเซอร์พื้นต่างระดับขัดข้อง")
            self.module.ultrasonic_break_two()
            
        if self.ulrasonic[0] and self.ulrasonic[1] and self.videostream.camera_available :
            self.module.play_audio("อุปกรณ์พร้อมใช้งาน")
            
        elif self.ulrasonic[0] and self.ulrasonic[1] and self.videostream.camera_available == False:
            self.module.play_audio("จะทำการแจ้งเตือนด้วยเซ็นเซอร์เป็นหลัก")
            
        elif self.ulrasonic[0]== False and self.ulrasonic[1]== False and self.videostream.camera_available:
            self.module.play_audio("โปรดตรวจสอบเซ็นเซอร์เพื่อความปลอดภัยในการใช้งาน")
            logger.error("Ultrasonic sensor not available")
            self.module.ultrasonic_break_one()
            self.module.ultrasonic_break_two()
            
        elif self.ulrasonic[0] == False and self.ulrasonic[1] == False and self.videostream.camera_available == False:
            self.module.play_audio("อุปกรณ์ไม่พร้อมใช้งาน")
            self.module.play_audio("หยุดการทำงาน")
            self.module.buzzer("shutdown")
            #Shutdown UPS
            os.system("sudo x708softsd.sh")
            
        logger.info("Setup complete")
        self.module.setup_complete()
            
    def run(self):
        logger.info("Running object detector")
        self.module.play_audio("เริ่มการทำงาน")
        self.module.buzzer("start")
        # Initialize frame rate calculation
        frame_rate_calc = 1
        freq = cv2.getTickFrequency()

        while self.videostream.camera_available:
            
            if self.mode == "realtime" and hasattr(self, "realtime_on") and not hasattr(self, "realtime_open"):
                # Initialize video stream
                setattr(self, "realtime_open", True)
                self.videostream.check_camera_open()
                self.videostream.start(resolution=(self.image_width,self.image_height))
                time.sleep(1)
                
            elif self.mode == "snapshot" and self.takesnap == True and self.onprocess == False:
                # Initialize video stream
                self.videostream.check_camera_open()
                self.videostream.start(resolution=(self.image_width,self.image_height))
                self.onprocess = True
                time.sleep(1)
                logger.info("Snapshot mode on")
                
            if hasattr(self, "realtime_on") or self.onprocess == True:
                # Start timer (for calculating frame rate)
                t1 = cv2.getTickCount()
                # Grab frame from video stream
                try:
                    # Acquire frame and resize to expected shape [1xHxWx3]
                    frame1 = self.videostream.read()
                    self.frame = frame1.copy()
                
                    # Acquire frame and resize to expected shape [1xHxWx3]
                    self.frame_rgb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
                    self.frame_resized = cv2.resize(self.frame_rgb, (self.width,self.height))
                    self.input_data = np.expand_dims(self.frame_resized, axis=0)

                    # Normalize pixel values if using a floating model (i.e. if model is non-quantized)
                    if self.floating_model:
                        self.input_data = (np.float32(self.input_data) - self.input_mean) / self.input_std

                    # Perform the actual detection by running the model with the image as input
                    self.interpreter.set_tensor(self.input_details[0]['index'],self.input_data)
                    self.interpreter.invoke()

                    # Retrieve detection results
                    self.boxes = self.interpreter.get_tensor(self.output_details[self.boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
                    self.classes = self.interpreter.get_tensor(self.output_details[self.classes_idx]['index'])[0] # Class index of detected objects
                    self.scores = self.interpreter.get_tensor(self.output_details[self.scores_idx]['index'])[0] # Confidence of detected objects

                    # Loop over all detections and draw detection box if confidence is above minimum threshold
                    for i in range(len(self.scores)):
                        if ((self.scores[i] > self.min_conf_threshold) and (self.scores[i] <= 1.0)):
                            # Get bounding box coordinates and draw box
                            # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                            ymin = int(max(1,(self.boxes[i][0] * self.image_height)))
                            xmin = int(max(1,(self.boxes[i][1] * self.image_width)))
                            ymax = int(min(self.image_height,(self.boxes[i][2] * self.image_height)))
                            xmax = int(min(self.image_width,(self.boxes[i][3] * self.image_width)))
                            center_x = (xmin + xmax) / 2
                            # cv2.rectangle(self.frame, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)
                            
                            # Draw label
                            self.object_name = self.labels[int(self.classes[i])] # Look up object name from "labels" array using class index
                            self.label = '%s: %d%%' % (self.object_name, int(self.scores[i]*100)) # Example: 'person: 72%'
                            labelSize, baseLine = cv2.getTextSize(self.label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                            label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                            # cv2.rectangle(self.frame, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                            # cv2.putText(self.frame, self.label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
                            
                            # Draw distance
                            bbox_width = (xmax - xmin)
                            self.distance = round(((self.model_config["config"]["known_distance"] * self.model_config["config"]["focal_length"]) / bbox_width)/1000, 2)
                            # cv2.putText(self.frame, f"Distance: {self.distance:.2f} m", (xmin, label_ymin-40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            
                            # Gather object information
                            self.object_gather(center_x, ymax, self.object_name, self.distance)
                            if self.mode == "snapshot":
                                self.object_warning_snapshot()
                except:
                    logger.error("frame1 is None")
                # Draw framerate in corner of frame
                # cv2.putText(self.frame,'FPS: {0:.2f}'.format(frame_rate_calc),(30,50),cv2.FONT_HERSHEY_SIMPLEX,1,(255,255,0),2,cv2.LINE_AA)
                
                # Draw distance lines (Delectable)
                # self.addDistanceLine()
            
                # All the results have been drawn on the frame, so it's time to display it.
                # cv2.imshow('Object detector', self.frame)
                # Calculate framerate
                t2 = cv2.getTickCount()
                time1 = (t2-t1)/freq
                frame_rate_calc= 1/time1
                
                if hasattr(self, "snapshot_on"):
                    try:
                        if self.videostream.thread_is_alive():
                            self.videostream.stop()
                            logger.info("Video thread stopped")
                    except:
                        logger.error("Video thread is not alive")
                    self.frame = None
                    self.onprocess = False
                    logger.info("Snapshot finish")

            else:
                logger.error("camera is not initialized")
                time.sleep(1)
            # # Press 'q' to quit
            # if cv2.waitKey(1) == ord('q'):
            #     break

        # # Clean up
        # cv2.destroyAllWindows()
        
    def addDistanceLine(self):
        # Add distance lines
        cv2.line(self.frame, (self.leftline,0), (self.leftline,480), (255, 255, 0), 2)
        cv2.line(self.frame, (self.rightline, 0), (self.rightline,480), (255, 255, 0), 2)
        
        cv2.line(self.frame, (0,self.distance1), (640,self.distance1), (255, 255, 0), 1)
        cv2.line(self.frame, (0,self.distance2), (640,self.distance2), (255, 255, 0), 1)
        cv2.line(self.frame, (0,self.distance3), (640,self.distance3), (255, 255, 0), 1)
        
    def object_gather(self, center_x, ymax, object_name, distance):
        # Choose the appropriate list based on ymin
        #middle bottom priority=1
        if ymax > self.distance1 and self.leftline <= center_x <= self.rightline:
            self.middle_bottom.append((distance, object_name))
            self.middle_bottom.sort()
            logger.info("middle_bottom")
            
        #middle center priority=2
        elif self.distance2 <= ymax <= self.distance1 and self.leftline <= center_x <= self.rightline:
            self.middle_center.append((distance, object_name))
            self.middle_center.sort()
            logger.info("middle_center")
            
        #middle top priority=3
        elif self.distance2 > ymax > self.distance3 and self.leftline <= center_x <= self.rightline:
            self.middle_top.append((distance, object_name))
            self.middle_top.sort()
            logger.info("middle_top")
            
        #sides bottom priority=4
        elif ymax > self.distance1 and (self.leftline >= center_x or center_x >= self.rightline):
            if center_x >= self.rightline:
                leftside = False
            else:
                leftside = True
            self.side_bottom.append((distance, object_name, leftside))
            self.side_bottom.sort()
            logger.info("sides_bottom")
            
        #sides center priority=5
        elif self.distance2 <= ymax <= self.distance1 and (self.leftline >= center_x or center_x >= self.rightline):
            if center_x >= self.rightline:
                leftside = False
            else:
                leftside = True
            self.side_center.append((distance, object_name, leftside))
            self.side_center.sort()
            logger.info("side_center")
            
        #sides top priority=6   
        # elif self.distance2 > ymax > self.distance3 and (self.leftline >= center_x or center_x >= self.rightline):  
        #     if center_x >= self.rightline:
        #         leftside = False
        #     else:
        #         leftside = True
        #     self.side_top.append((distance, object_name, leftside))
        #     self.side_top.sort()
        #     logger.info("side_top")
            
        else:
            logger.error("Object is not within the distance to be examined")
            return

    def object_warning_realtime(self):
        # Check if there are any objects in the lists
        while self.mode == "realtime":
            # if self.middle_bottom or self.middle_center or self.middle_top or self.side_bottom or self.side_center or self.side_top:
            if self.middle_bottom or self.middle_center or self.middle_top or self.side_bottom or self.side_center:
                # Check if there are any objects in the lists
                if self.middle_bottom:
                    # Check if the first object in the list is within the warning distance
                    if self.middle_bottom[0][0] :
                        # logger.info a warning
                        logger.info(f"WARNING: {self.middle_bottom[0][1]} is within {self.middle_bottom[0][0]} M")
                        self.module.play_audio(self.middle_bottom[0][1])
                        self.module.play_audio("ข้างหน้า")
                elif self.middle_center:
                    if self.middle_center[0][0] :
                        logger.info(f"WARNING: {self.middle_center[0][1]} is within {self.middle_center[0][0]} M")
                        # self.module.play_audio("ตรวจพบ")
                        self.module.play_audio(self.middle_center[0][1])
                        self.module.play_audio("ข้างหน้า")
                elif self.middle_top:
                    if self.middle_top[0][0] :
                        logger.info(f"WARNING: {self.middle_top[0][1]} is within {self.middle_top[0][0]} M")
                        # self.module.play_audio("ตรวจพบ")
                        self.module.play_audio(self.middle_top[0][1])
                        self.module.play_audio("ข้างหน้า")
                elif self.side_bottom:
                    if self.side_bottom[0][0] :  #< self.model_config["config"]["warning_distance"]
                        if self.side_bottom[0][2] != True:
                            logger.info(f"WARNING: {self.side_bottom[0][1]} is within {self.side_bottom[0][0]} M, Move to the right side")       #playsound for rightside
                            self.module.play_audio(self.side_bottom[0][1])
                            self.module.play_audio("ทางขวา")
                        else:
                            logger.info(f"WARNING: {self.side_bottom[0][1]} is within {self.side_bottom[0][0]} M, Move to the left side")       #playsound for leftside
                            self.module.play_audio(self.side_bottom[0][1])
                            self.module.play_audio("ทางซ้าย")
                elif self.side_center:
                    if self.side_center[0][0] :
                        if self.side_center[0][2] != True:
                            logger.info(f"WARNING: {self.side_center[0][1]} is within {self.side_center[0][0]} M, Move to the right side")  #playsound for rightside
                            self.module.play_audio(self.side_center[0][1])
                            self.module.play_audio("ทางขวา")
                        else:
                            logger.info(f"WARNING: {self.side_center[0][1]} is within {self.side_center[0][0]} M, Movve to the left side")  #playsound for leftside
                            self.module.play_audio(self.side_center[0][1])
                            self.module.play_audio("ทางซ้าย")
                # elif self.side_top:
                #     if self.side_top[0][0] :
                #         if self.side_top[0][2] != True:
                #             logger.info(f"WARNING: {self.side_top[0][1]} is within {self.side_top[0][0]} M, Move to the right side")  #playsound for rightside
                #             self.module.play_audio(self.side_top[0][1])
                #             self.module.play_audio("ทางขวา")
                #         else:   
                #             logger.info(f"WARNING: {self.side_top[0][1]} is within {self.side_top[0][0]} M, Move to the left side")  #playsound for leftside
                #             self.module.play_audio(self.side_top[0][1])
                #             self.module.play_audio("ทางซ้าย")
            else:  
                logger.info("No objects detected")
                    
            #Clear all list for next group of object
            self.middle_bottom.clear()
            self.middle_center.clear()
            self.middle_top.clear()
            self.side_bottom.clear()
            self.side_center.clear()
            # self.side_top.clear()
            time.sleep(self.model_config["config"]["warning_delay"])

    def object_warning_snapshot(self):
        # Check if there are any objects in the lists
        if self.middle_bottom or self.middle_center or self.middle_top or self.side_bottom or self.side_center:
            if self.middle_bottom:
                # Check if the first object in the list is within the warning distance
                if self.middle_bottom[0][0] :
                    # logger.info a warning
                    logger.info(f"WARNING: {self.middle_bottom[0][1]} is within {self.middle_bottom[0][0]} M")
                    self.module.play_audio(self.middle_bottom[0][1])
                    self.module.play_audio("ข้างหน้า")
            elif self.middle_center:
                if self.middle_center[0][0] :
                    logger.info(f"WARNING: {self.middle_center[0][1]} is within {self.middle_center[0][0]} M")
                    # self.module.play_audio("ตรวจพบ")
                    self.module.play_audio(self.middle_center[0][1])
                    self.module.play_audio("ข้างหน้า")
            elif self.middle_top:
                if self.middle_top[0][0] :
                    logger.info(f"WARNING: {self.middle_top[0][1]} is within {self.middle_top[0][0]} M")
                    # self.module.play_audio("ตรวจพบ")
                    self.module.play_audio(self.middle_top[0][1])
                    self.module.play_audio("ข้างหน้า")
            elif self.side_bottom:
                if self.side_bottom[0][0] :  #< self.model_config["config"]["warning_distance"]
                    if self.side_bottom[0][2] != True:
                        logger.info(f"WARNING: {self.side_bottom[0][1]} is within {self.side_bottom[0][0]} M, Move to the right side")       #playsound for rightside
                        self.module.play_audio(self.side_bottom[0][1])
                        self.module.play_audio("ทางขวา")
                    else:
                        logger.info(f"WARNING: {self.side_bottom[0][1]} is within {self.side_bottom[0][0]} M, Move to the left side")       #playsound for leftside
                        self.module.play_audio(self.side_bottom[0][1])
                        self.module.play_audio("ทางซ้าย")
            elif self.side_center:
                if self.side_center[0][0] :
                    if self.side_center[0][2] != True:
                        logger.info(f"WARNING: {self.side_center[0][1]} is within {self.side_center[0][0]} M, Move to the right side")  #playsound for rightside
                        self.module.play_audio(self.side_center[0][1])
                        self.module.play_audio("ทางขวา")
                    else:
                        logger.info(f"WARNING: {self.side_center[0][1]} is within {self.side_center[0][0]} M, Movve to the left side")  #playsound for leftside
                        self.module.play_audio(self.side_center[0][1])
                        self.module.play_audio("ทางซ้าย")
            # elif self.side_top:
            #     if self.side_top[0][0] :
            #         if self.side_top[0][2] != True:
            #             logger.info(f"WARNING: {self.side_top[0][1]} is within {self.side_top[0][0]} M, Move to the right side")  #playsound for rightside
            #             self.module.play_audio(self.side_top[0][1])
            #             self.module.play_audio("ทางขวา")
            #         else:   
            #             logger.info(f"WARNING: {self.side_top[0][1]} is within {self.side_top[0][0]} M, Move to the left side")  #playsound for leftside
            #             self.module.play_audio(self.side_top[0][1])
            #             self.module.play_audio("ทางซ้าย")
        else:
            self.module.play_audio("ไม่พบวัตถุในระยะ")
            logger.info("No objects detected")
    
        #Clear all list for next group of object
        self.middle_bottom.clear()
        self.middle_center.clear()
        self.middle_top.clear()
        self.side_bottom.clear()
        self.side_center.clear()
        # self.side_top.clear() 
        
    def set_value(self):
        while True:
        
            self.takesnap = self.module.get_snapshot()
            if self.takesnap :
                self.module.buzzer("snapshot")
                
            self.mode = self.module.get_current_mode()
            if self.mode == "snapshot" and not hasattr(self, "snapshot_on"):
                try:
                    if self.videostream.thread_is_alive():
                        self.videostream.stop()
                        self.frame = None
                        logger.info("Video thread stopped")
                except:
                    logger.error("Video thread is not alive")
                if hasattr(self, "realtime_on"):
                    delattr(self, "realtime_on")
                    delattr(self, "realtime_open")
                    logger.info("Realtime mode off")
                setattr(self, "snapshot_on", True)
                if self.videostream.camera_available:
                    self.module.play_audio("ใช้งานโหมดถ่ายภาพ")
                    self.module.buzzer("changemode")
                    logger.info("changed mode to snapshot")
                    if self.thread_object_warning.is_alive():
                        self.thread_object_warning.join()
                
            elif self.mode == "realtime" and not hasattr(self, "realtime_on"):
                setattr(self, "realtime_on", True)
                logger.info("Realtime mode on")
                self.module.play_audio("ใช้งานโหมดอัตโนมัติ")
                
                if hasattr(self, "snapshot_on"):
                    delattr(self, "snapshot_on")
                    logger.info("snapshot mode off")
                self.frame = None
                if self.videostream.camera_available:
                    if not self.thread_object_warning.is_alive():
                        self.thread_object_warning = Thread(target=self.object_warning_realtime, args=())
                        self.thread_object_warning.daemon = True
                        self.thread_object_warning.start()
                        
                        
            time.sleep(0.1)
            

if __name__ == "__main__":
    ObjectDetector()
else:
    logger.error("This file cannot be imported")
