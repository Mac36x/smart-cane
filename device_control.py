import RPi.GPIO as GPIO
import struct
import smbus
import time
import subprocess
import queue
import threading
from threading import Thread
import os
import json
from loguru import logger

class Module:
    def __init__(self):
        # Load config file
        config = "config.json"
        logger.info("Loading config file")
        with open(config, 'r') as config_file:
            self.module_config = json.load(config_file)
        logger.info("Loaded config file successfully")
        
        #load pin
        self.ultrasonic_one = self.module_config['ultrasonic']['trigger_pin1'], self.module_config['ultrasonic']['echo_pin1'], self.module_config['ultrasonic']['distant_one_alert']
        self.ultrasonic_two = self.module_config['ultrasonic']['trigger_pin2'], self.module_config['ultrasonic']['echo_pin2'], self.module_config['ultrasonic']['distant_two_alert']
        self.switch_mode = self.module_config['switch']['switch_mode']
        self.switch_snapshot = self.module_config['switch']['switch_snapshot']
        self.buzzer_pin = self.module_config['buzzer']['buzzer_pin']
        
        #set up ups bus
        self.bus = smbus.SMBus(1)
        self.address = 0x36
        
        #for calling for capacity
        self.capacity = 0
     
        #for calling for ultrasonic
        self.ultrasonic_one_status = False
        self.ultrasonic_two_status = False
        self.ultrasonic_one_break = False
        self.ultrasonic_two_break = False
        
        try:
            GPIO.cleanup()
        except:
            logger.info("GPIO cleanup failed")
        
        #for calling for mode
        self.mode = None
        
        #for calling for snapshot
        self.takesnap = False
        
        #enable all notification after setup
        self.setup_status = False
        
        self.lock = threading.Lock()
        self.audio_queue = queue.Queue()
        
        #start all function
        self.setup()
        self.thread_start()
        
    def setup(self):
        # set GPIO direction (IN / OUT)
        GPIO.setmode(GPIO.BCM)
        #ultrasonic 1
        GPIO.setup(self.ultrasonic_one[0], GPIO.OUT)
        GPIO.setup(self.ultrasonic_one[1], GPIO.IN)
        #ultrasonic 2
        GPIO.setup(self.ultrasonic_two[0], GPIO.OUT)
        GPIO.setup(self.ultrasonic_two[1], GPIO.IN)
        #switch mode
        GPIO.setup(self.switch_mode, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        #switch snapshot
        GPIO.setup(self.switch_snapshot, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(self.switch_snapshot, GPIO.FALLING, callback=self.take_snapshot, bouncetime=200)
        #buzzer
        GPIO.setup(self.buzzer_pin, GPIO.OUT)
        self.buzzer_set = GPIO.PWM(self.buzzer_pin, 100)
        
        logger.info("Setup GPIO complete")
        
    def setup_complete(self):
        self.setup_status = True    
        
    def thread_start(self):
        self.current_mode_thread = Thread(target=self.current_mode)
        self.current_mode_thread.daemon = True
        self.current_mode_thread.start()
        self.distance_one_thread = Thread(target=self.distance_one)
        self.distance_one_thread.daemon = True
        self.distance_one_thread.start()
        self.distance_two_thread = Thread(target=self.distance_two)
        self.distance_two_thread.daemon = True
        self.distance_two_thread.start()
        self.battery_thread = Thread(target=self.read_capacity)
        self.battery_thread.daemon = True
        self.battery_thread.start()
        self.audio_thread = Thread(target=self._play_audio_queue)
        self.audio_thread.daemon = True
        self.audio_thread.start()
    
    def distance_one(self):
        counter = 0 
        counter_time = 0
        while self.ultrasonic_one_break == False:
            GPIO.output(self.ultrasonic_one[0], False)
            time.sleep(0.01)
                
            # ส่งสัญญาณสั่นเพื่อทำการวัดระยะ
            GPIO.output(self.ultrasonic_one[0], True)
            time.sleep(0.00001)
            GPIO.output(self.ultrasonic_one[0], False)

            while GPIO.input(self.ultrasonic_one[1]) == 0:
                start_time = time.time()
                if self.ultrasonic_one_break:
                    break

            while GPIO.input(self.ultrasonic_one[1]) == 1:
                stop_time = time.time()
                if self.ultrasonic_one_break:
                    break
            
            if self.ultrasonic_one_break:
                    break
            # คำนวณเวลาที่ Echo pin สูงขึ้นเพื่อคำนวณระยะทาง
            elapsed_time = stop_time - start_time
            distance_1 = (elapsed_time * 17150)
            distance_1 = round(distance_1, 2)  # สูงคูณด้วยความเร็วของเสียงและหารด้วย 2 เนื่องจากสัญญาณต้องเดินไป-กลับ
            self.ultrasonic_one_status = True
            logger.info(distance_1)
            counter_time += 1
            if self.setup_status :
                if distance_1 <= self.ultrasonic_one[2]:
                    counter += 1
                    if counter > 0 and counter < 2:
                        if counter_time >= 6:
                            counter = 0
                            counter_time = 0
                    if counter >= 2:
                        counter = 0
                        self.buzzer("distance1")
                        self.play_audio("ใกล้วัตถุเกินไป")
                        time.sleep(1)
                    
            
            time.sleep(0.5)

    
    def distance_two(self):
        distancediff = 60
        counter = 0 
        counter_time = 0
        while self.ultrasonic_two_break == False:
            GPIO.output(self.ultrasonic_two[0], False)
            time.sleep(0.01)
                
            # ส่งสัญญาณสั่นเพื่อทำการวัดระยะ
            GPIO.output(self.ultrasonic_two[0], True)
            time.sleep(0.00001)
            GPIO.output(self.ultrasonic_two[0], False)

            while GPIO.input(self.ultrasonic_two[1]) == 0:
                start_time = time.time()
                if self.ultrasonic_two_break:
                    break
                
            while GPIO.input(self.ultrasonic_two[1]) == 1:
                stop_time = time.time()
                if self.ultrasonic_two_break:
                    break
            if self.ultrasonic_two_break:
                    break
            # คำนวณเวลาที่ Echo pin สูงขึ้นเพื่อคำนวณระยะทาง
            elapsed_time = stop_time - start_time
            distance_2 = (elapsed_time * 17150)
            distance_2 = round(distance_2, 2)  # สูงคูณด้วยความเร็วของเสียงและหารด้วย 2 เนื่องจากสัญญาณต้องเดินไป-กลับ
            
            self.ultrasonic_two_status = True
            
            logger.info(distance_2)
            counter_time += 1
            if self.setup_status:
                if distance_2 < 200 and distance_2 > 60:
                    mm = abs(distance_2 - distancediff)
                    logger.info("ค่าต่าง :"+str(mm))
                    logger.info("ค่าเก่า :"+ str(distancediff))
                    if mm > self.ultrasonic_two[2] and mm < self.ultrasonic_two[2]*2:
                        counter += 1
                        distancediff = distance_2
                        if counter > 0 and counter < 2:
                            if counter_time >= 6:
                                counter = 0
                                counter_time = 0
                        if counter >= 2:
                            counter = 0 
                            distancediff = 0
                            logger.info("Different ground level detected")
                            self.buzzer("distance2")
                            self.play_audio("ระวังพื้นต่างระดับ")
                            time.sleep(1.5)
                    
            time.sleep(0.1)
      
    def check_ultrasonic(self):
        return self.ultrasonic_one_status, self.ultrasonic_two_status
    
    def ultrasonic_break_one(self):
        self.ultrasonic_one_break = True
    def ultrasonic_break_two(self):
        self.ultrasonic_two_break = True
        
    def buzzer(self, event):
        if event == "start": #start raspi
            self.buzzer_set.start(20)
            time.sleep(0.05)
            self.buzzer_set.stop()
            logger.info("buzzer for START")
        elif event == "shutdown": #shutdown raspi
            self.buzzer_set.start(20)
            time.sleep(0.05)
            self.buzzer_set.stop()
            time.sleep(0.1) 
            self.buzzer_set.start(20)
            time.sleep(0.05)
            self.buzzer_set.stop()
            time.sleep(0.1) 
            self.buzzer_set.start(20)
            time.sleep(0.05)
            self.buzzer_set.stop()
            logger.info("buzzer for SHUTDOWN")
        elif event == "distance1": #warning distance 1 when object too close
            self.buzzer_set.start(20)
            time.sleep(0.05)
            self.buzzer_set.stop()
            time.sleep(0.5)
            self.buzzer_set.start(20)
            time.sleep(0.05)
            self.buzzer_set.stop()
            time.sleep(0.5)
            logger.info("buzzer for ULTRA1")
        elif event == "distance2": #warning distance 2 when detect different ground level
            self.buzzer_set.start(20)
            time.sleep(0.5)
            self.buzzer_set.stop()
            time.sleep(0.1)
            self.buzzer_set.start(20)
            time.sleep(0.5)
            self.buzzer_set.stop()
            time.sleep(0.1)
            self.buzzer_set.start(20)
            time.sleep(0.5)
            self.buzzer_set.stop()
            logger.info("buzzer for ULTRA2")
        elif event == "changemode": #alert when battery low
            self. buzzer_set.start(20)
            time.sleep(0.05)
            self.buzzer_set.stop()
            logger.info("buzzer for changemode")
        elif event == "snapshot": #alert when press snapshot button
            if self.takesnap:
                self.buzzer_set.start(20)
                time.sleep(0.05)
                self.buzzer_set.stop()
                time.sleep(2)
                self.takesnap = False
            logger.info("buzzer for snapshot")
        elif event == "batterylow": #alert when shutdown
            self.buzzer_set.start(20)
            time.sleep(0.05)
            self.buzzer_set.stop()
            time.sleep(0.1) 
            self.buzzer_set.start(20)
            time.sleep(0.05)
            self.buzzer_set.stop()
            logger.info("buzzer for batterylow")
        
    def current_mode(self):
        while True:
            if GPIO.input(self.switch_mode) == 1:
                self.mode = "realtime"
                logger.info("Realtime mode")
                time.sleep(1)
            else:
                self.mode = "snapshot"
                logger.info("Snapshot mode")
                time.sleep(1)
                
    def get_current_mode(self):
        return self.mode
    
    def take_snapshot(self, channel):
        if self.mode == "snapshot":
            self.takesnap = True
            self.buzzer("snapshot")
            logger.info("Take snapshot")
    
    def get_snapshot(self):
        return self.takesnap
    
        
    def _play_audio_queue(self):
        while True:
            file_name = self.audio_queue.get()
            file_path = self.module_config["audio"]["audio_path"] + file_name + ".wav"
            subprocess.call(["aplay", file_path])
            self.audio_queue.task_done()
    
    def play_audio(self, file_name):
        # set audio file path
        self.audio_queue.put(file_name)
        self.audio_queue.join()
     
    def read_capacity(self):
        while True:
            if self.setup_status:
                # Read the capacity from the UPS
                read = self.bus.read_word_data(self.address, 4)
                swapped = struct.unpack("<H", struct.pack(">H", read))[0]
                self.capacity = swapped / 256

                if self.capacity >= 100 and not hasattr(self, "alerted_100"):
                    self.capacity = 100
                    setattr(self, "alerted_100", True)
                    self.play_audio("แบตเตอรี่เต็ม 100%")
                
                if 80 < self.capacity < 100 and not hasattr(self, "alerted_more_80"):
                    logger.info("Alert: Capacity more than 80%!")
                    setattr(self, "alerted_more_80", True)  # Set a flag to prevent further alerts
                    self.play_audio("แบตเตอรี่มากกว่า 80%")
                
                # Alert for capacity below 20%, only once
                if 80 < self.capacity < 80 and not hasattr(self, "alerted_80"):
                    logger.info("Alert: Capacity below 80%!")
                    setattr(self, "alerted_80", True)  # Set a flag to prevent further alerts
                    self.play_audio("แบตเตอรี่ต่ำกว่า 80%")
                    
                    # Alert for capacity below 20%, only once
                if 40 < self.capacity < 60 and not hasattr(self, "alerted_60"):
                    logger.info("Alert: Capacity below 60%!")
                    setattr(self, "alerted_60", True)  # Set a flag to prevent further alerts
                    self.play_audio("แบตเตอรี่ต่ำกว่า 60%")
                    
                    # Alert for capacity below 20%, only once
                if 20 < self.capacity < 40 and not hasattr(self, "alerted_40"):
                    logger.info("Alert: Capacity below 40%!")
                    setattr(self, "alerted_40", True)  # Set a flag to prevent further alerts
                    self.play_audio("แบตเตอรี่ต่ำกว่า 40%")
                    
                # Alert for capacity below 20%, only once
                if 10 < self.capacity < 20 and not hasattr(self, "alerted_20"):
                    logger.info("Alert: Capacity below 20%!")
                    setattr(self, "alerted_20", True)  # Set a flag to prevent further alerts
                    self.play_audio("แบตเตอรี่ต่ำกว่า 20%")

                # Alert for capacity below 10%, only once
                if 5 < self.capacity < 10 and not hasattr(self, "alerted_10"):
                    logger.info("Alert: Capacity below 10%!")
                    setattr(self, "alerted_10", True)
                    self.buzzer("batterylow")
                    self.play_audio("แบตเตอรี่ต่ำกว่า 10%")

                # Alert for capacity below 5%, only once
                if 1 < self.capacity < 5 and not hasattr(self, "alerted_5"):
                    logger.info("Alert: Capacity critically low (below 5%)!")
                    setattr(self, "alerted_5", True)
                    self.buzzer("batterylow")
                    self.play_audio("แบตเตอรี่ต่ำกว่า 5%")

                if self.capacity <= 1 and hasattr(self, "alerted_1"):
                    logger.info("Alert: Capacity critically low (0%)!")
                    setattr(self, "alerted_1", True)
                    self.buzzer("batterylow")
                    self.play_audio("แบตเตอรี่ 1% ระบบกำลำจะปิดตัว")
                
                if self.capacity >= 100:
                    self.capacity = 100
                logger.info("Capacity:%5i%%" % self.capacity)
                time.sleep(self.module_config["battery"]["battery_check_time"])
        
    def get_capacity(self):
        return self.capacity
    

    