#!/usr/bin/python3

# Python
import copy

# Object detection
import cv2
import numpy as np
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator
from ultralytics.engine.results import Results

# ROS
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from turtlebot3_object_tracker.srv import Detection, DetectionResponse




class ImageProcessor:
    def __init__(self) -> None:
        # Image message
        self.image_msg = Image()

        self.image_res = 240, 320, 3 # Camera resolution: height, width
        self.image_np = np.zeros(self.image_res) # The numpy array to pour the image data into

        # TODO: Subscribe on your robot's camera topic
        # NOTE: Make sure you use the provided listener for this subscription
        self.camera_subscriber = rospy.Subscriber('/follower/camera/image', Image, self.camera_listener)

        # TODO: Instantiate your YOLO object detector/classifi'/follower/camera/image', Image, self.camera_listener)

        # TODO: Instantiate your YOLO object deer model
        self.model = YOLO()
        # TODO: You need to update results each time you call your model
        self.results: Results = None

        self.cv2_frame_size = 400, 320
        cv2.namedWindow("robot_view", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("robot_view", *self.cv2_frame_size)

        # TODO: Setup your "human detection" service
        self.human_detection_server = rospy.Service('detection_service', Detection, self.detection_service_callback)
        self.detection_info = []
        self.update_view()


    def camera_listener(self, msg: Image):
        self.image_msg.data = copy.deepcopy(msg.data)

    def detection_service_callback(self, req):
        label = req.label

        # Perform object detection on the current image frame
        self.results = self.model(self.image_np)
        
        # Extract the detection information and image size
        self.detection_info = []
        
        for result in self.results:

            detection_count = result.boxes.shape[0]

            for i in range(detection_count):
                cls = int(result.boxes.cls[i].item())
                name = result.names[cls]
                
                if name == label:
                    bounding_box = result.boxes.xyxy[i].cpu().numpy()
                    p1 = Point()
                    p2 = Point()
                    p1.x = int(bounding_box[0])
                    p1.y = int(bounding_box[1])
                    p2.x = int(bounding_box[2])
                    p2.y = int(bounding_box[3])

                    self.detection_info.append(p1)
                    self.detection_info.append(p2)


        image_size = [self.image_res[0], self.image_res[1]]

        response = DetectionResponse()
        response.detection_info = self.detection_info
        response.image_size = image_size

        return response

    def update_view(self):
        try:
            while not rospy.is_shutdown():
                if len(self.image_msg.data) == 0: # If there is no image data
                    continue

                # Convert binary image data to numpy array
                self.image_np = np.frombuffer(self.image_msg.data, dtype=np.uint8)
                self.image_np = self.image_np.reshape(self.image_res)

                frame = copy.deepcopy(self.image_np)

                 # TODO: You can use an "Annotator" to draw object bounding boxes on frame
                # annotator = Annotator(frame)
                rospy.loginfo(self.detection_info)
                image = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                if self.detection_info :
                    
                    x1,y1 = self.detection_info[0].x ,self.detection_info[0].y 
                    x2,y2 = self.detection_info[1].x ,self.detection_info[1].y
                    # print(((x1, y1), (x2, y2)))

                    box_color = (255, 0, 0)
                    line_thickness = 2
                    
                    cv2.rectangle(image, (x1, y1), (x2, y2), box_color, line_thickness)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    font_scale = 0.5
                    label_color = (255, 255, 255)  # White color
                    cv2.putText(image, "person", (x1, y1 - 10), font, font_scale, label_color, line_thickness)

            

                cv2.imshow("robot_view",image)
                
                cv2.waitKey(1)


        except rospy.exceptions.ROSInterruptException:
            pass


if __name__ == "__main__":
    rospy.init_node("image_processor", anonymous=True)

    rospy.on_shutdown(cv2.destroyAllWindows)

    image_processor = ImageProcessor()

    rospy.spin()


