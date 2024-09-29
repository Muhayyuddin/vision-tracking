import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import numpy as np
import importlib
import os
from pytracking.evaluation import Tracker
from SeqTrack.tracking.lib.test.evaluation import TrackerS
from pytracking.utils.plotting import draw_figure, overlay_mask
from collections import OrderedDict
from lib.test.tracker import * 


class ObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('object_tracking_node')
        self.subscription = self.create_subscription(
            Image,
            '/usv/slot0/image_raw',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.bridge = CvBridge()
        self.prev_output = OrderedDict()
        self.tracker_name = 'tomp'


        if self.tracker_name == 'seqtrack':
            self.tracker = self.initialize_tracker('seqtrack', 'seqtrack_b256_got')
        elif self.tracker_name =='tamos':
            self.tracker = self.initialize_tracker('tamos', 'tamos_resnet50')
        else:
            self.tracker = self.initialize_tracker('tomp', 'tomp50')     

        self.initialized = False  # To check if the tracker has been initialized
        self.center_x = None
        self.center_y = None
        self.image_width = None
        self.image_height = None
        # Define the initial bounding box (x, y, width, height)
        self.initial_bbox = [0, 345, 220, 100]
        #self.initial_bbox = [325, 371, 10, 10]
        
        # Tracker display colors (if needed for debugging/display)
        self._tracker_disp_colors = {1: (0, 255, 0), 2: (0, 0, 255), 3: (255, 0, 0),
                                     4: (255, 255, 255), 5: (0, 0, 0), 6: (0, 255, 128),
                                     7: (123, 123, 123), 8: (255, 128, 0), 9: (128, 0, 255)}

    def initialize_tracker(self, tracker_name, tracker_param):
        #print("Initializing Tracker 111111 ######### \n\n")

        if self.tracker_name == 'seqtrack':
            tracker = TrackerS(tracker_name, tracker_param,'got10k_test')
        else:
            tracker = Tracker(tracker_name, tracker_param)


        params = tracker.get_parameters()
        print(dir(params))

        tracker_instance = tracker.create_tracker(params)
        return tracker_instance
    
    def get_parameters(self):
        pass
        """Get parameters."""
        #print(" Tracker ######### \n\n")
        #param_module = importlib.import_module('pytracking_image.SeqTrack.lib.config.{}.{}'.format('seqtrack', 'config'))
        #param_module = importlib.import_module('pytracking.parameter.{}.{}'.format('tomp', 'tomp50'))
        #print(" Tracker ######### \n\n")
        #params = param_module.parameters()
        #return params

    def image_callback(self, msg):
        #print("WORKINGGGGGG")
        try:
            #print("CV_Bridge 55555 ######### \n\n")
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return
        #print("CV_Bridge 666666 ######### \n\n")
        # Initialize image dimensions
        if self.image_width is None or self.image_height is None:
            self.image_height, self.image_width, _ = cv_image.shape
            self.center_x = self.image_width / 2
            self.center_y = self.image_height / 2

        # Initialize tracker on the first frame with the initial bounding box
        if not self.initialized:
            self.prev_output = self.tracker.initialize(cv_image, {
                'init_bbox': self.initial_bbox,
                'init_object_ids': [1],
                'object_ids': [1],
                'sequence_object_ids': [1]
            })

            self.initialized = True

        # Run the tracker on the current frame
        if self.prev_output:
            info = OrderedDict()
            info['previous_output'] = self.prev_output
            info['sequence_object_ids'] = [1]
            out = self.tracker.track(cv_image, info)
            self.prev_output = OrderedDict(out)
            #print("OUTPUT", out)
#            if 'target_bbox' in out:
#                for obj_id, state in out['target_bbox'].items():
#                    self.publish_twist_message(state)


            if 'target_bbox' in out:
                bbox = out['target_bbox']
                print(bbox,"BOUNDINGBOX")
                #self.publish_twist_message(bbox)
                if self.tracker_name == 'tamos':
                    bbox_list = list(bbox.values())[0]
                    self.publish_twist_message(bbox_list)
                    x, y, w, h = bbox_list
                else:
                    self.publish_twist_message(bbox)
                    x, y, w, h = bbox
                # Draw the bounding box on the image
                #x, y, w, h = bbox
                
                cv.rectangle(cv_image, (int(x), int(y)), (int(x + w), int(y + h)), (0, 255, 0), 2)
                cv.putText(cv_image, f"Tracker: {self.tracker_name}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                # Display the image
                cv.imshow("Tracking", cv_image)
                cv.waitKey(1)


    def publish_twist_message(self, bbox):
        x, y, w, h = bbox
        bbox_center_x = x + w / 2
        bbox_center_y = y + h / 2

        error_x = bbox_center_x - self.center_x
        error_y = bbox_center_y - self.center_y

        # Assuming a simple P-controller for heading correction
        k_p = 0.01
        twist_msg = Twist()
        twist_msg.linear.x = 1.5  # constant forward velocity of 1.5 m/s
        twist_msg.angular.z = -k_p * error_x  # adjust heading based on x error
        print("twist msg value is ", twist_msg)
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTrackingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
