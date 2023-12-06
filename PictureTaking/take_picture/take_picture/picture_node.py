import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from example_interfaces.srv import Trigger
from cv_bridge import CvBridge
from .photo_points import * 
import matplotlib.pyplot as plt
from take_picture_interfaces.msg import Waypoint, WaypointList

class picture_node(Node):
    """A class representing a picture node.

    This node is responsible for capturing images from a camera and saving them.
    It also creates a waypoint list from the captured images and publishes it to a topic.

    Attributes:
        waypoints_list (WaypointList): The list of waypoints created from the captured images.
        camera_rgb_subscriber (Subscriber): The subscriber for the camera RGB image topic.
        waypoint_publisher (Publisher): The publisher for the waypoint list topic.
        save_service (Service): The service for saving the captured image.
        bridge (CvBridge): The bridge for converting ROS image messages to OpenCV images.
        timer (Timer): The timer for publishing the waypoint list.
        waypoint_list_subscriber (Subscriber): The subscriber for the waypoint list topic.
    """

    def __init__(self):
        super().__init__("picture_node")
        self.waypoints_list = WaypointList()

        self.camera_rgb_subscriber = self.create_subscription(Image, '/camera/color/image_raw', 
                                                              self.camera_rgb_callback, 10)
        self.waypoint_publisher = self.create_publisher(WaypointList, 'outline_waypoints', 10)
        
        self.save_service = self.create_service(Trigger, 'save_image', self.save_image_callback)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1/100, self.timer_callback)

        self.waypoint_list_subscriber = self.create_subscription(WaypointList, 'outline_waypoints',
                                                                 self.waypoint_list_callback, 10)


    def camera_rgb_callback(self, msg): 
        """Callback function for the camera image subscriber.

        Args:
            msg (Image): The received camera image message.
        """
        self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') 


    def save_image_callback(self, request, response):
        """Callback function for the save image service. Saves the last image taken by the camera.

        Args:
            request: The request object for the save image service.
            response: The response object for the save image service.

        Returns:
            The response object indicating the success or failure of saving the image.
        """
        if self.last_image is not None: 
            response.success = True
            response.message = "image saved successfully."
            self.create_waypoint_list_msg() 
        else:
            response.success = False
            response.message = "no image to save."
        return response
    
    def create_waypoint_list_msg(self): 
        """
        Create a WaypointList message based on the mapped points from the last image.

        """
        mapped_points = get_mapped_points(image_data=self.last_image)
        self.waypoints_list = WaypointList() #clear the list so can update if srv called again
        for point in mapped_points:
            waypoint = Waypoint()
            waypoint.x = point[0]
            waypoint.y = point[1]
            self.waypoints_list.waypoints.append(waypoint)

    def timer_callback(self):
        """
        Publishes the waypoints list to the outline_waypoints topic.

        :return: None
        """
        if self.waypoints_list.waypoints:
            self.waypoint_publisher.publish(self.waypoints_list)


def main(args=None):
    rclpy.init(args=args)
    node = picture_node()
    rclpy.spin(node)
    rclpy.shutdown()
