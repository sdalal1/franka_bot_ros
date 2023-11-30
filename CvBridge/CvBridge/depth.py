import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from sensor_msgs.msg import Image,CameraInfo
import numpy as np
import pyrealsense2 as rs2
import cv2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2
import tf2_ros
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ImageListener(Node):
    def __init__(self):
        
        super().__init__("imagelistener")
        self.camera_rgb_subscriber = self.create_subscription(Image,'/camera/color/image_raw',self.camera_rgb_callback,10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1/100, self.timer_callback)
        self.last_image = None
        
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.camera_info = self.create_subscription(CameraInfo,'/camera/color/camera_info',self.camera_info_cb,10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.cx_purple=None
        self.cy_purple=None
        self.cx_yellow=None
        self.cy_yellow=None
        self.cx_green=None
        self.cy_green=None
        
        self.intrinsics = None
        cv2.namedWindow('trackbar',cv2.WINDOW_NORMAL)
        # cv2.createTrackbar('LowH_purple','trackbar',0,179,self.nothing)
        # cv2.createTrackbar('HighH_purple','trackbar',0,179,self.nothing)
        # cv2.createTrackbar('LowS_purple','trackbar',0,255,self.nothing)
        # cv2.createTrackbar('HighS_purple','trackbar',0,255,self.nothing)
        # cv2.createTrackbar('LowV_purple','trackbar',0,255,self.nothing)
        # cv2.createTrackbar('HighV_purple','trackbar',0,255,self.nothing)

        cv2.createTrackbar('LowH_yellow','trackbar',0,179,self.nothing)
        cv2.createTrackbar('HighH_yellow','trackbar',0,179,self.nothing)
        cv2.createTrackbar('LowS_yellow','trackbar',0,255,self.nothing)
        cv2.createTrackbar('HighS_yellow','trackbar',0,255,self.nothing)
        cv2.createTrackbar('LowV_yellow','trackbar',0,255,self.nothing)
        cv2.createTrackbar('HighV_yellow','trackbar',0,255,self.nothing)

        cv2.createTrackbar('LowH_green','trackbar',0,179,self.nothing)
        cv2.createTrackbar('HighH_green','trackbar',0,179,self.nothing)
        cv2.createTrackbar('LowS_green','trackbar',0,255,self.nothing)
        cv2.createTrackbar('HighS_green','trackbar',0,255,self.nothing)
        cv2.createTrackbar('LowV_green','trackbar',0,255,self.nothing)
        cv2.createTrackbar('HighV_green','trackbar',0,255,self.nothing)
        
    def camera_info_cb(self,msg):
        value = msg.k
        self.intrinsics = rs2.intrinsics()
        self.intrinsics.coeffs = msg.d
        self.intrinsics.fx = value[0]
        self.intrinsics.fy = value[4]
        self.intrinsics.height = msg.height
        self.intrinsics.ppx = value[2]
        self.intrinsics.ppy = value[5]
        self.intrinsics.width=msg.width
        
             
    def camera_rgb_callback(self,msg): 
        """Callback function for the camera image subscriber."""
        self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') 
        
    def click_event(self,event, x, y, flags, params): 
        # checking for left mouse clicks 
        if event == cv2.EVENT_LBUTTONDOWN: 
    
            # displaying the coordinates 
            # on the Shell 
            print(x, ' ', y)
            self.get_logger().info(f"finish? {x, y}")
    
    def nothing(self,x):
        pass
    
    def taglistener(self):
        try:
            # get the latest transform between left and right
            # (rclpy.time.Time() means get the latest information)
                trans = self.buffer.lookup_transform("camera_color_optical_frame", "paint", rclpy.time.Time())
                paint_tag_x = trans.transform.translation.x
                paint_tag_y = trans.transform.translation.y
                paint_tag_z = trans.transform.translation.z
                tag_log = [paint_tag_x,paint_tag_y,paint_tag_z]
                # self.get_logger().info(f"Tag Location: {paint_tag_x,paint_tag_y,paint_tag_z}")
                return(tag_log)
        except tf2_ros.LookupException as e:
                # the frames don't exist yet
                self.get_logger().info(f"Lookup exception: {e}")
        
    def masking(self,image,tagpos):
        mask = np.zeros(image.shape[:2], dtype="uint8")
        tagposx = tagpos[0]
        tagposy = tagpos[1]
        tagposz = tagpos[2]
        # self.get_logger().info(f"Tag Location: {rs2.intrinsics,type(rs2.intrinsics)}")
        if self.intrinsics is not None:
            yoffset=0.25
            yoffset1 = 0.18
            xoffset1 = 0.08
            xoffset2 = 0.36
            rect1 = rs2.rs2_project_point_to_pixel(self.intrinsics,[tagposx-xoffset1,tagposy+yoffset,tagposz])
            rect2 = rs2.rs2_project_point_to_pixel(self.intrinsics,[tagposx-xoffset1,tagposy-yoffset1,tagposz])
            rect3 = rs2.rs2_project_point_to_pixel(self.intrinsics,[tagposx-xoffset2,tagposy-yoffset1,tagposz])
            rect4 = rs2.rs2_project_point_to_pixel(self.intrinsics,[tagposx-xoffset2,tagposy+yoffset,tagposz])
            # rect = np.array([[1002,257],[1028,526],[1231,509],[1207,207]])
            rect = np.array([rect1,rect2,rect3,rect4], np.int32)
            cv2.fillConvexPoly(mask,rect, 1)
            masked = cv2.bitwise_and(image, image, mask=mask)
            return masked
    
    def colordetenction(self,masked):
        hsv=cv2.cvtColor(masked, cv2.COLOR_BGR2HSV)
        # lower_purple = np.array([cv2.getTrackbarPos('LowH_purple','trackbar'),cv2.getTrackbarPos('LowS_purple','trackbar'),cv2.getTrackbarPos('LowV_purple','trackbar')])
        # upper_purple = np.array([cv2.getTrackbarPos('HighH_purple','trackbar'),cv2.getTrackbarPos('HighS_purple','trackbar'),cv2.getTrackbarPos('HighV_purple','trackbar')])

        lower_purple = np.array([114, 20, 86], np.uint8) 
        upper_purple = np.array([149, 149, 165], np.uint8) 
        purple_mask= cv2.inRange(hsv, lower_purple, upper_purple)
        
        # lower_yellow = np.array([cv2.getTrackbarPos('LowH_yellow','trackbar'),cv2.getTrackbarPos('LowS_yellow','trackbar'),cv2.getTrackbarPos('LowV_yellow','trackbar')])
        # upper_yellow = np.array([cv2.getTrackbarPos('HighH_yellow','trackbar'),cv2.getTrackbarPos('HighS_yellow','trackbar'),cv2.getTrackbarPos('HighV_yellow','trackbar')])

        lower_yellow = np.array([6, 118, 195], np.uint8) 
        upper_yellow = np.array([42, 170, 224], np.uint8) 

        # lower_yellow = np.array([136, 87, 111], np.uint8)
        # upper_yellow = np.array([180, 255, 255], np.uint8) 
        yellow_mask= cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        lower_green = np.array([cv2.getTrackbarPos('LowH_green','trackbar'),cv2.getTrackbarPos('LowS_green','trackbar'),cv2.getTrackbarPos('LowV_green','trackbar')])
        upper_green = np.array([cv2.getTrackbarPos('HighH_green','trackbar'),cv2.getTrackbarPos('HighS_green','trackbar'),cv2.getTrackbarPos('HighV_green','trackbar')])

        # lower_green = np.array([25, 52, 72], np.uint8) 
        # upper_green = np.array([102, 255, 255], np.uint8) 
        green_mask= cv2.inRange(hsv, lower_green, upper_green)
        
        kernel_size=(5,5)
        kernelo=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,kernel_size)
        kernel_sizec=(20,20)
        kernelc=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,kernel_sizec)
        img_open=cv2.morphologyEx(purple_mask, cv2.MORPH_OPEN, kernelo)
        img_open1=cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernelo)
        img_open2=cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernelo)
        img_close_purple=cv2.morphologyEx(img_open, cv2.MORPH_CLOSE, kernelc)
        img_close_yellow=cv2.morphologyEx(img_open1, cv2.MORPH_CLOSE, kernelc)
        img_close_green=cv2.morphologyEx(img_open2, cv2.MORPH_CLOSE, kernelc)

        
        res_purple= cv2.bitwise_and(masked, masked, mask=img_close_purple)
        res_yellow= cv2.bitwise_and(masked, masked, mask=img_close_yellow)
        res_green= cv2.bitwise_and(masked, masked, mask=img_close_green)
        contours, hierarchy=cv2.findContours(img_close_purple, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #print(len(contours))

        biggest_contour=0
        if len(contours)>0:
            biggest_contour=contours[0]
            for a in range(0,len(contours)):
                if cv2.contourArea(contours[a])>cv2.contourArea(biggest_contour):
                    biggest_contour=contours[a]
        
            M=cv2.moments(biggest_contour)
            self.cx_purple= int(M['m10']/M['m00'])
            self.cy_purple= int(M['m01']/M['m00'])

            cv2.circle(masked, (self.cx_purple, self.cy_purple), 3, (255,255,255), -1)
            cv2.putText(masked, 'purple_centroid', (self.cx_purple-10,self.cy_purple-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2, cv2.LINE_AA)
        
        contours, hierarchy=cv2.findContours(img_close_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #print(len(contours))
        
        biggest_contour=0
        if len(contours)>0:
            biggest_contour=contours[0]
            for a in range(0,len(contours)):
                if cv2.contourArea(contours[a])>cv2.contourArea(biggest_contour):
                    biggest_contour=contours[a]
        
            M=cv2.moments(biggest_contour)
            self.cx_yellow= int(M['m10']/M['m00'])
            self.cy_yellow= int(M['m01']/M['m00'])

            cv2.circle(masked, (self.cx_yellow, self.cy_yellow), 3, (255,255,255), -1)
            cv2.putText(masked, 'yellow_centroid', (self.cx_yellow-10,self.cy_yellow-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2, cv2.LINE_AA)
            
        contours, hierarchy=cv2.findContours(img_close_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #print(len(contours))
        
        biggest_contour=0
        if len(contours)>0:
            biggest_contour=contours[0]
            for a in range(0,len(contours)):
                if cv2.contourArea(contours[a])>cv2.contourArea(biggest_contour):
                    biggest_contour=contours[a]
        
            M=cv2.moments(biggest_contour)
            self.cx_green= int(M['m10']/M['m00'])
            self.cy_green= int(M['m01']/M['m00'])
            
            cv2.circle(masked, (self.cx_green, self.cy_green), 3, (255,255,255), -1)
            cv2.putText(masked, 'green_centroid', (self.cx_green-10,self.cy_green-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2, cv2.LINE_AA)
            
        conto=cv2.drawContours(masked, contours, -1, (0,255,0), 3)
        
        
        # cv2.imshow('Thresh',conto)
        # cv2.setMouseCallback('Thresh', self.click_event) 
        
        # cv2.namedWindow('purple', cv2.WINDOW_NORMAL)
        # cv2.imshow('purple',img_close_purple)

        # cv2.namedWindow('yellow', cv2.WINDOW_NORMAL)
        # cv2.imshow('yellow',img_close_yellow)

        # cv2.namedWindow('green', cv2.WINDOW_NORMAL)
        # cv2.imshow('green',img_close_green)
 
        # cv2.waitKey(1)
        # self.get_logger().info(f"finish? {cx_purple, cx_green, cx_yellow}")
        
    def broadcaster(self,tagpos):
        depth = tagpos[2]
        if self.cx_yellow is not None and self.cy_yellow is not None:
            yellow_pos = rs2.rs2_deproject_pixel_to_point(self.intrinsics,[self.cx_yellow,self.cy_yellow],depth)
            r = TransformStamped()

            # Read message content and assign it to
            # corresponding tf variables
            r.header.stamp = self.get_clock().now().to_msg()
            r.header.frame_id = 'camera_color_optical_frame'
            r.child_frame_id = 'yellow_color'

            # Turtle only exists in 2D, thus we get x and y translation
            # coordinates from the message and set the z coordinate to 0
            r.transform.translation.x = yellow_pos[0]
            r.transform.translation.y = yellow_pos[1]
            r.transform.translation.z = yellow_pos[2]
            
            self.tf_broadcaster.sendTransform(r)
            
        if self.cx_purple is not None and self.cy_purple is not None:
            purple_pos = rs2.rs2_deproject_pixel_to_point(self.intrinsics,[self.cx_purple,self.cy_purple],depth)
            b = TransformStamped()

            # Read message content and assign it to
            # corresponding tf variables
            b.header.stamp = self.get_clock().now().to_msg()
            b.header.frame_id = 'camera_color_optical_frame'
            b.child_frame_id = 'purple_color'

            # Turtle only exists in 2D, thus we get x and y translation
            # coordinates from the message and set the z coordinate to 0
            b.transform.translation.x = purple_pos[0]
            b.transform.translation.y = purple_pos[1]
            b.transform.translation.z = purple_pos[2]
            
            self.tf_broadcaster.sendTransform(b)
            
        if self.cx_green is not None and self.cy_green is not None:
            green_pos = rs2.rs2_deproject_pixel_to_point(self.intrinsics,[self.cx_green,self.cy_green],depth)
            g = TransformStamped()

            # Read message content and assign it to
            # corresponding tf variables
            g.header.stamp = self.get_clock().now().to_msg()
            g.header.frame_id = 'camera_color_optical_frame'
            g.child_frame_id = 'green_color'

            # Turtle only exists in 2D, thus we get x and y translation
            # coordinates from the message and set the z coordinate to 0
            g.transform.translation.x = green_pos[0]
            g.transform.translation.y = green_pos[1]
            g.transform.translation.z = green_pos[2]
            
            self.tf_broadcaster.sendTransform(g)
        
    def timer_callback(self):
        try:
            tagloc  = self.taglistener()
        except tf2_ros.LookupException as e:
                # the frames don't exist yet
                self.get_logger().info(f"Lookup exception: {e}")
        
        
        if self.last_image is not None and tagloc is not None:
            self.get_logger().info("Video Received" ,once=True)
            masked = self.masking(self.last_image,tagloc)
            self.colordetenction(masked)
            self.broadcaster(tagloc)
        else:
            self.get_logger().info("Waiting")

def main(args=None):
    rclpy.init(args=args)
    node = ImageListener()
    rclpy.spin(node)
    rclpy.shutdown()
