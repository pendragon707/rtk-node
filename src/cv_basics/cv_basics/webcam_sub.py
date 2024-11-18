# Import the necessary libraries
import rclpy # Python library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import time

from req_res_str_service.srv import ReqRes

from cv_basics.detector import Detector
from cv_basics.config import RunnerConfig
from cv_basics.signs import Signs
 
class ImageSubscriber(Node):
  """
  Create an ImageSubscriber class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('image_subscriber')
      
    # Create the subscriber. This subscriber will receive an Image
    # from the video_frames topic. The queue size is 10 messages.
    self.subscription = self.create_subscription(
      Image, 
      'image_raw/uncomp', 
      self.listener_callback, 
      10)
    self.subscription # prevent unused variable warning
    self.srv = self.create_service(ReqRes, 'cam_service', self.cam_serv_callback)
      
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    self.detector = Detector()    

    self.last_image = None # 1 last image        
    self.last_detected_sign = "forward"
    self.detected_traffic_light = False

  def cam_serv_callback(self, request, response):

      self.get_logger().info(f"{request.req}")

      # sonar_data = request.req.rstrip() # [dist1, dist2, dist3]

      detected_objects = self.detector.get_objects(self.last_image)
    #   self.get_logger().info(detected_objects)

      if len(detected_objects.keys()) == 0:
        #   self.get_logger().info(f"No object detected")          
          response.res = "NO"
          return response

      if "forward" in detected_objects.keys():
          response.res = "forward"
          #self.get_logger().info(f"Обнаружен знак Движение Прямо на расстоянии = {detected_objects['forward']}")          

      if "left" in detected_objects.keys():    
          response.res = "left"            
        #   self.get_logger().info(f"Обнаружен знак Поворот Налево на расстоянии = {detected_objects['left']}")          
     
      if "right" in detected_objects.keys():
          response.res = "right"
          #self.get_logger().info(f"Обнаружен знак Поворот Направо на расстоянии = {detected_objects['right']}")                  

      if "aruco" in detected_objects.keys():         
         # self.get_logger().info(f"Обнаружен Aruco-маркер на расстоянии = {detected_objects['aruco']}")             
          # if detected_objects['aruco'] < 11:
          response.res = "aruco"
          return response
      
      # if len(detected_objects.keys()) > 1:          
      #     min_dist_key = list(detected_objects.keys())[0]
      #     min_dist = list(detected_objects.values())[0][0]
      #     for key, val in detected_objects.items():                
      #         if sorted(val)[0] < min_dist:
      #             min_dist = sorted(val)[0]
      #             min_dist_key = key
          
      #     response.res = min_dist_key
      #     self.last_detected_sign = min_dist_key
      
      # if len(detected_objects.keys()) == 1:          
      #     response.res = list(detected_objects.keys())[0]
      #     self.last_detected_sign = list(detected_objects.keys())[0]



    #   if "traffic_light" in detected_objects.keys():          
    #       self.get_logger().info(f"Обнаружен красный сигнал светофора на расстоянии = {detected_objects['traffic_light']}") 
    #       if detected_objects['traffic_light'] < 11:
    #           response.res = "traffic_light"                

    #           self.detected_traffic_light = True
    #           return response    
    #   elif self.detected_traffic_light:
    #       self.detected_traffic_light = False
    #       response.res = self.last_detected_sign
              
      return response
   
  def listener_callback(self, data):
    """
    Callback function.
    """
    # Display the message on the console    
 
    # Convert ROS Image message to OpenCV image
    current_frame = self.br.imgmsg_to_cv2(data)    
       
    self.last_image = current_frame

    # self.last_images.append( current_frame )    
    # if len(self.last_images) > 20:
    #   self.last_images.pop(0)

    # Display image
    cv2.imshow("camera", cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB))    
    
    cv2.waitKey(1)
  
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  image_subscriber = ImageSubscriber()
  
  # Spin the node so the callback function is called.
  rclpy.spin(image_subscriber)
  
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  image_subscriber.destroy_node()
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()
