#Import related libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Topic_Sub(Node):
    def __init__(self,name):
        super().__init__(name)  
        #Create a subscriber using create_subscription. The passed parameters are: topic data type, topic name, callback function name, and queue length.
        self.sub = self.create_subscription(String,"/topic_demo",self.sub_callback,1) 
    #Callback function executes the program: prints the received message.
    def sub_callback(self,msg):
        # print(msg.data,flush=True)
        self.get_logger().info(msg.data)

def main():
    rclpy.init() #Initialize the ROS2 Python interface.
    sub_demo = Topic_Sub("subscriber_node")#Create the object and initialize it.
    rclpy.spin(sub_demo)
    sub_demo.destroy_node()  #Destroy the node object
    rclpy.shutdown()         #Shut down the ROS2 Python interface