#Import the rclpy library
import rclpy
from rclpy.node import Node
#Import String messages
from std_msgs.msg import String 
#Create a Topic_Pub node subclass that inherits from the Node base
class Topic_Pub(Node):
    def __init__(self,name):
        super().__init__(name)
        #Create a publisher using the create_publisher function. The passed parameters are:
        #Topic data type, topic name, and queue length for storing messages
        self.pub = self.create_publisher(String,"/topic_demo",1) 
        #Create a timer that enters the interrupt handler every 1 second. The passed parameters are:
        #Interval between interrupt executions, interrupt handler function
        self.timer = self.create_timer(1,self.pub_msg)
    #Define the interrupt handler function
    def pub_msg(self):
        msg = String()  #Create a String variable, msg
        msg.data = "Hi,I send a message." #Assign data to msg
        self.pub.publish(msg) #Publish topic data
        
#Main function
def main():
    rclpy.init() #Initialization
    pub_demo = Topic_Pub("publisher_node") #Create a Topic_Pub class object, passing in the node name as a parameter
    rclpy.spin(pub_demo)     #Execute the rclpy.spin function, passing in the Topic_Pub class object just created as a parameter
    pub_demo.destroy_node()  #Destroy the node object
    rclpy.shutdown()         #Shut down the ROS2 Python interface