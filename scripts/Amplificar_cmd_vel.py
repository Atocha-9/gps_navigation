#!/usr/bin/env python  
import rospy  
from geometry_msgs.msg import Twist

class GPSPubClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)   
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber('nav2d_vel', Twist, self.get_cmd)
        ####  PUBLISHERS ###
        self.cmd_vel_pub=rospy.Publisher("cmd_vel", Twist, queue_size = 1)
        
        ### CONSTANTS ###
        self.robot_vel=Twist()
        self.robot_vel.linear.x=0
        self.robot_vel.angular.z=0

        r = rospy.Rate(10) #10 Hz 
        #********** INIT NODE **********###  
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.robot_vel)
            r.sleep()
      
    def get_cmd (self, msg):

        self.robot_vel.linear.x = 2 * msg.linear.x
        self.robot_vel.angular.z = 10 * msg.angular.z
        print self.robot_vel.linear.x 
        print self.robot_vel.angular.z
     
    def cleanup(self):  

        #This function is called just before finishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.   
        print("entro")
        self.robot_vel.angular.z = 0
        self.robot_vel.linear.x = 0 
        self.cmd_vel_pub.publish(self.robot_vel)

      

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("gps_subscriber", anonymous=True)  

    try:  

        GPSPubClass()  

    except:  

        rospy.logfatal("gps_subscriber died")  
