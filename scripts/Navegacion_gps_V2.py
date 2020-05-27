#!/usr/bin/env python  
import rospy  
import math     #to calculate the tan inverse 
from sensor_msgs.msg import NavSatFix 
from sensor_msgs.msg import Imu #Odometry   #odometry like imu
from tf.transformations import euler_from_quaternion #library to transform the quaternion data of imu to euler angles
from geometry_msgs.msg import Twist

class GPSPubClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)   
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("fix", NavSatFix, self.gps_cb)   
        rospy.Subscriber("fixgoal", NavSatFix, self.gps_goal_cb)
        rospy.Subscriber('imu', Imu, self.get_rotation)
        ####  PUBLISHERS ###
        self.cmd_vel_pub=rospy.Publisher("cmd_vel", Twist, queue_size = 1)
        
        ### CONSTANTS ###
        self.robot_vel=Twist()
        self.robot_vel.linear.x=0
        self.robot_vel.angular.z=0
        r = rospy.Rate(10) #10 Hz 
        self.dif_latitude=0.0
        self.dif_longitude=0.0
        self.latitude_robot=0.0
        self.longitude_robot=0.0
        self.latitude_goal=0.0
        self.longitude_goal=0.0
        #********** INIT NODE **********###  
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.robot_vel)
            r.sleep()
       #cuando no hay while
        #rospy.spin() 


    def gps_cb(self, msg):  
        
        self.latitude_robot= msg.latitude
        self.longitude_robot= msg.longitude 

    def gps_goal_cb(self, msg):  
        ## This function receives a number
        self.latitude_goal= msg.latitude
        self.longitude_goal= msg.longitude
        self.dif_latitude= self.latitude_goal - self.latitude_robot
        self.dif_longitude= self.longitude_goal - self.longitude_robot
        #print("latitud goal: ", msg.latitude)
    
    def get_rotation (self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
        #print("dif latitude: ", self.dif_latitude)
        #print("dif longitude: ", self.dif_longitude)
        print yaw
        
        #Computing the angle to de goal
        ##Angulos de cuadrantes 1 y 2 respecto a x. En radianes de 0 a Pi (0 -> 180)##
        ##Angulos de cuadrantes 2 y 4 respecto a x. En radianes de 0 a -Pi (0 -> -180)##
        #print("yaw: ", yaw)

        if ((self.latitude_goal > self.latitude_robot) and (self.longitude_goal < self.longitude_robot)) :
            #cuadrante 1
            angle_1 = math.atan(self.dif_longitude/ (self.dif_latitude + 0.0000000000000000001) )
            angle_1 = (angle_1 * -1 )
            print("angle_1: ", angle_1) 
          

        elif ((self.latitude_goal > self.latitude_robot) and (self.longitude_goal > self.longitude_robot)) :
            #cuadrante 4
            angle_1 = math.atan(self.dif_longitude/ (self.dif_latitude + 0.0000000000000000001))
            angle_1 = (-1*angle_1)
            print("angle_4: ", angle_1)
          

        elif ((self.latitude_goal < self.latitude_robot) and (self.longitude_goal < self.longitude_robot)) :
            #cuadrante 2
            angle_1 = math.atan(self.dif_latitude/ (self.dif_longitude + 0.0000000000000000001))
            angle_1 = angle_1 + 1.57079632 #Sume 90 en radianes para que el angulo calculado sea respecto a x 
            print("angle_2: ", angle_1)
          

        elif ((self.latitude_goal < self.latitude_robot) and (self.longitude_goal > self.longitude_robot)) :
            #cuadrante 3
            angle_1 = math.atan(self.dif_latitude / (self.dif_longitude + 0.0000000000000000001))
            angle_1 = angle_1 - 1.57079632 #Sume 90 negativos para que el angulo calculado sea respecto a x. 
            print("angle_3: ", angle_1)
       
        
        else: 
            angle_1=0.0
            self.robot_vel.angular.z = 0.0
            self.robot_vel.linear.x = 0.0 
            dif_angle = 0.0
            print("no hay goal")
        
        ## Movimiento de motores para llegar al objetivo
        kdist = 25000        
        kang = 0.8
        vmax = 2.0
        latlong = 0.00001317690754  ## 1m se representa como 0.00001317690754 en terminos de  latitude y longitude 
        dif_angle = yaw - angle_1
        print dif_angle

        if (dif_angle < 6 and dif_angle > 0.2):
            self.robot_vel.linear.x = self.robot_vel.linear.x - (self.robot_vel.linear.x / 4)
            self.robot_vel.angular.z =  kang *-1*abs(dif_angle)
           
        elif (dif_angle < -0.2 and dif_angle > -6):
            self.robot_vel.linear.x = self.robot_vel.linear.x - (self.robot_vel.linear.x / 4)
            self.robot_vel.angular.z =  kang * abs(dif_angle)
              
        else:
            self.robot_vel.angular.z = 0.0
            ## Computing the distance to the goal
            dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5  
           
            ## lo dividi entre 2 para quedarme a 50cm de distancia de separacion del goal  y con un rango de 500m    
            if(dist_to_goal  > (latlong/2.0) and dist_to_goal  < (latlong*500)):
                self.robot_vel.linear.x = self.robot_vel.linear.x + 0.3 
                if (self.robot_vel.linear.x >= (kdist*dist_to_goal) ): 
                    self.robot_vel.linear.x = kdist*dist_to_goal
                if (self.robot_vel.linear.x >= vmax):
                    self.robot_vel.linear.x = vmax
            else:   
                self.robot_vel.linear.x = 0.0

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
