#!/usr/bin/env python  
import rospy  
import math     #to calculate the tan inverse 
from sensor_msgs.msg import NavSatFix 
from sensor_msgs.msg import Imu #Odometry   #odometry like imu
from tf.transformations import euler_from_quaternion #library to transform the quaternion data of imu to euler angles
from nav2d_operator.msg import cmd

class GPSPubClass():  
    def __init__(self):  
        rospy.on_shutdown(self.cleanup)   
        ############################### SUBSCRIBERS #####################################  
        rospy.Subscriber("fix", NavSatFix, self.gps_cb)   
        rospy.Subscriber("fixgoal", NavSatFix, self.gps_goal_cb)
        rospy.Subscriber('imu', Imu, self.get_rotation)
        ####  PUBLISHERS ###
        self.cmd_pub=rospy.Publisher("cmd", cmd, queue_size = 1)
        
        ### CONSTANTS ###
        self.robot_vel=cmd()
        self.robot_vel.Velocity = 0
        self.robot_vel.Turn = 0
        r = rospy.Rate(10) #10 Hz 
        self.dif_latitude=0.0
        self.dif_longitude=0.0
        self.latitude_robot=0.0
        self.longitude_robot=0.0
        self.latitude_goal=0.0
        self.longitude_goal=0.0
        #********** INIT NODE **********###  
        while not rospy.is_shutdown():
            self.cmd_pub.publish(self.robot_vel)
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
        #print yaw
        
        #Computing the angle to de goal
        ##Angulos de cuadrantes 1 y 2 respecto a x. En radianes de 0 a Pi (0 -> 180)##
        ##Angulos de cuadrantes 2 y 4 respecto a x. En radianes de 0 a -Pi (0 -> -180)##
        #print("yaw: ", yaw)

        angle_1 = -1* math.atan2(self.dif_longitude, (self.dif_latitude + 0.0000000000000000001) )
        print("angle_1: ", angle_1) 
        
        ## Movimiento de motores para llegar al objetivo
        kdist = 25000        
        kang = -0.19 #Para mantener los valores de angulo en el rango de -1 y 1 que son los valores que ocupa el cmd 
        vmax = 1.0
        latlong = 0.00001317690754  ## 1m se representa como 0.00001317690754 en terminos de  latitude y longitude 
        dif_angle = angle_1 - yaw
        #print dif_angle
        print("angulo robot: ", yaw)
        print ("dif_angle:", dif_angle)
        ## Computing the distance to the goal
        dist_to_goal = ((self.dif_latitude ** 2) + (self.dif_longitude ** 2)) ** 0.5  
  

        self.robot_vel.Turn =  kang * dif_angle
         
        if (abs(dif_angle) > 0.2 and abs(dif_angle) < 0.25 ):
            self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) + 0.000001
            self.robot_vel.Turn=  kang * dif_angle
        elif (abs(dif_angle) < 6 and abs(dif_angle) > 5 ): 
            self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) + 0.000001
            self.robot_vel.Turn =  kang * -0.5 * dif_angle
        elif (abs(dif_angle) < 5 and abs(dif_angle) > 0.25 ): 
                self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) + 0.000001
                self.robot_vel.Turn =  kang* dif_angle
                if(abs(angle_1) > 3 and abs(angle_1) < 3.16): 
                    self.robot_vel.Velocity = (self.robot_vel.Velocity * 0.6) +0.000001
                    self.robot_vel.Turn =  kang* abs(dif_angle)  
                  
        else:
            self.robot_vel.Turn = 0.0
        
            ## lo dividi entre 2 para quedarme a 50cm de distancia de separacion del goal  y con un rango de 500m    
        if(dist_to_goal  > (latlong/2.0) and dist_to_goal  < (latlong*500)):
            self.robot_vel.Velocity = (self.robot_vel.Velocity + 0.3)
            if (self.robot_vel.Velocity >= (kdist*dist_to_goal) ): 
                self.robot_vel.Velocity = (kdist*dist_to_goal)
            if (self.robot_vel.Velocity >= vmax):
                self.robot_vel.Velocity = vmax
        else: 
            if (self.robot_vel.Turn != 0): 
                 self.robot_vel.Velocity = 0.00000001
            else:
                self.robot_vel.Velocity = 0.0  

    def cleanup(self):  

        #This function is called just before inishing the node  
        # You can use it to clean things up before leaving  
        # Example: stop the robot before finishing a node.   
        print("entro")
        self.robot_vel.Velocity = 0.0
        self.robot_vel.Turn = 0.0 
        self.cmd_pub.publish(self.robot_vel)

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":  

    rospy.init_node("Navegacion_gps_V3", anonymous=True)  

    try:  

        GPSPubClass()  

    except:  

        rospy.logfatal("Navegacion_gps_V3 died")  
