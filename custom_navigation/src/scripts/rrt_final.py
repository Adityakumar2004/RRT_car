#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np

from rrt_trial2 import *
import random
import math
import cv2
import yaml
from typing import Union,List



path = '/home/aditya/catkin_ws_autonomous_navig/src/custom_navigation/src'

img = cv2.imread(f'{path}/map_res.pgm',-1)

with open(f'{path}/map_res.yaml','r') as file_reader:
    map_parameters = yaml.safe_load(file_reader)

resolution = map_parameters['resolution']
print('this is the resolution of the map ', resolution)
  
def rrt_publisher_rviz():
     
     rospy.init_node('rrt_graph_publisher')
     graph_pub = rospy.Publisher('/rrt_graph', MarkerArray, queue_size=10)





     rate = rospy.Rate(1)
     while not rospy.is_shutdown():
        graph_pub.publish(tree_node.array_marker)
        rate.sleep()
     




class pose_subscribers:

    def __init__(self,resolution,img):

        self.resolution = resolution
        self.img = img

        # set the class variables for transformation 
        self.Rotational,self.Translation_pixel = self.transformation_matrices_rviz_to_pixel()

        self.ros_topics = {"/move_base_simple/goal":{"data":None,"type":PoseStamped},
                           "/initialpose":{"data":None,"type":PoseWithCovarianceStamped}}
        
        self.length_car = 0.5 # robot model
        self.max_steer_angle = 0.5 # which is 28 degree 
        self.control_vt_samples = [0, 0.25, 0.5, 1, 1.5, 2, 3]  # 2.5 to 4.5
        self.control_vs_samples = [0, 0.5, 1]  # max of 60 degree/sec (1.047 radians) and max angle of 30 - 40 degree 
        self.Ts = 2  # simulation time 
        
        self.desired_error = 0.3;
        
        self.flag_start = 0  ## putting a flag to know when to start the graph making process
        
        
        # ## -------other class variables introduced in the middle 
        # initial_pose
        # goal_pose 


    ## defining the rotational and translational matrices 
    def transformation_matrices_rviz_to_pixel(self):
        
        Rotational = np.array([[1, 0, 0],
                            [0, -1, 0],
                            [0, 0, -1]])
        shape_img = self.img.shape
        Translation_pixel = (np.array([[0,shape_img[0],0]])).T
        # print(Translation_pixel)
        return(Rotational,Translation_pixel)
    
        
    def rviz_to_pixel(self,data:Union[Pose,np.ndarray]):
        
        if isinstance(data,Pose):
            position_wrt_rviz = np.array([[data.position.x],[data.position.y],[data.position.z]])

        if isinstance(data,np.ndarray):
            position_wrt_rviz = np.array([[data[0,0],data[1,0],0]]).T

        position_wrt_rviz = position_wrt_rviz/self.resolution
        
        position_np = np.dot(self.Rotational,position_wrt_rviz) 
        # print(position_np)
        position_np = position_np + self.Translation_pixel
        print('position pixels ',position_np)
        return(position_np)
    
    def pixel_to_rviz(self,position_np):

        position_rviz = position_np - self.Translation_pixel
        rot = self.Rotational.T
        position_rviz = np.dot(rot,position_rviz)*self.resolution
        # print('re transformed actual ',position_rviz)

        return(position_rviz)

    ## angle will be given in radians from 0 to 2pi
    def quaternions_to_angle(self,orientation:Quaternion):
        z = orientation.z 
        w = orientation.w
        # quaternion = [x, y, z, w]
        ## For rotation about the z-axis, the quaternion will be [0, 0, sin(theta/2), cos(theta/2)]
        
        ## acos range from 0 to pi
        ## angle is in radians with a range from 0 to pi/2 as we have taken absolute value 
        angle = 2*math.acos(abs(w))
        if z<0:
            angle = 2*math.pi - angle

        return(angle)


    ## angle in radians be given 
    def angle_to_quaternion(self,angle):
        
        ## making angle between -pi/2 to pi/2 
        if angle > 2*math.pi:
            angle = angle-2*math.pi

        z = math.sin(angle/2)
        w = math.cos(angle/2)

        orientation = Quaternion(0,0,z,w)

        return(orientation)

    def angle_vector(self,angle):

        x = math.cos(angle)
        y = math.sin(angle)
        z = 0
        vec = (np.array([[x,y,z]])).T
        return(vec)
    def vec_angle(self,vec):
        x = vec[0,0]
        y = vec[1,0]

        angle = math.atan2(y,x)
        if angle<0:
            angle = angle + 2*math.pi
        return(angle)

###-------------------------------rrt main functions ------------------------------------------------

    def distance_metric(self,position1,phi1,theta1,position2,phi2,theta2):
        
        d = np.dot((position2-position1).T,(position2 - position1))
        d = d**0.5

        p = abs(phi2-phi1)
        r = math.pi*2*(abs(theta2 - theta1)/(2*math.pi) - math.floor(0.5 + abs(theta2- theta1)/(2*math.pi)))
        w1 = 80
        w2 = 15
        w3 = 5

        distance = (w1*d + w2*r + w3*p) /(w1+w2+w3)
        return(distance)

    def random_generator(self,ub,lb,step):
        total = int((ub - lb)/step + 2)
        domain_random = [round(lb+step*i,3) for i in range(0,total)]
        return(domain_random)

    
    def nearest_node_recursive(self,node:tree_node,position_pt,phi_pt,theta_pt,best_distance,best_node):

        if node == None:
            
            return best_node,best_distance
        
        position = node.pose.position
        phi = node.steering_angle
        theta = self.quaternions_to_angle(node.pose.orientation)

        distance = self.distance_metric(position,phi,theta,position_pt,phi_pt,theta_pt)

        if distance<= best_distance:
            best_distance = distance
            best_node = node
        
        if len(node.children) == 0:
            return(best_node,best_distance)
        
        for child in node.children:
            best_node,best_distance = self.nearest_node_recursive(child,position_pt,phi_pt,theta_pt,best_distance,best_node) 
        
        return(best_node,best_distance)
    

    # def is_collision(self):

    def Extend_while_valid(self,position:np.ndarray,phi,theta,u1,u2):
        
        
        dt = 0.01 
        i = 1
        Tn = i*dt
        while Tn<=self.Ts:

            position_N = position + dt*u1*(np.array([[math.cos(theta,math.sin(theta))]]).T)
            theta_N = theta + dt*u1*math.tan(phi)/self.length_car
            phi_N = phi + dt*u2

            if theta_N>=2*math.pi:
                theta_N = 2*math.pi-theta_N

            if theta_N < 0:
                theta_N = 2*math.pi + theta_N

            if phi_N>self.max_steer_angle:
                phi_N = self.max_steer_angle
            
            if phi_N < -self.max_steer_angle:
                phi_N = -self.max_steer_angle


            ## for checking collision first its converted to pixel coordinates and then checking in the map (occupancy grid) 
            position_n_pixel = self.rviz_to_pixel(position_N)
            if self.img[position_n_pixel[0,0],position_n_pixel[1,0]] < 240:
                return(position,theta,phi,Tn)
                # collision = True
                
            else :
                position = position_N
                theta = theta_N
                phi = phi_N
                
                # collision = False
            
            i+=1
            Tn = i*dt
        
        return(position,theta,phi,Tn-dt)

     

    def rrt_graph_implement(self):

        root = tree_node()
        root.pose = self.initial_pose
        root.steering_angle = 0
        
        ni = 0
        n_max = 50
        ## generating a random position 
        ub = self.initial_pose.position.x + 3
        lb = self.initial_pose.position.x - 3
        step = 0.5
        total = int((ub - lb)/step + 2)
        domain_random_position_x = [round(lb+step*i,3) for i in range(0,total)]

        ub = self.initial_pose.position.y + 3
        lb = self.initial_pose.position.y - 3
        step = 0.5
        total = int((ub - lb)/step + 2)
        domain_random_position_y = [round(lb+step*i,3) for i in range(0,total)]

        ##generating random steer angle
        lb = -0.5
        ub = 0.5
        step = 0.1
        total = int((ub - lb)/step + 1)
        domain_random_steer_angle = [round(lb+step*i,3) for i in range(0,total)]

        ##generating random oreintation
        lb = 0
        ub = 2*math.pi
        step = 0.1
        total = int((ub - lb)/step + 1)
        domain_random_oreintation = [round(lb+step*i,3) for i in range(0,total)]




        while ni<n_max:
            
            position_s = (np.array([[random.choice(domain_random_position_x),random.choice(domain_random_position_y)]])).T
            phi_s = random.choice(domain_random_steer_angle)
            theta_s = random.choice(domain_random_oreintation)

            nearest_node,_ = self.nearest_node_recursive(root,position_s,phi_s,theta_s,float('inf'),None)

            min_dist = float('inf')

            ## ------------%%%%%%%%%%%------------dont forget to limit steering angle and also change the orientation after crossing 2*pi


            for vt in self.control_vt_samples:
                for vs in self.control_vs_samples:
                    
                    position_nearest_node = (np.array([[nearest_node.pose.position.x, nearest_node.pose.position.y]])).T
                    oreintation_nearest_node = self.quaternions_to_angle(nearest_node.pose.orientation)
                    positionN,thetaN,phiN,TN= self.Extend_while_valid(position_nearest_node,nearest_node.steering_angle,oreintation_nearest_node,vt,vs)

                    
                    distance = self.distance_metric(positionN,phiN,thetaN,position_s,phi_s,theta_s)

                    if distance<min_dist:
                        min_list = [vt,vs,positionN,thetaN,phiN,TN]
                    
            

            ## first convert the position and oreintation info into the pose of ros 
            
            vt,vs,positionN,thetaN,phiN,TN = min_list
            
            new_pose = Pose()
            new_pose.position = Point(positionN[0,0],positionN[1,0],0)
            new_pose.orientation =  self.angle_to_quaternion(thetaN)


            new_node = tree_node()

            new_node.pose = new_pose
            new_node.steering_angle = phiN
            # new_node.control_param = [vt,vs]

            nearest_node.add_child(new_node,vt,vs,TN)

            ## checking if the goal is reached 

            angle_goal = self.quaternions_to_angle(self.goal_pose.orientation)
            position_goal = (np.array([[self.goal_pose.position[0,0],self.goal_pose.position[1,0]]])).T
        

            if self.distance_metric(positionN,phiN,thetaN,position_goal,0,angle_goal) < self.desired_error:

                    print('goal reached')
                    break

            ni+=1

        # return()
        
        pass


        
####------------------------------------------%%%%%%%%%%%%%%%%%%%%%%%%%%%%-----------------------------------
    def callback_goal(self,data,args):

        
        # print('this is the data ',data)

         ## type of msg is PoseStamped
        if args == "/move_base_simple/goal":
            
           
            pose_data_goal = data.pose
            self.goal_pose = pose_data_goal
            position_goal = pose_data_goal.position
            print("pose of the goal ",pose_data_goal)

            # goal_np = self.rviz_to_pixel(pose_data_goal)
            self.flag_start+=1

         ## type of msg is PoseWithCovarianceStamped 
        if args == "/initialpose":
            pose_data_initial = data.pose.pose
            self.initial_pose = pose_data_initial
            position_initial = pose_data_initial.position
            print("pose of the initial ",pose_data_initial)
            # initial_np = self.rviz_to_pixel(pose_data_initial)
            # initial_rviz = self.pixel_to_rviz(initial_np)
            self.flag_start+=1

        if self.flag_start>=2:
            self.rrt_graph_implement()

            # show_img(img,initial_np)
        
        
            
    # def callback_initial(data:PoseWithCovarianceStamped):
    #     print(data.pose.pose)

    def subscribe_topics(self):
        for topic in self.ros_topics.keys():
            rospy.Subscriber(name=topic,data_class=self.ros_topics[topic]["type"],callback=self.callback_goal,callback_args=topic)



if __name__ == "__main__":
    rospy.init_node("goal_subscriber",anonymous=True)

    
    pose_class = pose_subscribers(resolution,img)

    
    
    pose_class.subscribe_topics()
    print('hi how r u')
    rospy.spin()
