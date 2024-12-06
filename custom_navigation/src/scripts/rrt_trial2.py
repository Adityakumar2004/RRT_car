#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose,Point
from std_msgs.msg import ColorRGBA
import random
from typing import Optional



def create_actual_path_marker(point1:Point,point2:Point,id):
        edge_marker = Marker()
        edge_marker.header.frame_id = 'robot_footprint'
        edge_marker.ns = 'actual_path'
        edge_marker.type = Marker.LINE_STRIP
        edge_marker.id  = id
        edge_marker.action = Marker.ADD
        edge_marker.pose.orientation.w = 1.0
        edge_marker.scale.x = 0.01
        edge_marker.color = ColorRGBA(0,0,1,1)
        edge_marker.points.append(point1)
        edge_marker.points.append(point2)
        return(edge_marker)



def create_node_marker(pose_node:Pose,id):

        Node_marker = Marker()
        Node_marker.header.frame_id = 'robot_footprint'
        Node_marker.ns = 'rrt_graph_node'
        Node_marker.id = id
        Node_marker.type = Marker.SPHERE
        Node_marker.action = Marker.ADD 
        Node_marker.pose = pose_node
        Node_marker.scale.x = 0.09
        Node_marker.scale.y = 0.09
        Node_marker.scale.z = 0.09
        Node_marker.color = ColorRGBA(0.6,0.4,0.1,1)
        # self.Marker_array.append(Node_marker)
        return(Node_marker)    

def create_line_marker(parent:Marker,child:Marker,id):
        edge_marker = Marker()
        edge_marker.header.frame_id = 'robot_footprint'
        edge_marker.ns = 'rrt_graph_edges'
        edge_marker.type = Marker.LINE_STRIP
        edge_marker.id  = id
        edge_marker.action = Marker.ADD
        edge_marker.pose.orientation.w = 1.0
        edge_marker.scale.x = 0.01
        edge_marker.color = ColorRGBA(0,1,0,1)
        edge_marker.points.append(parent.pose.position)
        edge_marker.points.append(child.pose.position)
        return(edge_marker)
        



     


class tree_node:
    array_marker = MarkerArray()
    id_node = 0
    id_marker = 0

    def __init__(self,parent = None):

        self.pose = Pose()
        self.steering_angle = 0
        self.id = tree_node.id_node
        self.parent = parent

        ## child.control_param = [u1n,u2n,Tn] this is the format of control param and is added using add_child
        self.control_param = None
        
        self.children = []
        # self.node_marker = create_node_marker(self.pose,self.id)
        # tree_node.array_marker.markers.append(self.node_marker)
        # tree_node.id_node+=1;
        
        

     # add_parent no more in use
    def add_parent(self,parent):
         self.parent = parent

    def add_child(self,child:'tree_node',u1n,u2n,Tn):
         
         self.children.append(child)
         child.parent = self
         child.control_param = [u1n,u2n,Tn]
     #     child_marker = child.node_marker
         
         edge_marker = create_line_marker(self.node_marker,child.node_marker,tree_node.id_marker)
         tree_node.id_marker+=1
         tree_node.array_marker.markers.append(edge_marker)
         
         
         
    # add_markers no more in use  
    def add_markers(self):
        
        
        self.node_marker = create_node_marker(self.pose,self.id)
        tree_node.array_marker.markers.append(self.node_marker)
        tree_node.id_node+=1;

        # if self.parent == None:
        #      self.node_array_marker = MarkerArray()

       
def publish_rrt_graph():
     
     rospy.init_node('rrt_graph_publisher')
     graph_pub = rospy.Publisher('/rrt_graph', MarkerArray, queue_size=10)

    

     node1 = tree_node()
     node1.pose.position = Point(0,0,0)
     node1.pose.orientation.w = 1

     node2 = tree_node()
     node2.pose.position = Point(0.7,0.7,0)
     node2.pose.orientation.w = 1
    
     node3 = tree_node()
     node3.pose.position = Point(0.0,0.7,0)
     node3.pose.orientation.w = 1
     print(tree_node.id_node)

     node1.add_child(node2)
     node2.add_child(node3)
     node1.add_child(node3)
     print(tree_node.id_marker)

     rate = rospy.Rate(1)
     while not rospy.is_shutdown():
        graph_pub.publish(tree_node.array_marker)
        rate.sleep()
     
     

        

        
if __name__ == '__main__':



     try:
        publish_rrt_graph()
        rospy.spin()
     except rospy.ROSInterruptException:
        pass


    



    
    



