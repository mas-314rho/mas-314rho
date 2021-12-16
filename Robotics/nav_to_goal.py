import rclpy
from rclpy.action import ActionClient,ActionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import random
import numpy as np
import sys
import time
from datetime import datetime
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from rclpy.qos import qos_profile_system_default

nb_tables=9 
nb_task=3
restaurant=[1,2,3,4,5,6,7,8,9]
pose_init=(0,0)
waypoint=[]
#useful function for algorithm

def random_tables(n): # we choose randomly 3 tables to serve
    table_selected=[]
    for i in range(n):
        index=random.randint(0,len(restaurant)-1)
        #print(index)
        table_selected.append(restaurant[index])
        del restaurant[index]
        
    return table_selected
        
        
service=random_tables(nb_task)
"""
print(service[0])
print("**")
print(service[1])
print("**")
print(service[2])
print("**")
print("Tables we must serve are numbers",service[0],service[1],service[2])
"""


def coord_tables(index_t): # each tables have an assign coordinates
    if index_t==1:
        coord=(-1,1)
    elif index_t==2:
        coord=(0,1)
    elif index_t==3:
        coord=(1,1)
    elif index_t==4:
        coord=(-1,0)
    elif index_t==5:
        coord=(0,0)
    elif index_t==6:
        coord=(1,0)
    elif index_t==7:
        coord=(-1,-1)
    elif index_t==8:
        coord=(0,-1)
    elif index_t==9:
        coord=(1,-1)
    else:
        coord=False
        print("there is no table ohter there")
    return coord
 

def d_euclidienne(x1,x2,y1,y2): # norm
    return np.sqrt((x2-x1)**2+(y2-y1)**2)

def way_init(p,i1,i2,i3): #main algorithm, we evaluate the distance between the spwan position
    xp,yp=p               #and the 3 tables. Then we choose the nearest until it 's done
    print(xp,yp)
    x,y=coord_tables(i1)
    print(x,y)
    a,b=coord_tables(i2)
    print(a,b)
    u,v=coord_tables(i3)
    print(u,v)
    
    d_1=d_euclidienne(xp, x, yp,y)
    d_2=d_euclidienne(xp,a, yp,b)
    d_3=d_euclidienne(xp,u,yp,v)
    """
    print("**")
    print(d_1)
    print(d_2)
    print(d_3)
    print("**")
    """
    choice=min(d_1,d_2,d_3)
    #print(choice)
    
    if (choice==d_1): #if index1 is the closest
        index=i1
        di=d_euclidienne(x,a, y, b)
        #print("**")
        #print(di)
        dj=d_euclidienne(x,u,y, v)
        #print(dj)
        #print("**")
        choice2=min(di, dj)
        if (choice2==di):
            index2=i2
            index3=i3
        else:
            index2=i3
            index3=i2
            
    elif (choice==d_2): #if index2 is the closest
        index=i1
        index=i2
        di=d_euclidienne(a,x, b, y)
        dj=d_euclidienne(a,u,b,v)
        choice2=min(di, dj)
        """
        print("**")
        print(di)
       
        print(dj)
        print("**")
        """
        if (choice2==di):
            index2=i1
            index3=i3
        else:
            index2=i3
            index3=i1
            
            
    elif (choice==d_3): #if index3 is the closest
        index=i1
        index=i3
        di=d_euclidienne(u,x,v,y)
        dj=d_euclidienne(u,a,v, b)
        """
        print("**")
        print(di)
        
        print(dj)
        print("**")
        """
        choice2=min(di, dj)
        if (choice2==di):
            index2=i1
            index3=i2
        else:
            index2=i2
            index3=i1
    #print(index)
    
    return index,index2,index3

def service(n,p):
        s=random_tables(n)
        move=way_init(p, s[0], s[1], s[2])
        return move


# Callbacks definition
#-----------------------------------------------------------------------------#
def active_cb(extra,node):
    node.get_logger().info("Goal pose being processed")

def feedback_cb(feedback,node):
    node.get_logger().info("Current location: "+str(feedback))

def done_cb(status, result,node):
    if status == 3:
        node.get_logger().info("Goal reached")
    if status == 2 or status == 8:
        node.get_logger().info("Goal cancelled")
    if status == 4:
        node.get_logger().info("Goal aborted")
  #---------------------------------------------------------------------------#  
def main(args=None):
    
    ############################## NODE ################################
    rclpy.init(args=sys.argv)

    node=rclpy.create_node('init_pose') # spawn point node #
    
    node_nav=rclpy.create_node('nave_pose') # navigation node #
    
    #####################################################################

    publisher= node.create_publisher(PoseWithCovarianceStamped,'/initialpose',qos_profile_system_default) 
    rdnposex = random.uniform(-2.,2.) #selecting a random position on the x axis
    rdnposey = random.uniform(-2.,2.) #selecting a random position on the y axis
    rdnorientz = random.uniform(0,360)    #selecting a random orientation around z axis
    
    pose_init=(rdnposex,rdnposey)

    initpose_msg = PoseWithCovarianceStamped()
    initpose_msg.header.frame_id = "map"
    initpose_msg.pose.pose.position.x = rdnposex
    initpose_msg.pose.pose.position.y = rdnposey
    initpose_msg.pose.pose.orientation.x = 0.0
    initpose_msg.pose.pose.orientation.y = 0.0
    initpose_msg.pose.pose.orientation.z = rdnorientz
    initpose_msg.pose.pose.orientation.w = 0.0
    
    #########################################################################

    node.get_logger().info( "Setting initial pose")
    publisher.publish(initpose_msg)
    
    
    ##########################################################################
    
    serv=service(nb_task,pose_init)
    for i in range(nb_task):
            waypoint.append(coord_tables(serv[i])) 
            
    ##########################################################################
                
    navclient = ActionClient(node_nav,MoveBaseAction,'move')
    navclient.wait_for_server()
    
    #  navigation goal 1
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = node_nav.Time.now()
    
    goal.target_pose.pose.position.x = waypoint[0][0]
    goal.target_pose.pose.position.y = waypoint[0][1]
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 0.0
    
    navclient.send_goal(goal, done_cb(node_nav), active_cb(node_nav), feedback_cb(node_nav))
    finished = navclient.wait_for_result()
    
    ##########################################################################
    
    if not finished:
        node_nav.get_logger().info("Action server not available!")
    else:
        node_nav.get_logger().info("We reached the 1st table")
        node_nav.get_logger().info( navclient.get_result())
        
    ##########################################################################
    
    #  navigation goal 2
    goal2 = MoveBaseGoal()
    goal2.target_pose.header.frame_id = "map"
    goal2.target_pose.header.stamp = node_nav.Time.now()
    
    goal2.target_pose.pose.position.x = waypoint[1][0]
    goal2.target_pose.pose.position.y = waypoint[1][1]
    goal2.target_pose.pose.position.z = 0.0
    goal2.target_pose.pose.orientation.x = 0.0
    goal2.target_pose.pose.orientation.y = 0.0
    goal2.target_pose.pose.orientation.z = 0.0
    goal2.target_pose.pose.orientation.w = 0.0
    
    navclient.send_goal(goal2, done_cb(node_nav), active_cb(node_nav), feedback_cb(node_nav))
    finished2 = navclient.wait_for_result()
        
    ##########################################################################
    
    if not finished2:
        node_nav.get_logger().info("Action server not available!")
    else:
        node_nav.get_logger().info("We reached the 2nd table")
        node_nav.get_logger().info( navclient.get_result())
        
    ##########################################################################
    
    
    ##########################################################################
    
    #  navigation goal 3
    goal3 = MoveBaseGoal()
    goal3.target_pose.header.frame_id = "map"
    goal3.target_pose.header.stamp = node_nav.Time.now()
    
    goal3.target_pose.pose.position.x = waypoint[2][0]
    goal3.target_pose.pose.position.y = waypoint[2][1]
    goal3.target_pose.pose.position.z = 0.0
    goal3.target_pose.pose.orientation.x = 0.0
    goal3.target_pose.pose.orientation.y = 0.0
    goal3.target_pose.pose.orientation.z = 0.0
    goal3.target_pose.pose.orientation.w = 0.0
    
    navclient.send_goal(goal3, done_cb(node_nav), active_cb(node_nav), feedback_cb(node_nav))
    finished3 = navclient.wait_for_result()
        
    ##########################################################################
    
    if not finished3:
        node_nav.get_logger().info("Action server not available!")
    else:
        node_nav.get_logger().info("We reached the 3rd table")
        node_nav.get_logger().info( navclient.get_result())
        
    ##########################################################################

if __name__ == '__main__':
    main()