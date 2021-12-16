# -*- coding: utf-8 -*-
"""
Created on Thu Dec 16 11:46:01 2021

@author: Masson
"""
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import rospy
import cv2
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from nav_msgs.srv import GetMap
from nav_msgs.msg import Path
from sys import exit
import numpy as np
import math as m
import random

class RRT:
    def __init__(self, K=0, dq=0):
        """ Constructor """
        rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.starting_pose_cb)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_pose_cb)
        self.pathPub = rospy.Publisher("/path", Path, queue_size=1)

        print("Waiting for map service to be available...")
        rospy.wait_for_service('/static_map')   #The node waits for the service that provides the map to be avaiblable
        try:    #GET the map data
            get_map = rospy.ServiceProxy('/static_map', GetMap)
            self.map = get_map().map
            print("Map received !")

            L = np.zeros((self.map.info.height,self.map.info.width,3),np.uint8)
            k=0
            for i in range (0,self.map.info.height):
                for j in range (0,self.map.info.width):
                    if(self.map.data[k]==-1):
                        L[-i,j]=[0,0,0]
                    elif(self.map.data[k]==100):
                        L[-i,j]=[0,0,0]
                    else:
                        L[-i,j]=[255,255,255]
                    k+=1
            
            self.M = L
            self.image = cv2.imshow('image',L)
            
            """ TODO - Add your attributes """
            #self.arrive=False
            self.node  = []    # tous les noeuds qu'on a dans le graphe 
            self.Lrand = []
            self.parent = []
            self.path_coord =[]
            self.path_coord_correct=[]
            self.path_coord_reg_1D=[]
            self.path_coord_reg_2D=[]
            """
            self.x=[]       #list of each nodes coordinates according to x
            self.y=[]       #list of each nodes coordinates according to y
            self.x.append(self.x)
            self.y.append(self.y)
            """
            self.path = []  # for the reduce path part
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        except rospy.ServiceException as e:
            print("Map service call failed: %s"%e)
            exit()

        self.starting_pose_received = False


    # ********************************
    def starting_pose_cb(self, msg):
        """ TODO - Get the starting pose """
        self.starting_pose_received = True
        x = int((msg.pose.pose.position.x - self.map.info.origin.position.x)/self.map.info.resolution)
        print("x_s: ",x)
        y = self.map.info.height - int((msg.pose.pose.position.y - self.map.info.origin.position.y)/self.map.info.resolution)
        print("y_s: ",y)
        self.start_img = (x, y)
        return(self.start_img)

    # ********************************
    def goal_pose_cb(self, msg):
        """ TODO - Get the goal pose """
        x = int((msg.pose.position.x - self.map.info.origin.position.x)/self.map.info.resolution)
        print("x_g: ",x)
        y = self.map.info.height - int((msg.pose.position.y - self.map.info.origin.position.y)/self.map.info.resolution)
        print("y_g: ",y)
        self.goal_img = (x, y)
        
        #TO DOT TOUCH
        if self.starting_pose_received:
            self.run()
        return (self.goal_img)

    # *******************************

    def Rand_free_conf(self):
        condition = True
        while (condition == True):
            x=random.randint(0,self.map.info.width)
            y=random.randint(0,self.map.info.height)
            for i in self.Lrand:
                if (i[0]==x and i[1]==y):
                    condition = True
                    break
            condition = False
        self.Lrand.append((x,y))
        return(x,y)

    # *******************************

    def add_vertex(self,qnew):
        """
        Cette fonction sert à ajouter le tuple q_new = (x_new,y_new) dans la liste node
        """
        self.node.append(qnew)      
        
    # *******************************

    def add_edge(self,node_parent,node_child):
        """
        cette fonction sert à tracer une ligne entre node_near et node_new sur l'image
        """
        # self.parent.insert(node_child[0],node_parent[0])
        cv2.line(self.M,node_parent, node_child, color=(170,170,0), thickness=3)
        cv2.imshow('image',self.M)
    # *******************************
    
    def distance(self,node_1,node_2):
        return(m.sqrt((node_1[0]-node_2[0])**2+(node_1[1]-node_2[1])**2))

     
    # *******************************

    def nearest_vertex(self,q_rand):
        dist=self.distance(self.node[0],q_rand)
        q_near = self.node[0]
        index_parent=0
        for i in range (len(self.node)):
            if (self.distance(self.node[i],q_rand)<dist):
                dist=self.distance(self.node[i],q_rand)
                q_near = self.node[i]
                index_parent = i
        return q_near,index_parent

    


    # ******************************

    def New_conf(self,qnear,qrand,delta_q):
        
        (x_rand,y_rand)=qrand
        (x_near,y_near)=qnear

        d_x = x_rand - x_near
        d_y = y_rand - y_near

        theta=m.atan2(d_y,d_x)

        x_new=int(x_near+delta_q*m.cos(theta))
        y_new=int(y_near+delta_q*m.sin(theta))

        if(x_new>self.map.info.width-1 or y_new>self.map.info.height-1): 
            return False
        else:
            print("inferieur!!!")
            return (x_new,y_new)
        
    # ******************************

    

     
    def run(self):
        """ TODO - Implement the RRT algorithm """
        self.add_vertex(self.start_img)
        K=10000
        delta_q = 25
        index_node = 0
        self.parent.append(0)
        # ****************RRT*********************#
        for k in range(K):
            print("**************************************")
            print(k+1," ième essai")
            q_rand = self.Rand_free_conf()
            q_near = self.nearest_vertex(q_rand)[0]
            i_qnear= self.nearest_vertex(q_rand)[1]
            q_new = self.New_conf(q_near,q_rand,delta_q)
            if (q_new != False):
                print("coord de new",q_new)
                if (np.any(self.M[q_new[1],q_new[0]] != [0,0,0])):
                    # si le noeud new n'est pas dans le bloc noir
                    self.add_vertex(q_new)
                    self.add_edge(q_near,q_new)
                    self.parent.append(i_qnear)
                    # sinon (s'il est dans l'obstacle, voir cas du dispo 25)
                    # alors on ne le prends pas en  compte, on ne le rajoute pas dans la notre arbre
                    index_node +=1
                    print("///////////////////////////////////////")
                    ## CHECK ##
                dist=self.distance(q_new,self.goal_img)
                if( dist<delta_q ):
                    self.add_vertex(self.goal_img)
                    self.add_edge(q_near,self.goal_img)
                    print(" Algorithm is done !!! ")
                    print(" The last new_node is ",self.node[index_node])
                    print(" remember that our goal is", self.goal_img)
                    print(" thus, we are very close to the goal")
                    self.path.append(len(self.parent)-1)
                    p=self.parent[-1]
                    self.path.append(p)
                    print("la taile de ma liste node :",len(self.node))
                    print("avant la boucle while, ma liste parent:",self.parent)
                    print("sa taile :",len(self.parent))
                    print("avant la boucle while, ma liste path:",self.path)
                    while( p != 0):
                        p=self.parent[p]
                        self.path.append(p)
                    break
        
        # *****************************************************************************#



        # *****************************************************************************#
        for i in range(len(self.node)):
            cv2.circle(self.M, (self.node[i][0],self.node[i][1]), radius=5, color=(170,170,0), thickness=-1)
        print(self.path)
        self.path_coord.append(self.goal_img)
        for p in range(len(self.path)-1):
            self.path_coord.append(self.node[self.path[p]])
            cv2.circle(self.M, (self.node[self.path[p]][0],self.node[self.path[p]][1]), radius=5, color=(115,8,0), thickness=-1)
            cv2.line(self.M,self.node[self.path[p]], self.node[self.path[p+1]], color=(115,8,0), thickness=3)
            cv2.imshow('image',self.M)
    
        cv2.line(self.M,self.path_coord[1], self.goal_img, color=(115,8,0), thickness=3)

        cv2.circle(self.M, (self.start_img[0],self.start_img[1]), radius=5, color=(255,255,0), thickness=-1)
        cv2.circle(self.M, (self.goal_img[0],self.goal_img[1]), radius=5, color=(0,0,255), thickness=-1)
        cv2.imshow('image',self.M)

        # *****************************************************************************#

        # *****************************************************************************#

        # create some waypoints 
        
        # withdraw the obvious error point
        for i in range(len(self.path_coord)-1):
            
            dist1=self.distance(self.path_coord[i],self.goal_img)
            dist2=self.distance(self.path_coord[i+1],self.goal_img)
            if(dist1<dist2):
                self.path_coord_correct.append(self.path_coord[i])
                print("waypoint: ",i) # we have our first waypoint !!
            else:
                print("no")
        print(self.path_coord_correct)
            
        # fit the model
         
        #print(self.path_coord_correct)
        List_x=[]
        List_y=[]
        for i in range (len(self.path_coord_correct)):
            List_x.append(self.path_coord_correct[i][0])
            List_y.append(self.path_coord_correct[i][1])

        #print(List_x)
        #print(List_y)

        p4 = np.poly1d(np.polyfit(List_x, List_y, 4))
        print(p4)
        xp = np.linspace(0,self.map.info.height , 100)
        #pp.plot(xp, p4(xp), c='r')

        i=0
        for i in range(100):
            self.path_coord_reg_1D.insert(2*i,int(xp[i]))
            self.path_coord_reg_1D.insert(2*i+1,int(p4(xp)[i]))
            
        #print(self.path_coord_reg_1D)

        for i in range(0,len(self.path_coord_reg_1D),2):
            self.path_coord_reg_2D.append( (self.path_coord_reg_1D[i],self.path_coord_reg_1D[i+1]))
            

        print(self.path_coord_reg_2D)
        #print(self.path_coord)
        

            

        self.publishPath()
        return self.node
        

    # ********************************
    def publishPath(self):
        """ Send the computed path so that RVIZ displays it """
        """ TODO - Transform the waypoints from pixels coordinates to meters in the map frame """
        msg = Path()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        path_RVIZ = []
        path_RVIZ_correct=[]
        
        for pose_img in self.path_coord:
            pose = PoseStamped()
            pose.pose.position.x = self.map.info.resolution*pose_img[0]+self.map.info.origin.position.x
            pose.pose.position.y = self.map.info.resolution*(-pose_img[1]+self.map.info.height)+self.map.info.origin.position.y
            path_RVIZ.append(pose)
        """
        for pose_img in self.path_coord_reg_2D:
            pose = PoseStamped()
            pose.pose.position.x = self.map.info.resolution*pose_img[0]+self.map.info.origin.position.x
            pose.pose.position.y = self.map.info.resolution*(-pose_img[1]+self.map.info.height)+self.map.info.origin.position.y
            path_RVIZ_correct.append(pose)
        msg.poses = path_RVIZ_correct
        """
        #print(path_RVIZ)
        #print(path_RVIZ_correct)
        self.pathPub.publish(msg)
if __name__ == '__main__':
    """ DO NOT TOUCH """
    rospy.init_node("RRT", anonymous=True)

    rrt = RRT()

    rospy.spin()