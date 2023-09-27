




import sys
sys.path.append('.')# one directory up
from math import cos, sin
import numpy as np
import matplotlib
from matplotlib import pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from mpl_toolkits.mplot3d import Axes3D
from lib .rotations import Quaternion2Euler, Quaternion2Rotation, Euler2Rotation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
matplotlib.use('TkAgg')

class UAV_animation():
    def __init__(self, state0, scale=0.25):


        self.scale=scale
        self.flag_init = True
        #self.fig = plt.figure(1)
        self.fig = plt.figure(figsize=(6,6))
        #self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax = self.fig.add_subplot(1, 2, 1, projection='3d')
        self.ax.set_xlim([-10,10])
        self.ax.set_ylim([-10,10])
        self.ax.set_zlim([-10,10])
        self.ax.set_title('3D Animation')
        self.ax.set_xlabel('East(m)')
        self.ax.set_ylabel('North(m)')
        self.ax.set_zlabel('Height(m)')


    def UAV_vertices(self,pn,pe,pd,phi,theta,psi):
        V = np.array([[2.8, 0, 0],       #Nose Vertex (1)
                     [0.1, 0.5, 0.5],   #Nose Base   (2)
                     [0.1, 0.5, -0.5],  #Nose Base   (3)
                     [0.1, -0.5, 0.5],  #Nose Base   (4)
                     [0.1, -0.5, -0.5], #Nose Base   (5)
                     [-5, 0, 0],        #Tail Vertex (6)
                     [0, 4.5, 0],       #Wing Corner (7)
                     [-1.5, 4.5, 0],    #Wing Corner (8)
                     [-1.5, -4.5, 0],   #Wing Corner (9)
                     [0, -4.5, 0],      #Wing Corner (10)
                     [-4.25, 1.65, 0],  #H_Tail Vert (11)
                     [-5, 1.65, 0],     #H_Tail Vert (12)
                     [-5, -1.65, 0],    #H_Tail Vert (13)
                     [-4.25, -1.65, 0], #H_Tail Vert (14)
                     [-4.25, 0, 0],     #V_Tail Vert (15)
                     [-5, 0, -1.5]])     #V_Tail Vert (16)
        
        #V = np.array([[0, 2.8, 0],       #Nose Vertex (1)
        #             [0.5, 0.1, 0.5],   #Nose Base   (2)
        #             [0.5, 0.1, -0.5],  #Nose Base   (3)
        #              [-0.5, 0.1, 0.5],  #Nose Base   (4)
        #              [-0.5, 0.1, -0.5], #Nose Base   (5)
        #              [0, -5, 0],        #Tail Vertex (6)
        #              [4.5, 0, 0],       #Wing Corner (7)
        #              [4.5, -1.5, 0],    #Wing Corner (8)
        #              [-4.5, -1.5, 0],   #Wing Corner (9)
        #              [-4.5, 0, 0],      #Wing Corner (10)
        #              [1.65, -4.25, 0],  #H_Tail Vert (11)
        #              [1.65, -5, 0],     #H_Tail Vert (12)
        #              [-1.65, -5, 0],    #H_Tail Vert (13)
        #              [-1.65, -4.25, 0], #H_Tail Vert (14)
        #              [0, -4.25, 0],     #V_Tail Vert (15)
        #              [0, -5, -1.5]])     #V_Tail Vert (16)
        
        pos_ned = np.array([pe, pn, pd])

        # create m by n copies of pos_ned and used for translation
        ned_rep= np.tile(pos_ned, (16,1))

        R=Euler2Rotation(phi,theta,psi)

        #rotate 
        vr=np.matmul(R,V.T).T
        # translate
        vr=vr+ned_rep
        # rotate for plotting north=y east=x h=-z
        R_plot=np.array([[0, 1, 0],
                        [1, 0, 0],
                        [0, 0, -1]])
        
        vr=np.matmul(R_plot,vr.T).T

        ## Faces: [1-2-3][1-3-4][1-4-5][1-5-2][2-3-6][3-4-6][4-5-6][5-2-6]
        ##        [7-8-10][9-10-8][11-12-14][13-14-12][6-15-16]

        faces = [[vr[0,:],vr[1,:],vr[2,:]],
                 [vr[0,:],vr[2,:],vr[3,:]],
                 [vr[0,:],vr[3,:],vr[4,:]],
                 [vr[0,:],vr[4,:],vr[1,:]],
                 [vr[1,:],vr[2,:],vr[5,:]],
                 [vr[2,:],vr[3,:],vr[5,:]],
                 [vr[3,:],vr[4,:],vr[5,:]],
                 [vr[4,:],vr[1,:],vr[5,:]],
                 [vr[6,:],vr[7,:],vr[9,:]],
                 [vr[8,:],vr[9,:],vr[7,:]],
                 [vr[10,:],vr[11,:],vr[13,:]],
                 [vr[12,:],vr[13,:],vr[11,:]],
                 [vr[5,:],vr[14,:],vr[15,:]]]
        return(faces)
    
    def update(self, pn, pe, pd, phi, theta, psi):
        
        # draw plot elements
        self.draw_UAV(pn,pe,pd,phi,theta,psi)
        # Set initialization flag to False after first call
        if self.flag_init == True:
            self.flag_init = False
        
        # move coordinate system
        #self.ax.set_xlim([pn-10,pn+10])
        #self.ax.set_ylim([pe-10,pe+10])
        #self.ax.set_zlim([pd-10,pd+10])

    def draw_UAV(self, pe, pn, pd, phi, theta, psi):
        faces=self.UAV_vertices(pe,pn,pd,phi,theta,psi)
        #x_axis = self.ax.xaxis
        #y_axis = self.ax.yaxis
        #z_axis = self.ax.zaxis
        if self.flag_init is True:
            poly = Poly3DCollection(faces, facecolors=['b','b','b','b','b','b','b','b','b','b','b','b','b'], alpha=.6)
            self.UAV =self.ax.add_collection3d(poly)# 
            self.ax.set_xlim([pe-10, pe+10])
            #x_axis.set_bounds(pe-10, pe+10)
            self.ax.set_ylim([pn-10, pn+10])
            #y_axis.set_bounds(pn-10, pn+10)
            self.ax.set_zlim([-pd-10, -pd+10])
            #z_axis.set_bounds(-pd-10, -pd+10)
            
            
            plt.pause(0.001)
            
        else:
            self.UAV.set_verts(faces)
            self.ax.set_xlim([pe-10, pe+10])
            #x_axis.set_bounds(pe-10, pe+10)
            self.ax.set_ylim([pn-10, pn+10])
            #y_axis.set_bounds(pn-10, pn+10)
            self.ax.set_zlim([-pd-10, -pd+10])
            #z_axis.set_bounds(-pd-10, -pd+10)
            plt.pause(0.001)
            




    def transformation_matrix(self):
        x = self.x
        y = self.y
        z = self.z
        roll = -self.roll
        pitch = -self.pitch
        yaw = self.yaw
        return np.array(
            [[cos(yaw) * cos(pitch), -sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll), x],
             [sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch)
              * sin(roll), -cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll), y],
             [-sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(yaw), z]
             ])

    def plot(self):  # pragma: no cover
        T = self.transformation_matrix()

        p1_t = np.matmul(T, self.p1)
        p2_t = np.matmul(T, self.p2)
        p3_t = np.matmul(T, self.p3)
        p4_t = np.matmul(T, self.p4)

        #plt.cla() # use handle 
        if self.flag_init is True:
            body, =self.ax.plot([p1_t[0], p2_t[0], p3_t[0], p4_t[0], p1_t[0], p3_t[0], p4_t[0], p2_t[0]],
                        [p1_t[1], p2_t[1], p3_t[1], p4_t[1], p1_t[1], p3_t[1], p4_t[1], p2_t[1]],
                        [p1_t[2], p2_t[2], p3_t[2], p4_t[2], p1_t[2], p3_t[2], p4_t[2], p2_t[2]], 'k-') # rotor
            self.handle.append(body)

            

            traj, =self.ax.plot(self.x_data, self.y_data, self.z_data, 'b:')# trajectory
            self.handle.append(traj)

            plt.xlim(-2, 2)
            plt.ylim(-2, 2)
            self.ax.set_zlim(0, 4)
            plt.xlabel('North')
            plt.ylabel('East')
            self.flag_init = False 
            plt.pause(0.001) # can be put in the main file
        else:
            self.handle[0].set_data([p1_t[0], p2_t[0], p3_t[0], p4_t[0], p1_t[0], p3_t[0], p4_t[0], p2_t[0]],
                        [p1_t[1], p2_t[1], p3_t[1], p4_t[1], p1_t[1], p3_t[1], p4_t[1], p2_t[1]])
            self.handle[0].set_3d_properties([p1_t[2], p2_t[2], p3_t[2], p4_t[2],p1_t[2], p3_t[2], p4_t[2], p2_t[2]])


            self.handle[1].set_data(self.x_data, self.y_data)
            self.handle[1].set_3d_properties(self.z_data)
            print(self.handle)
            plt.pause(0.001)


