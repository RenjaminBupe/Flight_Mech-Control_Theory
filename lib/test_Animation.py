import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from lib.rotations import Euler2Rotation


class animation():
    """
    Base class for 3D animations in MPL.
    
    Methods
    -------
    rotmove(verts, n, e, d, phi, theta, psi)
        Rotates and translates object of given vertices to specified location and euler angles.
    update(verts, n, e, d, phi, theta, psi, obj, facecolors)
        Updates position and rotation of object and draws it.
    drawObj(verts, n, e, d, phi, theta, psi, obj, facecolors)
        Draws the object to 3D plot.
    """
    verts = np.array([[2.8, 0, 0],       #Nose Vertex (1)
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
    
    obj = np.array([[verts[0], verts[1], verts[2]],
                     [verts[0], verts[1], verts[4]],
                     [verts[0], verts[2], verts[3]],
                     [verts[1], verts[3], verts[4]],
                     [verts[2], verts[3], verts[5]],
                     [verts[1], verts[2], verts[5]],
                     [verts[1], verts[4], verts[5]],
                     [verts[3], verts[4], verts[5]],
                     [verts[6], verts[7], verts[8]],
                     [verts[6], verts[8], verts[9]],
                     [verts[10], verts[11], verts[12]],
                     [verts[10], verts[12], verts[13]],
                     [verts[5], verts[14], verts[15]]
                     ])
    faces = ['b']

    def __init__(self, limits=10, alpha=0.6, flag=False): 
        # start plot
        self.fig = plt.figure(1)
        if flag == True:
            self.ax = self.fig.add_subplot(1, 2, 1, projection="3d")
        else:
            self.ax = self.fig.add_subplot(projection="3d")
        self.ax.set_xlim([-limits, limits])
        self.ax.set_ylim([-limits, limits])
        self.ax.set_zlim([-limits, limits])
        self.lim = limits
        self.ax.set_xlabel('East (m)')
        self.ax.set_ylabel('North (m)')
        self.ax.set_zlabel('Height (m)')
        self.alpha = alpha

        self.verts = np.array([[2.8, 0, 0],       #Nose Vertex (1)
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
        
        # set init flag
        self.flag_init = True


    def rotmove(self, verts, n, e, d, phi, theta, psi):
        """
        Applies rotation and translation to object of given vertices.
        
        Parameters
        ----------
        verts : np.ndarray
            Array of vertices of object to animate.
        n : float
            North position.
        e : float
            East position.
        d : float
            Downwards position.
        phi : float
            Pitch euler angle.
        theta : float
            Roll euler angle.
        psi : float
            Yaw euler angle.
        
        Returns
        -------
        newverts : np.ndarray
            Array of rotated and translated vertices.
        """
        # position matrix
        pos = np.array([n, e, d])
        posm = np.tile(pos.copy(), (np.shape(verts)[0], 1))
        
        # rotation matrix and rotate then translate verts
        R = Euler2Rotation(phi, theta, psi)
        vertsRot = np.matmul(R, verts.T).T

        vertsTrans = vertsRot + posm
        
        # rotation matrix for plotting and rotate
        R_plot = np.array([[0, 1, 0],
                           [1, 0, 0],
                           [0, 0, -1]])
        newverts = np.matmul(R_plot, vertsTrans.T).T
        
        return newverts
    
    
    def update(self, verts, n, e, d, phi, theta, psi, obj=None, facecolors=[]):
        """
        Updates position and rotation of object and draws it.
        
        Parameters
        ----------
        verts : np.ndarray
            Array of vertices of object to animate.
        obj : function
            Function that defines faces based on input vertices.
        n : float
            North position.
        e : float
            East position.
        d : float
            Downwards position.
        phi : float
            Pitch euler angle.
        theta : float
            Roll euler angle.
        psi : float
            Yaw euler angle.
        facecolors : list
            Optional list of face colors.
        """
        # draw obj
        self.drawObj(verts, n, e, d, phi, theta, psi, obj=obj, facecolors=facecolors)
        
        # set init flag
        if self.flag_init == True:
            self.flag_init = False
        
        
    def drawObj(self, verts, n, e, d, phi, theta, psi, obj=None, facecolors=[]):
        """
        Draws object and its faces.
        
        Parameters
        ----------
        verts : np.ndarray
            Array of vertices of object to animate.
        n : float
            North position.
        e : float
            East position.
        d : float
            Downwards position.
        phi : float
            Pitch euler angle.
        theta : float
            Roll euler angle.
        psi : float
            Yaw euler angle.
        obj : func
            Function that defines faces based on input vertices.
        facecolors : list
            Optional list of face colors.
        """
        # check for type of input
        if obj == None:
            # reshape to just vertices
            verts = np.reshape(verts.copy(), (-1, 3))
            # update positions
            objverts = self.rotmove(verts, n, e, d, phi, theta, psi)
            # get back to faces
            faces = np.reshape(objverts.copy(), (-1, 3, 3))
        else:
            # update position and get faces
            objverts = self.rotmove(verts, n, e, d, phi, theta, psi)
            faces = obj(objverts)
        
        # collect polys if first time
        if self.flag_init is True:
            poly = Poly3DCollection(faces, facecolors=facecolors, alpha=self.alpha)
            self.obj = self.ax.add_collection3d(poly)
            plt.pause(0.001)
            self.ax.set_xlim([e-self.lim, e+self.lim])
            self.ax.set_ylim([n-self.lim, n+self.lim])
            self.ax.set_zlim([-d-self.lim, -d+self.lim])
        # otherwise update vert location
        else:
            self.obj.set_verts(faces)
            plt.pause(0.001)
            self.ax.set_xlim([e-self.lim, e+self.lim])
            self.ax.set_ylim([n-self.lim, n+self.lim])
            self.ax.set_zlim([-d-self.lim, -d+self.lim])