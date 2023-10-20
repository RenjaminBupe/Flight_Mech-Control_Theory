import sys
sys.path.append('.')
sys.path.append('/Users/benru/OneDrive/Documents/Fall 2023/Control Systems/A1 Sim/parameters')
import numpy as np 
import lib.aerosonde_parameters as P
from lib.rotations import Euler2Rotation

class UAVdynamics:
    def __init__(self, Ts, state0):
        # Initial state conditions
        self.state = P.states0

        # simulation time step
        self.Ts = P.ts_simulation
        # Mass of the center, kg
        self.m = P.mass
        # , kg
        self.jx = P.jx
        # 
        self.jy = P.jy
        # 
        self.jz = P.jz
        #
        self.jxz = P.jxz

        self.g = self.jx*self.jz - self.jxz**2 # gamma
        self.g1 = (self.jxz*(self.jx - self.jy + self.jz))/self.g # gamma 1
        self.g2 = (self.jz*(self.jz - self.jy) + self.jxz**2)/self.g # gamma 2
        self.g3 = self.jz/self.g # gamma 3
        self.g4 = self.jxz/self.g # gamma 4
        self.g5 = (self.jz - self.jx)/self.jy # gamma 5
        self.g6 = self.jxz/self.jy # gamma 6
        self.g7 = (self.jx*(self.jx - self.jy) + self.jxz**2) # gamma 7
        self.g8 = self.jx/self.g # gamma 8

        

    def update(self, fx, fy, fz, l, m, n):

        self.rk4_step(fx, fy, fz, l, m, n)  # propagate the state by one time sample
        y = self.h()  # return the corresponding output
        return y
    
    def f(self, state, fx, fy, fz, l, m, n):
        # Return xdot = f(x,u)
        pn=state[0][0]
        pe=state[1][0]
        pd=state[2][0]

        u = state[3][0]
        v = state[4][0]
        w = state[5][0]

        phi=state[6][0]
        theta=state[7][0]
        psi=state[8][0]

        p = state[9][0]
        q = state[10][0]
        r = state[11][0]
        


        # The equations of motion.
       # pndot = np.cos(theta)*np.cos(psi)*u + (np.sin(phi)*np.sin(theta)*np.cos(psi) - np.cos(phi)*np.sin(psi))*v + (np.cos(phi)*np.sin(theta)*np.cos(psi) + np.sin(phi)*np.sin(psi))*w
       # pedot = np.cos(theta)*np.cos(psi)*u + (np.sin(phi)*np.sin(theta)*np.sin(psi) + np.cos(phi)*np.cos(psi))*v + (np.cos(phi)*np.sin(theta)*np.sin(psi) - np.sin(phi)*np.cos(psi))*w
       # pddot = -np.sin(theta)*u + np.sin(phi)*np.cos(theta)*v + np.cos(phi)*np.cos(theta)*w

       # udot = r*v - q*w + (1/self.m)*fx
       # vdot = p*w - r*u + (1/self.m)*fy
       # wdot = q*u - p*v + (1/self.m)*fz

       # phidot = p + np.sin(phi)*np.tan(theta)*q + np.sin(phi)*np.tan(theta)*q + np.cos(phi)*np.tan(theta)*r
       # thetadot = np.cos(phi)*q - np.sin(phi)*r
       # psidot = (np.sin(phi)/np.cos(theta))*q + (np.cos(phi)/np.cos(theta))*r

       # pdot = P.g1*p*q - P.g2*q*r + P.g3*l + P.g4*n
       # qdot = P.g5*p*r - P.g6*((p**2)-(r**2)) + m/self.jy
       # rdot = P.g7*p*q - P.g1*q*r + P.g4*l + P.g8*n

        # build xdot and return
       # xdot = np.array([[pndot], [pedot], [pddot], [udot], [vdot], [wdot], [phidot], [thetadot], [psidot], [pdot], [qdot], [rdot]])
       # return xdot
        
        eq_1a = np.array([[np.cos(theta)*np.cos(psi), np.sin(phi)*np.sin(theta)*np.cos(phi) - np.cos(phi)*np.sin(psi), np.cos(phi)*np.sin(theta)*np.cos(psi) + np.sin(phi)*np.sin(psi)],
                         [np.cos(theta)*np.sin(psi), np.sin(phi)*np.sin(theta)*np.sin(psi) + np.cos(phi)*np.cos(psi), np.cos(phi)*np.sin(theta)*np.sin(psi) - np.sin(phi)*np.cos(psi)],
                         [-np.sin(theta), np.sin(phi)*np.cos(theta), np.cos(phi)*np.cos(theta)]])
        eq_1b = np.array([[u], [v], [w]])
        eq_1 = np.dot(eq_1a, eq_1b)
        eq_2 = np.array([[r*v - q*w], [p*w - r*u], [q*u - p*v]]) + (1/self.m)*np.array([[fx], [fy], [fz]])
        eq_3a = np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],
                         [0, np.cos(phi), -np.sin(phi)],
                         [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])
        eq_3b = np.array([[p], [q], [r]])

        eq_3 = np.dot(eq_3a, eq_3b)
        eq_4 = np.array([[self.g1*p*q - self.g2*q*r], [self.g5*p*r - self.g6*(p**2 - r**2)],
            [self.g7*p*q - self.g1*q*r]]) + np.array([[self.g3*l + self.g4*n], [m/self.jy],
            [self.g4*l + self.g8*n]])
            
        # build and return output
        xdot = np.array([[eq_1[0][0]], [eq_1[1][0]], [eq_1[2][0]], [eq_2[0][0]], [eq_2[1][0]], [eq_2[2][0]],
                         [eq_3[0][0]], [eq_3[1][0]], [eq_3[2][0]], [eq_4[0][0]], [eq_4[1][0]], [eq_4[2][0]]])
        return xdot

    def h(self):
        # return y = h(x)
        pn=self.state[0][0]
        pe=self.state[1][0]
        pd=self.state[2][0]
        u=self.state[3][0]
        v=self.state[4][0]
        w=self.state[5][0]
        phi=self.state[6][0]
        theta=self.state[7][0]
        psi=self.state[8][0]
        p=self.state[9][0]
        q=self.state[10][0]
        r=self.state[11][0]

        y = np.array([[pn], [pe], [pd], [u], [v], [w], [phi], [theta], [psi], [p], [q], [r]])
        return y

    def rk4_step(self, fx, fy, fz, l, m, n):
        # Integrate ODE using Runge-Kutta RK4 algorithm
        F1 = self.f(self.state, fx, fy, fz, l, m, n)
        F2 = self.f(self.state + self.Ts/2*F1, fx, fy, fz, l, m, n)
        F3 = self.f(self.state + self.Ts/2*F2, fx, fy, fz, l, m, n)
        F4 = self.f(self.state + self.Ts*F3, fx, fy, fz, l, m, n)
        self.state += self.Ts/6*(F1 + 2*F2 + 2*F3 + F4)

    
#def saturate(u, limit):
    #if abs(u) > limit:
        #u = limit*np.sign(u)
    #return u