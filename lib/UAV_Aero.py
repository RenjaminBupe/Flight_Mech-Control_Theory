import sys
sys.path.append('.')# one directory up
import numpy as np
import time 
import scipy.linalg as linalg
from lib.aerosonde_parameters import *
import lib.aerosonde_parameters as P
import matplotlib.pyplot as plt
from lib.UAV_animation import UAV_animation
from lib.signalGenerator import signalGenerator
from lib.sliders import sliders
from lib.UAVDynamics_Assignment_3 import UAVdynamics

class Aero():

        

    def forces(self, state, d_e, d_a, d_r, d_t, alpha, beta, Va):
        """
        Calculates forces, returns vector [fx, fy, fz]^T
        """
        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = state.flatten()

        gravitational = np.array([[-mass*gravity*np.sin(theta)],
                        [mass*gravity*np.cos(theta)*np.sin(phi)],
                        [mass*gravity*np.cos(theta)*np.cos(phi)]])

        aerodynamic = 0.5*rho*(Va**2)*S_wing*np.array([[self.C_x(alpha) + self.C_xq(alpha)*(c/(2*Va))*q + self.C_xde(alpha)*d_e],
                        [C_Y_0 + C_Y_beta*beta + C_Y_p*(b/(2*Va))*p + C_Y_r*(b/(2*Va))*r + C_Y_delta_a*d_a + C_Y_delta_r*d_r],
                        [self.C_z(alpha) + self.C_zq(alpha)*(c/(2*Va))*q + self.C_zde(alpha)*d_e]])
        
        propulsion = 0.5*rho*S_prop*C_prop*np.array([[(k_motor*d_t)**2 - Va**2],
                                                             [0],
                                                             [0]])
        
        force = gravitational + aerodynamic + propulsion 
        fx, fy, fz = force.flatten()

        return fx, fy, fz

    def moments(self, state, d_e, d_a, d_r, d_t, alpha, beta, Va):

        pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = state.flatten()

        aerodynamic = 0.5*rho*(Va**2)*S_wing*np.array([[b*(C_ell_0 + C_ell_beta*beta + C_ell_p*(b/(2*Va))*p + C_ell_r*(b/(2*Va))*r +\
                                                            C_ell_delta_a*d_a + C_ell_delta_r*d_r)],
                                                       [c*(C_m_0 + C_m_alpha*alpha + C_m_q*(c/(2*Va))*q + C_m_delta_e*d_e)],
                                                       [b*(C_n_0 + C_n_beta*beta + C_n_p*(b/(2*Va))*p + C_n_r*(b/(2*Va))*r +\
                                                            C_n_delta_a*d_a + C_n_delta_r*d_r)]])
        
        propeller = np.array([[-k_Tp*((k_omega*d_t)**2)],
                              [0],
                              [0]])
        
        moment = aerodynamic + propeller
        l, m, n = moment.flatten()

        return l, m, n

    @staticmethod
    def sigma(alpha):
        eq1 = np.exp(-M*(alpha - alpha0))
        eq2 = np.exp(M*(alpha - alpha0))
        return (1 + eq1 + eq2)/((1 + eq1)*(1 + eq2))
    
    # Coefficient fucntions #
    def C_l(self, alpha):
        return C_L_0 + C_L_alpha*alpha
    
    def C_d(self, alpha): 
        return C_D_0 + C_D_alpha*alpha
   
    def C_x(self, alpha):
        return -self.C_d(alpha)*np.cos(alpha) + self.C_l(alpha)*np.sin(alpha)
    
    def C_xq(self, alpha):
        return -C_D_q*np.cos(alpha) + C_L_q*np.sin(alpha)
   
    def C_xde(self, alpha):
        return -C_D_delta_e*np.cos(alpha) + C_L_delta_e*np.sin(alpha)
    
    def C_z(self, alpha):
        return -self.C_d(alpha)*np.sin(alpha) - self.C_l(alpha)*np.cos(alpha)
    
    def C_zq(self, alpha):
        return -C_D_q*np.sin(alpha) - C_L_q*np.cos(alpha)
    
    def C_zde(self, alpha):
        return - C_D_delta_e*np.sin(alpha) - C_L_delta_e*np.cos(alpha)
    
