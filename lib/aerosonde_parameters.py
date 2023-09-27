import sys
sys.path.append('..')
import numpy as np

######################################################################################
                #  Inertial Params
######################################################################################
# Inertial parameters
jx= 0.824
jy= 1.135
jz= 1.759
jxz= 0.5
gravity=9.806650
mass=13.5
# inital states
states0=np.array([[0.], #pn
                  [0.], #pe
                  [-1.], # pd
                  [1.], # u
                  [1.], # v
                  [0.], # w
                  [0.], # phi
                  [0.], # theta
                  [0.], # psi
                  [0.], # p
                  [0.], # q
                  [0.]]) # r

gamma = jx*jz - jxz**2
g1 = (jxz*(jx-jy+jz))/gamma
g2 = (jz*(jx-jy)+jxz**2)/gamma
g3 = jz/gamma
g4 = jxz/gamma
g5 = (jz-jx)/jy
g6 = (jxz/jy)
g7 = ((jx-jy)*jx+jxz**2)/gamma
g8 = jx/gamma

## aerodynamic parameters
S_wing        = 0.55
b             = 2.90
c             = 0.19
S_prop        = 0.2027
rho           = 1.2682
e             = 0.9
AR            = b**2/S_wing
C_L_0         = 0.23
C_D_0         = 0.043
C_m_0         = 0.0135
C_L_alpha     = 5.61
C_D_alpha     = 0.030
C_m_alpha     = -2.74
C_L_q         = 7.95
C_D_q         = 0.0
C_m_q         = -38.21
C_L_delta_e   = 0.13
C_D_delta_e   = 0.0135
C_m_delta_e   = -0.99
M             = 50
alpha0        = 0.47
epsilon       = 0.16
C_D_p         = 0.0
C_Y_0         = 0.0
C_ell_0       = 0.0
C_n_0         = 0.0
C_Y_beta      = -0.98
C_ell_beta    = -0.13
C_n_beta      = 0.073
C_Y_p         = 0.0
C_ell_p       = -0.51 # ell=p
C_n_p         = -0.069
C_Y_r         = 0.0
C_ell_r       = 0.25
C_n_r         = -0.095
C_Y_delta_a   = 0.075
C_ell_delta_a = 0.17
C_n_delta_a   = -0.011
C_Y_delta_r   = 0.19
C_ell_delta_r = 0.0024
C_n_delta_r   = -0.069
C_prop        = 1
k_motor       = 80 #80
k_Tp          = 0
k_omega       = 0 

######################################################################################
                #   sample times, etc
######################################################################################
ts_simulation = 0.02  # smallest time step for simulation
start_time = 0.  # start time for simulation
end_time = 10.  # end time for simulation

ts_plotting = 0.1  # refresh rate for plots

ts_video = 0.1  # write rate for video

ts_control = ts_simulation  # sample rate for the controller