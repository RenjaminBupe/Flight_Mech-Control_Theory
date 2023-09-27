
import sys
sys.path.append('.')# one directory up
import numpy as np
import time 
from math import cos, sin, tan
import scipy.linalg as linalg
import parameters.simulation_parameters as SIM
import matplotlib.pyplot as plt
from viewers.UAV_animation import UAV_animation
from tools.signalGenerator import signalGenerator
from tools.sliders import sliders
from dynamics.UAVDynamics_Assignment_2 import UAVdynamics

UAV = UAVdynamics(alpha=0.0)
state=np.array([[0.], [0.], [-1.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.], [0.]])
UAV_anim=UAV_animation(state, scale=5)
#my_slider=sliders()
   
fref = signalGenerator(1000., 2.)
mref = signalGenerator(30., 2)

# initialize the simulation time
sim_time = SIM.start_time

# initialize plots
fig, (forces, moments, position, velocity, angles, rates) = plt.subplots(6, 1, sharex = True)


# initialize plot lists
pn_p = [0]
pe_p = [0]
pd_p = [0]
u_p = [0]
v_p = [0]
w_p = [0]
phi_p = [0]
theta_p = [0]
psi_p = [0]
p_p = [0]
q_p = [0]
r_p = [0]
fx_p = [0]
fy_p = [0]
fz_p = [0]
l_p = [0]
m_p = [0]
n_p = [0]
st_p = [0]


# main simulation loop
print("Press Command-Q to exit...")
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

while sim_time < SIM.end_time:
    if sim_time <= 0.5:
        fx = 0
        fy = fref.sin(sim_time)
        fz = 0
        l = 0
        m = 0
        n = 0
    elif sim_time <= 1.0:
        fy = 0
        fx = fref.sin(sim_time)
    elif sim_time <= 1.5:
        fx = 0
        fz = -fref.sin(sim_time)
    elif sim_time <= 2.0:
        fz = 0
        n = mref.sin(sim_time)
    elif sim_time <= 2.5:
        n = 0
        m = mref.sin(sim_time)
    elif sim_time <= 3.0:
        m = 0
        l = mref.sin(sim_time)
    else:
        l = 0

    #append force/moment plot lists
    fx_p.append(fx)
    fy_p.append(fy)
    fz_p.append(fz)
    l_p.append(l)
    m_p.append(m)
    n_p.append(n)
    st_p.append(sim_time)

    y = UAV.update(fx, fy, fz, l, m, n)
    pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = y.flatten()
    UAV_anim.update(pn, pe, pd, phi, theta, psi) # -pd for height

    #append state plot lists
    pn_p.append(pn)
    pe_p.append(pe)
    pd_p.append(pd)
    u_p.append(u)
    v_p.append(v)
    w_p.append(w)
    phi_p.append(phi)
    theta_p.append(theta)
    psi_p.append(psi)
    p_p.append(p)
    q_p.append(q)
    r_p.append(r)
    
    # plot
    forces.clear(); moments.clear(); position.clear(); velocity.clear(); angles.clear(); rates.clear()
    forces.plot(st_p, fx_p, label='$F_x$'); forces.plot(st_p, fy_p, label='$F_y$'); forces.plot(st_p, fz_p, label='$F_z$')
    forces.legend(loc='upper right'); forces.set_title('Input Forces'); forces.grid(); forces.set_ylabel('N')
    moments.plot(st_p, l_p, label='$l$'); moments.plot(st_p, m_p, label='$m$'); moments.plot(st_p, n_p, label='$n$')
    moments.legend(loc='upper right'); moments.set_title('Input Moments'); moments.grid(); moments.set_ylabel('N*m')
    position.plot(st_p, pn_p, label='$p_n$'); position.plot(st_p, pe_p, label='$p_e$'); position.plot(st_p, pd_p, label='$p_d$')
    position.legend(loc='upper right'); position.set_title('Position'); position.grid(); position.set_ylabel('m')
    velocity.plot(st_p, u_p, label='$u$'); velocity.plot(st_p, v_p, label='$v$'); velocity.plot(st_p, w_p, label='$w$')
    velocity.legend(loc='upper right'); velocity.set_title('Velocity'); velocity.grid(); velocity.set_ylabel('m/s')
    angles.plot(st_p, phi_p, label='$/phi$'); angles.plot(st_p, theta_p, label='$/theta$'); angles.plot(st_p, psi_p, label='$/psi$')
    angles.legend(loc='upper right'); angles.set_title('Attitude Angles'); angles.grid(); angles.set_ylabel('rad')
    rates.plot(st_p, p_p, label='$p$'); rates.plot(st_p, q_p, label='$q$'); rates.plot(st_p, r_p, label='$r$')
    rates.legend(loc='upper right'); rates.set_title('Rotation Rates'); rates.grid(); rates.set_ylabel('rad/s')

    
    # -------increment time-------------d
    plt.pause(0.1)
    sim_time += SIM.ts_simulation

    



