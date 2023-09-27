
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

state=np.array([[0], [0], [-1], [0], [0], [0], [0], [0], [0], [0], [0], [0]])
UAV_anim=UAV_animation(state, scale=5)
#my_slider=sliders()
   
temp = signalGenerator(amplitude=0.5, frequency=0.1)

# initialize the simulation time
sim_time = SIM.start_time

# initialize plots
fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

# initialize plot lists
plot_t = []
plot_pn = []
plot_pe = []
plot_pd = []
plot_phi = [] 
plot_theta = []
plot_psi = []

# main simulation loop
print("Press Command-Q to exit...")
pn=state[0,0]
pe=state[1,0]
pd=state[2,0]
phi=state[6,0]
theta=state[7,0]
psi=state[8,0]
while sim_time < SIM.end_time:
    # reading from sliders
    #phi=my_slider.roll_slider.val
    #theta=my_slider.pitch_slider.val
    #psi=my_slider.yaw_slider.val
    while sim_time < 2:
        #phi=temp.sin(sim_time)
        #theta=temp.sin(sim_time)
        psi=-2*temp.sin(sim_time)
        #pn=-5*(sin**2)*(sim_time)
        pe=10*(sim_time)
        pn=-5*(sim_time)**2

        plot_t.append(sim_time)
        plot_pn.append(pn)
        plot_pe.append(pe)
        plot_pd.append(pd)
        plot_phi.append(phi)
        plot_theta.append(theta)
        plot_psi.append(psi)
        
        
        UAV_anim.update(pn, pe, pd, phi, theta, psi) # -pd for height

        # -------increment time-------------d
        sim_time += SIM.ts_simulation

    while sim_time >= 2 and sim_time < 2+(np.pi/2):
        theta=(np.pi/2)*sin((sim_time-2))

        if sim_time > 2+(np.pi/4):
            phi=4.4428*sin(sim_time-(2+np.pi/4))
    
        pe=19.8+(15*sin((sim_time-1.98)))
        pn=-19.602-(15*sin((sim_time-1.98)))
        pd=-10*(sim_time-1.98)**1.5
        
        plot_t.append(sim_time)
        plot_pn.append(pn)
        plot_pe.append(pe)
        plot_pd.append(pd)
        plot_phi.append(phi)
        plot_theta.append(theta)
        plot_psi.append(psi)

        UAV_anim.update(pn, pe, pd, phi, theta, psi) # -pd for height

        # -------increment time-------------d
        sim_time += SIM.ts_simulation

    while sim_time >= 2+(np.pi/2) and sim_time < 1.7+(np.pi):
        theta=(np.pi)*sin(-sim_time)
        
        pe=19.8+(15*cos((sim_time-3.5708)))
        pn=-19.602-(15*cos((sim_time-3.5708)))
        pd=-19.86-(15*sin(sim_time-3.5708))
        
        plot_t.append(sim_time)
        plot_pn.append(pn)
        plot_pe.append(pe)
        plot_pd.append(pd)
        plot_phi.append(phi)
        plot_theta.append(theta)
        plot_psi.append(psi)

        UAV_anim.update(pn, pe, pd, phi, theta, psi) # -pd for height

        # -------increment time-------------d
        sim_time += SIM.ts_simulation

    while sim_time >= 1.7+(np.pi) and sim_time < 8:
        
        
        pe=14.256-20*(sim_time-5.34071)
        pn=-14.058+20*(sim_time-5.34071)
        
        plot_t.append(sim_time)
        plot_pn.append(pn)
        plot_pe.append(pe)
        plot_pd.append(pd)
        plot_phi.append(phi)
        plot_theta.append(theta)
        plot_psi.append(psi)

        UAV_anim.update(pn, pe, pd, phi, theta, psi) # -pd for height

        # -------increment time-------------d
        sim_time += SIM.ts_simulation
    
    

    x1 = ax1.set_xlabel('time elapsed (s)')
    y1 = ax1.set_ylabel('distance (m)')
    l1 = ax1.plot(plot_t, plot_pn, label='pn')
    l2 = ax1.plot(plot_t, plot_pe, label='pe')
    l3 = ax1.plot(plot_t, plot_pd, label='pd')
    leg1 = ax1.legend()

    x2 = ax2.set_xlabel('time elapsed (s)')
    y2 = ax2.set_ylabel('angle (rads)')
    l4 = ax2.plot(plot_t, plot_phi, label='phi')
    l5 = ax2.plot(plot_t, plot_theta, label='theta')
    l6 = ax2.plot(plot_t, plot_psi, label='psi')
    leg2 = ax2.legend()

    

    UAV_anim.update(pn, pe, pd, phi, theta, psi) # -pd for height
    

    # -------increment time-------------d
    sim_time += SIM.ts_simulation

    time.sleep(3)



