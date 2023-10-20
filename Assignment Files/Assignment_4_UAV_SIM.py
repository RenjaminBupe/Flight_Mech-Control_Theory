
import sys
sys.path.append('.')# one directory up
import numpy as np
import lib.simulation_parameters as SIM
import lib.aerosonde_parameters as P
import matplotlib.pyplot as plt
from lib.UAV_animation import UAV_animation
from lib.signalGenerator import signalGenerator
from lib.UAVDynamics import UAVdynamics
import lib.UAV_Aero as UAV_Aero
import lib.Wind as Wind
import lib.trim as trim
import keyboard
# Filter out the specific warning
import warnings; warnings.filterwarnings("ignore", category=UserWarning, module="control.matlab")



state= P.states0
UAV = UAVdynamics(SIM.ts_simulation, state)
UAV_anim=UAV_animation(state, scale=5)
Aero = UAV_Aero.Aero()
Trim = trim.ComputeTrim()
Vsteady = np.array([[0], [0], [0]])
wind = Wind.wind(Vsteady)
# initial values
d_a = 0
d_e = 0
d_r = 0
d_t = 1
Va = 35


# initialize the simulation time
sim_time = SIM.start_time

# initialize plots
forces = UAV_anim.fig.add_subplot(531)
for_p = forces.get_position(); for_p.x0 -= 0.075; for_p.x1 -= 0.05; forces.set_position(for_p)
moments = UAV_anim.fig.add_subplot(537)
mom_p = moments.get_position(); mom_p.x0 -= 0.075; mom_p.x1 -= 0.05; moments.set_position(mom_p)
position = UAV_anim.fig.add_subplot(5,3,13)
pos_p = position.get_position(); pos_p.x0 -= 0.075; pos_p.x1 -= 0.05; position.set_position(pos_p)
deflection = UAV_anim.fig.add_subplot(5,3,14)
def_p = deflection.get_position(); def_p.x0 -= 0.0; def_p.x1 -= 0.0; deflection.set_position(def_p)
throttle = UAV_anim.fig.add_subplot(20,3,2)
thr_p = throttle.get_position(); thr_p.x0 -= 0.0; thr_p.x1 -= 0.0; throttle.set_position(thr_p)
velocity = UAV_anim.fig.add_subplot(533)
vel_p = velocity.get_position(); vel_p.x0 += 0.075; vel_p.x1 += 0.05; velocity.set_position(vel_p)
angles = UAV_anim.fig.add_subplot(539)
ang_p = angles.get_position(); ang_p.x0 += 0.075; ang_p.x1 += 0.05; angles.set_position(ang_p)
rates = UAV_anim.fig.add_subplot(5,3,15)
rat_p = rates.get_position(); rat_p.x0 += 0.075; rat_p.x1 += 0.05; rates.set_position(rat_p)

#fig, (forces, moments, position, velocity, angles, rates) = plt.subplots(6, 1, sharex = True)


# initialize plot lists     
pn_p = np.array([0])
pe_p = np.array([0])
pd_p = np.array([0])
u_p = np.array([0])
v_p = np.array([0])
w_p = np.array([0])
phi_p = np.array([0])
theta_p = np.array([0])
psi_p = np.array([0])
p_p = np.array([0])
q_p = np.array([0])
r_p = np.array([0])
fx_p = np.array([0])
fy_p = np.array([0])
fz_p = np.array([0])
l_p = np.array([0])
m_p = np.array([0])
n_p = np.array([0])
d_ep = np.array([0])
d_ap = np.array([0])
d_rp = np.array([0])
st_p = np.array([0])


# main simulation loop
print("Press Command-Q to exit...")
#pn=state[0][0]
#pe=state[1][0]
#pd=state[2][0]
#u = state[3][0]
#v = state[4][0]
#w = state[5][0]
#phi=state[6][0]
#theta=state[7][0]
#psi=state[8][0]
#p = state[9][0]
#q = state[10][0]
#r = state[11][0]

#Va_ = 35; Y = np.deg2rad(7); R = np.inf

# Trim paramters
##  Param set 1  ##
Va_ = 35; Y = 0; R = np.inf
##  Param set 2  ##
#Va_ = 35; Y = 0; R = 200 #R = -200
##  Param set 3  ##
#Va_ = 35; Y = np.deg2rad(10); R = 100



x_trim, u_trim = Trim.compute_trim(Va_, Y, R)
d_e, d_t, d_a, d_r = u_trim.flatten()
pn = -100
pe = 0
pd = 0
u = x_trim.item(3)
v = x_trim.item(4)
w = x_trim.item(5)
phi = x_trim.item(6)
theta = x_trim.item(7)
psi = x_trim.item(8)
p = x_trim.item(9)
q = x_trim.item(10)
r = x_trim.item(11)

print("--- Trim Conditions ---")
print(f"E: {np.rad2deg(d_e):.2f} deg")
print(f"T: {d_t*100:.2f} %")
print(f"A: {np.rad2deg(d_a):.2f} deg")
print(f"R: {np.rad2deg(d_r):.2f} deg")

states = np.array([pn, pe, pd, u, v, w, phi, theta, psi, p, q, r])
state0 = np.array([[pn],[pe],[pd],[u],[v],[w],[phi],[theta],[psi],[p],[q],[r]])
UAV.state = np.ndarray.copy(state0)



while sim_time < SIM.end_time:
    
    # update values/state
    Va, alpha, beta = wind.wind_char(state, Va, sim_time)
    #x_trim, u_trim = Trim.compute_trim(Va_, Y, R, alpha, beta)
    d_e, d_t, d_a, d_r = u_trim.flatten()
    fx, fy, fz = Aero.forces(state, d_e, d_a, d_r, d_t, alpha, beta, Va)
    l, m, n = Aero.moments(state, d_e, d_a, d_r, d_t, alpha, beta, Va)  
    state = UAV.update(fx, fy, fz, l, m, n)
    pn, pe, pd, u, v, w, phi, theta, psi, p, q, r = state.flatten()
    UAV_anim.update(pn, pe, pd, phi, theta, psi) # -pd for height

    #append force/moment plot lists
    fx_p = np.append(fx_p, fx)
    fy_p = np.append(fy_p, fy)
    fz_p = np.append(fz_p, fz)
    l_p = np.append(l_p, l)
    m_p = np.append(m_p, m)
    n_p = np.append(n_p, n)
    st_p = np.append(st_p, sim_time)

    #append state plot lists
    pn_p = np.append(pn_p, pn)
    pe_p = np.append(pe_p, pe)
    pd_p = np.append(pd_p, pd)
    u_p = np.append(u_p, u)
    v_p = np.append(v_p, v)
    w_p = np.append(w_p, w)
    phi_p = np.append(phi_p, phi)
    theta_p = np.append(theta_p, theta)
    psi_p = np.append(psi_p, psi)
    p_p = np.append(p_p, p)
    q_p = np.append(q_p, q)
    r_p = np.append(r_p, r)
    d_ep = np.append(d_ep, d_e)
    d_ap = np.append(d_ap, d_a)
    d_rp = np.append(d_rp, d_r)
    
    # plot
    forces.clear(); moments.clear(); position.clear(); deflection.clear(); throttle.clear(); velocity.clear(); angles.clear(); rates.clear()
    forces.plot(st_p, fx_p, label='$F_x$'); forces.plot(st_p, fy_p, label='$F_y$'); forces.plot(st_p, fz_p, label='$F_z$')
    forces.legend(loc='upper right'); forces.set_title('Input Forces'); forces.grid(); forces.set_ylabel('N')
    moments.plot(st_p, l_p, label='$l$'); moments.plot(st_p, m_p, label='$m$'); moments.plot(st_p, n_p, label='$n$')
    moments.legend(loc='upper right'); moments.set_title('Input Moments'); moments.grid(); moments.set_ylabel('N*m')
    position.plot(st_p, pn_p, label='$p_n$'); position.plot(st_p, pe_p, label='$p_e$'); position.plot(st_p, pd_p, label='$p_d$')
    position.legend(loc='upper right'); position.set_title('Position'); position.grid(); position.set_ylabel('m')
    deflection.plot(st_p, d_ep, label='$d_e$'); deflection.plot(st_p, d_ap, label='$d_a$'); deflection.plot(st_p, d_rp, label='$d_r$')
    deflection.legend(loc='upper right'); deflection.set_title('Deflection'); deflection.grid(); deflection.set_ylabel('rads')
    throttle.bar(0, d_t); throttle.set_ylim(0, 1); throttle.set_xticklabels("")
    throttle.set_xticks([]); throttle.set_title("Throttle")
    velocity.plot(st_p, u_p, label='$u$'); velocity.plot(st_p, v_p, label='$v$'); velocity.plot(st_p, w_p, label='$w$')
    velocity.legend(loc='upper right'); velocity.set_title('Velocity'); velocity.grid(); velocity.set_ylabel('m/s')
    angles.plot(st_p, phi_p, label='$/phi$'); angles.plot(st_p, theta_p, label='$/theta$'); angles.plot(st_p, psi_p, label='$/psi$')
    angles.legend(loc='upper right'); angles.set_title('Attitude Angles'); angles.grid(); angles.set_ylabel('rad')
    rates.plot(st_p, p_p, label='$p$'); rates.plot(st_p, q_p, label='$q$'); rates.plot(st_p, r_p, label='$r$')
    rates.legend(loc='upper right'); rates.set_title('Rotation Rates'); rates.grid(); rates.set_ylabel('rad/s')
    
    
    if keyboard.is_pressed('q'): break
    plt.pause(0.1)
    sim_time += SIM.ts_simulation