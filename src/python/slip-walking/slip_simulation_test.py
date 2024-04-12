import numpy as np
from slip_model import stance_state_horz, stance_state_vert 
from com_traj_plotter import COMTrajectoryPlotter

T_1, p_0_1_x, p_0_1_y, p_T_1_x, p_T_1_y, r_0_1, r_T_1, angular_accel_1 = 0.3, 0.3, 0, 0.70, 0, 1.75, 1.75, 0
T_2, p_T_2_x, p_T_2_y, r_T_2, angular_accel_2 = 0.3, 0.9, 0, 20.3, 0
T_3 = 0.0
T_4, p_0_4_x, p_0_4_y, p_T_4_x, p_T_4_y, r_0_4, r_T_4, angular_accel_4 = 0.3, 1.4, 0, 1.8, 0, 20.3, 20.3, 0
T_5, p_T_5_x, p_T_5_y, r_T_5, angular_accel_5 = 0.3, 2.0, 0, 10.3, 0
T_6 = 0.0

c0 = [0.2, 0.0, 1.7]
cdot_0 = [1.5, 0.0, 1.5]
cddot_0 = [0.0, 0.0, -9.81]

ang_pos0, ang_vel0, ang_acc0 = 0.0, 0.0, 0.0

p0 = [p_0_1_x, p_0_1_y]
r0 = r_0_1


def simulate_stance_phase(c0, cdot0, cddot0, ang_pos0, ang_vel0, ang_acc0, T, p0, r0, pT, rT, angular_accel):
    traj_x = []
    traj_y = []
    traj_z = []

    for t in np.arange(0, T, 0.02):
        x, xdot, xddot = stance_state_horz(t, c0[0], cdot0[0], c0[2], cdot0[2], T, p0[0], pT[0])
        y, ydot, yddot = stance_state_horz(t, c0[1], cdot0[1], c0[2], cdot0[2], T, p0[1], pT[1])
        z, zdot, zddot = stance_state_vert(t, c0[2], cdot0[2], T, r0, rT)
        traj_x.append(x)
        traj_y.append(y)
        traj_z.append(z)

    return traj_x, traj_y, traj_z

# Main code
if __name__ == "__main__":
    traj_x_1, traj_y_1, traj_z_1 = simulate_stance_phase(c0, cdot_0, cddot_0, ang_pos0, ang_vel0, ang_acc0, T_1, [p_0_1_x, p_0_1_y], r0, [p_T_1_x, p_T_1_y], r_T_1, angular_accel_1)
    
    # Plot the trajectory
    plotter = COMTrajectoryPlotter()
    plotter.plot_3d(traj_x_1, traj_y_1, traj_z_1, p_0_1_x, p_0_1_y, p_T_1_x, p_T_1_y)