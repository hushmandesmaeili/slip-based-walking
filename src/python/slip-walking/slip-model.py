from math import exp, sin, cos, sqrt

# Initial state
x0, y0, z0 = 0, 0, 1.7
xdot0, ydot0, zdot0 = 0, 0, 0
ang_pos0, ang_vel0 = 0, 0

# Model parameters
g = 9.81
k = 1
m = 1

omega = sqrt(k/m)

def stance_state(t, T, p0, pT, r0, rT, ang_acc):
    p0_x, p0_y = p0
    pT_x, pT_y = pT

    # Calculate horizontal state
    x, xdot, xddot = stance_state_horz(t, T, p0_x, pT_x)
    y, ydot, yddot = stance_state_horz(t, T, p0_y, pT_y)

    # Calculate vertical state
    z, zdot, zddot = stance_state_vert(t, T, r0, rT)

    # Calculate rotational state
    ang_pos, ang_vel, ang_acc = stance_state_rot(t, T, ang_acc)

    c = [x, y, z]
    cdot = [xdot, ydot, zdot]
    
    return c, cdot, ang_pos, ang_vel


def flight_state(t, T):
    x = x0 + xdot0*t
    y = y0 + ydot0*t
    z = z0 + zdot0*t - 0.5*g*t**2
    c = [x, y, z]

    xdot = xdot0
    ydot = ydot0
    zdot = zdot0 - g*t
    cdot = [xdot, ydot, zdot]

    ang_pos = ang_pos0 + ang_vel0*t

    return c, cdot, ang_pos, ang_vel0


def stance_state_horz(t, T, p0, pT):
    h = z0

    alpha = sqrt(g/h)
    
    pt = p0 + (pT - p0) * t/T

    beta1 = (x0 - p0)/2 + (xdot0*T - (pT - p0))/(2*alpha*T)
    beta2 = (x0 - p0)/2 - (xdot0*T - (pT - p0))/(2*alpha*T)

    beta1_exp = beta1 * exp(alpha*t)
    beta2_exp = beta2 * exp(-alpha*t)

    # Return position, velocity, and acceleration 
    #(could be for x or y direction)
    x = beta1_exp + beta2_exp + pt
    xdot = alpha*beta1_exp - alpha*beta2_exp + (pT - p0)/T
    xddot = alpha**2*beta1_exp + alpha**2*beta2_exp

    return x, xdot, xddot
    

def stance_state_vert(t, T, r0, rT):
    rt = r0 + (rT - r0) * t/T

    d1 = z0 - r0 + g/(omega**2)
    d2 = zdot0/omega - (rT - r0)/(T*omega)

    d_1_exp = d1 * cos(omega*t)
    d_2_exp = d2 * sin(omega*t)

    z = d_1_exp + d_2_exp + rt - g/(omega**2)
    zdot = -omega*d1*sin(omega*t) + omega*d2*cos(omega*t) + (rT - r0)/T
    zddot = -omega**2*d_1_exp - omega**2*d_2_exp

    return z, zdot, zddot


def stance_state_rot(t, T, ang_acc):
    ang_pos = ang_pos0 + ang_vel0*t + 0.5*ang_acc*t**2
    ang_vel = ang_vel0 + ang_acc*t

    return ang_pos, ang_vel, ang_acc