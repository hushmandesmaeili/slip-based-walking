from math import exp, sin, cos, sqrt

# Model parameters
g = 9.81
k = 1
m = 1

omega = sqrt(k/m)

def stance_state(t, c0, cdot0, ang_pos0, ang_vel0, T, p0, pT, r0, rT, ang_acc):
    """
    Calculate the stance state of the slip model at a given time. Assumes initial time is 0.

    Parameters:
    t (float): The current time.
    c0 (list): The initial position of the center of mass [x0, y0, z0].
    cdot0 (list): The initial velocity of the center of mass [xdot0, ydot0, zdot0].
    ang_pos0 (float): The initial angular position.
    ang_vel0 (float): The initial angular velocity.
    T (float): The duration of the stance phase.
    p0 (list): The initial position of the COP [p0_x, p0_y].
    pT (list): The final position of the COP [pT_x, pT_y].
    r0 (float): The initial rest length.
    rT (float): The final rest length.
    ang_acc (float): The angular acceleration.

    Returns:
    tuple: A tuple containing the center of mass position (c), center of mass velocity (cdot),
        angular position (ang_pos), and angular velocity (ang_vel).
    """
    x0, y0, z0 = c0
    xdot0, ydot0, zdot0 = cdot0

    p0_x, p0_y = p0
    pT_x, pT_y = pT

    # Calculate horizontal state
    x, xdot, xddot = stance_state_horz(t, x0, xdot0, z0, zdot0, T, p0_x, pT_x)
    y, ydot, yddot = stance_state_horz(t, y0, ydot0, z0, zdot0, T, p0_y, pT_y)

    # Calculate vertical state
    z, zdot, zddot = stance_state_vert(t, z0, zdot0, T, r0, rT)

    # Calculate rotational state
    ang_pos, ang_vel, ang_acc = stance_state_rot(t, ang_pos0, ang_vel0, T, ang_acc)

    c = [x, y, z]
    cdot = [xdot, ydot, zdot]
    
    return c, cdot, ang_pos, ang_vel


def flight_state(t, c0, cdot0, ang_pos0, ang_vel0, T):
    x0, y0, z0 = c0
    xdot0, ydot0, zdot0 = cdot0

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


def stance_state_horz(t, x0, xdot0, z0, zdot0, T, p0, pT):
    h = z0

    alpha = sqrt(g/h)
    
    pt = p0 + (pT - p0) * t/T

    beta1 = (x0 - p0)/2 + (xdot0*T - (pT - p0))/(2*alpha*T)
    beta2 = (x0 - p0)/2 - (xdot0*T - (pT - p0))/(2*alpha*T)

    beta1_exp = beta1 * exp(alpha*t)
    beta2_exp = beta2 * exp(-alpha*t)

    x = beta1_exp + beta2_exp + pt
    xdot = alpha*beta1_exp - alpha*beta2_exp + (pT - p0)/T
    xddot = alpha**2*beta1_exp + alpha**2*beta2_exp

    return x, xdot, xddot
    

def stance_state_vert(t, z0, zdot0, T, r0, rT):
    rt = r0 + (rT - r0) * t/T

    d1 = z0 - r0 + g/(omega**2)
    d2 = zdot0/omega - (rT - r0)/(T*omega)

    d_1_exp = d1 * cos(omega*t)
    d_2_exp = d2 * sin(omega*t)

    z = d_1_exp + d_2_exp + rt - g/(omega**2)
    zdot = -omega*d1*sin(omega*t) + omega*d2*cos(omega*t) + (rT - r0)/T
    zddot = -omega**2*d_1_exp - omega**2*d_2_exp

    return z, zdot, zddot


def stance_state_rot(t, ang_pos0, ang_vel0, T, ang_acc):
    ang_pos = ang_pos0 + ang_vel0*t + 0.5*ang_acc*t**2
    ang_vel = ang_vel0 + ang_acc*t

    return ang_pos, ang_vel, ang_acc