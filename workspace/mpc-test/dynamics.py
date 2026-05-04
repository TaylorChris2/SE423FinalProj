import casadi as ca
import numpy as np

def EOM_kin_ca(X, dt, r):
    theta = X[2]
    
    A = ca.MX_eye(3)
    B = ca.vertcat(
        ca.horzcat(0.5 * dt * ca.cos(theta), 0.5 * dt * ca.cos(theta)),
        ca.horzcat(0.5 * dt * ca.sin(theta), 0.5 * dt * ca.sin(theta)),
        ca.horzcat(-0.5 * dt / r, 0.5 * dt / r)
    )
    
    return A, B

def EOM_kin(X, dt, r):
    theta = X[2]

    A = np.eye(3)
    B = np.array([[0.5 * dt * np.cos(theta), 0.5 * dt * np.cos(theta)],
                  [0.5 * dt * np.sin(theta), 0.5 * dt * np.sin(theta)],
                  [-0.5 * dt / r, 0.5 * dt / r]])
    
    return A, B

def EOM_dyn(X, u, dt):
    pass