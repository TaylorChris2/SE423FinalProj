import casadi as ca
import numpy as np

def EOM_kin_ca(X, dt):
    theta = X[2]
    
    A = ca.MX_eye(3)
    B = ca.vertcat(
        ca.horzcat(dt * ca.cos(theta), 0),
        ca.horzcat(dt * ca.sin(theta), 0),
        ca.horzcat(0, dt)
    )
    
    return A, B

def EOM_kin(X, dt):
    theta = X[2]

    A = np.eye(3)
    B = np.array([[dt * np.cos(theta), 0],
                  [dt * np.sin(theta), 0],
                  [0, dt]])
    
    return A, B

def EOM_dyn(X, u, dt):
    pass