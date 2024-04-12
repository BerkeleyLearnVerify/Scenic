import numpy as np
from scipy import linalg

def extract_AB(speed, dt, wheelbase=2.995):
    # if state is true, doing lane keeping
    A = np.array([[1.0, 0.0, dt*speed*5./18.],[0.0, 1.0, 0.0],[0.0, 0.0, 1.0]])
    B = np.array([[0.0], [0.0], [(dt*speed*5./18.)/wheelbase]])
    return A, B

def discrete_LQR(A, B, Q, R, c):
    nX, nU = np.shape(B)[0], np.shape(B)[1]

    A_new = np.concatenate((A, c[:, np.newaxis]), axis=1)
    A_new = np.concatenate((A_new, np.concatenate((np.zeros((1, nX)), np.ones((1, 1))), axis=1)), axis=0)

    B_new = np.concatenate((B, np.zeros((1, nU))), axis=0)

    Q_new = linalg.block_diag(Q, np.array([[0.]]))

    K_current = np.zeros((nU, nX + 1))
    P_current = Q_new
    flag = True

    while flag:
        M1 = R + B_new.T.dot(P_current).dot(B_new)
        K_new = -1 * np.linalg.inv(M1).dot(B_new.T).dot(P_current).dot(A_new)
        M2 = A_new + B_new.dot(K_new)
        P_new = Q_new + K_new.T.dot(R).dot(K_new) + M2.T.dot(P_current).dot(M2)
        if (np.linalg.norm(K_new - K_current, 2) <= 1e-4):
            flag = False
        else:
            K_current = K_new
            P_current = P_new
    K = K_current
    P = P_current

    K_actual = K[:nU, :nX]
    k_actual = K[:, -1]

    return K_actual, k_actual

def continuous_LQR(speed, Q, R, wheelbase=2.995):
    A= np.matrix([[0, speed], [0, 0]])
    B = np.matrix([[0], [(speed/wheelbase)]])
    V = np.matrix(linalg.solve_continuous_are(A, B, Q, R))
    K = np.matrix(linalg.inv(R) * (B.T * V))
    return K


