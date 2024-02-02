import numpy as np

def generate_v_ij(H, i, j):
    M = H.shape[0]
    v_ij = np.zeros((M, 6))
    v_ij[:, 0] = H[:, 0, i] * H[:, 0, j]
    v_ij[:, 1] = H[:, 0, i] * H[:, 1, j] + H[:, 1, i] * H[:, 0, j]
    v_ij[:, 2] = H[:, 1, i] * H[:, 1, j]
    v_ij[:, 3] = H[:, 2, i] * H[:, 0, j] + H[:, 0, i] * H[:, 2, j]
    v_ij[:, 4] = H[:, 2, i] * H[:, 1, j] + H[:, 1, i] * H[:, 2, j]
    v_ij[:, 5] = H[:, 2, i] * H[:, 2, j]

    return v_ij

def recover_intrinsics(homographies):
    H_stack = []
    for H in homographies:
        H_stack.append(H)
    H_stack = np.array(H_stack)

    v_00 = generate_v_ij(H_stack, 0, 0)
    v_01 = generate_v_ij(H_stack, 0, 1)
    v_11 = generate_v_ij(H_stack, 1, 1)

    V = np.vstack((v_01, v_00 - v_11))

    _, _, V_t = np.linalg.svd(V)
    B0, B1, B2, B3, B4, B5 = V_t[-1]

    v0      = (B1 * B3 - B0 * B4) / (B0 * B2 - B1**2)
    lambda_ = B5 - (B3**2 + v0 * (B1 * B3 - B0 * B4)) / B0
    alpha   = np.sqrt(lambda_ / B0)
    beta    = np.sqrt(lambda_ * B0 / (B0 * B2 - B1 * B1))
    gamma   = -B1 * alpha * alpha * beta / lambda_
    u0      = gamma * v0 / beta - B3 * alpha * alpha / lambda_

    K = np.array([[alpha, gamma, u0],
                  [   0.,  beta, v0],
                  [   0.,    0., 1.]])

    return K