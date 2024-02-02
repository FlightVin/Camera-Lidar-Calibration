import numpy as np
import util
import os
import glob
import cv2

def calculate_lens_distortion(model, all_data, K, extrinsic_matrices):
    M = len(all_data)
    N = model.shape[0]

    model = util.to_homogeneous_3d(model)

    u_c, v_c = K[0,2], K[1,2]

    r = np.zeros(2 * M * N)
    for e, E in enumerate(extrinsic_matrices):
        normalized_projection = np.dot(model, E.T)
        normalized_projection = util.to_inhomogeneous(normalized_projection)

        x_normalized_proj, y_normalized_proj = normalized_projection[:, 0], normalized_projection[:, 1]
        r_i = np.sqrt(x_normalized_proj**2 + y_normalized_proj**2)
        r[e*N:(e+1)*N] = r_i

    r[M*N:] = r[:M*N]

    obs = np.zeros(2 * M * N)
    u_data = v_data = np.zeros(M * N)

    for d, data in enumerate(all_data):
        u_i, v_i = data[:, 0], data[:, 1]
        u_data[d*N:(d+1)*N] = u_i
        v_data[d*N:(d+1)*N] = v_i

    obs[:M*N] = u_data
    obs[M*N:] = v_data
    
    pred = np.zeros(2 * M * N)
    pred_centered = np.zeros(2 * M * N)
    u_pred = v_pred = np.zeros(M * N)

    for e, E in enumerate(extrinsic_matrices):
        P = np.dot(K, E)
        projection = np.dot(model, P.T)
        projection = util.to_inhomogeneous(projection)
        u_pred_i = projection[:, 0]
        v_pred_i = projection[:, 1]

        u_pred[e*N:(e+1)*N] = u_pred_i
        v_pred[e*N:(e+1)*N] = v_pred_i

    pred[:M*N] = u_pred
    pred[M*N:] = v_pred
    pred_centered[:M*N] = u_pred - u_c
    pred_centered[M*N:] = v_pred - v_c

    D = np.zeros((2 * M * N, 2))
    D[:, 0] = pred_centered * r**2
    D[:, 1] = pred_centered * r**4

    b = obs - pred
    D_inv = np.linalg.pinv(D)
    k = np.dot(D_inv, b)

    return k