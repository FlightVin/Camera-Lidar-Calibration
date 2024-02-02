import cv2
import numpy as np
import util
from scipy.optimize import curve_fit

def pack_params(K, k, extrinsic_matrices):
    packed_params = []

    alpha, beta, gamma, u_c, v_c = K[0, 0], K[1, 1], K[0, 1], K[0, 2], K[1, 2]
    k0, k1 = k
    a = [alpha, beta, gamma, u_c, v_c, k0, k1]

    packed_params.extend(a)

    for E in extrinsic_matrices:
        R = E[:3, :3]
        t = E[:, 3]
        rho_x, rho_y, rho_z = cv2.Rodrigues(R)[0]

        e = [rho_x[0], rho_y[0], rho_z[0]]
        e.extend(t)

        packed_params.extend(e)
    
    return np.array(packed_params)

def unpack_refinement_params(params):
    intrinsics = params[:7]
    alpha, beta, gamma, u_c, v_c, k0, k1 = intrinsics

    K = np.array([[alpha, gamma, u_c],
                  [   0.,  beta, v_c],
                  [   0.,    0.,  1.]])

    k = np.array([k0, k1])

    extrinsic_matrices = []
    for i in range(7, len(params), 6):
        E_rodrigues = params[i:i + 6]
        rho_x, rho_y, rho_z, t_x, t_y, t_z = E_rodrigues

        R = cv2.Rodrigues(np.array([rho_x, rho_y, rho_z]))[0]
        t = [t_x, t_y, t_z]

        E = np.zeros((3, 4))
        E[:3, :3] = R
        E[:, 3] = t

        extrinsic_matrices.append(E)

    return K, k, extrinsic_matrices

def f_refine_all(xdata, *params):
    M = len(params[7:]) // 6
    N = xdata.shape[0] // (2 * M)

    X = xdata[:N]
    Y = xdata[N:2 * N]
    model = np.concatenate((X[:, None], Y[:, None]), axis=1)
    K, k, extrinsic_matrices = unpack_refinement_params(params)

    obs_x = np.zeros(N*M)
    obs_y = np.zeros(N*M)
    for e, E in enumerate(extrinsic_matrices):
        sensor_proj = util.project(K, k, E, model)
        x, y = sensor_proj.T.tolist()

        obs_x[e*N:(e+1)*N] = x
        obs_y[e*N:(e+1)*N] = y
        
    return np.concatenate((obs_x, obs_y))

def refine_all_parameters(model, all_data, K, k, extrinsic_matrices):
    M = len(all_data)
    N = model.shape[0]

    p0 = pack_params(K, k, extrinsic_matrices)

    X, Y = model[:, 0], model[:, 1]
    xdata = np.zeros(2 * M * N)
    xdata[:N] = X
    xdata[N:2 * N] = Y

    obs_x, obs_y = [], []
    for data in all_data:
        x, y = data[:, 0], data[:, 1]
        obs_x.append(x)
        obs_y.append(y)

    obs_x = np.hstack(obs_x)
    obs_y = np.hstack(obs_y)

    ydata   = np.hstack((obs_x, obs_y))
    popt, _ = curve_fit(f_refine_all, xdata, ydata, p0)

    K_opt, k_opt, extrinsic_matrices_opt = unpack_refinement_params(popt)

    return K_opt, k_opt, extrinsic_matrices_opt