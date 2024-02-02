import numpy as np

def recover_extrinsics(H, K):
    h0, h1, h2 = H.T.tolist()
    
    K_inv   = np.linalg.inv(K)
    lambda_ = 1 / np.linalg.norm(np.dot(K_inv, h0))
    
    r0  = (lambda_ * np.dot(K_inv, h0)).reshape((-1, 1))
    r1  = (lambda_ * np.dot(K_inv, h1)).reshape((-1, 1))
    r2  = (np.cross(r0.ravel(), r1.ravel())).reshape((-1, 1))
    t   = lambda_ * np.dot(K_inv, h2).reshape((-1, 1))

    R   = np.hstack((r0, r1, r2))
    U, _, V_t = np.linalg.svd(R)
    R = np.dot(U, V_t)

    return np.hstack((R, t))