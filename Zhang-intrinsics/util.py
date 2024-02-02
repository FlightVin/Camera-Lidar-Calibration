import numpy as np

def distort(k, normalized_proj):
   x, y = normalized_proj[:, 0], normalized_proj[:, 1]

   r = np.sqrt(x**2 + y**2)
   k0, k1 = k

   distorted_proj = np.zeros((x.shape[0], 2))
   distorted_proj[:, 0] = x * (1 + k0 * r**2 + k1 * r**4)
   distorted_proj[:, 1] = y * (1 + k0 * r**2 + k1 * r**4)

   return distorted_proj

def to_homogeneous(A):
   A = np.atleast_2d(A)

   N = A.shape[0]
   A_hom = np.hstack((A, np.ones((N,1))))

   return A_hom

def to_inhomogeneous(A):
   A = np.atleast_2d(A)

   A /= A[:,-1][:, np.newaxis]
   A_inhom = A[:,:-1]

   return A_inhom

def to_homogeneous_3d(A):
   N = A.shape[0]
   A_3d = np.hstack((A, np.zeros((N,1))))
   A_3d_hom = to_homogeneous(A_3d)

   return A_3d_hom

def project(K, k, E, model):
   model_hom = to_homogeneous_3d(model)
   normalized_proj = np.dot(model_hom, E.T)
   normalized_proj = to_inhomogeneous(normalized_proj)
   distorted_proj = distort(k, normalized_proj)
   distorted_proj_hom = to_homogeneous(distorted_proj)
   sensor_proj = np.dot(distorted_proj_hom, K[:-1].T)
   return sensor_proj