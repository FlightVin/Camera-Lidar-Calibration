import cv2
import numpy as np
import dataloader
import distortion
import extrinsics
import homography
import intrinsics
import refinement
import util

def zhang_calibration(model, all_data):
    homographies = []
    for data in all_data:
        H, _ = cv2.findHomography(model, data)
        H = homography.refine_homography(H, model, data)
        homographies.append(H)

    K = intrinsics.recover_intrinsics(homographies)

    model_hom_3d = util.to_homogeneous_3d(model)

    extrinsic_matrices = []
    for h, H in enumerate(homographies):
        E = extrinsics.recover_extrinsics(H, K)
        extrinsic_matrices.append(E)
        P = np.dot(K, E)

        predicted = np.dot(model_hom_3d, P.T)
        predicted = util.to_inhomogeneous(predicted)
        data = all_data[h]

    k = distortion.calculate_lens_distortion(model, all_data, K, extrinsic_matrices)

    print(K)
    print()
    K_opt, k_opt, extrinsics_opt = refinement.refine_all_parameters(model, all_data, K, k, extrinsic_matrices)

    return K_opt, k_opt, extrinsics_opt

model, all_data = dataloader.load_dataset("./zhang-coords")
K_opt, k_opt, extrinsics_opt = zhang_calibration(model, all_data)

print('   Focal Length: [ {:.5f}  {:.5f} ]'.format(K_opt[0,0], K_opt[1,1]))
print('Principal Point: [ {:.5f}  {:.5f} ]'.format(K_opt[0,2], K_opt[1,2]))
print('           Skew: [ {:.7f} ]'.format(K_opt[0,1]))
print('     Distortion: [ {:.6f}  {:.6f} ]'.format(k_opt[0], k_opt[1]))