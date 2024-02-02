import numpy as np
from scipy.optimize import curve_fit

def f_refine(xdata, *params):
    h11, h12, h13, h21, h22, h23, h31, h32, h33 = params

    num_coords = xdata.shape[0] // 2
    X = xdata[:num_coords]
    Y = xdata[num_coords:]

    x = (h11 * X + h12 * Y + h13) / (h31 * X + h32 * Y + h33)
    y = (h21 * X + h22 * Y + h23) / (h31 * X + h32 * Y + h33)

    return np.concatenate((x, y))

def refine_homography(H, model, data):
    X, Y = model.T.tolist()
    x, y = data.T.tolist()

    xdata = np.concatenate((X, Y))
    ydata = np.concatenate((x, y))

    h_refined, _ = curve_fit(f_refine, xdata, ydata, p0=H.reshape(-1))
    h_refined /= h_refined[-1]

    return h_refined.reshape((3,3))