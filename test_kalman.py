from ekf import ExtendedKalmanFilter
import numpy as np


state = np.array([0,0,0])
covariance = np.matrix([[0,0,0],[0,0,0],[0,0,0]])
width = 110
diameter = 45
N = 200
dt = 1
sig_left = 0.1
sig_right = 0.1
kf = ExtendedKalmanFilter(state,covariance,width,diameter,N,dt,sig_left,sig_right)
# print(kf.x)
# print(kf.g(1,2,3,4,5))
print(kf.state)
kf.prediction_step(np.array([300,300]))
print(kf.state)
kf.prediction_step(np.array([300,400]))
print(kf.state)
kf.prediction_step(np.array([400,300]))
print(kf.state)
kf.prediction_step(np.array([300,300]))
print(kf.state)
kf.prediction_step(np.array([300,300]))
print(kf.state)
