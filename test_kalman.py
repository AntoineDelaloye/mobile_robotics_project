from ekf import ExtendedKalmanFilter
import numpy as np


state = np.array([0,0,0])
covariance = np.matrix([[0,0,0],[0,0,0],[0,0,0]])
width = 110
speedfactor = 0.43478260869565216 # 0.31746
dt = 1
sig_left = 0.1
sig_right = 0.1
sig_x = 0.1
sig_y = 0.1
sig_theta = 0.1
kf = ExtendedKalmanFilter(state,covariance,width,speedfactor,dt,sig_left,sig_right,sig_x,sig_y,sig_theta)
# print(kf.x)
# print(kf.g(1,2,3,4,5))
print("[State] {}".format(kf.state))
print("[Covar] {}".format(kf.covariance))

kf.run_ekm(np.array([200,200]), np.array([87,0,0]))
print("[State] {}".format(kf.state))
# print("[Covar] {}".format(kf.covariance))
kf.run_ekm(np.array([200,200]), np.array([87,0,0]))
print("[State] {}".format(kf.state))
# print("[Covar] {}".format(kf.covariance))
kf.run_ekm(np.array([300,0]), np.array([87,0,0]))
print("[State] {}".format(kf.state))
# print("[Covar] {}".format(kf.covariance))
kf.run_ekm(np.array([300,0]), np.array([87,0,0]))
print("[State] {}".format(kf.state))
# print("[Covar] {}".format(kf.covariance))
kf.run_ekm(np.array([400,200]), np.array([87,0,0]))
print("[State] {}".format(kf.state))
# print("[Covar] {}".format(kf.covariance))
kf.run_ekm(np.array([-500,-500]), np.array([87,0,0]))
print("[State] {}".format(kf.state))
# print("[Covar] {}".format(kf.covariance))
kf.run_ekm(np.array([200,200]), np.array([87,0,0]))
print("[State] {}".format(kf.state))
# print("[Covar] {}".format(kf.covariance))
