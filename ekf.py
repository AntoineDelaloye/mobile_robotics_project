import numpy as np
from numpy.linalg import inv
import math

class ExtendedKalmanFilter:
    def __init__(self,state,covariance,width,speedfactor,dt,sig_left,sig_right,sig_x,sig_y,sig_theta): # constructor
        # covariance and state
        self.state = state # mu t-1
        self.covariance = covariance # Sigma t-1

        # Hyperparameters
        self.width = width # width of robot in mm
        self.speedfactor = speedfactor # nb ticks per full wheel rotation
        self.dt = dt # time interval in seconds
        self.sigma_control = np.matrix(np.diag([sig_left**2,sig_right**2]))
        self.sigma_camera = np.matrix(np.diag([sig_x**2,sig_y**2,sig_theta**2]))

    def convert_control(self, control): # convert control from velocity to mm
        vl,vr = control
        # l = np.pi*self.diameter*vl*self.dt/self.N
        l = self.speedfactor*vl*self.dt
        # r = np.pi*self.diameter*vr*self.dt/self.N
        r = self.speedfactor*vr*self.dt
        return np.array([l,r])

    def g(self,control):
        x,y,theta = self.state
        l,r = control
        if l!=r:
            alpha = (r-l)/self.width
            R = l/alpha
            g1 = x + (R+self.width/2)*(math.sin(theta+alpha)-math.sin(theta))
            g2 = y + (R+self.width/2)*(-math.cos(theta+alpha)+math.cos(theta))
            g3 = (theta + alpha) %(2*np.pi) # keep it between 0 and 2 pi
        else:
            g1 = x + l*math.cos(theta)
            g2 = y + l*math.sin(theta)
            g3 = theta
        return np.array([g1,g2,g3])

    def dg_dstate(self, control): # Gt
        x,y,theta = self.state
        l,r = control
        if l!=r:
            alpha = (r-l)/self.width
            R = l/alpha
            dg1_dx = 1
            dg1_dy = 0
            dg1_dtheta = (R+self.width/2)*(math.cos(theta+alpha)-math.cos(theta))
            dg2_dx = 0
            dg2_dy = 1
            dg2_dtheta = (R+self.width/2)*(math.sin(theta+alpha)-math.sin(theta))
            dg3_dx = 0
            dg3_dy = 0
            dg3_dtheta =  1
        else:
            dg1_dx = 1
            dg1_dy = 0
            dg1_dtheta = -l*math.sin(theta)
            dg2_dx = 0
            dg2_dy = 1
            dg2_dtheta = l*math.cos(theta)
            dg3_dx = 0
            dg3_dy = 0
            dg3_dtheta = 1
        return np.matrix([[dg1_dx,dg1_dy,dg1_dtheta],[dg2_dx,dg2_dy,dg2_dtheta],[dg3_dx,dg3_dy,dg3_dtheta]])

    def dg_dcontrol(self, control): # Vt
        x,y,theta = self.state
        l,r = control
        if l!=r:
            alpha = (r-l)/self.width
            R = l/alpha
            dalpha_dl = 1/self.width
            dalpha_dr = -1/self.width
            dR_dl = self.width*r*l**(-2)/((r/l-1)**2)
            dR_dr = -self.width/l/((r/l-1)**2)

            dg1_dl = dR_dl*(math.sin(theta+alpha)-math.sin(theta))+(R+self.width/2)*(math.cos(theta+alpha)*dalpha_dl)
            dg1_dr = dR_dr*(math.sin(theta+alpha)-math.sin(theta))+(R+self.width/2)*(math.cos(theta+alpha)*dalpha_dr)
            dg2_dl = dR_dl*(-math.cos(theta+alpha)+math.cos(theta))+(R+self.width/2)*(math.sin(theta+alpha)*dalpha_dl)
            dg2_dr = dR_dr*(-math.cos(theta+alpha)+math.cos(theta))+(R+self.width/2)*(math.sin(theta+alpha)*dalpha_dr)
            dg3_dl = dalpha_dl
            dg3_dr = dalpha_dr
        else:
            dg1_dl = math.cos(theta)
            dg1_dr = 0
            dg2_dl = math.sin(theta)
            dg2_dr = 0
            dg3_dl = 0
            dg3_dr = 0

        return np.matrix([[dg1_dl,dg1_dr],[dg1_dl,dg1_dr],[dg1_dl,dg1_dr]])

    def h(self, position):
        x,y,theta = position
        return np.matrix([[x],[y],[theta]])

    def prediction_step(self,control):
        control = self.convert_control(control) # convert control
        self.state = self.g(control) # get next state
        try:
            G = self.dg_dstate(control)
            V = self.dg_dcontrol(control)
            self.covariance = G*self.covariance*G.transpose(1,0)+V*self.sigma_control*V.transpose(1,0)
            # print("[Predict] Covariance: {}".format(self.covariance))
        except Exception as e:
            print("[Error] {}".format(e))

    def update_step(self,position): # Jacobian of h is just the identiy Matrix
        x,y,theta = position
        I = np.matrix(np.diag([1,1,1]))
        K = self.covariance * (self.covariance * inv(self.sigma_camera))# Kalman Gain
        # print("[Update] Kalman: {}".format(K))
        new_state = np.transpose(np.matrix(self.state)) + K*(self.h(position)-self.h(self.state))
        self.state = np.squeeze(np.asarray(np.transpose(new_state)))
        self.covariance = (I-K)*self.covariance

    def run_ekm(self, control, position):
        self.prediction_step(control)
        self.update_step(position)
