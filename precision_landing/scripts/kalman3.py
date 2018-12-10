import numpy as np
from numpy.linalg import inv, norm
from numpy.random import randn
from math import pi
from enum import Enum
import time

class Measurement(Enum):
    POS = 1
    VEL = 2
    BOTH = 3

class KalmanFilter(object):

    def __init__(self, initial_state, dt):
        self.x      = initial_state
        self.dt     = dt
        self.F      = np.array([[1,0,0,dt,0,0],
                                [0,1,0,0,dt,0],
                                [0,0,1,0,0,dt],
                                [0,0,0,1,0,0],
                                [0,0,0,0,1,0],
                                [0,0,0,0,0,1]])

        self.H_pos  = np.array([[1,0,0,0,0,0],
                                [0,1,0,0,0,0],
                                [0,0,1,0,0,0]])

        self.H_vel  = np.array([[0,0,0,1,0,0],
                                [0,0,0,0,1,0],
                                [0,0,0,0,0,1]])
                                
        self.H_both = np.array([[1,0,0,0,0,0],
                                [0,1,0,0,0,0],
                                [0,0,1,0,0,0],
                                [0,0,0,1,0,0],
                                [0,0,0,0,1,0],
                                [0,0,0,0,0,1]])

        self.P_min  = np.eye(6)
        self.P_plus = np.eye(6)

        self.x_hat_min  = self.x
        self.x_hat_plus = self.x

        self.last_timestamp = time.time()

        # Error matrices from Agus lecture
        measurementNoise    = 0.5
        modelNoise          = 3
        self.Q      = np.eye(6)*modelNoise**2*dt
        self.R      = np.eye(3)*measurementNoise**2/dt
        self.R3     = np.eye(3)*measurementNoise**2/dt
        self.R6     = np.eye(6)*measurementNoise**2/dt

        # Error matrices from youtube
        # measurementNoise = 0.5
        # self.R      = np.eye(2)*measurementNoise**2/dt
        # self.Q      = np.array([[pow(dt,4)/4,    0,              pow(dt,3)/2,    0],
        #                         [0,              pow(dt,4)/4,    0,              pow(dt,3)/2],
        #                         [pow(dt,3)/2,    0,              dt**2,          0],
        #                         [0,              pow(dt,3)/2,    0,              dt**2]])*dt

    def reset(self):
        # only reset if covariance matrix has been modified
        if not np.array_equal(self.P_plus, np.eye(6)):
            self.P_min  = np.eye(6)
            self.P_plus = np.eye(6)

    def update(self, y=None, y_type=Measurement.POS):
        timestamp = time.time()
        dt = timestamp - self.last_timestamp
        self.last_timestamp = timestamp

        dt = self.dt

        self.F      = np.array([[1,0,0,dt,0,0],
                                [0,1,0,0,dt,0],
                                [0,0,1,0,0,dt],
                                [0,0,0,1,0,0],
                                [0,0,0,0,1,0],
                                [0,0,0,0,0,1]])

        x, P = self.predict()

        # If there is no new measurement, then we can only make a prediction
        if type(y) != type(None):

            if y_type == Measurement.POS:
                self.R = self.R3
                self.H = self.H_pos
            elif y_type == Measurement.VEL:
                self.R = self.R3
                self.H = self.H_vel
            else:
                self.R = self.R6
                self.H = self.H_both

            x, P = self.correct(y)
        else:
            # The prediction was the best we could do in this iteration, so x_hat_plus and P_plus
            # are assigned to the result from the prediction
            self.x_hat_plus = x
            self.P_plus = P

        return x


    def predict(self):
        # For prediction we can only make a priori estimates
        # We need to increase the uncertainty in P
        self.P_min = self.F @ self.P_plus @ self.F.T + self.Q

        # Then we need to make our state prediction based on our model
        # a = randn()*
        self.x_hat_min = self.F @ self.x_hat_plus# + self.G * a

        # If this is all we do in this iteration then we need to update xhat_plus and P_plus as well
        # with our prediction estimates from this function, because they are used at the beginning of each
        # iteration

        # self.x_hat_plus = self.x_hat_min
        # self.P_plus     = self.P_min

        # This can also be taken care of in the update function

        return self.x_hat_min, self.P_min

    def correct(self, y):
        # Use long formula from book to calculate the kalman gain from P_min instead of P_plus
        # This is necessary if P_plus is changed in the predict function
        self.K = self.P_min@self.H.T@inv(self.H@self.P_min@self.H.T + self.R)

        # P_plus has to be the P_plus from the previous iteration; it must not be calculated in the
        # predict function
        # self.K          = self.P_plus @ self.H.T @ inv(self.R)

        # print("K: {}".format(self.K.shape))
        # print("H: {}".format(self.H.shape))

        self.x_hat_plus = self.x_hat_min + self.K @ (y - self.H @ self.x_hat_min)
        self.P_plus     = (np.eye(6) - self.K @ self.H) @ self.P_min

        return self.x_hat_plus, self.P_plus