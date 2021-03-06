import numpy as np
from numpy.linalg import inv, norm
from numpy.random import randn
from math import pi

class KalmanFilter(object):

    def __init__(self, initial_state, dt):
        self.x      = initial_state
        self.dt     = dt
        # included acceleration
        # x < - x + x_velocity
        # y < - y + y_velocity
        # x_velocity < - x_velocity + x_acceleration
        # y_velocity < - y_velocity + y_acceleration
        # x_acceleration < - x_acceleration
        # y_acceleration < - y_acceleration
        self.F      = np.array([[1, 0, dt, 0, dt*dt/2, 0],
                                [0, 1, 0, dt, 0, dt*dt/2],
                                [0, 0, 1, 0, dt, 0],
                                [0, 0, 0, 1, 0, dt],
                                [0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 1]])

        # Measurements include position and accelleration in x and y
        # TODO The accelleration should maybe be added as a control input instead of a measurement
        self.H      = np.array([[1, 0, 0, 0, 0, 0],
                                [0, 1, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 1, 0],
                                [0, 0, 0, 0, 0, 1]])

        self.P_min  = np.eye(6)
        self.P_plus = np.eye(6)

        self.x_hat_min  = self.x
        self.x_hat_plus = self.x

        # Members for detecting self-destruct condition
        # self.maxPredictions = maxPredictions
        self.predictionCount = 0
        self.selfDestruct = False

        # Members for delaying drawing feature
        self.correctionCount = 0
        # self.startDelay = startDelay
        # self.valid = False

        # Error matrices from Agus lecture
        measurementNoise    = 0.1
        modelNoise          = 2
        # TODO make sure matrix sizes make sense
        self.Q      = np.eye(6)*modelNoise**2*dt
        self.R      = np.eye(6)*measurementNoise**2/dt

        # Error matrices from youtube
        # measurementNoise = 0.5
        # self.R      = np.eye(2)*measurementNoise**2/dt
        # self.Q      = np.array([[pow(dt,4)/4,    0,              pow(dt,3)/2,    0],
        #                         [0,              pow(dt,4)/4,    0,              pow(dt,3)/2],
        #                         [pow(dt,3)/2,    0,              dt**2,          0],
        #                         [0,              pow(dt,3)/2,    0,              dt**2]])*dt

    def update(self, y=None):
        # If there is no new measurement, then we can only make a prediction

        x, P = self.predict()
        x0 = self.x_hat_plus


        # Members for detecting self-destruct condition
        # self.maxPredictions = maxPredictions
        self.predictionCount = 0
        self.selfDestruct = False
        if type(y) != type(None):
            x, P = self.correct(y)
            self.predictionCount = 0
            self.correctionCount += 1

        else:
            # The prediction was the best we could do in this iteration, so x_hat_plus and P_plus
            # are assigned to the result from the prediction
            self.x_hat_plus = x
            self.P_plus = P
            self.predictionCount += 1

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

        self.x_hat_plus = self.x_hat_min + self.K @ (y - self.H @ self.x_hat_min)
        self.P_plus     = (np.eye(6) - self.K @ self.H) @ self.P_min

        return self.x_hat_plus, self.P_plus

    def getState(self):
        return self.x_hat_plus

    def getPosition(self):
        return np.array([self.x_hat_plus[0,0], self.x_hat_plus[1,0]])

    def getAbsVelocity(self):
        v = np.array([self.x_hat_plus[2, 0], self.x_hat_plus[3, 0]])
        v = norm(v)
        return v
