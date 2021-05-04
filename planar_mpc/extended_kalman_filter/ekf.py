import sys
import numpy as np
from extended_kalman_filter.evolution import evolution

def is_pos_def(x):
    return np.all(np.linalg.eigvals(x) > 0)

class EKF:
    def __init__(self,  initial_x: np.array, 
                        P_0: np.array,
                        m: float, 
                        Ixx: float,
                        dt: float) -> None: # returns nothing
        # mean of state Gaussian RV
        self._x = initial_x.reshape(6,1)

        # covariance of initial state (Gaussian RV)
        self._P = P_0

        # mass of the drone
        self._m = m
        
        # inertia of the drone
        self._Ixx = Ixx

        # sample time of the extended kalman filter
        self._dt = dt

    def predict(self,   A: np.array, 
                        B: np.array, 
                        u: np.array, 
                        Q_alpha: np.array,
                        Q_beta: np.array) -> None:
        
        # prediction equations
        new_x = evolution(self._x, u, self._m, self._Ixx, self._dt)
        new_P = A @ self._P @ A.T + B @ Q_beta @ B.T + Q_alpha
        
        pos_def_bool = is_pos_def(new_P)
        if pos_def_bool == False:
            sys.exit("The covariance matrix is not positive definite")

        self._P = new_P
        self._x = new_x


    def update(self,    C: np.array, 
                        meas: np.array, 
                        meas_variance: np.array): 

        # reshaping the measurement vector for matrix multiplication
        y = meas.reshape(6,1)

        # measurement variance
        Q_gamma = meas_variance.reshape(6,6)

        # measurement prediction
        y_hat = C @ self._x

        # innovation
        innov = y - y_hat
        
        # Mobile robot equations
        K = self._P @ C.T @ np.linalg.inv( C @ self._P @C.T + Q_gamma)
        new_x = self._x + K @ innov
        # new_P = ( np.eye(len(new_x)) - K @ C) @ self._P
        new_P = ( np.eye(len(new_x)) - K @ C) @ self._P @ ( np.eye(len(new_x)) - K @ C).T + K @ Q_gamma @ K.T # (Joseph form, better conditioned)
        
        
        # equations from AUVE book
        # C_xy = self._P @ C.T
        # C_yy = C @ self._P @ C.T + Q_gamma
        # K = C_xy @ np.linalg.inv(C_yy)

        # state and covariance update
        # new_x = self._x + K @ innov
        # new_P = self._P - K @ C_xy.T
        # new_P = ( np.eye(len(new_x)) - K @ C) @ self._P @ ( np.eye(len(new_x)) - K @ C).T + K @ Q_gamma @ K.T (Joseph form, better conditioned)

        self._P = new_P
        self._x = new_x

    @property
    def cov(self) -> np.array:
        return self._P


    @property
    def state(self) -> np.array:
        return self._x