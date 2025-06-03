class KalmanFilter:
    def __init__(self):
        self.Q_angle = 0.001   # Process noise variance for the accelerometer
        self.Q_bias = 0.003    # Process noise variance for the gyro bias
        self.R_measure = 0.03  # Measurement noise variance

        self.angle = 0.0  # Reset the angle
        self.bias = 0.0   # Reset bias

        self.P = [[0, 0], [0, 0]]  # Error covariance matrix

    def get_angle(self, new_angle, new_rate, dt):
        # Predict phase
        rate = new_rate - self.bias
        self.angle += dt * rate

        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[1][0] - self.P[0][1] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        # Update phase
        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0] / S, self.P[1][0] / S]  # Kalman gain

        y = new_angle - self.angle  # Angle difference
        self.angle += K[0] * y
        self.bias += K[1] * y

        # Update the error covariance matrix
        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle
