import numpy as np

class KalmanFilter:
    def __init__(self, initial_state, initial_error_covariance, transition_matrix, control_matrix, observation_matrix, process_noise_covariance, observation_noise_covariance):
        self.state_estimate = initial_state
        self.error_covariance = initial_error_covariance
        self.transition_matrix = transition_matrix
        self.control_matrix = control_matrix
        self.observation_matrix = observation_matrix
        self.process_noise_covariance = process_noise_covariance
        self.observation_noise_covariance = observation_noise_covariance

    def predict(self, control_input):
        self.state_estimate = np.dot(self.transition_matrix, self.state_estimate) + np.dot(self.control_matrix, control_input)
        self.error_covariance = np.dot(np.dot(self.transition_matrix, self.error_covariance), self.transition_matrix.T) + self.process_noise_covariance

    def update(self, observation):
        innovation = observation - np.dot(self.observation_matrix, self.state_estimate)
        innovation_covariance = np.dot(np.dot(self.observation_matrix, self.error_covariance), self.observation_matrix.T) + self.observation_noise_covariance
        kalman_gain = np.dot(np.dot(self.error_covariance, self.observation_matrix.T), np.linalg.inv(innovation_covariance))
        self.state_estimate = self.state_estimate + np.dot(kalman_gain, innovation)
        self.error_covariance = self.error_covariance - np.dot(np.dot(kalman_gain, self.observation_matrix), self.error_covariance)

def main():
    initial_state = np.array([0, 0, 0, 0])
    initial_error_covariance = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    transition_matrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]])
    control_matrix = np.array([[0.5, 0], [0, 0.5], [1, 0], [0, 1]])
    observation_matrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])
    process_noise_covariance = np.array([[0.1, 0, 0, 0], [0, 0.1, 0, 0], [0, 0, 0.1, 0], [0, 0, 0, 0.1]])
    observation_noise_covariance = np.array([[0.1, 0], [0, 0.1]])

    kf = KalmanFilter(initial_state, initial_error_covariance, transition_matrix, control_matrix, observation_matrix, process_noise_covariance, observation_noise_covariance)
    kf.predict(np.array([0, 0]))
    kf.update(np.array([1, 1]))
    print(kf.state_estimate)

if __name__ == '__main__':  
    main()