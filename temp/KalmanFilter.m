function [innovation, e_cov, estimatedState, estimatedCov] = KalmanFilter(A, B, C, Q, ...
    R, y, u, estimatedState, estimatedCov)
    % set up
    n = size(A, 1);

    predictedState = A * estimatedState + B * u;
    estimatedCov = A * estimatedCov * A' + Q;

    % Error
    innovation = y - C * predictedState;
    e_cov = C * estimatedCov * C' + R;

    % Kalman gain
    K = estimatedCov * C' * inv(e_cov);
    estimatedState = predictedState + K * innovation;
    estimatedCov = (eye(n) - K * C) * estimatedCov;
end