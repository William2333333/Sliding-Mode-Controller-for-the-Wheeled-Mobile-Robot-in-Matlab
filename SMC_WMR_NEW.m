% Parameters
r = 0.0315; % Wheel radius (meters)
R = 0.09; % Distance between two wheels (meters)
c1 = 0.6; % Sliding surface parameter
eta1 = 1.2; % Reaching gain for sigma1
eta2 = 0.1; % Reaching gain for sigma2
dt = 0.01; % Time step (seconds)
t_end = 20; % Simulation end time (seconds)
t = 0:dt:t_end; % Time vector

% Reference trajectory parameters
vr_ref = 0.25; % Linear velocity (m/s)
wr_ref = 0.5; % Angular velocity (rad/s)

% Disturbance
disturbance = 0.001 * sin(2 * pi * t);


% Initial conditions
q = [-0.4; 0.4; 0]; % Initial state [x; y; theta]
qr = [0; 0; pi/4]; % Reference initial state
qe = [0; 0; 0]; % Error state

% Preallocate variables for storing results
q_trajectory = zeros(3, length(t));
q_ref_trajectory = zeros(3, length(t));
qe_trajectory = zeros(3, length(t));

% Simulation loop
for k = 1:length(t)
    % Compute error
    xe = (qr(1) - q(1)) * cos(q(3)) + (qr(2) - q(2)) * sin(q(3));
    ye = -(qr(1) - q(1)) * sin(q(3)) + (qr(2) - q(2)) * cos(q(3));
    theta_e = qr(3) - q(3);
    qe = [xe; ye; theta_e];
    
    % Sliding mode control law
    sigma1 = c1 * theta_e + atan(ye);
    sigma2 = xe;
    Jn = [0, 1/(1 + ye^2), 0; 1, 0, 0];
    E = [0, -(c1 + xe/(1 + ye^2)); -1, ye];
    u = -inv(E) * (Jn * [vr_ref * cos(theta_e); vr_ref * sin(theta_e); wr_ref] ...
        + [eta1 * sign(sigma1); eta2 * sign(sigma2)]);
    v = u(1);
    w = u(2);
    
    % Update state
    q_dot = [v * cos(q(3)); v * sin(q(3)); w];
    disturbance_array = [disturbance(k);disturbance(k);disturbance(k)];
    q = q + q_dot * dt+ disturbance_array;
    
    % Update reference trajectory
    qr_dot = [vr_ref * cos(qr(3)); vr_ref * sin(qr(3)); wr_ref];
    qr = qr + qr_dot * dt ;
    
    % Store data
    q_trajectory(:, k) = q;
    q_ref_trajectory(:, k) = qr;
    qe_trajectory(:, k) = qe;
end

% Plot results
figure;
subplot(3, 1, 1);
plot(t, qe_trajectory(1, :));
xlabel('Time (s)');
ylabel('x_e (m)');
title('Tracking Errors');
grid on;

subplot(3, 1, 2);
plot(t, qe_trajectory(2, :));
xlabel('Time (s)');
ylabel('y_e (m)');
grid on;

subplot(3, 1, 3);
plot(t, qe_trajectory(3, :));
xlabel('Time (s)');
ylabel('\theta_e (rad)');
grid on;

figure;
plot(q_ref_trajectory(1, :), q_ref_trajectory(2, :), 'r--', 'LineWidth', 1.5);
hold on;
plot(q_trajectory(1, :), q_trajectory(2, :), 'b-', 'LineWidth', 1.5);
xlabel('x (m)');
ylabel('y (m)');
legend('Reference Trajectory', 'Actual Trajectory');
title('Trajectory Tracking');
grid on;

