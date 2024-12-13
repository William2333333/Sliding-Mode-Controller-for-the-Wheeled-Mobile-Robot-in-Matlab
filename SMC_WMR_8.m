% Parameters
r = 0.0315; % Wheel radius (meters)
R = 0.09; % Distance between two wheels (meters)
c1 = 0.6; % Sliding surface parameter
eta1 = 1.2; % Reaching gain for sigma1
eta2 = 0.1; % Reaching gain for sigma2
dt = 0.01; % Time step (seconds)
t_end = 70; % Simulation end time (seconds)
t = 0:dt:t_end; % Time vector


A = 10; % Amplitude of the figure-8 (size of loops)
omega = 2 * pi / t_end; % Angular velocity based on period T
theta = omega * t; % Angle that changes with time
sigma1_list = zeros(1,length(t));
sigma2_list = zeros(1,length(t));
v_list = zeros(1,length(t));
w_list = zeros(1,length(t));

% Reference trajectory parameters
vr_ref = 0.25; % Linear velocity (m/s)
wr_ref = 0.5; % Angular velocity (rad/s)

% Disturbance
disturbance = 0.001 * sin(2 * pi * t);


% Initial conditions
q = [0; 5; 0]; % Initial state [x; y; theta]
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
    

    
    % Calculate the position in the "8" trajectory
    x_ref = A * sin(theta(k)); % Horizontal component
    y_ref = A * sin(2 * theta(k)) / 2; % Vertical component scaled to create "8" shape
    
    % Compute the desired velocity in the "8" shape
    x_dot_ref = A * omega * cos(theta(k));
    y_dot_ref = A * omega * cos(2 * theta(k));

    % Compute the desired acceleration in the "8" shape
    x_ddot_ref = -A * omega^2 * sin(theta(k));
    y_ddot_ref = -2 * A * omega^2 * sin(2 * theta(k));
    
    % Update reference orientation and angular velocity
    vr_ref = sqrt(x_dot_ref^2 + y_dot_ref^2); % Linear velocity
    wr_ref = (x_dot_ref * y_ddot_ref - y_dot_ref * x_ddot_ref) / vr_ref^2; % Angular velocity
    
    % Update the state
    qr_dot = [x_dot_ref; y_dot_ref; wr_ref];
    qr = qr + qr_dot * dt;
    
    % Store data
    q_trajectory(:, k) = q;
    q_ref_trajectory(:, k) = qr;
    qe_trajectory(:, k) = qe;
    sigma1_list(:,k) = sigma1;
    sigma2_list(:,k) = sigma2;
    v_list(:,k) = v;
    w_list(:,k) = w;
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
figure;
subplot(2,1,1);
plot(t,sigma1_list(1,:));
xlabel('Time (s)');
ylabel('\sigma1');
grid on;

subplot(2,1,2);
plot(t,sigma2_list(1,:));
xlabel('Time (s)');
ylabel('\sigma2');
grid on;

figure;
subplot(2,1,1);
plot(t,v_list(1,:));
xlabel('Time (s)');
ylabel('\v');
grid on;

subplot(2,1,2);
plot(t,w_list(1,:));
xlabel('Time (s)');
ylabel('\w');
grid on;



