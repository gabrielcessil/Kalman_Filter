% KALMAN FILTER: STATE ESTIMATION OF SIMPLE PENDULUM
clear
close all
clc


% SIMULATION SETUP --------------------------------------------------------
T               = 0.01;          % Sampling period
simuTime        = 10;            % Simulated interval
g               = 9.81;          % Gravity

t_hist          = 0:T:simuTime;  % Time array
n_timesteps     = length(t_hist);     % Number of time steps
% -------------------------------------------------------------------------

% SYSTEM INITIALIZATION ---------------------------------------------------
ang         = deg2rad(5);
angVel      = 0;
% Preditor initialization
xhat        = [0;0];
xhat1       = xhat; 
P0          = 1000 * eye(2);
P1          = P0;
% -------------------------------------------------------------------------


% PLANT SETUP -------------------------------------------------------------
% Plant specs
m = 1;      % Mass of pendulum
l = 0.5;    % Pendulum length 
% Continuous plant model
Ac = [0 , 1 ; -g/l 0];
Bc = [0 ; 1/(m*l^2)]; 
Cc = [1 0];
Dc = 0;
sys = ss(Ac,Bc,Cc,Dc);
% Discrete plant model (used by Kalman filters)
sysd = c2d(sys,T);
A = sysd.A;
B = sysd.B;
Gamma = sysd.B; 
C = sysd.C;
D = sysd.D;
% Noise modelling
Qv = 1e-3;   % Process noise covariance
Rv = 1e-3;   % Measurement noise covariance
% -------------------------------------------------------------------------

filter  = KalmanFilter(A, B, Gamma, C, D, Qv, Rv, P0, xhat);

% RUN IDENTIFICATION EXPERIMENT -------------------------------------------
% Stochastic signals
v               = sqrt(Rv)*randn(n_timesteps,1); % Create the noise for each time step
w               = sqrt(Qv)*randn(n_timesteps,1); % Create the noise for each time step
ang_truth       = zeros(n_timesteps,1);
angVel_truth    = zeros(n_timesteps,1);
ang_meas        = zeros(n_timesteps,1);
angVel_meas     = zeros(n_timesteps,1);
ang_hat         = zeros(n_timesteps,1);
angVel_hat      = zeros(n_timesteps,1);
KalmanGain      = zeros(n_timesteps,2);
res             = zeros(n_timesteps,2);
for k=2:n_timesteps
    % Command Input Signal
    u       = zeros(size(B,2), 1);
    
    % Simulation os Plant Signals
    ddtheta = 1/(m*l^2)*w(k) -g/l*sin(ang); % Plant model
    angVel  = angVel +ddtheta*T; % Integration 
    ang     = ang +angVel*T;   % Integration
    % Measuring signals
    ang_meas(k)     = ang +v(k); % Includes noise when reading    
    angVel_meas(k)  = (ang_meas(k) -ang_meas(k-1))/T; % Derivating readed signal
    
    % Filters
    filter  = filter.estimate(ang_meas(k), u);
    xhat    = filter.xhat;
    
    % Save signals
    ang_hat(k)      = xhat(1);
    angVel_hat(k)   = xhat(2);
    ang_truth(k)    = ang;
    angVel_truth(k) = angVel;
    KalmanGain(k,:) = filter.K;
    res(k,:)        = filter.e;
end
% -------------------------------------------------------------------------


% MAKE PLOTS --------------------------------------------------------------
figure
subplot(2,1,1)
plot(t_hist, ang_meas, 'w:', 'DisplayName', 'Measured Angle')
hold on
plot(t_hist, ang_truth, 'r', 'LineWidth', 1.5, 'DisplayName', 'Real Angle')
hold on
plot(t_hist, ang_hat, 'b', 'LineWidth', 1.5, 'DisplayName', 'Estimated Angle')
legend
ylabel('\theta (rad)')
title('Kalman Filter - Posição e Velocidade do Pêndulo')

subplot(2,1,2)
plot(t_hist, angVel_meas, 'w:', 'DisplayName', 'Measured Angular Velocity')
hold on
plot(t_hist, angVel_truth, 'r', 'LineWidth', 1.5, 'DisplayName', 'Real Angular Velocity')
hold on
plot(t_hist, angVel_hat, 'b', 'LineWidth', 1.5, 'DisplayName', 'Estimated Angular Velocity')
legend
ylabel('\omega (rad/s)')
xlabel('Time(s)')

figure
subplot(2,1,1);
semilogy(t_hist, abs(KalmanGain(:,1)), 'b', 'LineWidth', 1.5, 'DisplayName', 'K_\theta')
hold on
semilogy(t_hist, abs(KalmanGain(:,2)), 'r', 'LineWidth', 1.5, 'DisplayName', 'K_\omega')
legend
ylabel('Kalman matrix elements')
xlabel('Time(s)')
grid on
set(gca, 'YScale', 'log')   % ensure log axis
subplot(2,1,2);
plot(t_hist, res(:,2), 'b', 'LineWidth', 1.5, 'DisplayName', 'Kalman \xi_\theta before adjustment')
hold on
plot(t_hist, res(:,1), 'r', 'LineWidth', 1.5, 'DisplayName', 'Kalman \xi_\theta before adjustment')
legend
ylabel('Residual')
xlabel('Time(s)')
% -------------------------------------------------------------------------
