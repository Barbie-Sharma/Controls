%% Initialization for Two-Wheeled Robot
clear all;
close all;
clc;

% Times
T_c = 0.1; % sampling time
T = 260; % total number of time steps
t = [1:T]; % Time vector

% Gains
K_p1 = 1; % proportional
K_p2 = 1;
K_v1 = 1; % derivative
K_v2 = 1;

% Empty arrays
X = zeros(T,1);
Y = zeros(T,1);
THETA = zeros(T,1);

X_R = zeros(T, 1); % desired position
Y_R = zeros(T, 1);

DX_R = zeros(T,1); % first derivative (velocity) of desired
DY_R = zeros(T,1);

D2X_R = zeros(T,1); % second derivative
D2Y_R = zeros(T,1);

DCSI = zeros(T,1); % dynamic control signal input
V_D = zeros(T,1);
OMEGA_D = zeros(T,1);

EX = zeros(T,1);
EY = zeros(T,1);
ERR = zeros(T,1);

% Desired trajectory over time
for k = 1:T
  X_R(k) = k/200 + sin(k/100) / 1000;
  Y_R(k) = k/1000 + cos(k/20) / 10;
end

% Derivative of the desired trajectory over time
DX_R = deriva(X_R, T_c);
DY_R = deriva(Y_R, T_c);
% Doubled-derivative of the desired trajectory over time
D2X_R = deriva(DX_R, T_c);
D2Y_R = deriva(DY_R, T_c);

% Initial conditions
x0 = 0.5;
y0 = 0.5;
theta0 = pi;
X(1) = x0;
Y(1) = y0;
THETA(1) = theta0;

V_D(1) = sqrt((DX_R(1))^2 + (DY_R(1))^2);
OMEGA_D(1) = ((D2Y_R(1) * DX_R(1)) - (D2X_R(1) * DY_R(1))) / V_D(1);

%% TurtleBot URDF Initialization
turtlebot = importrobot('robotisTurtleBot3Burger.urdf');

% TurtleBot Parameters
wheelR = 0.033;
wheelSeparation = 0.16;
maxVelocity = 0.3;
time_step = T_c;
pose = [x0, y0, theta0];
jointPos = homeConfiguration(turtlebot);

LId = find(contains({jointPos.JointName}, 'wheel_left_joint'));
RId = find(contains({jointPos.JointName}, 'wheel_right_joint'));

% For plotting purposes
actualX = zeros(T, 1);
actualY = zeros(T, 1);
actualX(1) = x0;
actualY(1) = y0;

%% Main loop for Two-Wheeled Robot
for k = 2:T-1
  % Two-wheels robot model update
  X(k) = X(k-1) + V_D(k-1) * cos(THETA(k-1)) * T_c;
  Y(k) = Y(k-1) + V_D(k-1) * sin(THETA(k-1)) * T_c;
  THETA(k) = THETA(k-1) + OMEGA_D(k-1) * T_c;
  
  dx = (X(k) - X(k-1)) / T_c;
  dy = (Y(k) - Y(k-1)) / T_c;
  
  % Dynamic feedback linearization
  u1 = D2X_R(k) + K_v1 * (DX_R(k) - dx) + K_p1 * (X_R(k) - X(k));
  u2 = D2Y_R(k) + K_v1 * (DY_R(k) - dy) + K_p1 * (Y_R(k) - Y(k));
  DCSI(k) = u1 * cos(THETA(k)) + u2 * sin(THETA(k));
  
  V_D(k) = integra(DCSI, T_c); % integration
  OMEGA_D(k) = (u2 * cos(THETA(k)) - u1 * sin(THETA(k))) / V_D(k);
  
  % Controls saturation
  if abs(V_D(k)) > 5
    V_D(k) = 5 * sign(V_D(k));
  end
  if abs(OMEGA_D(k)) > 5
    OMEGA_D(k) = 5 * sign(OMEGA_D(k));
  end
  
  % Errors calculation
  EX(k) = X_R(k) - X(k);
  EY(k) = Y_R(k) - Y(k);
  ERR(k) = sqrt(EX(k)^2 + EY(k)^2);
  
  % TurtleBot URDF update
  v = V_D(k);
  omega = OMEGA_D(k);
  leftOmega = (v - (wheelSeparation / 2) * omega) / wheelR;
  rightOmega = (v + (wheelSeparation / 2) * omega) / wheelR;
  
  jointPos(LId).JointPosition = jointPos(LId).JointPosition + leftOmega * time_step;
  jointPos(RId).JointPosition = jointPos(RId).JointPosition + rightOmega * time_step;

  deltaTheta = wheelR / wheelSeparation * (rightOmega - leftOmega) * time_step;
  deltaX = wheelR / 2 * cos(pose(3) + deltaTheta / 2) * (leftOmega + rightOmega) * time_step;
  deltaY = wheelR / 2 * sin(pose(3) + deltaTheta / 2) * (leftOmega + rightOmega) * time_step;

  pose = pose + [deltaX, deltaY, deltaTheta];
  
  % For plotting purposes
  actualX(k) = pose(1);
  actualY(k) = pose(2);

  % Transform updates for visualization
  tform = trvec2tform([pose(1:2), 0]);
  tform(1:3, 1:3) = axang2rotm([0 0 1 pose(3)]);
  setFixedTransform(turtlebot.Bodies{1}.Joint, tform);

  % Show TurtleBot in updated position
  show(turtlebot, jointPos);
  axis([-1.5 1.5 -1.5 1.5 0 1.5]);
  pause(time_step);
end

%% Plot the trajectories
figure;
plot(X_R, Y_R, 'r', 'LineWidth', 2); hold on;
plot(actualX, actualY, 'b', 'LineWidth', 2);
legend('Desired Trajectory', 'Actual Trajectory');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('Desired vs. Actual Trajectory');

%% Plot velocity and omega
figure; plot(V_D); ylabel('v (m/s)'); xlabel('Time Step'); title('Linear Velocity');
figure; plot(OMEGA_D); ylabel('\omega (rad/s)'); xlabel('Time Step'); title('Angular Velocity');

%% Plot theta
figure; plot(THETA); ylabel('\theta (rad)'); xlabel('Time Step'); title('Orientation \theta');

%% Plot errors
figure; plot(EX); hold on; plot(EY, 'r'); ylabel('Error'); xlabel('Time Step');
legend('Error in X', 'Error in Y');
title('Tracking Errors in X and Y');

%% Support functions
function xint = integra(X, T)
N = length(X);
xint = 0;
for k = 2:N
  xint = xint + (T) * X(k-1) + (T / 2) * (X(k) - X(k-1));
end
end

function DX = deriva(X, T)
N = length(X);
DX = zeros(N,1);
for k = 2:N
  DX(k) = (X(k) - X(k-1)) / T;
end
DX(1) = DX(2);
end
