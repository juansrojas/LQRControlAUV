% This script simulates lqr control based on the current and target
% states using MATLAB functions.

clear;clear all;clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% State:
% [
%     x position (m)
%     y position (m)
%     z position [depth] (m)
%     roll (rad)
%     pitch (rad)
%     yaw (rad)
%     x velocity (m/s)
%     y velocity (m/s)
%     z velocity (m/s)
%     roll velocity (rad/s)
%     pitch velocity (rad/s)
%     yaw velocity (rad/s) 
% ]

% set initial state
state_0 = [0 0 0 0 0 0 0 0 0 0 0 0];

% set target state
target_state = [1 2 3 0 0 1.57 0 0 0 0 0 0];

% LQR tuning parameters
Q_elements = [1 1 4 4 1 1 3 3 3 4 4 4];  % Q value for each state element
R_value = 0.001;

% Set simulation parameters
t_lower = 0;  % time interval lower bound (s)
t_upper = 120; % time interval upper bound (s)
h = 2; % time step (s)
t_span = t_lower:h:t_upper;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Set up state space model
[df_dstate,df_dstate_sym,df_dcontrol,G,thrust_allocation] = robot_config();

% Simulate state space with lqr
[t_span,state] = ode45(@(t,state)state_eval(state,target_state,df_dstate,df_dcontrol,G,thrust_allocation, Q_elements, R_value),t_span,state_0);

% Plot simulation results
clf
figure(1)
subplot(2,1,1)
plot(t_span,state(:,1:6))
title('Robot Pose')
xlabel('time (s)')
ylabel('(m) or (rad)')
legend('x','y','z','roll','pitch','yaw')

hold on

subplot(2,1,2)
plot(t_span,state(:,7:12))
title('Robot Velocity')
xlabel('time (s)')
ylabel('(m/s) or (rad/s)')
legend('u','v','w','p','q','r')

%% Sub-Functions
function [ret] = state_eval(state,target_state,df_dstate,df_dcontrol,G,thrust_allocation, Q_elements, R_value)
% evaluates the state via ode45 with an lqr control loop

    persistent A B du
    
    [lqr_ret] = lqr_control_loop(state,target_state,df_dstate,df_dcontrol,G,thrust_allocation, Q_elements, R_value);

    A  = lqr_ret{1};
    B = lqr_ret{2};
    du = lqr_ret{3};
    
    ret = double(A*state + B*du);           
end