function [df_dstate,df_dstate_sym,df_dcontrol,G,thrust_allocation] = robot_config()
% This function populates the symbolic state space model with the robot's
% actual parameters, and generates the lqr cost matrices.

syms x y z roll_ pitch yaw u v w p q r du0 du1 du2 du3 du4 du5 radius

% thruster configuation
thruster_position = [ -0.028, -0.296,  0.013;  %thruster 0
                      -0.028,  0.296,  0.013;  %thruster 1
                      -0.352,  0.000,  0.101;  %thruster 2
                       0.352,  0.000,  0.101;  %thruster 3
                       0.000,  0.005, -0.184;  %thruster 4
                       0.000,  0.005,  0.184]; %thruster 5


thrust_direction =  [ 1, 0, 0;  %thruster 0
                      1, 0, 0;  %thruster 1
                      0, 0, 1;  %thruster 2
                      0, 0, 1;  %thruster 3
                      0, 1, 0;  %thruster 4
                      0, 1, 0]; %thruster 5

% set up symbolic linear state space model
[state_symbolic,du_symbolic,F_dot,G_sym,thrust_allocation] = symbolic_state_space(thruster_position, thrust_direction);

% upload constant parameters (all SI units)

% NOTE: Thruster positions and directions must be updated in symbolic_state_space.m

% Gravity matrix parameters
displaced_water_volume = 0.035;
water_density = 1000.0;
gx = -0.0056;
gy = 0.0002;
gz = 0.0181;
bx = -0.0055;
by = 0.0007;
bz = 0.0140;
gravity = 9.81;

% Mass matrix parameters
mass = 34;
Ix = 1.44;
Iy = 1.11;
Iz = 1.79;
Ixy = -0.01;
Ixz = 0.066;
Iyz = 0.006;
mzg = mass*abs(gz);

% Added mass matrix parameters
added_mass = water_density*displaced_water_volume;
mass_ratio = added_mass/mass;
Xu_dot = mass_ratio*mass;
Yv_dot = mass_ratio*mass;
Zw_dot = mass_ratio*mass;
Kp_dot = mass_ratio*Ix;
Mq_dot = mass_ratio*Iy;
Nr_dot = mass_ratio*Iz;
Xq_dot = mass_ratio*mzg;
Yp_dot = mass_ratio*mzg;

% Damping matrix parameters

    % Linear Damping
    Xu = 130.0;
    Yv = 300.0;
    Zw = 150.0;
    Kp = 65.0;
    Mq = 65.0;
    Nr = 65.0;

    % Quadratic Damping
    Xuu = 155.0;
    Yvv = 200.0;
    Zww = 175.0;
    Kpp = 95.0;
    Mqq = 95.0;
    Nrr = 95.0;


% Substitute constant parameters
state_dot = subs(F_dot);
G = subs(G_sym);

% the system is linearized via the jacobian
df_dstate_sym(x, y, z, roll_, pitch, yaw, u, v, w, p, q, r, radius) = jacobian(state_dot,state_symbolic);
df_dstate = matlabFunction(df_dstate_sym);
df_dcontrol_sym(du0, du1, du2, du3, du4, du5) = jacobian(state_dot,transpose(du_symbolic));
df_dcontrol = matlabFunction(df_dcontrol_sym);
df_dcontrol = df_dcontrol(1,1,1,1,1,1);

% Gravity Matrix G
G(roll_, pitch, radius) = G;
G = matlabFunction(G);
end