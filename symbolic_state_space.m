function [state,du,F_dot,G,thrust_allocation] = symbolic_state_space(thruster_position, thrust_direction)
% This function creates a fully symbolic non-linear state space model. The
% model is later used in to simulate the dynamics of the robot.

    % Parameters
    mn = size(thrust_direction);
    
    numthrusters = mn(1);
    
    syms x y z roll_ pitch yaw u v w p q r du0 du1 du2 du3 du4 du5 du6 du7 % state and control input

    syms mass Ix Iy Iz Ixy Ixz Iyz mzg % Mrb matrix

    syms Xu_dot Yv_dot Zw_dot Kp_dot Mq_dot Nr_dot Xq_dot Yp_dot % Ma matrix

    syms Xu Xuu Yv Yvv Zw Zww Kp Kpp Mq Mqq Nr Nrr % Damping matrices

    syms gx gy gz bx by bz gravity radius water_density % G matrix

    pose = [x y z roll_ pitch yaw];
    vel = [u v w p q r];

    state = sym(zeros(12,1));
    state(1:6) = transpose(pose);
    state(7:12) = transpose(vel);

    du = [du0 du1 du2 du3 du4 du5 du6 du7];
    du = du(1:numthrusters);

    gravity_center = [gx gy gz];
    buoyancy_center = [bx by bz];

    Mrb = sym([ mass,   0.0,   0.0,  0.0,  mzg,    0.0;
                 0.0,  mass,   0.0, -mzg,  0.0,    0.0;
                 0.0,  0.0,   mass,  0.0,  0.0,    0.0;
                 0.0, -mzg,    0.0,   Ix,  Ixy,    Ixz;
                 mzg,  0.0,    0.0,  Ixy,   Iy,    Iyz;
                 0.0,  0.0,    0.0,  Ixz,  Iyz,    Iz ]);

    Ma = sym([Xu_dot,    0.0,     0.0,    0.0, Xq_dot,   0.0;
                 0.0, Yv_dot,     0.0, Yp_dot,    0.0,   0.0;
                 0.0,    0.0,  Zw_dot,    0.0,    0.0,   0.0;
                 0.0, Yp_dot,     0.0, Kp_dot,    0.0,   0.0;
              Xq_dot,    0.0,     0.0,    0.0, Mq_dot,   0.0;
                 0.0,    0.0,     0.0,    0.0,    0.0, Nr_dot ]);

    linear_damping = sym([   -Xu,    0.0,     0.0,    0.0,    0.0,   0.0;
                             0.0,    -Yv,     0.0,    0.0,    0.0,   0.0;
                             0.0,    0.0,     -Zw,    0.0,    0.0,   0.0;
                             0.0,    0.0,     0.0,    -Kp,    0.0,   0.0;
                             0.0,    0.0,     0.0,    0.0,    -Mq,   0.0;
                             0.0,    0.0,     0.0,    0.0,    0.0,   -Nr]);

    quadratic_damping = sym([   -Xuu,    0.0,     0.0,    0.0,    0.0,   0.0;
                                 0.0,   -Yvv,     0.0,    0.0,    0.0,   0.0;
                                 0.0,    0.0,    -Zww,    0.0,    0.0,   0.0;
                                 0.0,    0.0,     0.0,   -Kpp,    0.0,   0.0;
                                 0.0,    0.0,     0.0,    0.0,   -Mqq,   0.0;
                                 0.0,    0.0,     0.0,    0.0,    0.0,  -Nrr]);

    %% Dynamics 
    M = sym(Mrb + Ma);

    C = sym(coriolisMatrix(M,state));

    D = sym(linear_damping + quadratic_damping);

    G = sym(gravityMatrix(state,mass,gravity,radius,water_density,gravity_center,buoyancy_center));

    %non-linear dynamics function f
    %(page 138 of Computer-Aided Control Systems Design, Chin 2013)
    f1 = sym(zeros(12,12));
    f1(1:6,7:12) = J(state);
    f1(7:12,7:12) = -M\(C - D);

    f2  = sym(zeros(12,1));
    f2(7:12,1) = -M\G;

    f = f1*state + f2;
    
    %% Control

    %thrust allocation matrix
    thrust_allocation= zeros(numthrusters,6);
    thrust_allocation(1:numthrusters,1:3)= thrust_direction; %maps XYZ forces

    for i = 1:numthrusters
        thrust_allocation(i,4:6) = cross(thruster_position(i,1:3),thrust_direction(i,1:3)); %maps XYZ torques
    end
    
    thrust_allocation = transpose(thrust_allocation);

    %control input u
    u_control = sym(zeros(1,numthrusters));
    
    for i = 1:numthrusters
    u_control(i) = du(i)*abs(du(i));
    end
    
    %generalized force tau
    tau = thrust_allocation*transpose(u_control);

    %control function g
    %(page 138 of Computer-Aided Control Systems Design, Chin 2013)
    g = sym(zeros(12,1));
    g(7:12,1) = M\tau;

    %% State space

    % non-linear state space model F_dot
    F_dot = sym(zeros(12,1));

    for i = 1:length(f)
        F_dot(i,1) = f(i,1) + g(i,1);
    end
end

%% Sub-Functions

function [ret] = s(vec)
% Creates the 3X3 anti-symmetric matrix from a 3 element input vector 
% (page 20 of Handbook of Marine Craft, Fossen 2011)
    ret = [0.0, -vec(3), vec(2); 
           vec(3), 0.0, -vec(1); 
          -vec(2), vec(1), 0.0 ];
end

function [C] = coriolisMatrix(M, state)
% Creates the coriolis matrix (page 53 of Handbook of Marine Craft, 2011)

    v1 = state(7:9);
    v2 = state(10:12);
    
    s1 = s(M(1:3,1:3)*v1 + M(1:3,4:6)*v2);
    s2 = s(M(4:6,1:3)*v1 + M(4:6,4:6)*v2);
    C = sym(zeros(6,6));
    C(1:3,4:6) = -s1;
    C(4:6,1:3) = -s1;
    C(4:6,4:6) = -s2;
end

function [G] = gravityMatrix(state,mass,gravity,radius,water_density,gravity_center,buoyancy_center)
% Creates the gravity matrix (page 60 of Handbook of Marine Craft, 2011)     

    [phi, theta, psi] = deal(state(4), state(5), state(6));

    %Weight, W and buoyancy force, F
    W = mass*gravity; %N
    
    pi = 3.14159265359;
    
    F_buoyancy = ((4/3)*pi*radius^3)*water_density*gravity;

    % gravity center position in the robot fixed frame (gx,gy,gz) [m]
    gx = gravity_center(1);
    gy = gravity_center(2);
    gz = gravity_center(3);
    
    % buoyancy center position in the robot fixed frame (bx,by,bz) [m]
    bx = buoyancy_center(1);
    by = buoyancy_center(2);
    bz = buoyancy_center(3);

    G = [(W - F_buoyancy)*sin(theta);
         -(W - F_buoyancy)*cos(theta)*sin(phi);
         -(W - F_buoyancy)*cos(theta)*cos(phi);
         -(gy*W - by*F_buoyancy)*cos(theta)*cos(phi) + (gz*W - bz*F_buoyancy)*cos(theta)*sin(phi);
         (gz*W - bz*F_buoyancy)*sin(theta) + (gx*W - bx*F_buoyancy)*cos(theta)*cos(phi);
         -(gx*W - bx*F_buoyancy)*cos(theta)*sin(phi) - (gy*W - by*F_buoyancy)*sin(theta)];
end

function [ret] = J(state)
% Transpforms from BODY to NED coordinates (page 26 of Handbook of Marine Craft, 2011)

    [phi, theta, psi] = deal(state(4), state(5), state(6));

    %the velocity is transformed from BODY to NED cooridnate system
    vel_NED = [cos(psi)*cos(theta), -sin(psi)*cos(phi)+cos(psi)*sin(theta)*sin(phi), sin(psi)*sin(phi)+cos(psi)*cos(phi)*sin(theta);
               sin(psi)*cos(theta), cos(psi)*cos(phi)+sin(phi)*sin(theta)*sin(psi), -cos(psi)*sin(phi)+sin(theta)*sin(psi)*cos(theta);
               -sin(theta), cos(theta)*sin(phi), cos(theta)*cos(phi)];

    %angular velocity is transformed from BODY to NED coordinate system
    angular_vel_NED = [1.0, sin(phi)*tan(theta), cos(phi)*tan(theta);
                       0.0, cos(phi), -sin(phi);
                       0.0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

    ret = sym(zeros(6,6));

    ret(1:3,1:3) = vel_NED;
    ret(4:6,4:6) = angular_vel_NED;
end