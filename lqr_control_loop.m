function [ret] = lqr_control_loop(state,target_state,df_dstate,B,G,thrust_allocation, Q_elements, R_value)
% This function generates an lqr control loop using the current and target states.

    persistent du % If the LQR function fails to converge, then the previous du is returned
  
    mn = size(B);
    numthrusters = mn(2);
    
    % set the current state
    x = state(1);
    y = state(2);
    z = state(3);
    roll_ = state(4);
    pitch = state(5);
    yaw = state(6);
    u = state(7);
    v = state(8);
    w = state(9);
    p = state(10);
    q = state(11);
    r = state(12); 

    % Tune Parameters
    vehicle_radius = 0.2;
    Q = diag(Q_elements);
    R_elements = R_value*ones(1,numthrusters);
    R = diag(R_elements); 
    
    % Thruster conversion factors
    force_to_thrusteffort_forward = 0.1; % Conversion factor for forwards direction
    force_to_thrusteffort_backward = 1.0*force_to_thrusteffort_forward; % Conversion factor for backwards direction

    % If the vehicle moves out of the water the flotability decreases
    if z < 0.0
        radius = vehicle_radius - abs(z);
        if radius < 0.0
            radius = 0.0;
        end
    else 
        radius = vehicle_radius; 
    end
    
    gravity_effects = G(roll_,pitch,radius);
    
    % Calculate A matrix for current state
    A = df_dstate(x,y,z,roll_,pitch,yaw,u,v,w,p,q,r,radius);
    
    % Calculate lqr error
    lqr_error = state - transpose(target_state);
    
    % Ensure that the minimum distances between angles are used in the error
    % https://stackoverflow.com/a/2007279
    euler_diff = zeros(1,3);
    for i = 1:length(euler_diff)
        euler_diff(1,i) = atan2(sin(state(3 + i) - target_state(3 + i)),cos(state(3 + i) - target_state(3 + i)));
        lqr_error(3 + i, 1) = euler_diff(1,i);
    end
     
    % LQR Loop:
    try
        K = lqr(A,B,Q,R);
        
        du = -K*lqr_error; % Control thrust du in Newtons
        
        % Convert thrust-force(N) to thrust-effort(%)
        force_to_thrusteffort = zeros(1,numthrusters); 
        i1 = find(du >= 0.0);
        i2 = find(du <= 0.0);
        force_to_thrusteffort(i1) = force_to_thrusteffort_forward;
        force_to_thrusteffort(i2) = force_to_thrusteffort_backward;
        du = diag(force_to_thrusteffort)*du;

        % Add du to counteract gravity and buoyancy effects
        force_to_thrusteffort = eye(numthrusters).*force_to_thrusteffort_forward; 
        du_gravity_effects = thrust_allocation\gravity_effects; % Maps force to thrust (N)
        du_gravity_effects = force_to_thrusteffort*du_gravity_effects; % Convert thrust-force to thrust-effort
        %du = du - du_gravity_effects;
    catch  
    end
    
    ret = {A,B,du};
end