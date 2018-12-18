%% Solve equations of motion
% Note: eqns.m defines the equations of motion to be solved by this script
% This function returns the time vector T, the solution Y, the event time
% TE, solution at the event time YE.
% q0, dq0 are the initial angles and angular velocities, num_steps are the
% number of steps the robot is supposed to take
% As an example you can use q0 = [pi/6; -pi/3; 0] and dq0 = [0;0;8].

function sln = solve_eqns(q0, dq0, num_steps)

global optData h_ i_ j_ k_ m_ n_ p_ q_ h; %optimization data structure, along with global indices so it can be populated from other functions

options = odeset('RelTol',1e-5, 'Events', @event_func);
h = 0.001; % time step
tmax = 2; % max time that we allow for a single step
tspan = 0:h:tmax;
y0 = [q0; dq0];
t0 = 0;

% we define the solution as a structure to simplify the post-analyses and
% animation, we initialize it to null
sln.T = {};
sln.Y = {};
sln.TE = {};
sln.YE = {};

optData(h_,i_,j_,k_,m_,n_,p_,q_).ith_step_velocity = 0;
[~, ~, ~, l1, ~, ~, ~] = set_parameters;
t_help=0;


for i = 1:num_steps
    
        
    [T, Y, TE, YE] = ode45(@(t, y) eqns(t, y, y0, i), t0 + tspan, y0, options);
    sln.T{i} = T;
    sln.Y{i} = Y;
    sln.TE{i} = TE;
    sln.YE{i} = YE;
    if T(end) == tmax
        break
    end
    
    % Impact map
    q_m = YE(1:3)';
    dq_m = YE(4:6)';
    [q_p, dq_p] = impact(q_m, dq_m);
    
    y0 = [q_p; dq_p];
    t0 = T(end);
    
    step_time = sln.TE{i}(1)-t_help;
    q1i = sln.Y{1,i}(1,1);
    q1f = sln.Y{1,i}(end,1);
    step_dist = l1*(sin(q1f)-sin(q1i));
    optData(h_,i_,j_,k_,m_,n_,p_,q_).ith_step_velocity = step_dist/step_time; %avg step velocity measured at the hip
    t_help=sln.TE{i}(1);
end
if isfield(optData,'time')
    optData(h_,i_,j_,k_,m_,n_,p_,q_).time = t0;
    
    [~, ~, ~, l1, ~, ~, ~] = set_parameters;
    
    step_time = t0-T(1);
    q1i = Y(1,1);
    q1f = Y(end,1);
    step_dist = l1*(sin(q1f)-sin(q1i));
    optData(h_,i_,j_,k_,m_,n_,p_,q_).stepVel = step_dist/step_time; %avg step velocity measured at the hip
    
    
    %finding the final position, in much the same way as animate does
    r0 = [0; 0];
    for j = 1:num_steps-1
        Y = sln.Y{j};
        [x0, ~, ~, ~] = kin_swf(Y(end,1:3));
        r0 = r0 + [x0; 0]; %find the starting position after jth step
    end
    Y_f = sln.Y{end}; %
    q_f = Y_f(end,1:3); %angles of limbs at end
    x0 = r0(1); %offset position of hip after num_steps steps
    x_h_final = l1*sin(q_f(1)) + x0;
    optData(h_,i_,j_,k_,m_,n_,p_,q_).dist = x_h_final; %final hip position
    optData(h_,i_,j_,k_,m_,n_,p_,q_).avgVel = x_h_final/T(end); %average velocity measured at the hip after num_steps number of steps
end
end


