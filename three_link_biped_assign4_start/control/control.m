function u = control(t, q, dq, q0, dq0, step_number)
% You may call control_hyper_parameters and desired_outputs in this
% function
% you don't necessarily need to use all the inputs to this control function
global optData h_ i_ j_ k_ m_ n_ p_ q_ plotsOn; %optimization data structure, along with global indices so it can be populated from other functions
global h %stores timestep from solve_eqns.m
%%IF THE INDEXING IS CHANGED, CHANGES NEED TO BE REFLECTED IN THIS FILE,
%%control.m AND solve_eqns.m

% if isfield(optData,'Kd1') %check to see if optData was initialized properly. assume if 1 field is initialized, they all are
%     maxStep = optData(h_,i_,j_,k_,m_,n_,p_,q_).maxStep;
%     minStep = optData(h_,i_,j_,k_,m_,n_,p_,q_).minStep;
%     xScale = optData(h_,i_,j_,k_,m_,n_,p_,q_).xScale; 
%     Kp=[optData(h_,i_,j_,k_,m_,n_,p_,q_).Kp1; optData(h_,i_,j_,k_,m_,n_,p_,q_).Kp2];
%     Kd=[optData(h_,i_,j_,k_,m_,n_,p_,q_).Kd1; optData(h_,i_,j_,k_,m_,n_,p_,q_).Kd2];
%     torsoAngle = optData(h_,i_,j_,k_,m_,n_,p_,q_).torsoAngle;
%     Kd3=0;%optData(h_,i_,j_,k_,m_,n_,p_,q_).Kd3;    
% else
%     minStep = pi/100; %length of first step
%     maxStep = pi/35; %length of infinity-th step
%     xScale = 2;
%     Kp=[100; -100];
%     Kd=[10 ; -10];
%     Kd3 = 0;
%     torsoAngle = 0;
% end
% 
% maxStep=maxStep.*(tanh(-step_number/1600)+1);
% SCALE=20;
% fac=1;%(1+tanh(-step_number/295));
% torso_target = tanh(step_number/SCALE)*torsoAngle+.2;
% swing_target = tanh(fac*minStep*q(1)+0.001)*maxStep+0.01;
% swing_vel_target = (fac*minStep*maxStep)/(cosh(fac*minStep*q(1)))^2*dq(1);
% 
% y=[q(3)-torso_target;q(2)+swing_target];
% dy=[dq(3);dq(2)+swing_vel_target];
% 
% u=  y.* Kp  + dy.*  Kd +Kd3.*(1.5-optData.ith_step_velocity);
% 
% % Kd3=1000;
% % u=  y.* Kp  + dy.*  Kd + Kd3*(1.5-bestParams.velocity) ;
% 
% 
% if(abs(u(1))>30)
%    u(1)=sign(u(1))*30;
% end
%     
% if (abs(u(2))>30)
%     u(2)=sign(u(2))*30;
% end


if isfield(optData,'Kd1') %check to see if optData was initialized properly. assume if 1 field is initialized, they all are
    maxStep = optData(h_,i_,j_,k_,m_,n_,p_,q_).maxStep;
    minStep = optData(h_,i_,j_,k_,m_,n_,p_,q_).minStep;
    xScale = optData(h_,i_,j_,k_,m_,n_,p_,q_).xScale; 
    Kp=[optData(h_,i_,j_,k_,m_,n_,p_,q_).Kp1; optData(h_,i_,j_,k_,m_,n_,p_,q_).Kp2];
    Kd=[optData(h_,i_,j_,k_,m_,n_,p_,q_).Kd1; optData(h_,i_,j_,k_,m_,n_,p_,q_).Kd2];
    torsoAngle = optData(h_,i_,j_,k_,m_,n_,p_,q_).torsoAngle;
else
    minStep = pi/100; %length of first step
    maxStep = pi/30; %length of infinity-th step
    xScale = 2;
    Kp=[100; -100];
    Kd=[10 ; -10];
    torsoAngle = 0;
end
sigmoid = @(x) (maxStep-minStep).*(2./(1+exp(-x./xScale))-1)+minStep; %default sigmoid function reaches 99% in 4.5 steps
stepangle = sigmoid(step_number);

maxStep=1;
xScale=1.;
max_angle = pi/4*xScale;
if(q(1)<-max_angle || q(1) >max_angle)
   q2_target = -maxStep*sign(xScale*q(1));
   dq2_target = 0;
else
    q2_target = -maxStep*sin(xScale*q(1));
    dq2_target = -maxStep*xScale*cos(xScale*q(1))*dq(1);
end

y=[q(3)-torsoAngle;q(2)-q2_target+stepangle];
dy=[dq(3);dq(2)-dq2_target];

u=  y.* Kp  + dy.*  Kd;

if(abs(u(1))>30)
   u(1)=sign(u(1))*30;
end
    
if (abs(u(2))>30)
    u(2)=sign(u(2))*30;
end

if isfield(optData,'uNet')
    %keep track of the total control input 
    dBeta1 = dq(1) - dq(3);
    dBeta2 = dq(2) - dq(3);
    dt = h;
    incrEnergy1 = u(1)*dBeta1*dt; %incremental energy 
    incrEnergy2 = u(2)*dBeta2*dt;
    
    if(incrEnergy1 < 0)
        incrEnergy1 = 0; %only consider positive work
    end
    if(incrEnergy2 < 0)
        incrEnergy2 = 0; %only consider positive work
    end
    
    optData(h_,i_,j_,k_,m_,n_,p_,q_).uNet(1) = optData(h_,i_,j_,k_,m_,n_,p_,q_).uNet(1) + incrEnergy1; 
    optData(h_,i_,j_,k_,m_,n_,p_,q_).uNet(2) = optData(h_,i_,j_,k_,m_,n_,p_,q_).uNet(2) + incrEnergy2;
end
end