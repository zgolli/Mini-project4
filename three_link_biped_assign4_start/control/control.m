function u = control(t, q, dq, q0, dq0, step_number)
% You may call control_hyper_parameters and desired_outputs in this
% function
% you don't necessarily need to use all the inputs to this control function
global optData h_ i_ j_ k_ m_ n_ p_ plotsOn; %optimization data structure, along with global indices so it can be populated from other functions

if isfield(optData,'Kd1') %check to see if optData was initialized properly. assume if 1 field is initialized, they all are
    maxStep = optData(h_,i_,j_,k_,m_,n_,p_).maxStep;
    minStep = optData(h_,i_,j_,k_,m_,n_,p_).minStep;
    xScale = optData(h_,i_,j_,k_,m_,n_,p_).xScale; 
    Kp=[optData(h_,i_,j_,k_,m_,n_,p_).Kp1; optData(h_,i_,j_,k_,m_,n_,p_).Kp2];
    Kd=[optData(h_,i_,j_,k_,m_,n_,p_).Kd1; optData(h_,i_,j_,k_,m_,n_,p_).Kd2];
else
    minStep = pi/100; %length of first step
    maxStep = pi/30; %length of infinity-th step
    xScale = 2;
    Kp=[100; -100];
    Kd=[10 ; -10];
end
sigmoid = @(x) (maxStep-minStep).*(2./(1+exp(-x./xScale))-1)+minStep; %default sigmoid function reaches 99% in 4.5 steps
stepangle = sigmoid(step_number);

y=[q(3);q(2)-q(3)+stepangle];
dy=[dq(3);dq(2)-dq(3)];

u=  y.* Kp  + dy.*  Kd ;

if(abs(u(1))>30)
   u(1)=sign(u(1))*30;
end
    
if (abs(u(2))>30)
    u(2)=sign(u(2))*30;
end

% if exist('plotsOn','var')
%     if plotsOn == 1
%         figure (5)
%         plot(t,u(1),'ro');
%         plot(t,u(2),'bo');
%         legend('u1','u2');
%         hold on;
%     end
% end

if isfield(optData,'uNet')
    %keep track of the total control input 
    %NOTE: THE VALIDITY OF USING THE TOTAL RELIES ON THE TIME STEP BEING 
    %CONSTANT. TBD IF THIS IS THE CASE WITH ODE45
    optData(h_,i_,j_,k_,m_,n_,p_).uNet(1) = optData(h_,i_,j_,k_,m_,n_,p_).uNet(1) +u(1);
    optData(h_,i_,j_,k_,m_,n_,p_).uNet(2) = optData(h_,i_,j_,k_,m_,n_,p_).uNet(2) +u(2);
end
end