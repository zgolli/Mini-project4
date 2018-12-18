%% Read the README_ASSIGN4.pdf to see what results you need to analyze here. 
function sln = analyze(sln)

    close all;    
    %Solution, sln, has 4 fields: T, Y, TE, YE
    %analyze the resulting walking gait and plots the results which include
    set(groot,'DefaultAxesFontSize',14);
    set(groot,'DefaultLineLineWidth',1.5);
    [m1, m2, m3, l1, l2, ~, g] = set_parameters();
    
    Y = cell2mat(sln.Y');
    t = cell2mat(sln.T');
    
    q = Y(:,1:3); 
    dq = Y(:,4:6);
    u = Y(:,7:8);

    d = l1*sin(q(:,1)); 
    index_shift = 1;
    %distance travelled by the hip
    for i=1:length(sln.T)
        [x0, ~, ~, ~] = kin_swf(sln.Y{i}(end,1:3),sln.Y{i}(end,4:6));
        index_shift = index_shift + length(sln.T{i});
        d(index_shift:end) = d(index_shift:end)+x0;
    end
    qdq_impact = cell2mat(sln.YE');
    t_impact = cell2mat(sln.TE');
    
    %convert to degrees...
    q = q*180/pi;
    dq = dq*180/pi;
    
    %plot angles over time
    figure(1);
    subplot(2,2,1);
    hold on;
    plot(t,q(:,1));
    plot(t,q(:,2));
    plot(t,q(:,3));
    legend('Stance Leg','Swing Leg','Torso');
    xlabel('Time (s)');
    ylabel('Joint Angle (deg)');
    title('Joint Angles Over Time');
    
    %-velocity of the robot vs time, 
    v = l1*cosd(q(:,1)).*dq(:,1);
    subplot(2,2,3);
    hold on;
    plot(t,v);
    xlabel('Time (s)');
    ylabel('Hip Velocity (m/s)');
    title('Hip Velocity Over Time');
    
    %-torques vs time, 
    subplot(2,2,2);
    hold on;
    plot(t,u(:,1));
    plot(t,u(:,2));
    legend('u1','u2');
    xlabel('Time (s)');
    ylabel('Actuator Torque (Nm)');
    title('Torque vs Time');
    
    %-cost of transport
    dBeta1 = dq(:,1)-dq(:,3);
    dBeta2 = dq(:,1)-dq(:,3);
    w1 = cumsum(dBeta1.*u(:,1));
    w2 = cumsum(dBeta2.*u(:,2));
    wnet = w1+w2;
    mnet = m1+m2+m3;
    COT = wnet./(mnet*g*d);    
    subplot(2,2,4);
    hold on;
    plot(t,COT);
    xlabel('Time (s)');
    ylabel('Cost of Transport');
    title('COT vs Time');    
    
    %-dispacement in each step vs. step number, 
    x_swf = l1*sin(qdq_impact(:,1)) - l2*sin(qdq_impact(:,2));
    figure(2);
    subplot(2,1,1);
    hold on;
    plot(x_swf,'o');
    xlabel('Step Number');
    ylabel('Step Length (m)');
    title('Step Length vs Step Number');
    
    %-step frequency vs step number, 
    subplot(2,1,2);
    f = 1./(t_impact(1:end)-[0;t_impact(1:(end-1))]);
    plot(f, 'o');
    xlabel('Step Number');
    ylabel('Step Frequency (Hz)');
    title('Step Length vs Step Number');    
    
    %plots of q_dot vs q for all 3 angles
    %for limit cycle - periodic orbit of legs -> should be able to detect
    %closed curve for any stable trajectory 
    figure(3);
    hold on;
    plot(q(:,1),dq(:,1));
    plot(q(:,2),dq(:,2));
    plot(q(:,3),dq(:,3));
    legend('Stance Leg','Swing Leg','Torso');
    xlabel('Joint Angle (deg)');
    ylabel('Joint Angular Velocity (deg/s)');
    title('Joint Angles vs Joint Angular Velocity');
        
end