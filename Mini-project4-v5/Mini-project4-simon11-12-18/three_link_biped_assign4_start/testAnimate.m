%%Testing the animate function with different initial conditions here:
%clc;
clear all;
clf;
close all;
clearvars –global

global optData h_ i_ j_ k_ m_ n_ p_ q_ plotsOn; %optimization data structure, along with global indices so it can be populated from other functions

%number of search steps for each variable
NumKd = 1*[1, 1];
NumKp = 1*[1, 1];

%step sizes for each variable. start coarse, improve resolution around
%points of interest
StepKd = [1, 1];
StepKp = [10, 10];

if ~exist('bestParams.mat','file')
    %nominal gains to be optimized
    Kp0=[100; -100];
    Kd0=[10 ; -10];
    
    minSteps = [.35 .45 .55];
    maxSteps = [.1 .13 .16];
    xScales = [.46]; %manually input for now.. don't think it'll make a huge difference.
    %xScales correspond with ~convergence in 2, 4, 6, 12, and 24 steps
    %respectively
    
    torsoAngles = [0,.05,.1,.15,.2];
else
    load bestParams
    Kp0=[bestParams.Kp1; bestParams.Kp2];
    Kd0=[bestParams.Kd1 ; bestParams.Kd2];
    
    StepKd = .5*StepKd;%converge step size
    StepKp = .5*StepKp;
    
    minStep = bestParams.minStep;
    maxStep = bestParams.maxStep;
    xScale = bestParams.xScale;
    torsoAngle = 0.65;%.35;%bestParams.torsoAngle;
    minSteps = 1.048;%*[.8 .9 1 1.1 1.2];%minStep;%*(.5:.1:1.5);
    maxSteps =1.05;%*[.8 .9 1 1.1 1.2];%maxStep;%*(.5:.1:1.5);
    xScales = xScale;%*[.9 1 1.1];
    torsoAngles = torsoAngle;%+[-.05,-.025,0,.025,.05];
end

NumStepAngle = length(xScales);
NumMaxStep = length(maxSteps);
NumMinStep = length(minSteps);
NumTorsoAngles = length(torsoAngles);

%initial conditions for the walker
% q0 = [0;-pi/6;pi/10];
q0 = [0;0;0];
dq0 = [0;0;0];
num_steps = 25;

numIter = NumKd(1)*NumKd(2)*NumKp(1)*NumKp(2)*NumStepAngle*NumMaxStep*NumMinStep*NumTorsoAngles;

if(numIter < 5)
    plotsOn = 1;
else
    plotsOn = 0;
end

iterCount = 1;
%preallocating struct memory
for h_=NumKd(1):-1:1
    for i_=NumKd(2):-1:1
        for j_=NumKp(1):-1:1
            for k_=NumKp(2):-1:1
                for m_=NumStepAngle:-1:1
                    for n_=NumMaxStep:-1:1
                        for p_ = NumMinStep:-1:1
                            for q_ = NumTorsoAngles:-1:1
                                %parameters to be optimized
                                optData(h_,i_,j_,k_,m_,n_,p_,q_).Kd1 = Kd0(1)+StepKd(1)*(h_-ceil(NumKd(1)/2));
                                optData(h_,i_,j_,k_,m_,n_,p_,q_).Kp1 = Kp0(1)+StepKp(1)*(i_-ceil(NumKp(1)/2));
                                optData(h_,i_,j_,k_,m_,n_,p_,q_).Kd2 = Kd0(2)+StepKd(2)*(j_-ceil(NumKd(2)/2));
                                optData(h_,i_,j_,k_,m_,n_,p_,q_).Kp2 = Kp0(2)+StepKp(2)*(k_-ceil(NumKp(2)/2));
                                optData(h_,i_,j_,k_,m_,n_,p_,q_).xScale = xScales(m_); %how quickly step angle increases, using sigmoid function
                                optData(h_,i_,j_,k_,m_,n_,p_,q_).maxStep = maxSteps(n_); %max target step angle achieved by sigmoid function
                                optData(h_,i_,j_,k_,m_,n_,p_,q_).minStep = minSteps(p_); %max target step angle achieved by sigmoid function
                                optData(h_,i_,j_,k_,m_,n_,p_,q_).torsoAngle = torsoAngles(q_); %max target step angle achieved by sigmoid function
                                
                                %data to be output
                                optData(h_,i_,j_,k_,m_,n_,p_,q_).uNet = zeros(1,2); %total input given to each actuator, Nm
                                optData(h_,i_,j_,k_,m_,n_,p_,q_).dist = 0; %total distance travelled, measured by the hip, m
                                optData(h_,i_,j_,k_,m_,n_,p_,q_).time = 0; %total time, s
                                optData(h_,i_,j_,k_,m_,n_,p_,q_).stepVel = 0; %avg step velocity of the final step
                                optData(h_,i_,j_,k_,m_,n_,p_,q_).solution = solve_eqns(q0,dq0,num_steps);
                                
                                if(plotsOn==1)
                                    animate(optData(h_,i_,j_,k_,m_,n_,p_,q_).solution);
                                end
                                
                                strcat(['Processing Combination ',num2str(iterCount),' of ',num2str(numIter)])
                                iterCount = iterCount+1;
                            end
                        end
                    end
                end
            end
        end
    end
end

optDataVec = reshape(optData, [numel(optData),1]);
dist = [optDataVec.dist];
netEnergy = sum(abs([optDataVec.uNet]),2).*[optDataVec.time];
speed = [optDataVec.dist]./[optDataVec.time];
COT = netEnergy./dist; %value proportional to cost of transport (b/c gravity and mass are fixed)

[maxDist, i_maxDist] = maxk(dist,25); %start by taking the top distance to avoid high efficiency single step cases
[maxSpeed, i_maxSpeed] = maxk(speed,10);
[minCOT, i_minCOT] = mink(COT(i_maxDist),10);
i_minCOT = i_maxDist(i_minCOT);%swap around indices so they make more sense
[minEnergy, i_minEnergy] = mink(netEnergy(i_maxDist),10);
i_minEnergy = i_maxDist(i_minEnergy);%swap around indices so they make more sense

i = i_minCOT(1);
if exist('bestParams','var')
    priorBest = bestParams %display prior parameters to compare with new parameters
end

Kd1s = [optDataVec.Kd1];
bestParams.Kd1= Kd1s(i);
Kd2s = [optDataVec.Kd2];
bestParams.Kd2= Kd2s(i);
Kp1s = [optDataVec.Kp1];
bestParams.Kp1= Kp1s(i);
Kp2s = [optDataVec.Kp2];
bestParams.Kp2 = Kp2s(i);
xScales = [optDataVec.xScale];
bestParams.xScale = xScales(i);
maxSteps = [optDataVec.maxStep];
bestParams.maxStep = maxSteps(i);
minSteps= [optDataVec.minStep];
bestParams.minStep = minSteps(i);
torsoAngles = [optDataVec.torsoAngle];
bestParams.torsoAngle = torsoAngles(i);
bestParams.minCOT = minCOT(1);
bestParams.dist = dist(i);
bestParams.index = i;
Solutions = [optDataVec.solution];
bestParams.solution = Solutions(i);
velocities = [optDataVec.stepVel];
bestParams.velocity = velocities(i);
%bestParams.avgvelocity = optDataVec.avgVel(i);
save('bestParams','bestParams');

bestParams %display new optimal parameters

%testControllers([bestParams.solution],1); %animate the winner