%%Testing the animate function with different initial conditions here:
clc;
clear;
clf; 
close all;

q0 = [0;pi/3;pi/10];
dq0 = [0;0;0];
num_steps = 10;

sln = solve_eqns(q0,dq0,num_steps);

animate(sln);
