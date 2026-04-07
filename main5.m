clc;clear;close all;

%% Task 2.1
ttwistor(); % Load aircraft_parameters structure

tspan = [0 10];
% Case 1
% var_0 = [0 0 -1609.34
%         0 0 0
%         21 0 0
%         0 0 0]'; 
% 
% aircraft_surfaces = [0 0 0 0]';

% Case 2
% var_0 = [0 0 -1800
%         0 0.0278 0
%         20.99 0 0.5837
%         0 0 0]'; 
% 
% aircraft_surfaces = [0.1079 0 0 0.3182]';

% Case 3
var_0 = [0 0 -1800
        deg2rad(15) deg2rad(-12) deg2rad(270)
        19 3 -2
        deg2rad(0.08) deg2rad(-0.2) 0]'; 

aircraft_surfaces = [deg2rad(5) deg2rad(2) deg2rad(-13) 0.3]';
wind_inertial = [0 0 0]';
ode_func = @(time,aircraft_state) AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters);
[time,x] = ode45(ode_func,tspan,var_0);
% [~, Control] = cellfun(ode_func, num2cell(t), num2cell(x,2),'UniformOutput',0);
% Z_c = zeros(length(t),1); 
% L_c = zeros(length(t),1);
% M_c = zeros(length(t),1);
% N_c = zeros(length(t),1);

PlotAircraftSim(time,x,aircraft_surfaces,[1 2 3 4 5 6],['b', 'b', 'b', 'b', 'b', 'b', 'k']')
