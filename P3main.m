% Contributors: 
% Course number: ASEN 3801
% File name: P3main
% Created: 4/7/26
clc
clear
close all
ttwistor();

% Case 2
var_0 = [0 0 -1800
        0 0.0278 0
        20.99 0 0.5837
        0 0 0]'; 

aircraft_surfaces = [0.1079 0 0 0.3182]';
wind_inertial = [0 0 0]';

doublet_time = 0.25;
doublet_size = deg2rad(15);

%% 3.2
% tspan = 0:0.1:3;
% 
% ode_func1 = @(time,aircraft_state) AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size, ...
%     doublet_time, wind_inertial, aircraft_parameters);
% [time,x] = ode45(ode_func1,tspan,var_0);
% PlotAircraftSim(time,x,aircraft_surfaces,[1 2 3 4 5 6],['b', 'b', 'b', 'b', 'b', 'b', 'k']')

%% 3.3
tspan = 0:0.1:100;
ode_func2 = @(time,aircraft_state) AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size, ...
    doublet_time, wind_inertial, aircraft_parameters);
[time,x] = ode45(ode_func2,tspan,var_0);

PlotAircraftSim(time,x,aircraft_surfaces,[1 2 3 4 5 6],['b', 'b', 'b', 'b', 'b', 'b', 'k']')