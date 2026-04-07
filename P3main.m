% Contributors: 
% Course number: ASEN 3801
% File name: P3main
% Created: 4/7/26
clc
clear
close all

% Case 3
var_0 = [0 0 -1800
        deg2rad(15) deg2rad(-12) deg2rad(270)
        19 3 -2
        deg2rad(0.08) deg2rad(-0.2) 0]'; 
aircraft_surfaces = [deg2rad(5) deg2rad(2) deg2rad(-13) 0.3]';
wind_inertial = [0 0 0]';

doublet_time = 0.25;
doublet_size = deg2rad(15);

tspan = 0:0.01:3;


[time, x] = ode45(@(time,aircraft_state) AircraftEOMDoublet(time, var_0, aircraft_surfaces, doublet_size, ...
    doublet_time, wind_inertial, aircraft_parameters), tspan, var_0);

PlotAircraftSim(time,x,aircraft_surfaces,[1 2 3 4 5 6],['b', 'b', 'b', 'b', 'b', 'b', 'k']')
