% Contributors: Aidan McCarty
% Course number: ASEN 3801
% File name: AircraftEOM
% Created: 4/7/26

function xdot = AircraftEOMDoublet(time, aircraft_state, aircraft_surfaces, doublet_size, doublet_time, wind_inertial,aircraft_parameters)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:   time = simulation time
%           aircraft_state = 12x1 state vector [x,y,z,phi,theta,psi,u,v,w,p,q,r] 
%           aircraft_surfaces = 4 x 1 control surface vector
%           wind_inertial = 3 x 1 inertial wind velocity in inertial coordinates
%           aircraft_parameters = aircraft parameter structure
%
%
% Output:   xdot: 12x1 derivative of the state vector
%
% Methodology: Using derivations from class, calculate the dotted state
% vaiables given values for the previous step and control forces and
% moments. This function will be used by ode45 to simulate the QR flight. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Doublet value 
if time < doublet_time
    delta_e = aircraft_surfaces(1) + doublet_size;
elseif time < (2*doublet_time)
    delta_e = aircraft_surfaces(1) - doublet_size;
else 
    delta_e = aircraft_surfaces(1);
end

new_aircraft_surfaces = [delta_e, aircraft_surfaces(2), aircraft_surfaces(3), aircraft_surfaces(4)]';

xdot = AircraftEOM(time, aircraft_state, new_aircraft_surfaces, wind_inertial, aircraft_parameters);

end

