% Contributors: Victor Turpin Aguayo
% Course number: ASEN 3801
% File name: AircraftEOM
% Created: 4/7/26

function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)
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
g= aircraft_parameters.g;
m = aircraft_parameters.m;
%% Assign Useful Names to Variables (x,y,z,phi,theta,etc...)
State.x=aircraft_state(1); State.y=aircraft_state(2); State.z=aircraft_state(3);             % Position
State.phi=aircraft_state(4); State.theta=aircraft_state(5); State.psi=aircraft_state(6);     % Attitude
State.u=aircraft_state(7); State.v=aircraft_state(8); State.w=aircraft_state(9);             % Velocity
State.p=aircraft_state(10); State.q=aircraft_state(11); State.r=aircraft_state(12);          % Angular Rate

%% Call aerodynamics force and moments function
density = stdatmo(-State.z);
[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);
Aero.X = aero_forces(1); Aero.Y = aero_forces(2); Aero.Z = aero_forces(3); 
Aero.L = aero_moments(1); Aero.M = aero_moments(2); Aero.N = aero_moments(3); 

%% Set Up Euler Angle Structure 

% Calculate Trig values of Psi
Trig.cpsi = cos(State.psi); Trig.spsi = sin(State.psi);
Trig.tpsi = tan(State.psi);

% Calculate Trig values of Theta
Trig.ctheta = cos(State.theta); Trig.stheta = sin(State.theta);
Trig.ttheta = tan(State.theta);

% Calculate Trig values of Phi
Trig.cphi = cos(State.phi); Trig.sphi = sin(State.phi);
Trig.tphi = tan(State.phi);

%% Extracting Inertia Values

In.x = aircraft_parameters.Ix; In.y = aircraft_parameters.Iy; In.z = aircraft_parameters.Iz;
In.xz = aircraft_parameters.Ixz;

%% X_dot, Y_dot, Z_dot

% Calculate d/dt[Inertial Position]
Pos_dot=[Trig.ctheta.*Trig.cpsi,... % Calculate B to E DCM
    (Trig.sphi.*Trig.stheta.*Trig.cpsi)-(Trig.cphi.*Trig.spsi),...
    (Trig.cphi.*Trig.stheta.*Trig.cpsi)-(Trig.sphi.*Trig.spsi);...
    (Trig.ctheta.*Trig.spsi),...
    (Trig.sphi.*Trig.stheta.*Trig.spsi)+(Trig.cphi.*Trig.cpsi),...
    (Trig.cphi.*Trig.stheta.*Trig.spsi)-(Trig.sphi.*Trig.cpsi);...
    (-Trig.stheta),...
    (Trig.ctheta.*Trig.sphi),...
    (Trig.ctheta.*Trig.cphi)] * [State.u;State.v;State.w];

% Allocate d/dt[Inertial Position]
X_dot = Pos_dot(1);
Y_dot = Pos_dot(2);
Z_dot = Pos_dot(3);


%% Phi_dot, Theta_dot, Psi_dot

% Calculate d/dt[Attiude]
Angle_dot = [1, Trig.sphi.*Trig.ttheta, Trig.cphi.*Trig.ttheta;
             0,        Trig.cphi,             -Trig.sphi      ;
             0, Trig.sphi./Trig.ctheta, Trig.cphi./Trig.ctheta]...
            *[State.p;State.q;State.r]; 


% Allocate d/dt[Attiude]
Phi_dot = Angle_dot(1); Theta_dot = Angle_dot(2); Psi_dot = Angle_dot(3);

%% U_dot, V_dot, W_dot

% Calculate d/dt[Velocity]
v_dot = cross([State.u,State.v,State.w],[State.p,State.q,State.r])' ...
    + g.*[-Trig.stheta;Trig.ctheta.*Trig.sphi ;Trig.ctheta.*Trig.cphi] ...
    +[Aero.X; Aero.Y; Aero.Z]./m;
% +[0;0;Control.Z]./m;

% ALlocate d/dt[Velocity]
U_dot = v_dot(1); V_dot = v_dot(2); W_dot = v_dot(3);


%% P_dot, Q_dot, R_dot

% Calculate d/dt[Angular Rate]
Gamma = In.x * In.z - (In.xz)^2;
Gamma1 = In.xz * (In.x - In.y + In.z) / Gamma;
Gamma2 = (In.z * (In.z - In.y) + (In.xz)^2) / Gamma;
Gamma3 = In.z / Gamma;
Gamma4 = In.xz / Gamma;
Gamma5 = (In.z - In.x) / In.y;
Gamma6 = In.xz / In.y;
Gamma7 = (In.x * (In.x - In.y) + (In.xz)^2) / Gamma;
Gamma8 = In.x / Gamma;

Omega_dot = [ Gamma1*State.p*State.q - Gamma2*State.q.*State.r ;...
              Gamma5*State.p*State.r - Gamma6.*(State.p^2 - State.r^2) ;...
              Gamma7*State.p*State.q - Gamma1.*State.q.*State.r ]...
           + [ Gamma3*Aero.L + Gamma4.*Aero.N ;...
              Aero.M/In.y;...
              Gamma4*Aero.L + Gamma8.*Aero.N  ];
           
% +[ (1/In.x).*Control.L ;...
%               (1/In.y).*Control.M ;...
%               (1/In.z).*Control.N];

% Allocate d/dt[Angular Rate]
P_dot = Omega_dot(1); Q_dot = Omega_dot(2); R_dot = Omega_dot(3);

%% Compile All Dotted States

xdot=[ X_dot; Y_dot; Z_dot; Phi_dot; Theta_dot; Psi_dot; U_dot; V_dot; W_dot; P_dot; Q_dot; R_dot];

end

