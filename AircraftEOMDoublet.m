% Contributors: Victor Turpin Aguayo
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

%% Assign Useful Names to Variables (x,y,z,phi,theta,etc...)

State.x=var(1); State.y=var(2); State.z=var(3);             % Position
State.phi=var(4); State.theta=var(5); State.psi=var(6);     % Attitude
State.u=var(7); State.v=var(8); State.w=var(9);             % Velocity
State.p=var(10); State.q=var(11); State.r=var(12);          % Angular Rate

%% Doublet value 
if time < doublet_time
    aircraft_surfaces(1) = aircraft_surfaces(1) + doublet_size;
elseif time < (2*doublet_time)
    aircraft_surfaces(1) = aircraft_surfaces(1) - doublet_size;
else
    
end

%% Call aerodynamics force and moments function
density = stdatmo(-State.z);
[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);

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

In.x = I(1,1); In.y = I(2,2); In.z = I(3,3);

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
            *[State.p;State.q;State.r;]; 


% Allocate d/dt[Attiude]
Phi_dot = Angle_dot(1); Theta_dot = Angle_dot(2); Psi_dot = Angle_dot(3);

%% U_dot, V_dot, W_dot

% Calcualte d/dt[Velocity]
v_dot = cross([State.u,State.v,State.w],[State.p,State.q,State.r])' ...
    + g.*[-Trig.stheta;Trig.ctheta.*Trig.sphi ;Trig.ctheta.*Trig.cphi] ...
    +[aero_forces(1); aero_forces(2); aero_forces(3)]./m;

% ALlocate d/dt[Velocity]
U_dot = v_dot(1); V_dot = v_dot(2); W_dot = v_dot(3);


%% P_dot, Q_dot, R_dot

% Calculate d/dt[Angular Rate]
Omega_dot = [ ((In.y-In.z)./In.x).*State.q.*State.r ;...
              ((In.z-In.x)./In.y).*State.p.*State.r ;...
              ((In.x-In.y)./In.z).*State.q.*State.p ]...
           +[ (1/In.x).*aero_moments(1) ;...
              (1/In.y).*aero_moments(2) ;...
              (1/In.z).*aero_moments(3) ]...
           +[ (1/In.x).*aero_moments(1) ;...
              (1/In.y).*aero_moments(2) ;...
              (1/In.z).*aero_moments(3)];

% Allocate d/dt[Angular Rate]
P_dot = Omega_dot(1); Q_dot = Omega_dot(2); R_dot = Omega_dot(3);

% %% Ground Collison
% if(State.z>=0)        
%     Z_dot=0; 
%     U_dot=0;
%     V_dot=0;
%     W_dot=0;
%     P_dot=0;
%     Q_dot=0;
%     R_dot=0;
% end

%% Compile All Dotted States

xdot=[ X_dot; Y_dot; Z_dot; Phi_dot; Theta_dot; Psi_dot; U_dot; V_dot; W_dot; P_dot; Q_dot; R_dot];

end

