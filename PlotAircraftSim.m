% Contributors: Victor Turpin Aguayo
% Course number: ASEN 3801
% File name: PlotAircraftSim

function PlotAircraftSim(time,aircraft_state_array,control_input_array,fig,col)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Inputs:   time = Time corresponding to the nth set of variables
%           aircraft_state_array = 12xn array of aircraft states
%           control_input_array = 4xn array of control inputs (Ec,Ac,Rc,Tc)
%           fig = 6x1 vector of figure numbers to plot over
%           col = string indicating the plotting option for each plot
%                           
% Outputs:  NO NUMERICAL. Only plots the following 6 figures:
%               - Four figures each with 3 subplots for intertial position,
%               euler angles, intertial velocity, and angular velocity
%               - One figure with subplots for each control input variable
%               - One figure that shows the 3D path for the aircraft, using
%               up as positive, and marking the start (green) and end (red)
%
% Methodology: Using inputted data from a simulation run, plot the aircraft
% states, control inputs, and 3D location over time, WITHOUT clearing the
% figures before plotting
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Assign Useful Names to Variables (x,y,z,phi,theta,etc...)

x = aircraft_state_array(:,1); y = aircraft_state_array(:,2); 
z = aircraft_state_array(:,3); phi = aircraft_state_array(:,4);
theta = aircraft_state_array(:,5); psi = aircraft_state_array(:,6);
u = aircraft_state_array(:,7); v = aircraft_state_array(:,8);
w = aircraft_state_array(:,9); p = aircraft_state_array(:,10);
q = aircraft_state_array(:,11); r = aircraft_state_array(:,12);
Ec = control_input_array(1); Ac = control_input_array(2);
Rc = control_input_array(3); Tc = control_input_array(4);

%% Plotting A/C State Vector Components (4 Figs w/Subplots)

figure(fig(1)); % Figure 1 for A/C Inertial Positions
sgtitle('Inertial Position Components vs Time')
subplot(3,1,1); plot(time,x,col(1,:),LineWidth=1); grid on; hold on;
ylabel('X-Position (m)'); xlim([0,max(time)]);
subplot(3,1,2); plot(time,y,col(2,:),LineWidth=1); grid on; hold on;
ylabel('Y-Position (m)'); xlim([0,max(time)]);
subplot(3,1,3); plot(time,z,col(3,:),LineWidth=1); grid on; hold on;
ylabel('Z-Position (m)'); xlim([0,max(time)]); xlabel('Time (s)');

figure(fig(2)); % Figure 2 for A/C Euler Angles
sgtitle('3-2-1 Body Euler Angles vs Time')
subplot(3,1,1); plot(time,phi,col(4,:),LineWidth=1); grid on; hold on;
ylabel('Roll Angle (rad)'); xlim([0,max(time)]);
subplot(3,1,2); plot(time,theta,col(5,:),LineWidth=1); grid on; hold on;
ylabel('Pitch Angle (rad)'); xlim([0,max(time)]);
subplot(3,1,3); plot(time,psi,col(6,:),LineWidth=1); grid on; hold on;
ylabel('Yaw Angle (rad)'); xlim([0,max(time)]); xlabel('Time (s)');

figure(fig(3)); % Figure 3 for A/C Air Relative Velocity
sgtitle('Air-Relative Velocity Components')
subplot(3,1,1); plot(time,u,col(1,:),LineWidth=1); grid on; hold on;
ylabel('X-Velocity (m/s)'); xlim([0,max(time)]);
subplot(3,1,2); plot(time,v,col(2,:),LineWidth=1); grid on; hold on;
ylabel('Y-Velocity (m/s)'); xlim([0,max(time)]);
subplot(3,1,3); plot(time,w,col(3,:),LineWidth=1); grid on; hold on;
ylabel('Z-Velocity (m/s)'); xlim([0,max(time)]); xlabel('Time (s)');

figure(fig(4)); % Figure 4 for A/C Angular Rates
sgtitle('3-2-1 Body Angular Rates')
subplot(3,1,1); plot(time,p,col(4,:),LineWidth=1); grid on; hold on;
ylabel('Roll Rate (rad/s)'); xlim([0,max(time)]);
subplot(3,1,2); plot(time,q,col(5,:),LineWidth=1); grid on; hold on;
ylabel('Pitch Rate (rad/s)'); xlim([0,max(time)]);
subplot(3,1,3); plot(time,r,col(6,:),LineWidth=1); grid on; hold on;
ylabel('Yaw Rate (rad/s)'); xlim([0,max(time)]); xlabel('Time (s)');

%% Plotting Control Inputs

figure(fig(5)); % Figure 5 for A/C Control Inputs
sgtitle('Control Inputs')
subplot(4,1,1); plot(time,rad2deg(Ec)*ones(length(time),1),col(3,:),LineWidth=1); grid on; hold on;
ylabel('Elevator deflection (deg)'); xlim([0,max(time)]);
subplot(4,1,2); plot(time,rad2deg(Ac)*ones(length(time),1),col(4,:),LineWidth=1); grid on; hold on;
ylabel('Aileron Deflection (deg)'); xlim([0,max(time)]);
subplot(4,1,3); plot(time,rad2deg(Rc)*ones(length(time),1),col(5,:),LineWidth=1); grid on; hold on;
ylabel('Rudder deflection (deg)'); xlim([0,max(time)]);
subplot(4,1,4); plot(time,Tc*ones(length(time),1),col(6,:),LineWidth=1); grid on; hold on;
ylabel('Throttle'); xlim([0,max(time)]); xlabel('Time (s)');

%% Plotting 3D Trajectory 

figure(fig(6)); % Figure 6 for A/C 3D Trajectory
hold on; grid on; view(3);
title('3D Trajectory')
plot3(x,y,-z,col(7,:),LineWidth=1); % 3D Trajectory
plot3(x(1),y(1),-z(1),'go'); plot3(x(end),y(end),-z(end),'ro') % Markers
xlabel('X-Positon (m)'); ylabel('Y-Position (m)'); zlabel('Altitude (m)')

end
