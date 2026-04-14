function wind_body = TransformFromInertialToBody(wind_inertial, aircraft_state);

State.phi = aircraft_state(1);
State.theta = aircraft_state(2);
State.psi = aircraft_state(3);

% Calculate Trig values of Psi
Trig.cpsi = cos(State.psi); Trig.spsi = sin(State.psi);
Trig.tpsi = tan(State.psi);

% Calculate Trig values of Theta
Trig.ctheta = cos(State.theta); Trig.stheta = sin(State.theta);
Trig.ttheta = tan(State.theta);

% Calculate Trig values of Phi
Trig.cphi = cos(State.phi); Trig.sphi = sin(State.phi);
Trig.tphi = tan(State.phi);


wind_body = [Trig.ctheta.*Trig.cpsi,... % Calculate E to B DCM
    (Trig.sphi.*Trig.stheta.*Trig.cpsi)-(Trig.cphi.*Trig.spsi),...
    (Trig.cphi.*Trig.stheta.*Trig.cpsi)-(Trig.sphi.*Trig.spsi);...
    (Trig.ctheta.*Trig.spsi),...
    (Trig.sphi.*Trig.stheta.*Trig.spsi)+(Trig.cphi.*Trig.cpsi),...
    (Trig.cphi.*Trig.stheta.*Trig.spsi)-(Trig.sphi.*Trig.cpsi);...
    (-Trig.stheta),...
    (Trig.ctheta.*Trig.sphi),...
    (Trig.ctheta.*Trig.cphi)]' * wind_inertial;

end