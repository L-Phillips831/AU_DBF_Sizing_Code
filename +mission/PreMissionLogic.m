%% PreMissionLogic %%
% Description: Logic that joins together the resizer methods and the
% mission model. Calculates required values for the mission model such as
% optimal climb characteristics, etc. Probably needs to be run for
% multiple mission configurations due to changing weights (payload/battery)

% Required values:
% CL_max
% mass
% g (gravitational acceleration)
% Sref
% rho
% k

% Additional options:
% - Have hard coded values like ground friction coefficient and nVals
% - logic that differentiates tricycle gear configs (aoaSet, vRot, etc)

% aoaSet = The aoa at forward ground run. Often determined by the landing
% gear configuration and the wing incidence angle. For tricycle gear and 0
% wing incidence, this is probably 0. For tail dragger, this is often the
% aoaLOF because the takeoff rotation segment doesn't exist.

% aoaLOF = The angle of attack for takeoff
aoaLOF = (CL_TO - CL0) / CL_Alpha;

% vLOF = velocity required for liftoff, assumed to be 1.3 * stall
weight = mass * g;
vStall = sqrt(weight*2/(Sref * rho * CL_Max));
vLOF = 1.3 * vStall;

% vROT = The speed at which elevator authority starts. Literally where the
% moment about the rear gear is above zero. Need distance that CG is
% forward of rear gear, the aircraft weight, thrust, centerline height
% above ground, and dCL/dE elevator effectiveness or d_alpha/dE and
% dCL/d_alpha for the elevator. vRot = vLOF for tail draggers.

% vClimb = The optimal airspeed to climb at. It is the point (airspeed) at which
% maximum available power/thrust occurs.

% hDotClimb = from vClimb, determine the hDot that would be achieved at
% vClimb
aoaTest = linspace(0, aoaMax, nVals);
vTest   = linspace(0, vMax, nVals);
for i = 1 : nVals % aoa iterations
    aoa = aoaTest(i);
    CL = CL0 + CL_Alpha * aoa;
    CD = CD0 + k*CL^2;
    for j = 1 : nVals % v iterations
        v = vTest(j);
        thrust = ThrustCalc(v);
        hDot   = (thrust - 0.5*rho*v^2 * CD * Sref)*v / weight;
        vGround 
    end
end