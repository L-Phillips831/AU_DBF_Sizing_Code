clear
close all
clc

% Create propellor object
prop = propeller(7,11*0.0254,"Propeller Data Files\PER3_11x8.dat");

% Create battery object
bt = battery(10000,6,100,"LiPo");

% Create motor object
mt = motor(470,130,0.0129);

% Create ESC object
esc = esc(60,80);

% Collect all objects into one propulsion system
propulsion = Propulsion(bt,esc,mt,prop);

n = 100;

tbl = table;
tbl.V = linspace(0,100,n)';
tbl.alt = ones(n,1).*0;
[tbl.T,tbl.a,tbl.P,tbl.rho] = atmosisa(tbl.alt);
tbl.q = 0.5.*tbl.rho.*tbl.V.^2;

tbl.Throttle = ones(n,1).*1;

tbl.RPM = propulsion.calcRPM(tbl.Throttle);
tbl.J = tbl.V ./ tbl.RPM .* 60 ./ propulsion.propeller.diameter;
tbl.Thrust = propulsion.propeller.GI_Thrust_fcn_V_RPM(tbl.V,tbl.RPM);
tbl.Power = propulsion.propeller.GI_Power_fcn_V_RPM(tbl.V,tbl.RPM);

tbl.dt = ones(n,1).*1;
tbl.dE = tbl.Power .* tbl.dt;


figure
hold on
yyaxis left
plot(tbl.V,tbl.Thrust)
ylim([0,ceil((max(tbl.Thrust)+10)/10)*10])
ylabel('Thrust (N)')

yyaxis right
plot(tbl.V,tbl.Power)
ylim([0,(ceil(max(tbl.Power)+100)/1000)*1000])
ylabel('Power (W)')

xlabel('Velocity (m/s)')

% 
% GI_T = propulsion.propeller.GI_Thrust_fcn_V_RPM;
% 
% maxThrust = max(max(GI_T.Values));
% 
% Tvec = linspace(0,maxThrust,10);
% Vvec = GI_T.GridVectors{1,1}';
% 
% Vmat = repmat(Vvec,1,length(Tvec));
% Tmat = repmat(Tvec,length(Vvec),1);
% 
% RPMvec = GI_T.GridVectors{1,2};
% RPMmat = zeros(length(Vvec),length(Tvec));
% 
% for i=1:length(Vvec)
%     T = GI_T(Vvec(i).*ones(1,length(RPMvec)),RPMvec);
% 
%     RPMmat(i,:) = interp1(T,RPMvec,Tvec,'linear');
% end

%propulsion.propeller.GI_RPM_fcn_V_Thrust = griddedInterpolant(Vmat,Tmat,RPMmat,'linear','linear');

rpmtest = propulsion.propeller.GI_RPM_fcn_V_Thrust(0,10);

thrusttest = propulsion.propeller.GI_Thrust_fcn_V_RPM(0,rpmtest);