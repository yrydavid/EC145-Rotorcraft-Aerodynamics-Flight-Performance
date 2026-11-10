%% EC145/BK117 C-2: simplified power-balance coursework model
% This estimates power-limited intersections, not operational V_NE or a
% certified flight envelope. It does not include retreating-blade stall,
% advancing-tip Mach limits, rotor trim, or temperature corrections.
clear; clc; close all;

m = 3585;             % Reference mass [kg]
NB = 4;               % Blades
RB = 5.5;             % Rotor radius [m]
NR = 383;             % Rotor speed [rev/min]
CB = 0.32;            % Blade chord [m]
rho0 = 1.225;         % Density at sea level [kg/m^3]
g = 9.8;              % Acceleration due to gravity [m/s^2]
P0max = 1032e3;       % Combined continuous power [W]
kh = 1.15;            % Induced power correction in hover
kf = 1.15;            % Induced power correction in forward flight
K = 1.15;             % Profile-power speed correction
fA = 1.20;            % Equivalent drag area [m^2]
reserve = 0.04;       % Coursework power margin
xi = 1.15;            % Additional-loss factor

sigma = NB*CB/(pi*RB);
omega = NR*2*pi/60;
Vtip = omega*RB;
Arotor = pi*RB^2;
H = 0:1000:6000;      % Pressure-altitude cases [m]
V = [0,4:103];        % Shared physical speed grid [m/s]

rhoAt = @(h) rho0.*(20000-h)./(20000+h);
powerAt = @(h) P0max.*(1.11*rhoAt(h)./rho0-0.11);

nH = numel(H);
nV = numel(V);
Pavail = zeros(nH,1);
Preq = zeros(nH,nV);
wClimb = zeros(nH,nV);
wAuto = zeros(nH,nV);
aForward = nan(nH,nV);
Vmin = nan(nH,1);
Vmax = nan(nH,1);
Vopt = nan(nH,1);

for i = 1:nH
    rho = rhoAt(H(i));
    Pavail(i) = powerAt(H(i));
    CPav = Pavail(i)/(rho*Arotor*Vtip^3);
    CT = m*g/(rho*Arotor*Vtip^2);
    CL = 6*CT/sigma;
    CD = 0.011+0.065*(CL-0.5)^2;
    lambdaHover = sqrt(CT/2);

    for j = 1:nV
        speed = V(j);
        mu = speed/Vtip;
        Rd = 0.5*rho*fA*speed^2;

        if speed == 0
            lambdaI = lambdaHover;
            CP = kh*lambdaI*CT+sigma*CD/8;
            climbFactor = kh;
        else
            % Unique positive real root of r^4+a*r^2-1=0.
            a = (speed/(Vtip*lambdaHover))^2;
            r = sqrt(2/(sqrt(a^2+4)+a));
            lambdaI = r*lambdaHover;
            CP = kf*lambdaI*CT+sigma*CD*(1+K*mu^2)/8 ...
               +mu*Rd/(m*g)*CT;
            climbFactor = kf;
        end

        Preq(i,j) = (1+reserve)*CP*rho*Arotor*Vtip^3;
        lambdaClimb = (1-reserve)*CPav/(xi*CT) ...
            -climbFactor*lambdaI/xi ...
            -sigma*CD*(1+K*mu^2)/(8*xi*CT) ...
            -mu^3*fA/(2*xi*CT*Arotor);
        wClimb(i,j) = lambdaClimb*Vtip;
        lambdaAuto = climbFactor*lambdaI ...
            +sigma*CD*(1+K*mu^2)/(8*CT) ...
            +mu^3*fA/(2*CT*Arotor);
        wAuto(i,j) = -lambdaAuto*Vtip;

        if speed > 0
            aForward(i,j) = (Pavail(i)-Preq(i,j))/(m*speed);
        end
    end

    % Intersections use V(j) for both Preq(i,j) and Preq(i,j+1).
    % A missing power intersection is NaN, not a zero airspeed.
    if Preq(i,1) <= Pavail(i)
        Vmin(i) = 0;
    end
    for j = 1:nV-1
        d1 = Preq(i,j)-Pavail(i);
        d2 = Preq(i,j+1)-Pavail(i);
        if d1*d2 <= 0 && d1 ~= d2
            vCross = V(j)-d1*(V(j+1)-V(j))/(d2-d1);
            if d1 > 0 && d2 < 0
                Vmin(i) = vCross;
            elseif d1 < 0 && d2 > 0
                Vmax(i) = vCross;
            end
        end
    end
    [~,k] = min(Preq(i,:));
    Vopt(i) = V(k);
end

fprintf('EC145 simplified power-balance results (model estimates)\n');
fprintf(' H [m]  Vmin  Vopt  VpowerMax [km/h]\n');
for i = 1:nH
    fprintf('%5.0f  %6.2f  %6.2f  %8.2f\n',H(i), ...
        3.6*Vmin(i),3.6*Vopt(i),3.6*Vmax(i));
end

% Height above ground modifies induced power, not ambient altitude.
heightAGL = 0:0.5:15;
rho = rho0;
CT = m*g/(rho*Arotor*Vtip^2);
CL = 6*CT/sigma;
CD = 0.011+0.065*(CL-0.5)^2;
lambdaHover = sqrt(CT/2);
groundFactor = 1-0.5./(1+4*(heightAGL/RB).^2);
Pground = (groundFactor*kh*lambdaHover*CT+sigma*CD/8) ...
          *rho*Arotor*Vtip^3;

if ~exist('figures','dir'), mkdir('figures'); end
colors = lines(nH);
speedKmh = V*3.6;

f1=figure('Color','w'); hold on;
for i=1:nH
    plot(speedKmh,Preq(i,:)/1000,'Color',colors(i,:), ...
        'LineWidth',1.4,'DisplayName',sprintf('%d m',H(i)));
    yline(Pavail(i)/1000,'--','Color',colors(i,:), ...
        'HandleVisibility','off');
end
grid on; xlabel('Forward speed [km/h]'); ylabel('Power [kW]');
title('Required power and available power (simplified model)');
legend('Location','eastoutside');
exportgraphics(f1,fullfile('figures','power_required_available.svg'),'ContentType','vector');

f2=figure('Color','w'); hold on;
plot(3.6*Vmin,H,'o-','DisplayName','Lower power intersection');
plot(3.6*Vopt,H,'s-','DisplayName','Minimum-power speed');
plot(3.6*Vmax,H,'d-','DisplayName','Upper power intersection');
grid on; xlabel('Power-balance speed [km/h]'); ylabel('Altitude [m]');
title('Power-limited speed intersections (not an operating envelope)');
legend('Location','best');
exportgraphics(f2,fullfile('figures','flight_ceiling_envelope.svg'),'ContentType','vector');

f3=figure('Color','w'); hold on;
for i=1:nH
    plot(speedKmh,wClimb(i,:),'Color',colors(i,:), ...
        'DisplayName',sprintf('%d m',H(i)));
end
grid on; xlabel('Forward speed [km/h]'); ylabel('Climb-rate estimate [m/s]');
title('Climb-rate estimates from the simplified power balance');
legend('Location','eastoutside');
exportgraphics(f3,fullfile('figures','rate_of_climb.svg'),'ContentType','vector');

f4=figure('Color','w'); hold on;
for i=1:nH
    plot(speedKmh,wAuto(i,:),'Color',colors(i,:), ...
        'DisplayName',sprintf('%d m',H(i)));
end
grid on; xlabel('Forward speed [km/h]'); ylabel('Vertical velocity [m/s]');
title('Autorotation sink-rate approximation');
legend('Location','eastoutside');
exportgraphics(f4,fullfile('figures','autorotation_descent.svg'),'ContentType','vector');

f5=figure('Color','w');
plot(heightAGL,Pground/1000,'LineWidth',1.8);
grid on; xlabel('Rotor height above ground [m]');
ylabel('Estimated power [kW]');
title('Hover induced-power correction near the ground');
exportgraphics(f5,fullfile('figures','ground_effect_power.svg'),'ContentType','vector');

f6=figure('Color','w'); hold on;
plot(speedKmh,aForward(1,:),'DisplayName','Sea level');
plot(speedKmh,aForward(2,:),'DisplayName','1000 m');
grid on; xlabel('Forward speed [km/h]');
ylabel('Longitudinal acceleration estimate [m/s^2]');
title('Excess-power acceleration (nonzero forward speed)');
legend('Location','best');
exportgraphics(f6,fullfile('figures','horizontal_acceleration.svg'), ...
    'ContentType','vector');
