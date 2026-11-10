%% EC145 / BK117 C-2: blade-element / momentum coursework model
% Prescribed blade-pitch and flight conditions (not a trimmed flight state).
% Rotor totals are azimuthal averages over four identical blades.
clear; clc; close all;

R0 = 1.10;                % Inner blade station [m]
RB = 5.50;                % Main rotor radius [m]
NB = 4;                   % Blade count
NR = 395;                 % Rotor speed [rev/min]
CB = 0.30;                % Constant blade chord [m]
rho = 1.225;              % Reference air density [kg/m^3]
theta0 = 10;              % Root twist parameter [deg]
thetaTwist = 2;           % Twist rate [deg/m]
CL0 = 0.20;
CD0 = 0.01;

aspectRatio = (RB-R0)/CB;
CLalpha = (5.5*pi*aspectRatio)/(5.5+pi*aspectRatio); % [1/rad]
omega = NR*2*pi/60;       % [rad/s]
diskArea = pi*RB^2;
vTip = omega*RB;

nPsi = 60;                % Unique azimuth stations over [0, 360)
nR = 20;                  % Radial blade elements
dR = (RB-R0)/nR;
psi = (0:nPsi-1)*360/nPsi;
r = R0 + ((1:nR)-0.5)*dR;

caseNames = {'Hover input', 'Axial-climb input', ...
             'Forward-flight input', 'Lateral-control input'};
VcAll = [0, 5, 0, 0];           % Axial velocity [m/s]
UAll = [0, 0, 10, 5];           % Forward velocity [m/s]
alphaAll = [0, 0, 5, 2];       % Shaft incidence [deg]
AAll = [0, 0, 5, 2];           % Cyclic cosine [deg]
BAll = [0, 0, 0, 2];           % Cyclic sine [deg]
CAll = [1, 5, 2, 2];           % Prescribed collective offset [deg]

TpsiByCase = zeros(4,nPsi);
rotorT = zeros(4,1);
rotorQ = zeros(4,1);
rotorP = zeros(4,1);
CT = zeros(4,1);
CQ = zeros(4,1);
CP = zeros(4,1);

for cas = 1:4
    Vc = VcAll(cas);
    U = UAll(cas);
    alphaT = alphaAll(cas);
    A = AAll(cas);
    B = BAll(cas);
    C = CAll(cas);
    Qpsi = zeros(1,nPsi);
    nonConverged = 0;

    for i = 1:nPsi
        bladeT = 0;
        bladeQ = 0;
        beta = C + A*cosd(psi(i)) + B*sind(psi(i));

        for j = 1:nR
            rj = r(j);
            vi = 5;                   % Initial induced velocity [m/s]
            converged = false;

            for iteration = 1:500
                V0 = vi + Vc + U*sind(alphaT);
                V2 = omega*rj + U*cosd(alphaT)*sind(psi(i));
                Vrel = hypot(V0,V2);
                phi = atan2d(V0,V2);
                theta = theta0 - thetaTwist*rj;
                alphaEff = beta + theta - phi;

                CL = CL0 + CLalpha*deg2rad(alphaEff);
                CD = CD0 + CL^2/(pi*aspectRatio);
                dL = 0.5*rho*Vrel^2*CB*dR*CL;
                dD = 0.5*rho*Vrel^2*CB*dR*CD;
                dT = dL*cosd(phi) - dD*sind(phi);
                % Positive Q is resistive shaft torque for positive inflow.
                dQ = rj*(dL*sind(phi) + dD*cosd(phi));

                % Annular momentum uses total thrust from NB blades.
                viNew = NB*dT/(4*pi*rho*rj*max(V0,0.001)*dR);
                viNew = max(0,viNew);  % Elementary positive-inflow branch

                if abs(viNew-vi) < 0.01
                    converged = true;
                end
                vi = 0.9*vi + 0.1*viNew;
                if converged
                    break;
                end
            end

            if ~converged
                nonConverged = nonConverged+1;
            end
            bladeT = bladeT+dT;
            bladeQ = bladeQ+dQ;
        end

        TpsiByCase(cas,i) = bladeT;
        Qpsi(i) = bladeQ;
    end

    % Do not multiply by solidity: each strip represents one blade.
    rotorT(cas) = NB*mean(TpsiByCase(cas,:));
    rotorQ(cas) = NB*mean(Qpsi);
    rotorP(cas) = omega*rotorQ(cas);
    CT(cas) = rotorT(cas)/(rho*diskArea*vTip^2);
    CQ(cas) = rotorQ(cas)/(rho*diskArea*RB*vTip^2);
    CP(cas) = rotorP(cas)/(rho*diskArea*vTip^3);

    assert(abs(CP(cas)-CQ(cas)) < 1e-12, ...
        'The power and torque coefficients must agree when P = omega Q.');
    fprintf('%s: T = %.2f N, Q = %.2f N m, P = %.2f kW, ', ...
        caseNames{cas},rotorT(cas),rotorQ(cas),rotorP(cas)/1000);
    fprintf('CT = %.6f, CP = %.6f\n',CT(cas),CP(cas));
    if nonConverged > 0
        warning('EC145:BEMConvergence', ...
            '%s: %d blade elements reached the iteration limit.', ...
            caseNames{cas},nonConverged);
    end
end

% With prescribed pitch the rotor is not trimmed to equal aircraft weight.
referenceWeight = 3585*9.81;
fprintf('Hover-input thrust / reference aircraft weight = %.3f\n', ...
    rotorT(1)/referenceWeight);

fig = figure('Color','w','Position',[100 100 1050 620]);
hold on;
for cas = 1:4
    plot(psi,NB*TpsiByCase(cas,:)/1000,'LineWidth',1.6, ...
        'DisplayName',caseNames{cas});
end
grid on;
xlabel('Blade azimuth [deg]');
ylabel('Instantaneous four-blade thrust [kN]');
title('EC145/BK117 C-2: prescribed rotor-input cases');
xlim([0 360]);
legend('Location','eastoutside');
hold off;
if ~exist('figures','dir'), mkdir('figures'); end
exportgraphics(fig,fullfile('figures','bem_thrust_distribution.svg'), ...
    'ContentType','vector');
