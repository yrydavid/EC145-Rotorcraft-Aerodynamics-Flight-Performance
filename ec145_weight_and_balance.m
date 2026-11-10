%% EC145/BK117 C-2: estimated longitudinal mass and CG
% Recomputes the six coursework payload cases from reported masses/arms.
% Envelope vertices are illustrative course inputs, not a certified
% flight-manual envelope. Some scenarios may lie outside this polygon.
clear; clc; close all;

g = 9.81;
emptyMass = 1792;                % Coursework empty mass [kg]
emptyMoment = 76468.46/g;       % Empty moment from Table 11 [kg m]
rotorAxis = 4.50;                % Datum station of main rotor [m]

% [CG station (m), gross mass (kg)]: idealized coursework polygon
envCG   = [4.337,4.377,4.544,4.667,4.337];
envMass = [2000, 3585, 3585, 1750, 2000];

% Scenario inputs from Tables 12-17 of the university report:
% [passengers row 1, row 2, row 3, fuel mass], all in kg.
cases = [ ...
      0    0    0   640;  % No passengers, full fuel
      0  160   80   640;  % Partial passengers, full fuel
     80  160  240   640;  % Maximum passengers, full fuel
      0    0    0     0;  % No passengers, no fuel
      0  160   80     0;  % Partial passengers, no fuel
     80  160  240     0];% Maximum passengers, no fuel

caseNames = { ...
    '0 passengers, full fuel'; ...
    'Partial passengers, full fuel'; ...
    'Max passengers, full fuel'; ...
    '0 passengers, no fuel'; ...
    'Partial passengers, no fuel'; ...
    'Max passengers, no fuel'};
paxArms = [3.30,3.95,4.10];     % Passenger positions [m]
pilotMass = 80; pilotArm = 2.85;
baggageMass = 70; baggageArm = 5.55;
fuelArm = 4.60;

nCases = size(cases,1);
totalMass = zeros(nCases,1);
xCG = zeros(nCases,1);
isWithin = false(nCases,1);
for i = 1:nCases
    passengers = cases(i,1:3);
    fuelMass = cases(i,4);
    totalMass(i) = emptyMass+pilotMass+baggageMass ...
                   +sum(passengers)+fuelMass;
    moment = emptyMoment+pilotMass*pilotArm ...
           +baggageMass*baggageArm ...
           +sum(passengers.*paxArms)+fuelMass*fuelArm;
    xCG(i) = moment/totalMass(i);
    [in,on] = inpolygon(xCG(i),totalMass(i),envCG,envMass);
    isWithin(i) = in | on;
end

fprintf('EC145 load cases recomputed from university report tables\n');
fprintf('These are not the manually specified Figure 21 point coordinates.\n');
for i = 1:nCases
    fprintf('%-30s : %5.0f kg, xCG = %.4f m, inside course polygon = %d\n', ...
        caseNames{i},totalMass(i),xCG(i),isWithin(i));
end

fig = figure('Color','w','Position',[100 100 1050 650]);
hold on;
fill(envCG,envMass,[0.94 0.95 0.98], ...
    'EdgeColor',[0.35 0.45 0.65],'LineWidth',1.8, ...
    'DisplayName','Coursework reference polygon');
xline(rotorAxis,'k--','LineWidth',1.3,'DisplayName','Rotor axis');
colors = lines(nCases);
for i = 1:nCases
    if isWithin(i)
        marker = 'o';
    else
        marker = 'x';
    end
    plot(xCG(i),totalMass(i),marker,'Color',colors(i,:), ...
        'MarkerSize',9,'LineWidth',1.6, ...
        'DisplayName',caseNames{i});
    text(xCG(i)+0.007,totalMass(i), ...
        sprintf('%.3f m, %.0f kg',xCG(i),totalMass(i)), ...
        'FontSize',8);
end
grid on;
xlabel('Longitudinal CG station [m]');
ylabel('Gross mass [kg]');
title('EC145 - estimated CG from coursework mass inputs');
xlim([4.18 4.72]); ylim([1700 3700]);
legend('Location','eastoutside');
hold off;

if ~exist('figures','dir'), mkdir('figures'); end
exportgraphics(fig,fullfile('figures','cg_envelope.svg'),'ContentType','vector');
