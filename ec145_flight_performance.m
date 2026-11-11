%% Main 
clc; clear all; close all; 
 
m = 3585; % system mass [kg] 
N_B = 4; % number of rotating arms 
R_B = 5.5; % arm radius [m] 
N_R = 383; % angular speed [RPM] 
C_B = 0.32; % arm chord (constant) [m] 
rho_0 = 1.225; % air density [kg/m3] 
g = 9.8; % gravity constant [m/s2] 
P0_max = 1032; % Max continous power at H=0m [kW] 
k_h = 1.15; % Coefficient for induced velocity distribution irregularities 
k_f = 1.15; 
K = 1.15; % empirical constant 
f_A = 1.2; % Equivalent flat plate area [m2] 
sigma = N_B*C_B/(pi*R_B); 
V_T = R_B*N_R*2*pi/60; 
reserve = 0.04; % reserve power for electronics
xsi = 1.15; % additional losses 
 
% Available power 
pwr_f = @(H) P0_max*(1.11*(rho_0*(20000-H)/(20000+H))/rho_0- 0.11); % Variation of power [kW] with altitude H[m] 
rho_f = @(H) rho_0*(20000-H)/(20000+H); 
 
% Setare specifica a inaltimilor cerute 
H = [0 1000 2000 3000 4000 5000 6000];
V = 4:103; % Forward-speed grid [m/s]; the first P column is hover  
 
for i=1:length(H) 
    P_AV(i) = pwr_f(H(i))*1000; 
    CP_AV(i) = P_AV(i)/(rho_f(H(i))*pi*R_B^2*V_T^3); 
end 
 
% Required power 
for i=1:length(H) 
    CT(i) = m*g/(rho_f(H(i))*R_B^2*pi*(R_B*N_R*2*pi/60)^2); 
    CL(i) = 6*CT(i)/sigma; 
    CD(i) = 0.011+0.065*(CL(i)-0.5)^2; 
     
    % STATIONARY MODE 
    lambda_h = sqrt(CT(i)/2); 
    CP(i,1) = k_h*lambda_h*CT(i)+sigma/8*CD(i); 
    P(i,1) = (1+reserve)*CP(i,1)*rho_f(H(i))*pi*R_B^2*V_T^3; 
    w_c(i,1) = V_T*((1-reserve)*CP_AV(i)/xsi/CT(i)-k_h*lambda_h/xsi-sigma/8*CD(i)/xsi/CT(i)); 
    w_aut(i,1) = -V_T*(k_h*lambda_h+sigma/8*CD(i)/CT(i)); 
     
    % HORIZONTAL MOTION 
    for j=1:100  
        % V(j) is associated with column j+1 of the power arrays 
        miu = V(j)/V_T; 
        p = [1 0 (V(j)/(V_T*sqrt(CT(i)/2)))^2 0 -1]; 
        r = roots(p); 
        for k=1:4 
            if (imag(r(k))==0)&&(real(r(k))>=0) 
                t=r(k); 
                lambda_i = r(k)*sqrt(CT(i)/2); 
            else 
            end 
        end 
        R_D = 1/2*rho_f(H(i))*f_A*V(j)^2; 
        CP(i,j+1) = k_f*lambda_i*CT(i)+sigma/8*CD(i)*(1+K*miu^2)+miu*R_D/(m*g)*CT(i); 
        P(i,j+1) =(1+reserve)*CP(i,j+1)*rho_f(H(i))*pi*R_B^2*V_T^3; 
         
        % Climb in progressive motion 
        lambda_c = (1-reserve)*CP_AV(i)/xsi/CT(i)-k_f*lambda_i/xsi-sigma/8*CD(i)/xsi/CT(i)*(1+K*miu^2)- miu^3/2/xsi/CT(i)*f_A/pi/R_B^2; 
        w_c(i,j+1) = lambda_c*V_T; 
         
        % Rate of descent 
           lambda_aut = k_f*lambda_i+sigma/8*CD(i)/CT(i)*(1+K*miu^2)+1/2*miu^3/CT(i)*f_A/pi/R_B^2; 
        w_aut(i,j+1) = -lambda_aut*V_T; 
         
        % Acceleration 
        a_h(i,j+1) = (P_AV(i)-P(i,j+1))/m/V(j); 
    end 
end 
 
% Surface effect 
HG = [0:0.5:15]; 
for i=1:length(HG) 
    CT_G(i) = m*g/(rho_f(HG(i))*R_B^2*pi*(R_B*N_R*2*pi/60)^2); 
    CL_G(i) = 6*CT_G(i)/sigma; 
    CD_G(i) = 0.011+0.065*(CL_G(i)-0.5)^2; 
     
    % STATIONARY MODE 
    lambda_hG = sqrt(CT_G(i)/2);  
    xsi_G(i) = 1-0.5/(1+4*(HG(i)/R_B)^2); 
    PG(i) = (xsi_G(i)*k_h*lambda_hG*CT_G(i)+sigma/8*CD_G(i))*rho_f(HG(i))*pi*R_B^2*V_T^3; 
end 
 
V_plot = [0,V]; % Exactly one velocity for every column of the power matrices

%% Maximum, Minimum and Optimum Power (Intersection calculation) 
for i=1:length(H) 
    Vmin(i)=NaN; Vmax(i)=NaN;
    if P(i,1) <= P_AV(i), Vmin(i) = 0; end 
     
    for j=1:length(V_plot)-1  
        if (P(i,j)>P_AV(i))&&(P(i,j+1)<P_AV(i)) 
            Vmin(i) = V_plot(j)+((P_AV(i)-P(i,j))/(P(i,j)-P(i,j+1)))*(V_plot(j)-V_plot(j+1)); 
        elseif (P(i,j)<P_AV(i))&&(P(i,j+1)>P_AV(i)) 
            Vmax(i) = V_plot(j)+((P_AV(i)-P(i,j))/(P(i,j)-P(i,j+1)))*(V_plot(j)-V_plot(j+1)); 
        else 
        end 
    end 
    [Pmin(i),index]=min(P(i,:)); 
    Vopt(i)=V_plot(index);
    wc_max(i)=max(w_c(i,:));
    waut_least(i)=max(w_aut(i,:)); 
end 
 
fprintf('     REZULTATE PERFORMANTE LA DIVERSE INALTIMI\n'); 
for i = 1:length(H) 
    fprintf('H = %4.0f m | Vmin = %6.2f km/h | Vopt = %6.2f km/h | Vpower = %6.2f km/h | climb = %6.2f m/s | autorot = %6.2f m/s\n', ...
        H(i), Vmin(i)*3.6, Vopt(i)*3.6, Vmax(i)*3.6, wc_max(i), waut_least(i)); 
end 
 
colors = jet(length(H));  
 
for i=1:length(H) 
    figure(1); hold on 
    plot(V_plot*3.6,P(i,:)/1000,'linewidth',1.5,'color',colors(i,:)) 
    plot(V_plot*3.6,ones(1,length(V_plot))*P_AV(i)/1000,'--','linewidth',1.3,'color',colors(i,:))
    figure(2); hold on 
plot(V_plot*3.6,w_c(i,:),'linewidth',1.5,'color',colors(i,:)) 
figure(3); hold on 
plot(V_plot*3.6,w_aut(i,:),'linewidth',1.5,'color',colors(i,:)) 
end 
figure(4);hold on 
plot(Vmin*3.6,H,'.-','linewidth',1.5) 
plot(Vmax*3.6,H,'.-','linewidth',1.5) 
plot(Vopt*3.6,H,'.-','linewidth',1.5) 
figure(5); hold on 
plot(HG, PG/1000) 
xline(R_B) 
yline(PG(end)/1000,'--') 
figure(6); hold on 
plot(V*3.6,a_h(1,2:end)) 
plot(V*3.6,a_h(2,2:end)) 
figure(1); 
grid on; grid minor; 
xlabel('V [km/h]'); ylabel('Power [kW]')
% Line style distinguishes required and available power; colors mark altitude.
hRequired = plot(nan,nan,'k-','LineWidth',1.5);
hAvailable = plot(nan,nan,'k--','LineWidth',1.3);
legend([hRequired,hAvailable],{'Power required','Power available'}, ...
    'Location','northeast'); 
xlim([0 380]) 
figure(2); 
grid on; grid minor; 
xlabel('V [km/h]'); ylabel('Climb rate [m/s]'); ylim([-10 18]) 
figure(3); 
grid on; grid minor; 
ylabel('Autorotation vertical speed [m/s]'); xlabel('V [km/h]'); ylim([-30 0]) 
figure(4); 
grid on; grid minor; 
xlabel('V [km/h]'); ylabel('H [m]'); legend('Vmin', 'Vmax','Vopt') 
figure(5); 
grid on; grid minor; 
xlabel('H [m]'); ylabel('P_G [kW]'); 
figure(6); 
grid on; grid minor; 
xlabel('V [km/h]'); ylabel('a [m/s^2]'); ylim([0 30])

% Export all six coursework performance figures with updated axis limits.
figDir = fullfile(fileparts(mfilename('fullpath')),'figures');
if ~exist(figDir,'dir'), mkdir(figDir); end
exportgraphics(figure(1),fullfile(figDir,'power_required_available.png'),'Resolution',200);
exportgraphics(figure(2),fullfile(figDir,'rate_of_climb.png'),'Resolution',200);
exportgraphics(figure(3),fullfile(figDir,'autorotation_descent.png'),'Resolution',200);
exportgraphics(figure(4),fullfile(figDir,'flight_ceiling_envelope.png'),'Resolution',200);
exportgraphics(figure(5),fullfile(figDir,'ground_effect_power.png'),'Resolution',200);
exportgraphics(figure(6),fullfile(figDir,'horizontal_acceleration.png'),'Resolution',200);
