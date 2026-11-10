clear; clc; close all; 
 
%% a) INITIALIZARE DATE EC145 
 
R0 = 1.1;          % raza butucului [m] 
RB = 5.5;          % raza rotorului [m] 
 
NB = 4;            % numarul de pale [-] 
NR = 395;          % turatia rotorului [RPM] 
CB = 0.30;         % coarda medie a palei [m] 
 
ro = 1.225;        % densitatea aerului [kg/m^3] 
 
theta0 = 10;       % torsiunea palei la butuc [deg] 
theta_tw = 2;      % rata de torsiune a palei [deg/m] 
 
CL0 = 0.2;         % coeficient portanta la incidenta nula [-] 
CD0 = 0.01;        % coeficient rezistenta la incidenta nula [-] 
 
lambda = (RB - R0)/CB; 
CL_alpha = (5.5*pi*lambda)/(5.5 + pi*lambda); 
 
sigma = (NB*CB*(RB-R0))/(pi*(RB-R0)^2); 
 
n_psi = 61; 
n_r = 20;
dR = (RB - R0)/(n_r - 1); 
 
%% CAZURI CERUTE 
 
nume_caz = { 
    'Punct fix' 
    'Urcare' 
    'Zbor orizontal longitudinal' 
    'Zbor orizontal cu comanda laterala' 
}; 
 
Vc_all = [0 5 0 0]; 
U_all = [0 0 10 5]; 
alpha_T_all = [0 0 5 2]; 
 
A_all = [0 0 5 2]; 
B_all = [0 0 0 2]; 
C_all = [1 5 2 2]; 
 
%% RULARE PENTRU TOATE CAZURILE 
 
for caz = 1:4 
 
    Vc = Vc_all(caz); 
    U = U_all(caz); 
    alpha_T = alpha_T_all(caz); 
 
    A = A_all(caz); 
    B = B_all(caz); 
    C = C_all(caz); 
 
    psi = zeros(1,n_psi); 
    R = zeros(1,n_r); 
 
    x = zeros(n_psi,n_r); 
    y = zeros(n_psi,n_r); 
 
    Beta = zeros(n_psi,n_r); 
    Theta = zeros(n_psi,n_r); 
 
    V_I = zeros(n_psi,n_r); 
    V0 = zeros(n_psi,n_r); 
    V2 = zeros(n_psi,n_r); 
    V1 = zeros(n_psi,n_r); 
 
    phi = zeros(n_psi,n_r); 
    A_eff = zeros(n_psi,n_r); 
 
    dL = zeros(n_psi,n_r); 
    dD = zeros(n_psi,n_r); 
    dT = zeros(n_psi,n_r);
     dQ = zeros(n_psi,n_r); 
    dP = zeros(n_psi,n_r); 
 
    T = zeros(1,n_psi); 
    Q = zeros(1,n_psi); 
    P = zeros(1,n_psi); 
 
    %% b) For i = 1:n_psi 
 
    for i = 1:n_psi 
 
        %% Se calculeaza psi_i 
        psi(i) = (i-1)*360/(n_psi-1); 
 
        %% c) For j = 1:n_r 
 
        for j = 1:n_r 
 
            %% Se calculeaza R(j), x(i,j), y(i,j), Beta(i,j), Theta(i,j) 
 
            R(j) = R0 + (j-1)*(RB-R0)/(n_r-1); 
 
            x(i,j) = R(j)*cosd(psi(i)); 
            y(i,j) = R(j)*sind(psi(i)); 
 
            Beta(i,j) = C + A*cosd(psi(i)) + B*sind(psi(i)); 
 
            Theta(i,j) = theta0 - theta_tw*R(j); 
 
            %% d) Se initializeaza bucla while 
 
            V_I(i,j) = 5; 
            condition = false; 
            c = 0; 
 
            %% e) while(~condition) 
 
            while ~condition 
 
                %% Se calculeaza V0, V2, V1, phi, A_eff, dL, dD, dT, dQ, dP, new_V_I 
 
                V0(i,j) = V_I(i,j) + Vc + U*sind(alpha_T); 
 
                V2(i,j) = NR*2*pi/60*R(j) + ... 
                          U*cosd(alpha_T)*sind(psi(i)); 
 
                V1(i,j) = sqrt(V0(i,j)^2 + V2(i,j)^2); 
 
                phi(i,j) = atan2d(V0(i,j),V2(i,j)); 
 
                A_eff(i,j) = Beta(i,j) + Theta(i,j) - phi(i,j); 
                    CL = CL0 + CL_alpha*deg2rad(A_eff(i,j)); 
 
                CD = CD0 + (1/(pi*lambda))*CL^2; 
 
                dL(i,j) = 0.5*ro*V1(i,j)^2*CB*dR*CL; 
 
                dD(i,j) = 0.5*ro*V1(i,j)^2*CB*dR*CD; 
 
                dT(i,j) = dL(i,j)*cosd(phi(i,j)) - ... 
                          dD(i,j)*sind(phi(i,j)); 
 
                dQ(i,j) = R(j)*( ... 
                         -dL(i,j)*sind(phi(i,j)) + ... 
                          dD(i,j)*cosd(phi(i,j)) ); 
 
                dP(i,j) = NR*2*pi*dQ(i,j)/60; 
 
                new_V_I = dT(i,j)/(4*pi*ro*R(j)*max(V0(i,j),0.001)*dR); 
 
                %% f) if abs(new_V_I - V_I(i,j)) < 1.0e-2 
 
                if abs(new_V_I - V_I(i,j)) < 1.0e-2 
                    condition = true; 
                end 
 
                %% g) V_I(i,j) = 0.9*V_I(i,j) + 0.1*new_V_I 
 
                V_I(i,j) = 0.9*V_I(i,j) + 0.1*new_V_I; 
 
                %% h) If V_I(i,j)<0; V_I(i,j)=0; end 
 
                if V_I(i,j) < 0 
                    V_I(i,j) = 0; 
                end 
 
                %% i) c=c+1; if c==500; condition=true; end 
 
                c = c + 1; 
 
                if c == 500 
                    condition = true; 
                end 
 
            end 
 
            %% j) end while 
 
        end 
 
        %% k) end for j
           %% Se calculeaza T(i), Q(i), P(i) 
 
        T(i) = sum(dT(i,:)); 
        Q(i) = sum(dQ(i,:)); 
        P(i) = sum(dP(i,:)); 
 
    end 
 
    %% l) end for i 
    %% Se calculeaza T_a, Q_a, P_a, Ct_a, Cq_a si Cp_a 
 
    T_a = sigma*sum(T); 
    Q_a = sigma*sum(Q); 
    P_a = sigma*sum(P); 
 
    Ct_a = T_a/(0.5*ro*(NR*2*pi/60*RB)^2*NB*CB*(RB-R0)); 
    Cq_a = Q_a/(0.5*ro*(NR*2*pi/60*RB)^2*RB*NB*CB*(RB-R0)); 
    Cp_a = P_a/(0.5*ro*(NR*2*pi/60*RB)^2*NB*CB*(RB-R0)); 
 
    fprintf('\n==============================\n'); 
    fprintf('CAZ %d: %s\n', caz, nume_caz{caz}); 
    fprintf('T_a  = %.3f N\n', T_a); 
    fprintf('Q_a  = %.3f Nm\n', Q_a); 
    fprintf('P_a  = %.3f W\n', P_a); 
    fprintf('Ct_a = %.6f\n', Ct_a); 
    fprintf('Cq_a = %.6f\n', Cq_a); 
    fprintf('Cp_a = %.6f\n', Cp_a); 
 
    %% Se prezinta grafic variatiile T, Q si P cu unghiul psi 
 
    figure; 
    plot(psi,T,'LineWidth',1.5); 
    grid on; 
    title(['T(\psi) - ', nume_caz{caz}]); 
    xlabel('\psi [deg]'); 
    ylabel('T [N]'); 
 
    figure; 
    plot(psi,Q,'LineWidth',1.5); 
    grid on; 
    title(['Q(\psi) - ', nume_caz{caz}]); 
    xlabel('\psi [deg]'); 
    ylabel('Q [Nm]'); 
 
    figure; 
    plot(psi,P,'LineWidth',1.5); 
    grid on; 
    title(['P(\psi) - ', nume_caz{caz}]); 
    xlabel('\psi [deg]'); 
    ylabel('P [W]');
      %% Se prezinta grafic distributiile in planul rotorului 
 
    valori = {Beta, Theta, phi, A_eff, V_I, V0, V1, V2, ... 
              dL, dD, dT, dQ, dP}; 
 
    titluri = {'\beta', '\theta', '\phi', '\alpha_{eff}', ... 
               'V_I', 'V_0', 'V_1', 'V_2', ... 
               'dL', 'dD', 'dT', 'dQ', 'dP'}; 
 
    for k = 1:length(valori) 
 
        figure; 
        surf(x,y,valori{k}); 
        axis equal; 
        view([0 90]); 
        colorbar; 
        title([titluri{k}, '(x,y) - ', nume_caz{caz}]); 
        xlabel('x [m]'); 
        ylabel('y [m]'); 
 
    end 
 
end
