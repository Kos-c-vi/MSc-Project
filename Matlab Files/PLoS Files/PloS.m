%% Matlab script to evaluate and plot the Probability of Line of Sight
% by Kossivi Fangbemi
% Part of the Masters disertation

% Resources: https://www.mathworks.com/matlabcentral/fileexchange/26737-legendre-laguerre-and-hermite-gauss-quadrature

%***********************************************************************%
set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');

% The integral expression for the probability 
% See in the dissertation

% Computing the nodes xi and their coresponding weight wi for the Gauss-Hermite
% Quadrature.
m = 128; % Order of the Hermite Plynomial and upper limit of the summation
f = zeros(m,1); f1 = zeros(m,1); f2 = zeros(m,1); 
fD = zeros(m,1); fD1 = zeros(m,1); fD2 = zeros(m,1);
% Calling function for get the value of xi and wi
[x, w] = GaussHermite_2(m); 
%% Computing the probability as function of antenna height.
% Antenna heigth
hant = linspace(0,10,1000);
Plos = zeros(1,1000); Plos1 = zeros(1,1000); Plos2 = zeros(1,1000);
%% Case 1: U10 = 7.5 m/s
% The parameters of the wave field
    % The wind speed 
    U10 = 7.5;
    % Distance separation the two nodes in metres.
    D = 5000;
    % The Earth curvature
    Deltah = 6371000 - sqrt((6371000^2) - (D/2)^2);
    % Definition of the  PM spectrum
    U19 = 1.075*U10;
    g = 9.81;
    alpha = 0.0081;
    beta = 0.71;
    syms omega;
    SPM2 = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);
    % The variance of the surface elevation from the PM SPECTRUM
    variances = double(int(SPM2,omega,0,Inf));
    % The std of the surface height
    sigmas = sqrt(variances);
    % The RMS Height of the wave
    H = 2*sqrt(2)*sigmas;
    % The wavelenght of the wave
    lambda = 2.*pi*(U19^2)/((0.877^2)*g);
    % The  nuber of wave sampled
    N = D/lambda;
% Calculating the Probability for every value of hant
for j = 1:length(hant)
   %Computing the summation 
    for i = 1:1:m
        f(i) = w(i)*(1/sqrt(pi))*(1 - exp(-4*(sigmas*x(i) + hant(j) - Deltah)^2/(H^2)))^N;
    end 
    Plos(j) = sum(f);
end

%% Case 2: U10 = 10 m/s
% The parameters of the wave field
    % The wind speed 
    U10 = 10;
    % Distance separation the two nodes.
    D = 5000;
    % The Earth curvature
    Deltah = 6371000 - sqrt((6371000^2) - (D/2)^2);
    % Definition of the  PM spectrum
    U19 = 1.075*U10;
    g = 9.81;
    alpha = 0.0081;
    beta = 0.71;
    syms omega;
    SPM2 = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);
    % The variance of the surface elevation from the PM SPECTRUM
    variances = double(int(SPM2,omega,0,Inf));
    % The std of the surface height
    sigmas = sqrt(variances);
    % The RMS Height of the wave
    H = 2*sqrt(2)*sigmas;
    % The wavelenght of the wave
    lambda = 2.*pi*(U19^2)/((0.877^2)*g);
    % The  nuber of wave sampled
    N = D/lambda;
% Calculating the Probability for every value of hant
for j = 1:length(hant)
   %Computing the summation 
    for i = 1:1:m
        f1(i) = w(i)*(1/sqrt(pi))*(1 - exp(-4*(sigmas*x(i) + hant(j) - Deltah)^2/(H^2)))^N;
    end 
    Plos1(j) = sum(f1);
end

%% Case 3: U10 = 12.5 m/s
% The parameters of the wave field
    % The wind speed 
    U10 = 12.5;
    % Distance separation the two nodes.
    D = 5000;
    % The Earth curvature
    Deltah = 6371000 - sqrt((6371000^2) - (D/2)^2);
    % Definition of the  PM spectrum
    U19 = 1.075*U10;
    g = 9.81;
    alpha = 0.0081;
    beta = 0.71;
    syms omega;
    SPM2 = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);
    % The variance of the surface elevation from the PM SPECTRUM
    variances = double(int(SPM2,omega,0,Inf));
    % The std of the surface height
    sigmas = sqrt(variances);
    % The RMS Height of the wave
    H = 2*sqrt(2)*sigmas;
    % The wavelenght of the wave
    lambda = 2.*pi*(U19^2)/((0.877^2)*g);
    % The  nuber of wave sampled
    N = D/lambda;
% Calculating the Probability for every value of hant
for j = 1:length(hant)
   %Computing the summation 
    for i = 1:1:m
        f2(i) = w(i)*(1/sqrt(pi))*(1 - exp(-4*(sigmas*x(i) + hant(j) - Deltah)^2/(H^2)))^N;
    end 
    Plos2(j) = sum(f2);
end

figure('Name','PLOS_Hant','Units','centimeters','position',[5 5 23 9]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'FontWeight','bold','fontsize',12,'FontName','Romain')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 9],'PaperSize',[21 9]);
plot(hant,Plos,hant,Plos1,hant,Plos2,'LineWidth',2), ylabel('\bf{Probability of Line of sight}'),
xlabel('$\mathbf{h_{ant}~~(m)}$'),box on, grid on, legend('$\mathbf{U_{10}~ =~ 7.5 ~m/s}$','$\mathbf{U_{10}~ =~ 10 ~m/s}$','$\mathbf{U_{10}~ =~ 12.5 ~m/s}$','Location','northeastoutside');
%print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\Plos_hant.pdf','-dpdf');

%% Computing the probability as function of the distance D between the two nodes.
% The distance D
D = linspace(0,10000,1000);
PlosD = zeros(1,1000); PlosD1 = zeros(1,1000); PlosD2 = zeros(1,1000);
Delth = 0*D; Delth1 = 0*D; Delth2 = 0*D;
N = 0*D; N1 = 0*D; N2 = 0*D;
%% Case 1: U10 = 7.5 m/s
% The parameters of the wave field
    % The wind speed 
    U10 = 7.5;
    % Antenna height in metres.
    hant = 2.00;
    % Wind speed at 19.5 metres above sea level
    U19 = 1.075*U10;
    % Gravitational acceleration
    g = 9.81;
    % Definition of the  PM spectrum
    alpha = 0.0081;
    beta = 0.71;
    syms omega;
    SPM2 = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);
    % The variance of the surface elevation from the PM SPECTRUM
    variances = double(int(SPM2,omega,0,Inf));
    % The std of the surface height
    sigmas = sqrt(variances);
    % The RMS Height of the wave
    H = 2*sqrt(2)*sigmas;
    % The wavelenght of the wave
    lambda = 2.*pi*(U19^2)/((0.877^2)*g);
% Calculating the Probability for every value of hant
for j = 1:length(D)
   % The Earth curvature
   Delth(j) = 6371000 - sqrt((6371000^2) - (D(j)/2)^2);
   % The  nuber of wave sampled
   N(j) = D(j)/lambda;
   %Computing the summation 
    for i = 1:1:m
        fD(i) = w(i)*(1/sqrt(pi))*(1 - exp(-4*(sigmas*x(i) + hant - Delth(j))^2/(H^2)))^N(j);
    end 
    PlosD(j) = sum(fD);
end

%% Case 2: U10 = 10 m/s
% The parameters of the wave field
    % The wind speed 
    U10 = 10;
    % Antenna height in metres.
    hant = 2.00;
    % Wind speed at 19.5 metres above sea level
    U19 = 1.075*U10;
    % Gravitational acceleration
    g = 9.81;
    % Definition of the  PM spectrum
    alpha = 0.0081;
    beta = 0.71;
    syms omega;
    SPM2 = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);
    % The variance of the surface elevation from the PM SPECTRUM
    variances = double(int(SPM2,omega,0,Inf));
    % The std of the surface height
    sigmas = sqrt(variances);
    % The RMS Height of the wave
    H = 2*sqrt(2)*sigmas;
    % The wavelenght of the wave
    lambda = 2.*pi*(U19^2)/((0.877^2)*g);
% Calculating the Probability for every value of hant
for j = 1:length(D)
   % The Earth curvature
   Delth1(j) = 6371000 - sqrt((6371000^2) - (D(j)/2)^2);
   % The  nuber of wave sampled
   N1(j) = D(j)/lambda;
   %Computing the summation 
    for i = 1:1:m
        fD1(i) = w(i)*(1/sqrt(pi))*(1 - exp(-4*(sigmas*x(i) + hant - Delth1(j))^2/(H^2)))^N1(j);
    end 
    PlosD1(j) = sum(fD1);
end

%% Case 3: U10 = 12.5 m/s
% The parameters of the wave field
    % The wind speed 
    U10 = 12.5;
    % Antenna height in metres.
    hant = 2.00;
    % Wind speed at 19.5 metres above sea level
    U19 = 1.075*U10;
    % Gravitational acceleration
    g = 9.81;
    % Definition of the  PM spectrum
    alpha = 0.0081;
    beta = 0.71;
    syms omega;
    SPM2 = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);
    % The variance of the surface elevation from the PM SPECTRUM
    variances = double(int(SPM2,omega,0,Inf));
    % The std of the surface height
    sigmas = sqrt(variances);
    % The RMS Height of the wave
    H = 2*sqrt(2)*sigmas;
    Hb2 = H;
    % The wavelenght of the wave
    lambda = 2.*pi*(U19^2)/((0.877^2)*g);
% Calculating the Probability for every value of hant
for j = 1:length(D)
   % The Earth curvature
   Delth2(j) = 6371000 - sqrt((6371000^2) - (D(j)/2)^2);
   % The  nuber of wave sampled
   N2(j) = D(j)/lambda;
   %Computing the summation 
    for i = 1:1:m
        fD2(i) = w(i)*(1/sqrt(pi))*(1 - exp(-4*(sigmas*x(i) + hant - Delth2(j))^2/(H^2)))^N2(j);
    end 
    PlosD2(j) = sum(fD2);
end
D1 = linspace(0,10,1000);
figure('Name','PLOS_D','Units','centimeters','position',[5 5 23 9]); set(0,'defaultfigurecolor',[1 1 1 ]); set(gca,'FontWeight','bold','fontsize',12,'FontName','Romain')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 9],'PaperSize',[21 9]);
plot(D1,PlosD,D1,PlosD1,D1,PlosD2,'LineWidth',2), ylabel('\bf{Probability of Line of sight}'),
xlabel('$\mathbf{D~~(km)}$'),box on, grid on, legend('$\mathbf{U_{10}~ =~ 7.5 ~m/s}$','$\mathbf{U_{10}~ =~ 10 ~m/s}$','$\mathbf{U_{10}~ =~ 12.5 ~m/s}$','Location','northeastoutside');
%print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\Plos_distance.pdf','-dpdf');

%% Computing the probability as function of the wind speed.
clear U19 SPM3 variances sigmas lambda H
% The wind speed
U10 = linspace(3/1.075,10/1.075,1000);
PlosU = 0.*U10; U19 = 0.*U10; SPM3 = 0.*U10; variances = 0.*U10; 
sigmas = 0.*U10; lambda = 0.*U10; H = 0.*U10; N3 = 0.*U10;
%% Case 1: D = 5 km
% The parameters of the wave field
    % The distance 
    D = 5000;
    % The Earth curvature
    Delh = 6371000 - sqrt((6371000^2) - (D/2)^2);
    % Antenna height in metres.
    hant = 1.00;
    % Gravitational acceleration
    g = 9.81;
    % Definition of the  PM spectrum constants
    alpha = 0.0081;
    beta = 0.71;
% Calculating the Probability for every value of hant
for j = 1:length(U10)
   % Wind speed at 19.5 metres above sea level
   U19(j) = 1.075.*U10(j);
   syms omega;
   SPM3 = (alpha.*g.*g./(omega.^5)).*exp(-beta.*(g./(U19(j).*omega)).^4);
   % The variance of the surface elevation from the PM SPECTRUM
   variances(j) = double(int(SPM3,omega,0,Inf));
   % The std of the surface height
   sigmas(j) = sqrt(variances(j));
   % The RMS Height of the wave
   H(j) = 2*sqrt(2)*sigmas(j);
   % The wavelenght of the wave
   lambda(j) = 2.*pi*(U19(j)^2)/((0.877^2)*g);
   % The  nuber of wave sampled
   N3(j) = D/lambda(j);
   %Computing the summation 
    for i = 1:1:m
        fU(i) = w(i)*(1/sqrt(pi))*(1 - exp(-4.*(sigmas(j)*x(i) + hant - Delh)^2/(H(j)^2)))^N3(j);
    end 
    PlosU(j) = sum(fU);
end

%% Case 2: D = 10 km
% The parameters of the wave field
    % The distance 
    D = 7000;
    % The Earth curvature
    Delh = 6371000 - sqrt((6371000^2) - (D/2)^2);
    % Antenna height in metres.
    hant = 1.00;
    % Gravitational acceleration
    g = 9.81;
    % Definition of the  PM spectrum constants
    alpha = 0.0081;
    beta = 0.71;
% Calculating the Probability for every value of hant
for j = 1:length(U10)
   % Wind speed at 19.5 metres above sea level
   U19(j) = 1.075.*U10(j);
   syms omega;
   SPM3 = (alpha.*g.*g./(omega.^5)).*exp(-beta.*(g./(U19(j).*omega)).^4);
   % The variance of the surface elevation from the PM SPECTRUM
   variances(j) = double(int(SPM3,omega,0,Inf));
   % The std of the surface height
   sigmas(j) = sqrt(variances(j));
   % The RMS Height of the wave
   H(j) = 2*sqrt(2)*sigmas(j);
   % The wavelenght of the wave
   lambda(j) = (2.*pi*((U19(j))^2))/((0.877^2)*g);
   % The  nuber of wave sampled
   N3(j) = D/lambda(j);
   %Computing the summation 
    for i = 1:1:m
        fU1(i) = w(i)*(1/sqrt(pi))*(1 - exp(-4.*(sigmas(j)*x(i) + hant - Delh)^2/(H(j)^2)))^N3(j);
    end 
    PlosU1(j) = sum(fU1);
end

figure('Name','PLOS_wind','Units','centimeters','position',[5 5 23 9]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'FontWeight','bold','fontsize',12,'FontName','Romain')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 9],'PaperSize',[21 9]);
plot(U19,PlosU,U19,PlosU1,'LineWidth',2), ylabel('\bf{Probability of Line of sight}'),
xlabel('$\mathbf{U_{19.5}~~(m/s)}$'),box on, grid on, legend('$D~ =~ 5~ km$','$D~ =~ 10~km$','Location','northeastoutside');
%print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\Plos_windspeed10p.pdf','-dpdf');
%print('C:\Users\Kossivi\Onedrive_UCT\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\Plos_windspeed10p.pdf','-dpdf');


% figure(1)
% plot(U10,PlosU);