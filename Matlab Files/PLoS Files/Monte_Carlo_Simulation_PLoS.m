% Matlab Script for the Monte Carlo simulation for the PloS
% by Kossivi Fangbemi

set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex'); 

number_of_runs = 1500000; %Number of runs for the Monte Carlo simulation

% Declaring parametres
U_19_5 = linspace(3,10,100);        % Wind speed at 19.5 metres above the ocean surface
g = 9.81;           % Gravitional acceleration
h_ANT = 1;          % Antenna height
D = 5000;           % Distance points A and B
R = 6371000;        % The radius of Earth
Delta_H = R - sqrt(R^2 - (D/2)^2); % The effect of the Earth's curvature

% Declaring PLoS for memory allocation!
PLoS = zeros(length(U_19_5),1);
for j = 1:length(U_19_5)
    Count = 0;  % Reinitiating the count
    for i = 1:number_of_runs
        % Generating random normal distributed numbers for the surface elevation at point A and B
            % Mean is zero
        Sigma = 2.74e-3*((U_19_5(j)).^4/g^2); % Variance of the normal distribution
        eta_A = sqrt(Sigma).*randn(1,1);   % Surface elevation at A
        eta_B = sqrt(Sigma).*randn(1,1);   % Surface elevation at B
        
        % Calculating the number of waves along the distance D
        lambda = (2*pi*U_19_5(j).^2)/(g*0.877^2);
        N = floor(D/lambda);
        %N = D/lambda;
        
        % Expressing the equation of the surface and the line of sight
        x = linspace(-D/2,D/2,N);
        y_A = eta_A  + h_ANT;
        y_B = eta_B  + h_ANT;
        y_s = -(4*Delta_H/(D^2)).*(x.^2) + Delta_H;     % Paraboloid Surface
        y_ls = ((y_A - y_B).*x./D) + ((y_A + y_B)./2); % Line of sight
        
        % Calculating minimum distance between y_ls and ys
        Delta_y_min = -(y_A - y_B).^2/(16*Delta_H) + (y_A + y_B)/2 - Delta_H;
        
        % Generating random numbers chosen from the Rayleigh distribution with for wave height
        b = 2*sqrt(Sigma);      % The parameter of the Rayleigh distribution
        H = raylrnd(b,[N 1]);
        A = H'./2;              % Getting the wave amplitudes into a row vector
        
        % Extracting the extreme wave heigth He and calculating Extreme wave
        % amplitude Ae
        % He = max(H);
        % Ae = He/2;
        
        % Checking whether y_ls is above y_s + A at each point x along D and record it
        count = find(y_ls > (y_s + A));
        if (length(count) == N)
            Count = Count + 1;
        end
    end
    
    % Calculating the probability of LoS, PloS
    PLoS(j) = Count/number_of_runs;
end


%% Analylitic slution 

% The integral expression for the probability 
% See in the dissertation

% Computing the nodes xi and their coresponding weight wi for the Gauss-Hermite
% Quadrature.
m = 128; % Order of the Hermite Plynomial and upper limit of the summation
f = zeros(m,1); f1 = zeros(m,1); f2 = zeros(m,1); 
fD = zeros(m,1); fD1 = zeros(m,1); fD2 = zeros(m,1);
% Calling function for get the value of xi and wi
[x, w] = GaussHermite_2(m); 
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


figure(1)
plot(U_19_5,PLoS,'LineWidth',2);

figure('Name','PLOS_wind','Units','centimeters','position',[5 5 20 12]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'FontWeight','bold','fontsize',18,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20 12],'PaperSize',[20 12]);
plot(U19,PlosU,U_19_5,PLoS,'LineWidth',2), ylabel('\bf{Probability of Line of sight}'),
xlabel('$\mathbf{U_{19.5}~~(m/s)}$'),box on, grid on, legend('\bf{Analytical Solution}','\bf{Monte Carlo Simulation}','Location','northeastoutside');
print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\Plos_windspeed10p.pdf','-dpdf');
%print('C:\Users\Kossivi\Onedrive_UCT\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\Plos_windspeed10pp.pdf','-dpdf');
