% Matlab Script for the ocean surface simulation
% by Kossivi Fangbemi
% Resources exploited: https://www.mathworks.com/matlabcentral/answers/uploaded_files/74101/SurfaceWavesTutorial.pdf
%                      https://agupubs.onlinelibrary.wiley.com/doi/epdf/10.1029/97JC00467
                       


set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex'); 

tic;

Bound_C = 0;  % Boundary Condition setting: 1 ---> Providing Boudary condition at the instant
                               %            0 ---> Only rounding the domain

U_10 = 10.0;          % Wind speed at 10 metres above the ocean surface
Theta_wind = 1*pi/2 + 1*pi/3;  % Wind direction ==> Wave propagartion direction
% Significant wave height for the measurment data
if U_10 == 7.5
    Hm0 = 2;
elseif U_10 == 10
    Hm0 = 3;
elseif U_10 == 12.5
    Hm0 = 4.5;
end
g = 9.82;           % Gravitational Acceleration  

    % Dimension: 30 m x 30 m
    L = 30.0;
    % Grid resolution: N_x x N_y = 1024 x 1024
    N = 32;
    % Measurement Possition
    P = [3 25; 5 26];
%% Definition of constants for the Variance Spectrum
                    % Sea age constant for the given wind speed U_10:
Omega_c = 0.84;     % Fully developped sea
%Omega_c = 1;       % Matures sea
%Omega_c = 5;       % Very Yound sea
Cd10N = 0.00144;    % Drag Coefficient
a_0 = 0.1733;       % 
a_p = 4.0;          %
k_m = 370.0;        % rad/m
c_m = 0.23;         % Phase speed of the wave with spatial frequency km
% Parameters depending on the above constant
a_m = 0.13*(sqrt(Cd10N)*U_10/c_m);
sigma =  0.08*(1 + 4*(Omega_c^(-3))); % % Sigma
if (sqrt(Cd10N)*U_10) <= c_m
    Alpha_m = 0.01*(1 + log(sqrt(Cd10N)*U_10/c_m));
else     
    Alpha_m = 0.01*(1 + 3*log(sqrt(Cd10N)*U_10/c_m));
end
Alpha_p =  0.006*(Omega_c^(0.55));
if (Omega_c) <= 1       % % Lambda
    Lambda = 1.7;
else     
    Lambda = 1.7 + 6*log(Omega_c);
end

k_p = (g/((U_10)^2))*(Omega_c^2);   % Spatial frequency of the maximum of the 
                                    % spectrum
c_p = sqrt(g/k_p);     % Phase speed of the wave with spatial frequency kp

Tp = 1/(sqrt(g.*k_p)/(2*pi)); % Dominant wave period

ts = min([(L/(N*c_p*sqrt(2))),(L/(N*c_p*sqrt(2))), 0.10]);  % Sampling Time for the transition Matrix 

% The measurement y 
[y, BC_1, BC_2] = Measurement(Hm0, Tp, L, N, P, Theta_wind, ts);
System.y = gpuArray(y);
% The Measurement Matrix
System.C = gpuArray(Matrix_C(L, N, P));

%% The 2-D surface elevation 
% Fundamental Spatial Frequencies:
k_x = (2*pi)/L; % In the x-direction
k_y = (2*pi)/L; % In the y-direction
% Nyquist Spatial Frequencies:
k_Nx = 0.5*k_x*N; 
k_Ny = 0.5*k_y*N;
% The frequency grid
u = ((-(N/2 -1):(N/2))*k_x)'; % x-direction
v = ((-(N/2 -1):(N/2))*k_y)'; % y-direction
for i = 0: N/2
    u(i+1,1) = i.*k_x;
    v(i+1,1) = i.*k_y;
end 
for i = -(N/2 -1) : -1
    u(N+i+1,1) = i.*k_x;
    v(N+i+1,1) = i.*k_y;
end
% Generartion random values for rho and sigma (White Noise)
%Rd_r_s = 1.*randn(N,N,2);
Rd_r_s = unifrnd(-500,500,N,N,2)/500;

% Getting the Hermatian Fourier Series
    % Delcaring the Matrices size for speed optimisations
k = zeros(N,N);     k_ = zeros(N,N);
phi = zeros(N,N);   phi_ = zeros(N,N);
Omega = zeros(N,N);  Omega_ = zeros(N,N);
c = zeros(N,N);     c_ = zeros(N,N);
L_pm = zeros(N,N);  L_pm_ = zeros(N,N);
Gamma = zeros(N,N); Gamma_ = zeros(N,N);
J_p = zeros(N,N);   J_p_ = zeros(N,N);
B_l = zeros(N,N);   B_l_ = zeros(N,N);
B_h = zeros(N,N);   B_h_ = zeros(N,N);
Phi = zeros(N,N);   Phi_ = zeros(N,N);
D_Phi = zeros(N,N);     D_Phi_ = zeros(N,N);
z_0hat = zeros(N,N);    z_0hat_ = zeros(N,N);
z_hat = zeros(N,N);     z_hat_ = zeros(N,N);
D_z_0hat = zeros(N,N);    D_z_0hat_ = zeros(N,N);
D_z_hat = zeros(N,N);     D_z_hat_ = zeros(N,N);
ii = 1;
Sum_zhatsqrt0 = zeros(N*N,1);
      % Parameters for the cosine-2s spreading function\
Mu = 4.1.*ones(N,N);    Mu_ = 4.1.*ones(N,N);
sp = 7.3.*ones(N,N);    sp_ = 7.3.*ones(N,N);
s = zeros(N,N);         s_ = zeros(N,N);

for p = 2:N
    for q = 2:N
        % The function parameters for the spectrum
        k(p,q) = sqrt(u(p).^2 + v(q).^2);
        phi(p,q) = angle(u(p) + 1i*v(q));
        Omega(p,q) = sqrt(g.*k(p,q));
        if ((p ~= (N/2 + 1))||(q ~= (N/2 + 1)))
            k_(p,q) = sqrt((u(N-p+2)).^2 + (v(N-q+2)).^2);
            phi_(p,q) = angle(u(N-p+2) + 1i.*v(N-q+2)) - Theta_wind;
            Omega_(p,q) = sqrt(g.*k(p,q));
        end
        c(p,q) = sqrt((g./k(p,q)).*(1+ ((k(p,q)./k_m).^2)));  % Phase speed of the wave
        c_(p,q) = sqrt((g./k_(p,q)).*(1+ ((k_(p,q)./k_m).^2)));
        L_pm(p,q) = exp(-1.25.*((k_p./k(p,q)).^2));    % Shape of the sperctrum (from PM SPECTRUM)
        L_pm_(p,q) = exp(-1.25.*((k_p./k_(p,q)).^2)); 
        Gamma(p,q) = exp(-((sqrt(k(p,q)./k_p) - 1).^2)./(2.*(sigma.^2)));
        Gamma_(p,q) = exp(-((sqrt(k_(p,q)./k_p) - 1).^2)./(2.*(sigma.^2)));
        J_p(p,q) = Lambda.^Gamma(p,q);
        J_p_(p,q) = Lambda.^Gamma_(p,q);
                                        % Low Frequency comtribution
                                        % function to the spectrum.
        B_l(p,q) = 0.5.*Alpha_p.*(c_p./c(p,q)).*L_pm(p,q).*J_p(p,q).*exp(-0.3162.*Omega_c.*(sqrt(k(p,q)./k_p) - 1));
        B_l_(p,q) = 0.5.*Alpha_p.*(c_p./c_(p,q)).*L_pm_(p,q).*J_p_(p,q).*exp(-0.3162.*Omega_c.*(sqrt(k_(p,q)./k_p) - 1));
                                        % High Frequency comtribution
                                        % function to the spectrum.
        B_h(p,q) = 0.5.*Alpha_m.*(c_m./c(p,q)).*L_pm(p,q).*J_p(p,q).*exp(-0.25.*((sqrt(k(p,q)./k_m) - 1).^2));
        B_h_(p,q) = 0.5.*Alpha_m.*(c_m./c_(p,q)).*L_pm_(p,q).*J_p_(p,q).*exp(-0.25.*((sqrt(k_(p,q)./k_m) - 1).^2));
        
                                        % Spreading Function for the
                                        % directional spectrum from 
        Phi(p,q) = (1/(2.*pi)).*(1 + tanh(a_0 + a_p.*((c(p,q)./c_p).^2.5) + a_m.*((c_m./c(p,q)).^2.5)).*cos(2.*phi(p,q) - 2.*Theta_wind));
        Phi_(p,q) = (1/(2.*pi)).*(1 + tanh(a_0 + a_p.*((c_(p,q)./c_p).^2.5) + a_m.*((c_m./c_(p,q)).^2.5)).*cos(2.*phi_(p,q) - 2.*Theta_wind));
                                        % Cosine-2s Spreading function for
                                        % propagation in the downwind
                                        % direction (Hasselmann et al.
                                        % 1980)
        if(k(p,q) >= k_p)
            Mu(p,q) = - 2.39 - 1.5.*((U_10/c(p,q)) - 1.17);
            sp(p,q) = 9.8;
        end
        if(k_(p,q) >= k_p)
            Mu_(p,q) = - 2.39 - 1.5.*((U_10/c_(p,q)) - 1.17);
            sp_(p,q) = 9.8;
        end
        s(p,q) = sp(p,q).*(sqrt(k(p,q)/k_p))^(Mu(p,q));
        s_(p,q) = sp_(p,q).*(sqrt(k_(p,q)/k_p))^(Mu_(p,q));
        D_Phi(p,q) = (2^(2.*s(p,q)-1)/pi).*((gamma(s(p,q)+1))^2/(gamma(2.*s(p,q)+1))).*(cos((0.5.*(phi(p,q) - Theta_wind))))^(2.*s(p,q)).*(abs(phi(p,q)) < pi);
        D_Phi_(p,q) = (2^(2.*s_(p,q)-1)/pi).*((gamma(s_(p,q)+1))^2/(gamma(2.*s_(p,q)+1))).*(cos((0.5.*(phi_(p,q) - Theta_wind))))^(2.*s_(p,q)).*(abs(phi(p,q)) < pi);
    end
end
% The spectrum with the first spreading function Phi
        % Normalization of the spreading function
        Phi_norm = Phi/max(Phi(:));
        Phi_norm_ = Phi_/max(Phi_(:));
        Psi = (B_l + B_h).*Phi_norm./(k.^4);
        Psi_ = (B_l_ + B_h_).*Phi_norm_./(k_.^4);
        %Psi = (B_l + B_h)./(k.^3);
        
% The spectrum with the cosine-2s spreading function 
        % Normalization of the spreading function
        D_phi_norm = D_Phi/max(D_Phi(:));
        D_phi_norm_ = D_Phi_/max(D_Phi_(:));
        
        D_Psi = (B_l + B_h).*D_phi_norm./(k.^4);
        D_Psi_ = (B_l_ + B_h_).*D_phi_norm_./(k_.^4);
        
        % Colour-filtering for the first spectrum
        DPsiroot = sqrt((Psi/2).*(k_x.*k_y));
        DPsiroot_ = sqrt((Psi_/2).*(k_x.*k_y));
        
        D_DPsiroot = sqrt((D_Psi/1).*(k_x.*k_y));
        D_DPsiroot_ = sqrt((D_Psi_/1).*(k_x.*k_y));

%% Settings for videos of the waves
% clear M_frame; clear D_M_frame;
% v = VideoWriter('SurfaceWaveMovie.avi','Motion JPEG AVI');
% v.FrameRate = 20; % number of frames per second.
% v.Quality = 95; % Quality of the video.
% open(v);
% D_v = VideoWriter('D_SurfaceWaveMovie.avi','Motion JPEG AVI');
% D_v.FrameRate = 20; % number of frames per second.
% D_v.Quality = 95; % Quality of the video.
% open(D_v);

t = 0:ts:10;  %0:0.05:180;
for tt = 1:2 %length(t)
   for p = 2:N
    for q = 2:N
        z_0hat(p,q) =(1/sqrt(2)).*((Rd_r_s(p,q,1) + 1i.*Rd_r_s(p,q,2)).*DPsiroot(p,q));
                                        % Z_0hat at k_uv 
        z_0hat_(p,q) =(1/sqrt(2)).*((Rd_r_s(N-p+2,N-q+2,1) - 1i.*Rd_r_s(N-p+2,N-q+2,2)).*DPsiroot_(p,q));
                                        % Z_0hat conjugate at - k_uv 
                                        
        z_hat(p,q) = N*N*(1/sqrt(2)).*(z_0hat(p,q).*exp(-1i.*Omega(p,q).*t(tt)) + z_0hat_(p,q).*exp(1i.*Omega_(p,q).*t(tt)));
        
        % Colour-filtering for the second spectrum

        D_z_0hat(p,q) =(1/sqrt(2)).*((Rd_r_s(p,q,1) + 1i.*Rd_r_s(p,q,2)).*D_DPsiroot(p,q));
                                        % Z_0hat at k_uv 
        D_z_0hat_(p,q) =(1/sqrt(2)).*((Rd_r_s(N-p+2,N-q+2,1) - 1i.*Rd_r_s(N-p+2,N-q+2,2)).*D_DPsiroot_(p,q));
                                        % Z_0hat conjugate at - k_uv 
        
        D_z_hat(p,q) = N*N*(1/sqrt(2)).*(D_z_0hat(p,q).*exp(-1i.*Omega(p,q).*t(tt)) + D_z_0hat_(p,q).*exp(1i.*Omega_(p,q).*t(tt)));
    end
   end 
   
%% Getting the Surface elevation and plotting it.
    z_hat(1,1) = 0;
    z_hat((N/2 +1), (N/2  + 1)) = 0;

    D_z_hat(1,1) = 0;
    D_z_hat((N/2 +1), (N/2  + 1)) = 0;
% The inverse FFT
    zz_c = ifft2(z_hat, 'symmetric');
    zz = real(zz_c);
    zz_i = imag(zz_c); % This should be zero. 

    D_zz_c = ifft2(D_z_hat, 'symmetric');
    D_zz = real(D_zz_c);
    D_zz_i = imag(D_zz_c); % This should be zero. 
    if( tt == 1) % Getting the initalial conditions at T+2
        X_init1 = reshape(zz,[],1);
        D_X_init1 = reshape(D_zz,[],1);
    end
    if( tt == 2) % Getting the initalial conditions at T+1
        X_init2 = reshape(zz,[],1);
        D_X_init2 = reshape(D_zz,[],1);
    end
%     pause(0.005);
end
% Getting the the initial condition of the state space model
State_X1 = zeros(2*N*N, 1);
State_X2 = zeros(2*N*N, 1);
j = 1;
 for i = 1:2:2*length(X_init1)
     State_X1(i) = X_init2(j);
     State_X1(i+1) = X_init1(j);
     j = j + 1;
 end 
 j = 1;
 for i = 1:2:2*length(D_X_init1)
     State_X2(i) = D_X_init2(j);
     State_X2(i+1) = D_X_init1(j);
     j = j + 1;
 end 

% The transition Matirx A 
System.A = gpuArray(Matrix_A(U_10,Omega_c,ts,N,L,Theta_wind));

%% Kalman Filtering
% Defining the covariance Matrices
% The initial state covariance matrix Pn_       
       % The variance of the surface elavation
       Var_State = 0.01^2;%2.75e-3 *((1.075*U_10)^4)/(g^2);
       Pn_mns =  Var_State*ones(1, length(State_X1));
       Pn_moins = sparse(length(Pn_mns)*length(Pn_mns));
       for i = 1:2:length(Pn_mns)
           Pn_moins(i,i) = Pn_mns(i);
           Pn_moins(i+1,i+1) = Pn_mns(i);
       end
       System.P = gpuArray(Pn_moins);
       % The inverse of the state covariavnce matrix
%        Pn_moins_inv = inv(Pn_moins);
% The white noise covariance Q_n
       Var_Error = (5e-4)^2;
       Qn_mns =  Var_Error*ones(1, length(State_X1));
       Q_m = sparse(length(Pn_mns)*length(Pn_mns));
       for i = 1:2:length(Pn_mns)
%            if (i <= 1*N)
               Q_m(i,i) = Qn_mns(i);%0.01*Hm0/2;
%            elseif ((i >= 1*N) || (i <= 5*N))
%                Q_m(i,i) = 0.1*Qn_mns(i);%0.01*Hm0/12;
%            elseif ((i >= 25*N) || (i <= 30*N))
%                Q_m(i,i) = 0.1*Qn_mns(i);%0.01*Hm0/12;
%            else
%                Q_m(i,i) = 0.025*Qn_mns(i);
%            end
           Q_m(i+1,i+1) = Qn_mns(i);
       end
       System.Q = gpuArray(Q_m);
% The measurment error covariance R_n
System.R = gpuArray(zeros(length(P)));
Mean_y = zeros(length(P));
for i =  1:length(P)    
    System.R(i,i) = 0.05^2;
end
% *abs(mean(System.y(:,i + 1)))

% Assigning the initial Condition for the wave field
System.State_X = gpuArray(State_X1);            % The Symetrical spreading function 
%System.State_X = State_X2;           % The Cos-2s spreading function
System.State_X2 = State_X1;           % The Symetrical spreading function
%System.State_X2 = State_X2;          % The Cos-2s spreading function
y_p = zeros(2,length(System.y));        % An array for the surface elevation at the measurement locations

for i = 1:length(System.y)
%     System.A = gpuArray(System.A);
%     System.C = gpuArray(System.C);
%     System.y = gpuArray(y);
%     System.Q = gpuArray(System.Q);
%     System.R = gpuArray(System.R);
%     System.State_X = gpuArray(System.State_X);
%     System.P = gpuArray(System.P);
    
    System = KalmaFilter(System,i);
    % Extracting the predicted surface elevation at the measurement locations
    y_p(:,i) = gather(System.C * System.State_X);
    
    % Saving the system state for plotting
    if (i == 500) 
        X_state_500 = System.State_X;
        P_500 = System.P;
    end
    if (i == 1000) 
        X_state_1000 = System.State_X;
        P_1000 = System.P;
    end
    if (i == 1500) 
        X_state_1500 = System.State_X;
        P_1500 = System.P;
    end
    % The wave equation
    System.State_X2 = System.A*System.State_X2;
    % Updating boundary conditions
    if (Bound_C == 1)
        ij = 0;
        for jj = 1:2:2*N
            System.State_X(jj) = BC_1(i,ij+1);
        end
        ij = 0;
        for ji = 2*N-1:2*N:2*N*N
            System.State_X(ji) = BC_2(i,ij+1);
        end
    end
end

%Retreiving the surface elevation from the column vector
Surf_X_state_500 = reshape(X_state_500(1:2:length(X_state_500)),[N,N]);
Surf_X_state_1000 = reshape(X_state_1000(1:2:length(X_state_1000)),[N,N]);
Surf_X_state_1500 = reshape(X_state_1500(1:2:length(X_state_1500)),[N,N]);

% Extraction the covariance of each system state
Diag_Cov = zeros(N*N*2,1);
P_full_500 = full(gather(P_500));
for i = 1:2:N*N*2
    Diag_Cov(i,1) = sqrt(P_full_500(i,i));
end
Diag_Cov_Surf_500 = reshape(Diag_Cov(1:2:length(Diag_Cov)),[N,N]);

P_full_1000 = full(gather(P_1000));
for i = 1:2:N*N*2
    Diag_Cov(i,1) = sqrt(P_full_1000(i,i));
end
Diag_Cov_Surf_1000 = reshape(Diag_Cov(1:2:length(Diag_Cov)),[N,N]);

P_full_1500 = full(gather(P_1500));
for i = 1:2:N*N*2
    Diag_Cov(i,1) = sqrt(P_full_1500(i,i));
end
Diag_Cov_Surf_1500 = reshape(Diag_Cov(1:2:length(Diag_Cov)),[N,N]);

%% Plotting 
Folder_name = 'C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\Figs1\';
[x,y] = meshgrid(linspace(0, L, N),linspace(-0, L, N));
% direction = [1 1 1];
% figure('Name','surface 1','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',11,'FontName','Helvetica')
% set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
% mesh(x,y,zz), ylabel('Distance $(m)$'), zlabel('Surface elevation $(m)$'),
% xlabel('Distance $(m)$'),box on, grid on,
% print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\INIT_SURF_7_0.pdf','-dpdf');
% 
% figure('Name','surface 1','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',11,'FontName','Helvetica')
% set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
% mesh(x,y,surf+cur_w_e), ylabel('Distance $(m)$'), zlabel('Surface elevation $(m)$'),
% xlabel('Distance $(m)$'),box on, grid on,
% print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\CUR_SURF_EXC_WIND_7_0.pdf','-dpdf');
% 
% figure('Name','surface 1','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',11,'FontName','Helvetica')
% set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
% mesh(x,y,surf+zz), ylabel('Distance $(m)$'), zlabel('Surface elevation $(m)$'),
% xlabel('Distance $(m)$'),box on, grid on,
% print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\INIT_SURF_PLUS_EXC_WIND_7_0.pdf','-dpdf');
% 
% figure('Name','surface 1','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',11,'FontName','Helvetica')
% set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
% mesh(x,y,surf), ylabel('Distance $(m)$'), zlabel('Surface elevation $(m)$'),
% xlabel('Distance $(m)$'),box on, grid on,
% print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\EXC_WIND_7_0.pdf','-dpdf');

figure('Name','surface 10','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
hSurface = mesh(x,y,zz); ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{Surface~ elevation~ (m)}$'),
xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on, zlim([-1 1]),
print( strcat(Folder_name, 'INIT_SURF_10_1n.pdf'),'-dpdf');

figure('Name','surface 11','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
mesh(x,y,Surf_X_state_500), ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{Surface~ elevation~ (m)}$'),
xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on, zlim([-4 1.2]),
print( strcat(Folder_name, 'CUR_SURF_500_10_10n.pdf'),'-dpdf');

figure('Name','surface 12','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
mesh(x,y,Diag_Cov_Surf_500), ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{ Surface ~elevation ~Error~ (m)}$'),
xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on, %zlim([-3 3]),
print( strcat(Folder_name, 'ICUR_COV_500_10_10n.pdf'),'-dpdf');

figure('Name','surface 13','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
mesh(x,y,Surf_X_state_1000), ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{Surface~ elevation~ (m)}$'),
xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on, zlim([-4 1.2]),
print(strcat(Folder_name, 'CUR_SURF_1000_10_10n.pdf'),'-dpdf');

figure('Name','surface 14','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
mesh(x,y,Diag_Cov_Surf_1000), ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{ Surface ~elevation ~Error~ (m)}$'),
xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on, %zlim([-3 3]),
print(strcat(Folder_name, 'ICUR_COV_1000_10_10n.pdf'),'-dpdf');

figure('Name','surface 15','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
mesh(x,y,Surf_X_state_1500), ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{Surface~ elevation~ (m)}$'),
xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on, zlim([-4 1.2]),
print(strcat(Folder_name, 'CUR_SURF_1500_10_10n.pdf'),'-dpdf');

figure('Name','surface 16','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
mesh(x,y,Diag_Cov_Surf_1500), ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{ Surface ~elevation ~Error~ (m)}$'),
xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on, %zlim([-3 3]),
print(strcat(Folder_name, 'ICUR_COV_1500_10_10n.pdf'),'-dpdf');

figure('Name','surface 17','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
     subplot(2,1,1);
     plot(System.y(:,1), System.y(:,2),'LineWidth',2), title('\bf{Measurement at Coordinates 1}');
     ylabel('$\mathbf{Surface elevation~ (m)}$'),
     xlabel('$\mathbf{Time ~(s)}$'),box on, grid on, xlim([0 84]), ylim([-3 3]),
     subplot(2,1,2);
     plot(System.y(:,1), System.y(:,3),'LineWidth',2), title('\bf{Measurement at Coordinates 2}'); 
     ylabel('$\mathbf{Surface elevation~ (m)}$'),
     xlabel('$\mathbf{Time ~(s)}$'),box on, grid on, xlim([0 84]), ylim([-3 3]),
print(strcat(Folder_name, 'MEAS_10_10n.pdf'),'-dpdf');

figure('Name','surface 18','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);   
     subplot(2,1,1);
     plot(System.y(:,1), System.y(:,2), System.y(:,1),y_p(1,:),'LineWidth',2), title('\bf{Location 1}');
     ylabel('$\mathbf{Surface elevation~ (m)}$'), xlim([0 84]), ylim([-3 3]),
     xlabel('$\mathbf{Time ~(s)}$'),box on, grid on, legend('\bf{Generated measurement}', '\bf{Predicted surface elevation}' ,'Location','northeastoutside');
     subplot(2,1,2);
     plot(System.y(:,1), System.y(:,3), System.y(:,1),y_p(2,:),'LineWidth',2), title('\bf{Location 2}'); 
     ylabel('$\mathbf{Surface elevation~ (m)}$'), xlim([0 84]), ylim([-3 3]),
     xlabel('$\mathbf{Time ~(s)}$'),box on, grid on, legend('\bf{Generated measurement}', '\bf{Predicted surface elevation}' ,'Location','northeastoutside');
print(strcat(Folder_name, 'MEAS_PRED_10_10n.pdf'),'-dpdf');

toc