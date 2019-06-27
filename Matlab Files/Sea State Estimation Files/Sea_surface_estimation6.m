
%clear
set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex'); 
tic
Bnd_C = 1;  % Boundary Condition setting: 1 ---> Providing Boudary condition at the instant
                               %            0 ---> Only rounding the domain

    % Sea age constant for the given wind speed U_10:
    %Omega_c = 0.84;     % Fully developped sea
    %Omega_c = 1;       % Matures sea
    %Omega_c = 5;       % Very Yound sea
                               
U_10 = 10.0;          % Wind speed at 10 metres above the ocean surface
Theta_wind = 0*pi/2 + 1*pi/6;  % Wind direction ==> Wave propagartion direction
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
    L = 60.0;
    % Grid resolution: N_x x N_y = 1024 x 1024
    N = 64;
    % Measurement Possition
    P = [3 55; 5 56];
    
    % Sea surface paremeters given wind speed U_10:
    Omega_c = 0.84;                     % Fully developped sea
    k_p = (g/((U_10)^2))*(Omega_c^2);   % Spatial frequency of the maximum of the 
                                        % spectrum
    c_p = sqrt(g/k_p);                  % Phase speed of the wave with spatial frequency kp

    Tp = 1/(sqrt(g.*k_p)/(2*pi));           % Dominant wave period
    ts = min([(L/(N*c_p*sqrt(2))),(L/(N*c_p*sqrt(2))), 0.10]);  % Sampling Time for the transition Matrix 
    
    % The Initital conditions using the FFT2
    [Init_surf, Init_vec_X] = Spec_FFT2_surf(U_10, Omega_c, g, L, N, Theta_wind);
    
% The measurement y 
[y, BC_1, BC_2, Surf_Surf] = Measmt(Hm0, Tp, L, N, P, Theta_wind, ts);
System.y = y;
% The Measurement Matrix
System.C = Matrix_C(L, N, P);

% % Getting the initalial conditions and defining the system state
X_init1 = reshape(Surf_Surf(:,:,1),[],1);
X_init2 = reshape(Surf_Surf(:,:,2),[],1);
    % Getting the the initial condition of the state space model
    State_X1 = zeros(2*N*N, 1);
    j = 1;
    for i = 1:2:2*length(X_init1)
        State_X1(i) = X_init2(j);
        State_X1(i+1) = X_init1(j);
        j = j + 1;
    end 

%% %% %% %% %% %% %% %% %% 
    % Setting The initial Conditions 
    System.State_X = State_X1;
    %System.State_X = Init_vec_X;
    %System.State_X = zeros(length(State_X1),1);

% The transition matrix
System.A = Matrix_A_simple(U_10,Omega_c,ts,N,L,Theta_wind);

% Defining the covariance Matrices
% The initial state covariance matrix Pn_       
       % The variance of the surface elavation
       Var_State = 0.000^2;
       Pn_mns =  Var_State*ones(1, length(System.State_X));
       Pn_moins = sparse(length(Pn_mns)*length(Pn_mns));
       for i = 1:2:length(Pn_mns)
           Pn_moins(i,i) = Pn_mns(i);
           Pn_moins(i+1,i+1) = Pn_mns(i);
       end
       System.P = Pn_moins;
       % The inverse of the state covariavnce matrix
%        Pn_moins_inv = inv(Pn_moins);
% The white noise covariance Q_n
       Var_Error = (5e-4)^2;
       Qn_mns =  Var_Error*ones(1, length(System.State_X));
       Q_m = sparse(length(Pn_mns)*length(Pn_mns));
       for i = 1:2:length(Pn_mns)
           Q_m(i,i) = Qn_mns(i);%0.01*Hm0/2;
           Q_m(i+1,i+1) = Qn_mns(i);
       end
       System.Q = Q_m;
% The measurment error covariance R_n
System.R = sparse(length(P));
Mean_y = zeros(length(P));
for i =  1:length(P)    
    System.R(i,i) = (0.05)^2;
end

Max_error = zeros(1,length(System.y));

for i = 1:length(System.y)
    
    %State_X1 = System.A*State_X1;
    %System.State_X = System.A*System.State_X;
    System = KalmaFilter(System,i);
    
    Error = Surf_Surf(:,:,i) - reshape(System.State_X(1:2:length(System.State_X)),[N,N]); 
    Max_error(i) = max(Error(:));
    
    % Updating boundary conditions
    if (Bnd_C == 1)
        ij = 1;
        for jj = 1:2:2*N
            System.State_X(jj) = Surf_Surf(ij,1,i);
            %System.P(jj,jj) = 0;
            ij = ij + 1;
        end
        ij = 1;
        for ji = 2*N-1:2*N:2*N*N
            System.State_X(ji) = Surf_Surf(N,ij,i);
            %System.P(jj,jj) = 0;
            ij = ij + 1;
        end
        ij = 1;
        for ji = 1:2*N:2*N*N-(2*N-1)
            System.State_X(ji) = Surf_Surf(1,ij,i);
            %System.P(jj,jj) = 0;
            ij = ij + 1;
        end
        ij = 1;
        for ji = 2*N*N-(2*N-1):2:(2*N*N)-1
            System.State_X(ji) = Surf_Surf(ij,N,i);
            %System.P(jj,jj) = 0;
            ij = ij + 1;
        end
    end
    if (i == 2) 
        %X_state_10 = System.State_X;
        P_10 = System.P;
    end
    if (i == 5) 
        %X_state_20 = System.State_X;
        P_20 = System.P;
    end
    if (i == 9) 
        %X_state_30 = System.State_X;
        P_30 = System.P;
    end
    % Extracting the predicted surface elevation at the measurement locations
    y_p(:,i) = gather(System.C * System.State_X);
end

Surf_X_state = reshape(System.State_X(1:2:length(System.State_X)),[N,N]);

% Extraction the covariance of each system state
Diag_Cov = zeros(N*N*2,1);
P_full_10 = full(gather(P_10));
for i = 1:2:N*N*2
    Diag_Cov(i,1) = sqrt(P_full_10(i,i));
end
Diag_Cov_Surf_10 = reshape(Diag_Cov(1:2:length(Diag_Cov)),[N,N]);

P_full_20 = full(gather(P_20));
for i = 1:2:N*N*2
    Diag_Cov(i,1) = sqrt(P_full_20(i,i));
end
Diag_Cov_Surf_20 = reshape(Diag_Cov(1:2:length(Diag_Cov)),[N,N]);

P_full_30 = full(gather(P_30));
for i = 1:2:N*N*2
    Diag_Cov(i,1) = sqrt(P_full_30(i,i));
end
Diag_Cov_Surf_30 = reshape(Diag_Cov(1:2:length(Diag_Cov)),[N,N]);
 
%% Plotting and saving figures
Folder_name = 'C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Conf Papers\Figures\Fig11_New_Mtd\';
[x,y] = meshgrid(linspace(0, L, N),linspace(0, L, N));

figure(1)
mesh(x,y,Surf_X_state);

figure(2)
mesh(x,y,Surf_Surf(:,:,end));

figure(3)
mesh(x,y,(Surf_Surf(:,:,end) - Surf_X_state));

figure(4)
mesh(x,y,Diag_Cov_Surf_10);
figure(5)
mesh(x,y,Diag_Cov_Surf_20);
figure(6)
mesh(x,y,Diag_Cov_Surf_30);
figure(7)
plot(System.y(:,1),Max_error);

% figure('Name','surface 10','Units','centimeters','position',[5 5 23 7]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
% set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 7],'PaperSize',[21 7]);
% hSurface = mesh(x,y,Surf_Surf(:,:,1)); ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{Surface~ elevation~ (m)}$'),
% xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on, %rotate(hSurface,direction,25);
% print( strcat(Folder_name, 'INIT_SURF_10_n.pdf'),'-dpdf');
% 
% figure('Name','surface 16','Units','centimeters','position',[5 5 23 7]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
% set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 7],'PaperSize',[21 7]);
% mesh(x,y,Init_surf), ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{ Surface ~elevation ~Error~ (m)}$'),
% xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on,
% print(strcat(Folder_name, 'INIT_SURF_zz_10_n.pdf'),'-dpdf');
% 
% 
% figure('Name','surface 11','Units','centimeters','position',[5 5 23 7]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
% set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 7],'PaperSize',[21 7]);
% mesh(x,y,Surf_X_state), ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{Surface~ elevation~ (m)}$'),
% xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on,
% print( strcat(Folder_name, 'SURF_10_10_n.pdf'),'-dpdf');
% 
% figure('Name','surface 12','Units','centimeters','position',[5 5 23 7]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
% set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 7],'PaperSize',[21 7]);
% mesh(x,y,Diag_Cov_Surf_10), ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{ Surface ~elevation ~Error~ (m)}$'),
% xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on,
% print( strcat(Folder_name, 'ICOV_10_10_10n.pdf'),'-dpdf');
% 
% figure('Name','surface 13','Units','centimeters','position',[5 5 23 7]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
% set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 7],'PaperSize',[21 7]);
% mesh(x,y,Surf_Surf(:,:,end)), ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{Surface~ elevation~ (m)}$'),
% xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on,
% print(strcat(Folder_name, 'ACT_SURF_10_n.pdf'),'-dpdf');
% 
% figure('Name','surface 14','Units','centimeters','position',[5 5 23 7]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
% set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 7],'PaperSize',[21 7]);
% mesh(x,y,Diag_Cov_Surf_20), ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{ Surface ~elevation ~Error~ (m)}$'),
% xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on,
% print(strcat(Folder_name, 'I_COV_20_10_n.pdf'),'-dpdf');
% 
% figure('Name','surface 14','Units','centimeters','position',[5 5 23 7]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
% set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 7],'PaperSize',[21 7]);
% mesh(x,y,Diag_Cov_Surf_30), ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{ Surface ~elevation ~Error~ (m)}$'),
% xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on,
% print(strcat(Folder_name, 'I_COV_30_10_n.pdf'),'-dpdf');
% 
% % figure('Name','surface 15','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
% % set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
% % mesh(x,y,Surf_X_state_1500), ylabel('$\mathbf{Distance~ (m)}$'), zlabel('$\mathbf{Surface~ elevation~ (m)}$'),
% % xlabel('$\mathbf{Distance~ (m)}$'),box on, grid on,
% % print(strcat(Folder_name, 'CUR_SURF_1500_10_10n.pdf'),'-dpdf');
% 
% figure('Name','surface 17','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
% set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
%      subplot(2,1,1);
%      plot(System.y(:,1), System.y(:,2),'LineWidth',2), title('\bf{Measurement at Coordinates 1}');
%      ylabel('$\mathbf{Surface elevation~ (m)}$'),
%      xlabel('$\mathbf{Time ~(s)}$'),box on, grid on, %xlim([0 90.35]), ylim([-3 3]),
%      subplot(2,1,2);
%      plot(System.y(:,1), System.y(:,3),'LineWidth',2), title('\bf{Measurement at Coordinates 2}'); 
%      ylabel('$\mathbf{Surface elevation~ (m)}$'),
%      xlabel('$\mathbf{Time ~(s)}$'),box on, grid on,% xlim([0 90.35]), ylim([-3 3]),
% print(strcat(Folder_name, 'MEAS_10_10n.pdf'),'-dpdf');
% 
% figure('Name','surface 18','Units','centimeters','position',[5 5 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',12,'FontName','Helvetica')
% set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);   
%      subplot(2,1,1);
%      plot(System.y(:,1), System.y(:,2), System.y(:,1),y_p(1,:),'LineWidth',2), title('\bf{Location 1}');
%      ylabel('$\mathbf{Surface elevation~ (m)}$'), %xlim([0 90.35]), ylim([-3 3]),
%      xlabel('$\mathbf{Time ~(s)}$'),box on, grid on, legend('\bf{Generated measurement}', '\bf{Predicted surface elevation}' ,'Location','southeastoutside');
%      subplot(2,1,2);
%      plot(System.y(:,1), System.y(:,3), System.y(:,1),y_p(2,:),'LineWidth',2), title('\bf{Location 2}'); 
%      ylabel('$\mathbf{Surface elevation~ (m)}$'), %xlim([0 90.35]), ylim([-3 3]),
%      xlabel('$\mathbf{Time ~(s)}$'),box on, grid on, legend('\bf{Generated measurement}', '\bf{Predicted surface elevation}' ,'Location','southeastoutside');
% print(strcat(Folder_name, 'MEAS_PRED_10_10n.pdf'),'-dpdf');

toc
