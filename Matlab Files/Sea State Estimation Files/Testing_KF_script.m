
%% %% %% %% %% %% %% %% %% 
    % Setting The initial Conditions 
    %System.State_X = State_X1;
    System.State_X = Init_vec_X;
    %System.State_X = zeros(length(State_X1),1);

    Theta_wind = 1*pi/3 + 1*pi/3;
% The transition matrix
System.A = Matrix_A_simple(U_10,Omega_c,ts,N,L,Theta_wind);

% Defining the covariance Matrices
% The initial state covariance matrix Pn_       
       % The variance of the surface elavation
       Var_State = 0.01^2;
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
       Var_Error = (1e-2)^2;
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
    System.R(i,i) = 0.01^2;
end

for i = 1:length(System.y)
    for ii = 1:1
        %State_X1 = System.A*State_X1;
        %System.State_X = System.A*System.State_X;
        System = KalmaFilter(System,i);
    
    % Updating boundary conditions
    if (Bnd_C == 1)
        ij = 1;
        for jj = 1:2:2*N
            System.State_X(jj) = Surf_Surf(ij,1,i);
            ij = ij + 1;
        end
        ij = 1;
        for ji = 2*N-1:2*N:2*N*N
            System.State_X(ji) = Surf_Surf(N,ij,i);
            ij = ij + 1;
        end
        ij = 1;
        for ji = 1:2*N:2*N*N-(2*N-1)
            System.State_X(ji) = Surf_Surf(1,ij,i);
            ij = ij + 1;
        end
        ij = 1;
        for ji = 2*N*N-(2*N-1):2:(2*N*N)-1
            System.State_X(ji) = Surf_Surf(ij,N,i);
            ij = ij + 1;
        end
    end
    end 
    if (i == 10) 
        %X_state_10 = System.State_X;
        P_10 = System.P;
    end
    if (i == 20) 
        %X_state_20 = System.State_X;
        P_20 = System.P;
    end
    if (i == 30) 
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
Folder_name = 'C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Conf Papers\Figures\Fig10_New_Mtd\';
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
