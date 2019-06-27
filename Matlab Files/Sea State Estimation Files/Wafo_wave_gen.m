
set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');  
% 
Hm0 = 5; Tp = 11; gamma = 3.3; sa = 0.1; sb = 0.11; plotflag = 0; clf
ST = bretschneider([],[Hm0 Tp gamma sa sb],plotflag);
% dt = 0.01; N = 2000;
% xs = spec2sdat(ST,N,dt);

plotflag = 1; 
Nt = 10001; % number of angles
th0 = 1*pi/2; % primary direction of waves
Sp = 15; % spreading parameter
% D1 = spreading(Nt,'cos',th0,Sp,[],0); %frequency independent
D12 = spreading(Nt,'cos2s',th0,Sp,ST.w,1); %frequency dependent
% STD1 = mkdspec(ST,D1); 
STD12 = mkdspec(ST,D12);
% plotspec(STD1,plotflag), hold on
plotspec(STD12,plotflag,'-.');

rng('default'); clf
opt = simoptset('Nt',2,'dt',1,'Nu',100,'du',1,'Nv',100,'dv',1);
% W1 = spec2field(STD1,opt);
W12 = spec2field(STD12,opt);

% figure(1); clf
% Movie1 = seamovie(W1,1);
figure(3); 
axis off
Movie12 = seamovie(W12,1);%,'GaussianSea12.avi')

%% 
% Hm0 = 5; Tp = 10; sa = 0.1; sb = 0.11;
% D = 0.036-0.0056*Tp/sqrt(Hm0);
% gamma = exp(3.484*(1-0.1975*D*Tp^4/(Hm0^2)));
% S_pec = jonswap([], [Hm0, Tp, gamma, sa, sb], 0); % JONSWAP omnidirectional spectrum
% N_t = 20001; th0 = 1*pi/4; Sp = 15;
% D = spreading(N_t,'cos2s',th0,Sp,S_pec.w,1); % Frequency dependent spreding function
% S_pec_D = mkdspec(S_pec,D,0); %  Directional Spectrum
% 
% % Simultion option of the 3D wave field
%     % 128 points with 0.15625 m between two point giving 20 m in x and y
%     % directions
%     % Nt give the number of time samples of the wave field and dt is the
%     % time step size in seconds.
% Sim_Opt = simoptset('Nt',200,'dt',0.25,'Nu',512,'du',0.15625,'Nv',512,'dv',0.15625);
% 
% % Generating the elementary wave field processes
% [W,X,Y] = spec2ldat3D(S_pec_D, Sim_Opt, 'lalpha', 1.5);
% 
% % Gen_Opt3D_movies = genoptset('type', 'movie');
% % % Generatiing time series Lagragnge waves at two points.
% % Gen_Opt3D_movies = genoptset(Gen_Opt3D_movies, 'start',[2 2]);
% % L_waves_movies = ldat2lwav3D(W,X,Y,Gen_Opt3D_movies);
% % 
% % figure(1)
% % Mv_wave = seamovie(L_waves_movies,1);
% 
% % Generatiing time series Lagragnge waves at two points.
% Gen_Opt3D = genoptset('type', 'timeseries', 'PP', [3 17; 3 17]);
% L_waves = ldat2lwav3D(W,X,Y,Gen_Opt3D);
% 
% % Extracting the measurement data for the extimation
% y_n = [L_waves.t; L_waves.Z{1}; L_waves.Z{2}];
% 
% figure(4)
% plot(L_waves.t, L_waves.Z{1});
% figure(5)
% plot(L_waves.t, L_waves.Z{2});

%% Multiple time series
% Hm0 = 5; Tp = 10; sa = 0.1; sb = 0.11;
% D = 0.036-0.0056*Tp/sqrt(Hm0);
% gamma = exp(3.484*(1-0.1975*D*Tp^4/(Hm0^2)));
% S_pec = jonswap([], [Hm0, Tp, gamma, sa, sb], 0); % JONSWAP omnidirectional spectrum
% N_t = 20001; th0 = 1*pi/4; Sp = 15;
% D = spreading(N_t,'cos2s',th0,Sp,S_pec.w,1); % Frequency dependent spreding function
% S_pec_D = mkdspec(S_pec,D,0); %  Directional Spectrum
% 
% P1 = [13 27; 27 13];
% P2 = [10:30; 11*ones(1,21)];
% P3 = [11*ones(1,21); 10:30];
% PPP = [P1, P2, P3];
% Nt = 200;
% Sim_Opt = simoptset('Nt',Nt,'dt',0.25,'Nu',256,'du',0.15625,'Nv',256,'dv',0.15625); 
% L = spec2lseries(S_pec_D,PPP,Sim_Opt);
% 
% y = [L.t, L.Z{1,1}', L.Z{1,2}'];
%     
%     % Extracting the boundary condition data
%     BC_1 = L.Z{1,3}';
%     for i=1:length(P2)-1
%         BC_1 = [BC_1, L.Z{1,3+i}'];
%     end
%     
%     BC_2 = L.Z{1,length(P2)+2}';
%     for i=1:length(P2)-1
%         BC_2 = [BC_2, L.Z{1,length(P2)+2+i}']; 
%     end
%     
%     P_s = zeros(2,length(ones(1,21))^2);
% for i = 1:length(ones(1,21))
%     P_s(:,(21*(i-1)+1):(21*(i)))= [(10+i-1)*ones(1,21);10:30];
%     
% end 
% L_S = spec2lseries(S_pec_D,P_s,Sim_Opt);
% 
% Surf_vec = L_S.Z{1,1}';
%     for i=2:length(P_s)
%         Surf_vec = [Surf_vec, L_S.Z{1,i}'];
%     end
% 
% 
% for jj = 1:Nt
%     Surf_Surf(:,:,jj) = reshape(Surf_vec(jj,:),length(10:30),length(10:30));
% end
