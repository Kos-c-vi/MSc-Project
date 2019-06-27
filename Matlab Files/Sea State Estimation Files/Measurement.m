% Matlab Script for the ocean surface simulation
% by Kossivi Fangbemi
% Part of the Masters disertation

% Function to compute the measurements at points whose coordinates in metres are given in matrix Position.
% Position = [x1 x2 x3 ....; y1 y2 y3 ....];
% L is the lenghth in metres of the sides of the spatial domain.
% N is the spatial discratisation number 

function [y, BC_1, BC_2] = Measurement(Hm0,Tp, L, N, Position, Theta_wind, ts)
    Delta_x = L/N;
    Delta_t = 1*ts;
    sa = 0.1; sb = 0.11;
    D = 0.036-0.0056*Tp/sqrt(Hm0);
    gamma = exp(3.484*(1-0.1975*D*Tp^4/(Hm0^2)));
    S_pec = jonswap([], [Hm0, Tp, gamma, sa, sb], 0); % JONSWAP omnidirectional spectrum
    N_t = 20001; th0 = Theta_wind; Sp = 15;
    D = spreading(N_t,'cos2s',th0,Sp,S_pec.w,1); % Frequency dependent spreding function
    S_pec_D = mkdspec(S_pec,D,0); %  Directional Spectrum
    % Simultion option of the 3D wave field
        % 512 points with L/N =  Delta_x between two point
        % Nt give the number of time samples of the wave field and dt is the
        % time step size in seconds.
    Sim_Opt = simoptset('Nt',1505,'dt',Delta_t,'Nu',64,'du',Delta_x,'Nv',64,'dv',Delta_x);

    % Generating the elementary wave field processes
    %[W,X,Y] = spec2ldat3D(S_pec_D, Sim_Opt, 'lalpha', 1.5);
    % Generatiing time series Lagragnge waves at the points.
    Position_Act = Position + 15;
    % The boundary condition calues
    P2 = [16:Delta_x:45.0625; 16*ones(1,length(16:Delta_x:45.0625))];
    P3 = [16*ones(1,length(16:Delta_x:45.0625)); 16:Delta_x:45.0625];
    PPP = [Position_Act, P2, P3];
    %Gen_Opt3D = genoptset('type', 'timeseries', 'PP', Position_Act);
    %L_waves = ldat2lwav3D(W,X,Y,Gen_Opt3D);
    L_waves = spec2lseries(S_pec_D,PPP,Sim_Opt);

    % Extracting the measurement data and the boundary condition data for the estimation
    y = [L_waves.t, L_waves.Z{1,1}', L_waves.Z{1,2}'];
    
    % Extracting the boundary condition data
    BC_1 = L_waves.Z{1,3}';
    for i=1:length(P2)-1
        BC_1 = [BC_1, L_waves.Z{1,3+i}'];
    end
    
    BC_2 = L_waves.Z{1,length(P2)+2}';
    for i=1:length(P3)-1
        BC_2 = [BC_2, L_waves.Z{1,length(P2)+2+i}'];
    end
end