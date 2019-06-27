% Matlab Script for the ocean surface simulation
% by Kossivi Fangbemi
% Part of the Masters disertation, Second version.

% Function to compute the measurements at points whose coordinates in metres are given in matrix Position.
% Position = [x1 x2 x3 ....; y1 y2 y3 ....];
% L is the lenghth in metres of the sides of the spatial domain.
% N is the spatial discratisation number 

function [y, BC_1, BC_2, Surf_Surf] = Measmt(Hm0,Tp, L, N, Position, Theta_wind, ts)
    Delta_x = L/N;
    Delta_t = 1*ts;
    sa = 0.1; sb = 0.11;
    D_1 = 0.036-0.0056*Tp/sqrt(Hm0);
    gamma = exp(3.484*(1-0.1975*D_1*Tp^4/(Hm0^2)));
    S_pec = jonswap([], [Hm0, Tp, gamma, sa, sb], 0); % JONSWAP omnidirectional spectrum
    N_t = 20001; th0 = Theta_wind; Sp = 15;
    D = spreading(N_t,'cos2s',th0,Sp,S_pec.w,1); % Frequency dependent spreding function
    S_pec_D = mkdspec(S_pec,D,0); %  Directional Spectrum
    % Simultion option of the 3D wave field
        % 512 points with L/N =  Delta_x between two point
        % Nt give the number of time samples of the wave field and dt is the
        % time step size in seconds.
        Nt = 300;
        Nu = N*2;
    Sim_Opt = simoptset('Nt',Nt,'dt',Delta_t,'Nu',Nu,'du',Delta_x,'Nv',Nu,'dv',Delta_x);

    % Generating the elementary wave field processes
    %[W,X,Y] = spec2ldat3D(S_pec_D, Sim_Opt, 'lalpha', 1.5);
    % Generatiing time series Lagragnge waves at the points.
        xStart = (Nu*Delta_x)/4;
    Position_Act = Position + xStart;
    % The boundary condition values and the surface elevation
        Pp = xStart + (0:N-1)*Delta_x;
    P2 = [Pp; 15*ones(1,length(Pp))];
    P3 = [15*ones(1,length(Pp)); Pp];
    P_s = zeros(2,length(ones(1,length(Pp)))^2);
    for i = 1:length(ones(1,length(Pp)))
        P_s(:,((length(Pp)*(i-1))+1):(length(Pp))*(i))= [(15+i-1)*ones(1,length(Pp));Pp];
    end 
    PPP = [Position_Act, P2, P3, P_s];
    %L_waves = ldat2lwav3D(W,X,Y,Gen_Opt3D);
    L_waves = spec2lseries(S_pec_D,PPP,Sim_Opt);
    
    % Extracting the measurement data and the boundary condition data for the estimation
    y = [L_waves.t, L_waves.Z{1,1}', L_waves.Z{1,2}'];
    
    % Extracting the boundary condition data
    BC_1 = L_waves.Z{1,3}';
    for i=1:length(P2)-1
        BC_1 = [BC_1, L_waves.Z{1,3+i}'];
    end
    
    BC_2 = L_waves.Z{1,length(P2)+ 3}';
    for i=1:length(P3)-1
        BC_2 = [BC_2, L_waves.Z{1,length(P2)+3+i}'];
    end
    
    Surf_vec = L_waves.Z{1,length([Position_Act, P2, P3])+ 1}';
    for i=1:length(P_s)-1
        ii = i;
        Surf_vec = [Surf_vec, L_waves.Z{1,length([Position_Act, P2, P3]) + 1 + i}'];
    end
    for jj = 1:Nt
    Surf_Surf(:,:,jj) = reshape(Surf_vec(jj,:),length(Pp),length(Pp));
    end
end