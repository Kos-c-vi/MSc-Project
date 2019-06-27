%% Matlab script to evaluate the controlability of the Discret State Space system for the Kalman Filter application
% by Kossivi Fangbemi
% Part of the Masters disertation

% System Model: X(n+1) = A X(n) 
%               y(n) = C X(n)
% Checking the Observability of A and C
% ********************************************************************* %
%% Getting the transition matrix A
% Definition of constants to calculkate the pase speed of the waves
                    % Sea age constant for the given wind speed U_10:
                    Omega_c = 0.84;     % Fully developped sea
                    %Omega_c = 1;       % Matures sea
                    %Omega_c = 5;       % Very Yound sea
                    U_10 = 10;          % Wind speed at 10 metres above the ocean surface
                    Theta_wind = 1*pi/4;  % Wind direction ==> Wave propagartion direction
                    g = 9.82;           % Gravitational Acceleration
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
% Defining the dimensions of theregion of interest
L = 60;
% Defining the grid cell of the region of interest
N = 2;
% Calculating the sampling period for stability
tsc = min([(L/(N*c_p*sqrt(2))),(L/(N*c_p*sqrt(2))), 0.01]) ; % Sampling Time 
% Computing the transition matrix A
Ac = Matrix_A_simple(U_10,Omega_c,tsc,N,L,Theta_wind);

    % Measurement Possition
    P = [3 55; 5 50];
%% Creating the measurement Matrix C
Cc = Matrix_C(L, N, P);

% Computing the Observability matrix
OBV_Mx = zeros(2*N*N*length(P),2*N*N);
%OBV_Mx = spalloc(2*N*N,2*N*N,2*N*N*2*N*N);
Row_Vec = Cc;
OBV_Mx(1:length(P),:) = Row_Vec;
for i = 3:2:length(OBV_Mx)
    Row_Vec = Row_Vec*Ac;
    mag1 = sqrt(sum(Row_Vec(1,:).*Row_Vec(1,:))); % Getting the magnitude to normalise the vector
    mag2 = sqrt(sum(Row_Vec(2,:).*Row_Vec(2,:)));
    Row_Vec(1,:) = Row_Vec(1,:)/mag1;
    Row_Vec(2,:) = Row_Vec(2,:)/mag2;
    OBV_Mx(i:i+1,:) = Row_Vec;
end 

Ob_mat = obsv(Ac,Cc);
 OB = rank(Ob_mat)
 Cond0 = cond(Ob_mat)
 OB_0 = rank(OBV_Mx)
 Cond1 = cond(OBV_Mx)
 %non_ob_states = length(A) - OB