%% Definition of the Directional Spectrum 
% Declaring the constants for the Power Spectrum
Omega_C = zeros(3,1);  % Sea age constant for the given wind speed U_10:
Omega_C(1) = 0.84;     % Fully developped sea
Omega_C(2) = 1;       % Matures sea
Omega_C(3) = 5;       % Very Yound sea
U_ten = linspace(2,35,25);          % Wind speed at 10 metres above the ocean surface
U_10 = 10;
g = 9.82;           % Gravitational Acceleration
Cd10N = 0.00144;    % Drag Coefficient
a_0 = 0.1733;       % 
a_p = 4.0;          %
k_m = 370.0;        % rad/m
c_m = 0.23;         % Phase speed of the wave with spatial frequency km
k = 1e-3:0.005:1e+5;    % The angular spatial frequency in rad m?1

for i = 1:length(Omega_C)
    Omega_c = Omega_C(i);
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

    % The function parameters
    phi = 0; %linspace(-pi, pi, 20000000);
    c = sqrt((g./k).*(1+ ((k./k_m).^2)));  % Phase speed of the wave
    L_pm = exp(-1.25.*((k_p./k).^2));    % Shape of the sperctrum (from PM SPECTRUM)
    Gamma = exp(-((sqrt(k./k_p) - 1).^2)./(2.*(sigma.^2)));
    J_p = Lambda.^Gamma;
    B_l = 0.5.*Alpha_p.*(c_p./c).*L_pm.*J_p.*exp(-0.3162.*Omega_c.*(sqrt(k./k_p) - 1));
                                    % Low Frequency comtribution function
    B_h = 0.5.*Alpha_m.*(c_m./c).*L_pm.*J_p.*exp(-0.25.*((sqrt(k./k_m) - 1).^2));
                                    % High Frequency comtribution function
    Phi = (1/2.*pi).*(1 + tanh(a_0 + a_p.*((c./c_p).^2.5) + a_m.*((c_m./c).^2.5)).*cos(2.*phi));
                                    % Spreading Function for the directional
                                    % spectrum 
    % The spectrum
%     Psi = (B_l + B_h).*Phi./(k.^4);
     Psi = (B_l + B_h)./(k.^3);

    figure(5)
    loglog(k,Psi)
    ylim([10e-8 5])
    grid on
    hold on;
    
    figure(6)
    loglog(k,Psi)
    ylim([10e-8 5])
    grid on
    hold on;
    
    figure(7)
    loglog(k,Psi)
    ylim([10e-8 5])
    grid on
    hold on;
end
 
% for i = 1:length(U_ten)
%     U_10 = U_ten(i);
%     Omega_c = Omega_C(1);
%     % Parameters depending on the above constant
%     a_m = 0.13*(sqrt(Cd10N)*U_10/c_m);
%     sigma =  0.08*(1 + 4*(Omega_c^(-3))); % % Sigma
%     if (sqrt(Cd10N)*U_10) <= c_m
%         Alpha_m = 0.01*(1 + log(sqrt(Cd10N)*U_10/c_m));
%     else     
%         Alpha_m = 0.01*(1 + 3*log(sqrt(Cd10N)*U_10/c_m));
%     end
%     Alpha_p =  0.006*(Omega_c^(0.55));
%     if (Omega_c) <= 1       % % Lambda
%         Lambda = 1.7;
%     else     
%         Lambda = 1.7 + 6*log(Omega_c);
%     end
% 
%     k_p = (g/((U_10)^2))*(Omega_c^2);   % Spatial frequency of the maximum of the 
%                                     % spectrum
%     c_p = sqrt(g/k_p);     % Phase speed of the wave with spatial frequency kp
% 
%     % The function parameters
%     k = 1e-3:0.005:1e+5;
%     phi = 0; %linspace(-pi, pi, 20000000);
%     c = sqrt((g./k).*(1+ ((k./k_m).^2)));  % Phase speed of the wave
%     L_pm = exp(-1.25.*((k_p./k).^2));    % Shape of the sperctrum (from PM SPECTRUM)
%     Gamma = exp(-((sqrt(k./k_p) - 1).^2)./(2.*(sigma.^2)));
%     J_p = Lambda.^Gamma;
%     B_l = 0.5.*Alpha_p.*(c_p./c).*L_pm.*J_p.*exp(-0.3162.*Omega_c.*(sqrt(k./k_p) - 1));
%                                     % Low Frequency comtribution function
%     B_h = 0.5.*Alpha_m.*(c_m./c).*L_pm.*J_p.*exp(-0.25.*((sqrt(k./k_m) - 1).^2));
%                                     % High Frequency comtribution function
%     Phi = (1/2.*pi).*(1 + tanh(a_0 + a_p.*((c./c_p).^2.5) + a_m.*((c_m./c).^2.5)).*cos(2.*phi));
%                                     % Spreading Function for the directional
%                                     % spectrum 
%     % The spectrum
%     Psi = (B_l + B_h).*Phi./(k.^4);
%     %Psi_1 = (B_l + B_h)./(k.^3);
% 
%     figure(8)
%     hold on
%     loglog(k,Psi);
% end

