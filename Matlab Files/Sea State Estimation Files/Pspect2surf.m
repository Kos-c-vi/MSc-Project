% Matlab Script for the ocean surface simulation
% by Kossivi Fangbemi
% Part of the Masters disertation

% Function to convert 2D power spectrum to 3d Surface elevation

function surf_WIND = Pspect2surf(Psi,Psi_, N, L) 
% Fundamental Spatial Frequencies:
k_x = (2*pi)/L; % In the x-direction
k_y = (2*pi)/L; % In the y-direction
        % Colour-filtering for the first spectrum
        DPsiroot = sqrt((Psi/2).*(k_x.*k_y));
        DPsiroot_ = sqrt((Psi_/2).*(k_x.*k_y));
        
%         D_DPsiroot = sqrt((D_Psi/1).*(k_x.*k_y));
%         D_DPsiroot_ = sqrt((D_Psi_/1).*(k_x.*k_y));

% Generartion random values for rho and sigma (White Noise)
%Rd_r_s = 1.*randn(N,N,2);
Rd_r_s = unifrnd(-500,500,N,N,2)/500;

z_0hat = zeros(N,N);    z_0hat_ = zeros(N,N);
z_hat = zeros(N,N);     
% Sum_zhatsqrt0 = zeros(N*N,1);
for p = 2:length(Psi)
    for q = 2:length(Psi)
        z_0hat(p,q) =(1/sqrt(2)).*((Rd_r_s(p,q,1) + 1i.*Rd_r_s(p,q,2)).*DPsiroot(p,q));
                                        % Z_0hat at k_uv 
        z_0hat_(p,q) =(1/sqrt(2)).*((Rd_r_s(N-p+2,N-q+2,1) - 1i.*Rd_r_s(N-p+2,N-q+2,2)).*DPsiroot_(p,q));
                                        % Z_0hat conjugate at - k_uv                                        
        z_hat(p,q) = N*N*(1/sqrt(2)).*(z_0hat(p,q) + z_0hat_(p,q));
        
        % Colour-filtering for the second spectrum 
%         D_z_0hat(p,q) =(1/sqrt(2)).*((Rd_r_s(p,q,1) + 1i.*Rd_r_s(p,q,2)).*D_DPsiroot(p,q));
%                                         % Z_0hat at k_uv 
%         D_z_0hat_(p,q) =(1/sqrt(2)).*((Rd_r_s(N-p+2,N-q+2,1) - 1i.*Rd_r_s(N-p+2,N-q+2,2)).*D_DPsiroot_(p,q));
%                                         % Z_0hat conjugate at - k_uv 
%         D_z_hat(p,q) = N*N*(1/sqrt(2)).*(D_z_0hat(p,q).*exp(-1i.*Omega(p,q).*t(tt)) + D_z_0hat_(p,q).*exp(1i.*Omega_(p,q).*t(tt)));
    end
end 
   
%% Getting the Surface elevation and plotting it.
    z_hat(1,1) = 0;
    z_hat((N/2 +1), (N/2  + 1)) = 0;

% The inverse FFT
    zz_c = ifft2(z_hat, 'symmetric');
    surf_WIND = real(zz_c);
    %zz_i = imag(zz_c); % This should be zero.
end