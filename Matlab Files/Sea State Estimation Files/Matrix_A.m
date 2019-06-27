% Function to compute the state transition Matric D depending on:
% The wind speed U_10
% The wind age factor Omega_c
% The sampling time ts
% N is the spatial discratisation number 
% L is the lenghth in metres of the region sides
% And the main wind direction Theta_wind

function A = Matrix_A(U_10,Omega_c,ts,N,L,Theta_wind)
    g = 9.82;            % Gravitational Acceleration  
    k_p = (g/((U_10)^2))*(Omega_c^2);   % Spatial frequency of the maximum of the 
                                    % spectrum
    c_p = sqrt(g/k_p);     % Phase speed of the wave with spatial frequency kp
    
    Delta_x = L/N;
    Delta_y = L/N;
    c_x = c_p*cos(Theta_wind);
    c_y = c_p*sin(Theta_wind);
    Alpha_x = (c_x*ts/Delta_x)^2;
    Alpha_y = (c_y*ts/Delta_y)^2;
    Alpha = 2*(1 - Alpha_x - Alpha_y);
    
    A = sparse(N*N*2,N*N*2);
    j = 0;
    for i = 1:2:N*N*2
        A(i,i) = Alpha;
        A(i,i+1) = -1;
        A(i+1,i) = 1;
        if (i < (N*N*2 - 3))
            A(i,i+2) = Alpha_x;
            A(i+2,i) = Alpha_x;
        end
        % Boundary Conditions in x-direction
        if((mod(i+1,2*N) == 0) && (i < (N*N*2 - 3)))
            A(i,i+2) = 0;
            A(i+2,i) = 0;
            j = j+1;
        end
        if((mod(i+1,2*N) == 0) && (i < (N*N*2 - 2*N-1)))
            A((j-1)*2*N +1 + 2*N - 2,(j-1)*2*N +1) = Alpha_x;
            A((j-1)*2*N +1, (j-1)*2*N +1 + 2*N - 2) = Alpha_x;
        end
        if (i < (N*N*2 - (2*N + 1)))
            A(i,i+(2*N)) = Alpha_y;
            A(i+(2*N),i) = Alpha_y;
        end
        % Boundary Conditions in x-direction
        if (i < (N*N*2 - ((N-1)*2*N + 1)))
            A(i,i+(N-1)*2*N) = Alpha_y;
            A(i+(N-1)*2*N, i) = Alpha_y;
        end
    end
    A = sparse(A);
end 