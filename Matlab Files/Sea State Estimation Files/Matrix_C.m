% Matlab Script for the ocean surface simulation
% by Kossivi Fangbemi
% Part of the Masters disertation

% Function to compute the measurement (Observation) Matric C depending on the measurement
% coordinates P in metres.
% L is the lenghth in metres of the region sides
% N is the spatial discratisation number 
function C = Matrix_C(L, N, P)
    if(all(P <= L))
        C = zeros(length(P), 2*N*N); % Declaring C for memory allocation
        P_in_C = zeros(length(P), 1); % Declaring an dummy array P_in_C for memory allocation
                                      % This array will contain the
                                      % position of the 1's entry in the C
                                      % Matrix
        Delta_x = L/N;                % Calciulation the spartial step size 
        P_N = ceil(P/Delta_x);        % Getting the integer coordinates of the measuremet cells
                                      
                                      % Forming Measurement Matrix C
        for i = 1:length(P)
            P_in_C(i) =  (N*2)*(P_N(2,i)-1) + (2)*(P_N(1,i)-1) +1;
            C(i,P_in_C(i)) = 1;
        end
        C = sparse(C);
    else
        disp('Measurement Coodinates beyong the limit of the region L*L m^2')
    end
end