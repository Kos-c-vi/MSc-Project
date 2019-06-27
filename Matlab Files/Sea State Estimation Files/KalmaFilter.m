% Matlab Script for the ocean surface simulation
% by Kossivi Fangbemi
% Part of the Masters disertation

% Function for the Kalman Filter algorighm

function Syst = KalmaFilter(Syst,i)

        % Computing Kalman gain factor:
        C_T = sparse(Syst.C');
        Dm_K1 = sparse(Syst.P * C_T);
        Dm_K2 = sparse(Syst.C * Syst.P);
        Dm_K3 = sparse(Dm_K2 * C_T);
        clear Dm_K2; % Clearing the variables to free memory.
        Dm_K4 = Dm_K3 + Syst.R;
        clear Dm_K3;
        Dm_K5 = inv(Dm_K4);
        clear Dm_K4;
        K = sparse(Dm_K1 * Dm_K5);
        
        % Correction based on observation:
        Dm_CF = (Syst.y(i,2:end)' - Syst.C * Syst.State_X);
        Syst.State_X = Syst.State_X + K * Dm_CF;
        clear Dm_CF;
        % Computation of the error covariance for the updated estimate:
        Syst.P = Syst.P - K * Syst.C * Syst.P;  
        
   for j = 1:1 
        % Prejection for the next state vector and covariance:
        Syst.State_X = Syst.A * Syst.State_X;% + Syst.B*Syst.Input;
        % Update of the state error covariance
        A_T = Syst.A';
        Dm_P1 = sparse( Syst.P* A_T);
        clear A_T
        Dm_P11 = sparse( Syst.A*Dm_P1 );
        clear AA
        clear Dm_P1
        Syst.P = Dm_P11 + Syst.Q;
        clear Dm_P11
   end
end

% Dm_P11 = full(gather(Dm_P1));
%         Dm_P12 = full(gather(Syst.A));
%         %Dm_P2 = sparse(Syst.A *Dm_P1 );
%         for ki = 1:length(Dm_P12)
%             for kki = 1:length(Dm_P11)
%                 Summ_0 = Dm_P12(ki,:).*Dm_P11(:,kki)';
%                 Dm_P2(ki,kki) = sum(Summ_0);
%             end
%         end