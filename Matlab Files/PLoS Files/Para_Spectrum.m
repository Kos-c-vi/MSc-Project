
U10 = 10;
% Definition of the  PM spectrum
U19 = 1.075*U10;
g = 9.81;
alpha = 0.0081;
beta = 0.71;
sigma0 = 0.07;
%syms omega;
omega = linspace(0.0,3.00,1200); 
SPM2 = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);

% Definition of the  JONSWAP spectrum
X = 250000;
alpha = 0.0076.*(g.*X/U19^2)^(-0.22);
omega_P = 7.*pi.*(g/U19).*(g.*X/U19^2)^(-0.33);
delta = exp(-((omega - omega_P).^2)./(2.*(sigma0.^2).*(omega_P.^2)));
index = (omega < omega_P);
sigma0 = 0.09;
delta(index) = exp(-((omega(index) - omega_P).^2)./(2.*(sigma0.^2).*(omega_P.^2)));
JONSWAP = SPM2.*(3.3.^(delta));

figure('Name','spectrum_Comp','Units','centimeters','position',[0 0 23 10]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',11,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
plot(omega,JONSWAP,omega,SPM2,'LineWidth',2), ylabel('\bf{Spectral Density ($m^2/s$)}'),
xlabel('$\mathbf{frequency~~(rad/s)}$'),box on, grid on, legend('\bf{JONSWAP spectrum}','\bf{Pierson-Moskowitz Spectrum}');
print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\spectrum_Comp.pdf','-dpdf');
