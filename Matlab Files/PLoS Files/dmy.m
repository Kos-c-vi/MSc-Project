%% Plotting the RSSI from pactical experiment.
% By Kossivi 

set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex'); 

Dt = [0.0, 00.25, 0.5, 0.75];  % The vertical height from the lower point of the antenna
RSSIavg = [-82.54929577, -81.61607143, -74.52212389, -72.75675676 ];
scatter(Hant,RSSIdBm, 'filled')

p = polyfit(Dt,RSSIavg,3);
h1 = linspace(0,2,2);
f = polyval(p,Dt);
hold on
plot(Dt,f,'--r','LineWidth',2)

figure('Name','RSSI_PRAC','Units','centimeters','position',[0 0 25 15]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',18,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 25 15],'PaperSize',[25 15]);
scatter(Hant,RSSIdBm, 'filled'), ylabel('\bf{RSSI ($dBm$)}'),
xlabel('$\mathbf{Antenna ~Heigth~~(m)}$'), hold on
plot(Dt,f,'--r','LineWidth',2), legend('\bf{Received Power}','\bf{Line joining the mean received powers}','Location','northwest');
%print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\RSSI_PRAC.pdf','-dpdf');

