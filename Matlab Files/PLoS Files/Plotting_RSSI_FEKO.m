
% Plotting the RSSI from Feko and experiment
% By Kossivi FAngbemi

set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex'); 

Power_lvl_xz = [-85.31, -79.14, -75.52, -73.17, -71.79, -71.33];
Power_lvl_yz = [-85.07, -77.35, -74.34, -72.58, -71.63, -71.33];
Ang = [75, 60, 45, 30, 15, 0];
H_ant0 = 1*cos(Ang*pi/180);

Power_lvl_prac = [-82.55, -81.61, -74.52, -72.77];
H_ant = [0, 0.25, 0.50, 0.75];


figure(1)
plot(H_ant0,Power_lvl_xz,H_ant0,Power_lvl_yz,H_ant,Power_lvl_prac);

figure('Name','RSSI_FEKO_5K','Units','centimeters','position',[5 5 20 9]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'FontWeight','bold','fontsize',11,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20 9],'PaperSize',[20 9]);
plot(H_ant0,Power_lvl_xz,H_ant0,Power_lvl_yz,H_ant,Power_lvl_prac,'LineWidth',2), ylabel('\bf{RSSI ($dBm$)}'),
xlabel('\bf{Visible Antenna Height~~(m)}'),box on, grid on, legend('\bf{Power Level for antenna moving in ZX plane}','\bf{Power Level for antenna moving in Zy plane}','\bf{ Average Power Level from the field experiment}' ,'Location','northwest');
print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\RSSI_FEKO_5K.pdf','-dpdf');
