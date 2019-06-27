% Matlab Script for the ocean surface simulation
% by Kossivi Fangbemi
% Part of the Masters disertation

% Plots of the different probability density functions (pdf)

%% Probability of LOS 
clear all
set(groot,'defaulttextinterpreter','latex');  
set(groot, 'defaultAxesTickLabelInterpreter','latex');  
set(groot, 'defaultLegendInterpreter','latex');  

% Definition of the Wind speed and the distance D
U10 = 7.5;
%U10 = ones(1,3);
%U10(1) = 5; U10(1) = 8.5; U10(1) = 10;
D = 5000;
% Definition of the  PM spectrum
U19 = 1.075*U10;
g = 9.81;
alpha = 0.0081;
beta = 0.71;
syms omega;
SPM = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);

% The Randon variables
s = linspace(-3.0,3.0,1200); 
y = linspace(0,7,1200); 
h = linspace(0,10,1200); 
a = linspace(0,5,1200); 
% The variance of the surface elevation from the PM SPECTRUM
variances = (1/pi)*double(int(SPM,omega,0,Inf));
var7_5 = variances;
% The std of the surface height
sigmas = sqrt(variances);
% The RMS Height of the wave
H = 2*sqrt(2)*sigmas;
Hb = H;
% The wavelenght of the wave
lambda = 2.*pi*(U19^2)/((0.877^2)*g);
% The  nuber of wave sampled
N = D/lambda;

% The pdf of surface elevation
pdfs = (exp(-(s.^2/(2*variances))))/(sqrt(2.*pi).*sigmas);
pdfs1 = normpdf(s,0,sigmas);
% The PDF of surface elevation
%PDFs = int(pdfs,0,6)

% The pdf of Height
pdfH = (2.*y.*exp(-(y.^2)/(H^2)))/(H^2);

% The pdf of extreme Height
pdfeH = 2.*N.*(h/H.^2).*exp(-(h.^2)/(H^2)).*(1 - exp(-(h.^2)/(H^2))).^(N-1);

% The pdf of extreme Height
pdfeA = 4.*N.*(a/H.^2).*exp(-4.*(a.^2)/(H^2)).*(1 - exp(-4.*(a.^2)/(H^2))).^(N-1);

U10 = 10;
%U10 = ones(1,3);
%U10(1) = 5; U10(1) = 8.5; U10(1) = 10;
D = 5000;
% Definition of the  PM spectrum
U19 = 1.075*U10;
g = 9.81;
alpha = 0.0081;
beta = 0.71;
syms omega;
SPM1 = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);

% The Randon variables
s = linspace(-3.0,3.0,1200); 
y = linspace(0,7,1200); 
h = linspace(0,10,1200); 
a = linspace(0,5,1200); 
% The variance of the surface elevation from the PM SPECTRUM
variances = (1/pi)*double(int(SPM1,omega,0,Inf));
var10 = variances;
% The std of the surface height
sigmas = sqrt(variances);
% The RMS Height of the wave
H = 2*sqrt(2)*sigmas;
Hb1 = H;
% The wavelenght of the wave
lambda = 2.*pi*(U19^2)/((0.877^2)*g);
% The  nuber of wave sampled
N = D/lambda;

% The pdf of surface elevation
pdfs1 = (exp(-(s.^2/(2*variances))))/(sqrt(2.*pi).*sigmas);
%pdfs1 = normpdf(s,0,sigmas);
% The PDF of surface elevation
%PDFs = int(pdfs,0,6)

% The pdf of Height
pdfH1 = (2.*y.*exp(-(y.^2)/(H^2)))/(H^2);

% The pdf of extreme Height
pdfeH1 = 2.*N.*(h/H.^2).*exp(-(h.^2)/(H^2)).*(1 - exp(-(h.^2)/(H^2))).^(N-1);

% The pdf of extreme Height
pdfeA1 = 4.*N.*(a/H.^2).*exp(-4.*(a.^2)/(H^2)).*(1 - exp(-4.*(a.^2)/(H^2))).^(N-1);

U10 = 12.5;
%U10 = ones(1,3);
%U10(1) = 5; U10(1) = 8.5; U10(1) = 10;
D = 5000;
% Definition of the  PM spectrum
U19 = 1.075*U10;
g = 9.81;
alpha = 0.0081;
beta = 0.71;
syms omega;
SPM2 = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);

% The Randon variables
s = linspace(-3.0,3.0,1200); 
y = linspace(0,7,1200); 
h = linspace(0,10,1200); 
a = linspace(0,5,1200); 
% The variance of the surface elevation from the PM SPECTRUM
variances = (1/pi)*double(int(SPM2,omega,0,Inf));
var12_5 = variances;
% The std of the surface height
sigmas = sqrt(variances);
% The RMS Height of the wave
H = 2*sqrt(2)*sigmas;
Hb2 = H;
% The wavelenght of the wave
lambda = 2.*pi*(U19^2)/((0.877^2)*g);
% The  nuber of wave sampled
N = D/lambda;

% The pdf of surface elevation
pdfs2 = (exp(-(s.^2/(2*variances))))/(sqrt(2.*pi).*sigmas);
%pdfs1 = normpdf(s,0,sigmas);
% The PDF of surface elevation
%PDFs = int(pdfs,0,6)

% The pdf of Height
pdfH2 = (2.*y.*exp(-(y.^2)/(H^2)))/(H^2);

% The pdf of extreme Height
pdfeH2 = 2.*N.*(h/H.^2).*exp(-(h.^2)/(H^2)).*(1 - exp(-(h.^2)/(H^2))).^(N-1);

% The pdf of extreme Amplitude
pdfeA2 = 4.*N.*(a/H.^2).*exp(-4.*(a.^2)/(H^2)).*(1 - exp(-4.*(a.^2)/(H^2))).^(N-1);

figure('Name','pdf_surface','Units','centimeters','position',[0 0 23 9]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',11,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 8],'PaperSize',[21 8]);
plot(s,pdfs,s,pdfs1,s,pdfs2,'LineWidth',2), ylabel('\bf{Probability Density Function}'),
xlabel('$\mathbf{\eta~~(m)}$'),box on, grid on, legend('$\mathbf{U_{10}~ =~ 7.5~ m/s}$','$\mathbf{U_{10}~ =~ 10 ~m/s}$','$\mathbf{U_{10}~ =~ 12.5~ m/s}$');
print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\pdf_surface.pdf','-dpdf');
 
figure('Name','pdf_surface1','Units','centimeters','position',[0 0 23 9]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',11,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 8],'PaperSize',[21 8]);
plot(y,pdfH,y,pdfH1,y,pdfH2,'LineWidth',2), ylabel('\bf{Probability Density Function}'),
xlabel('\bf{H~~(m)}'),box on, grid on, legend('$\mathbf{U_{10}~ =~ 7.5~ m/s}$','$\mathbf{U_{10}~ =~ 10 ~m/s}$','$\mathbf{U_{10}~ =~ 12.5~ m/s}$');
print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\pdf_heigth.pdf','-dpdf');

figure('Name','pdf_surface1','Units','centimeters','position',[0 0 23 9]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',11,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 8],'PaperSize',[21 8]);
plot(h,pdfeH,h,pdfeH1,h,pdfeH2,'LineWidth',2), ylabel('\bf{Probability Density Function}'),
xlabel('$\mathbf{H_{e}}$~~\bf{(m)}'),box on, grid on, legend('$\mathbf{U_{10}~ =~ 7.5~ m/s}$','$\mathbf{U_{10}~ =~ 10 ~m/s}$','$\mathbf{U_{10}~ =~ 12.5~ m/s}$');
print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\pdf_Ext_height.pdf','-dpdf');

figure('Name','pdf_surface3','Units','centimeters','position',[0 0 23 9]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',11,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 8],'PaperSize',[21 8]);
plot(a,pdfeA,a,pdfeA1,a,pdfeA2,'LineWidth',2), ylabel('\bf{Probability Density Function}'),
xlabel('$\mathbf{A_{e}}$~~\bf{(m)}'),box on, grid on, legend('$\mathbf{U_{10}~ =~ 7.5~ m/s}$','$\mathbf{U_{10}~ =~ 10 ~m/s}$','$\mathbf{U_{10}~ =~ 12.5~ m/s}$');
print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\pdf_Ext_Amplitude.pdf','-dpdf');


U10 = 10;
D = 100;
% Definition of the  PM spectrum
U19 = 1.075*U10;
g = 9.81;
alpha = 0.0081;
beta = 0.71;
syms omega;
SPM2 = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);

% The Randon variables
h = linspace(0,7,1200); 
a = linspace(0,4,1200); 
% The variance of the surface elevation from the PM SPECTRUM
variances = (1/pi)*double(int(SPM2,omega,0,Inf));
var12_5 = variances;
% The std of the surface height
sigmas = sqrt(variances);
% The RMS Height of the wave
H = 2*sqrt(2)*sigmas;
Hb2 = H;
% The wavelenght of the wave
lambda = 2.*pi*(U19^2)/((0.877^2)*g);
% The  nuber of wave sampled
N = D/lambda;

% The pdf of surface elevation
pdfs3 = (exp(-(s.^2/(2*variances))))/(sqrt(2.*pi).*sigmas);
%pdfs1 = normpdf(s,0,sigmas);
% The PDF of surface elevation
%PDFs = int(pdfs,0,6)

% The pdf of Height
pdfH3 = (2.*y.*exp(-(y.^2)/(H^2)))/(H^2);

% The pdf of extreme Height
pdfeH3 = 2.*N.*(h/H.^2).*exp(-(h.^2)/(H^2)).*(1 - exp(-(h.^2)/(H^2))).^(N-1);

% The pdf of extreme Amplitude
pdfeA3 = 4.*N.*(a/H.^2).*exp(-4.*(a.^2)/(H^2)).*(1 - exp(-4.*(a.^2)/(H^2))).^(N-1);

U10 = 10;
D = 1000;
% Definition of the  PM spectrum
U19 = 1.075*U10;
g = 9.81;
alpha = 0.0081;
beta = 0.71;
syms omega;
SPM2 = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);

% The Randon variables
h = linspace(0,7,1200); 
a = linspace(0,4,1200); 
% The variance of the surface elevation from the PM SPECTRUM
variances = (1/pi)*double(int(SPM2,omega,0,Inf));
var12_5 = variances;
% The std of the surface height
sigmas = sqrt(variances);
% The RMS Height of the wave
H = 2*sqrt(2)*sigmas;
Hb2 = H;
% The wavelenght of the wave
lambda = 2.*pi*(U19^2)/((0.877^2)*g);
% The  nuber of wave sampled
N = D/lambda;

% The pdf of surface elevation
pdfs3 = (exp(-(s.^2/(2*variances))))/(sqrt(2.*pi).*sigmas);
%pdfs1 = normpdf(s,0,sigmas);
% The PDF of surface elevation
%PDFs = int(pdfs,0,6)

% The pdf of Height
pdfH4 = (2.*y.*exp(-(y.^2)/(H^2)))/(H^2);

% The pdf of extreme Height
pdfeH4 = 2.*N.*(h/H.^2).*exp(-(h.^2)/(H^2)).*(1 - exp(-(h.^2)/(H^2))).^(N-1);

% The pdf of extreme Amplitude
pdfeA4 = 4.*N.*(a/H.^2).*exp(-4.*(a.^2)/(H^2)).*(1 - exp(-4.*(a.^2)/(H^2))).^(N-1);

U10 = 10;
D = 5000;
% Definition of the  PM spectrum
U19 = 1.075*U10;
g = 9.81;
alpha = 0.0081;
beta = 0.71;
syms omega;
SPM2 = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);

% The Randon variables
h = linspace(0,7,1200); 
a = linspace(0,4,1200); 
% The variance of the surface elevation from the PM SPECTRUM
variances = (1/pi)*double(int(SPM2,omega,0,Inf));
var12_5 = variances;
% The std of the surface height
sigmas = sqrt(variances);
% The RMS Height of the wave
H = 2*sqrt(2)*sigmas;
Hb2 = H;
% The wavelenght of the wave
lambda = 2.*pi*(U19^2)/((0.877^2)*g);
% The  nuber of wave sampled
N = D/lambda;

% The pdf of surface elevation
pdfs5 = (exp(-(s.^2/(2*variances))))/(sqrt(2.*pi).*sigmas);
%pdfs1 = normpdf(s,0,sigmas);
% The PDF of surface elevation
%PDFs = int(pdfs,0,6)

% The pdf of Height
pdfH5 = (2.*y.*exp(-(y.^2)/(H^2)))/(H^2);

% The pdf of extreme Height
pdfeH5 = 2.*N.*(h/H.^2).*exp(-(h.^2)/(H^2)).*(1 - exp(-(h.^2)/(H^2))).^(N-1);

% The pdf of extreme Amplitude
pdfeA5 = 4.*N.*(a/H.^2).*exp(-4.*(a.^2)/(H^2)).*(1 - exp(-4.*(a.^2)/(H^2))).^(N-1);

U10 = 10;
D = 10000;
% Definition of the  PM spectrum
U19 = 1.075*U10;
g = 9.81;
alpha = 0.0081;
beta = 0.71;
syms omega;
SPM2 = (alpha*g*g./(omega.^5)).*exp(-beta.*(g./(U19.*omega)).^4);

% The Randon variables
h = linspace(0,7,1200); 
a = linspace(0,4,1200); 
% The variance of the surface elevation from the PM SPECTRUM
variances = (1/pi)*double(int(SPM2,omega,0,Inf));
var12_5 = variances;
% The std of the surface height
sigmas = sqrt(variances);
% The RMS Height of the wave
H = 2*sqrt(2)*sigmas;
Hb2 = H;
% The wavelenght of the wave
lambda = 2.*pi*(U19^2)/((0.877^2)*g);
% The  nuber of wave sampled
N = D/lambda;

% The pdf of surface elevation
pdfs5 = (exp(-(s.^2/(2*variances))))/(sqrt(2.*pi).*sigmas);
%pdfs1 = normpdf(s,0,sigmas);
% The PDF of surface elevation
%PDFs = int(pdfs,0,6)

% The pdf of Height
pdfH6 = (2.*y.*exp(-(y.^2)/(H^2)))/(H^2);

% The pdf of extreme Height
pdfeH6 = 2.*N.*(h/H.^2).*exp(-(h.^2)/(H^2)).*(1 - exp(-(h.^2)/(H^2))).^(N-1);

% The pdf of extreme Amplitude
pdfeA6 = 4.*N.*(a/H.^2).*exp(-4.*(a.^2)/(H^2)).*(1 - exp(-4.*(a.^2)/(H^2))).^(N-1);

figure('Name','pdf_surface4','Units','centimeters','position',[0 0 23 9]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',11,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
plot(h,pdfeH3,h,pdfeH4,h,pdfeH5,h,pdfeH6,'LineWidth',2), ylabel('\bf{Probability Density Function}'),
xlabel('$\mathbf{H_e~~(m)}$'),box on, grid on, legend('$\mathbf{D~ =~0.1 }$ km','$\mathbf{D~ =~1 }$ km','$\mathbf{D~ =~5 }$ km','$\mathbf{D~ =~10 }$ km');
print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\pdf_He_Comp.pdf','-dpdf');

figure('Name','pdf_surface4','Units','centimeters','position',[0 0 23 9]); set(0,'defaultfigurecolor',[1 1 1 ]);  set(gca,'fontsize',11,'FontName','Helvetica')
set(gcf,'PaperUnits','centimeters','PaperPosition',[0 0 20.9 10],'PaperSize',[21 10]);
plot(a,pdfeA3,a,pdfeA4,a,pdfeA5,a,pdfeA6,'LineWidth',2), ylabel('\bf{Probability Density Function} '),
xlabel('$\mathbf{A_e~~(m)}$'),box on, grid on, legend('$\mathbf{D~ =~0.1 }$ km','$\mathbf{D~ =~1 }$ km','$\mathbf{D~ =~5 }$ km','$\mathbf{D~ =~10 }$ km');
print('C:\Users\user\OneDrive - University of Cape Town\#School2015--\MSc Research\Thesis Report New\Figures\pdf_Ae_Comp.pdf','-dpdf');
