clear all; close all; clc;
%% Part 1 - Plant Model
load('system.mat');
G11=system(1,1);
G22=system(2,2);
G33=system(3,3);
G44=system(4,4);
G55=system(5,5);
G12=system(1,2);
[freq,zeta,poles]=damp(G22);

%% Part 2 - Performance of various control laws - Damping of the First mode
%sisotool(G22)
PPF1_1=load('PPF1_1.mat'); %C'est celui là que je décide de garder
PPF1_1=PPF1_1.PPF1_1;
sys1_1=feedback(system,PPF1_1,2,2,+1);
G11_11=sys1_1(1,1);
G22_11=sys1_1(2,2);

PPF1_2=load('PPF1_2.mat');
PPF1_2=PPF1_2.PPF1_2;
sys1_2=feedback(system,PPF1_2,2,2,+1);
G11_12=sys1_2(1,1);
G22_12=sys1_2(2,2);

PPF1_3=load('PPF1_3.mat');
PPF1_3=PPF1_3.PPF1_3;
sys1_3=feedback(system,PPF1_3,2,2,+1);
G11_13=sys1_3(1,1);
G22_13=sys1_3(2,2);

PPF1_4=load('PPF1_4.mat');
PPF1_4=PPF1_4.PPF1_4;
sys1_4=feedback(system,PPF1_4,3,3,+1);
G11_14=sys1_4(1,1);
G22_14=sys1_4(2,2);
G33_14=sys1_4(3,3);

%% Damping of the Second mode
PPF2_1=load('PPF2_1.mat');
PPF2_1=PPF2_1.PPF2_1;
sys2_1=feedback(sys1_1,PPF2_1,2,2,+1); %On rajoute une rétroaction
G11_21=sys2_1(1,1);
G22_21=sys2_1(2,2);

PPF2_2=load('PPF2_2.mat');
PPF2_2=PPF2_2.PPF2_2;
sys2_2=feedback(sys1_1,PPF2_2,2,2,+1); %On rajoute une rétroaction, c'est celui là que je garde
G11_22=sys2_2(1,1);
G22_22=sys2_2(2,2);

PPF2_3=load('PPF2_3.mat');
PPF2_3=PPF2_3.PPF2_3;
sys2_3=feedback(sys1_1,PPF2_3,2,2,+1); %On rajoute une rétroaction
G11_23=sys2_3(1,1);
G22_23=sys2_3(2,2);

%% First Order PPF
PPF3_1=load('PPF3_1'); %On redesign sur le système initial
PPF3_1=PPF3_1.PPF3_1;
sys3_1=feedback(system,PPF3_1,2,2,+1);
G11_31=sys3_1(1,1);
G22_31=sys3_1(2,2);

PPF3_2=load('PPF3_2');
PPF3_2=PPF3_2.PPF3_2;
sys3_2=feedback(system,PPF3_2,2,2,+1);
G11_32=sys3_2(1,1);
G22_32=sys3_2(2,2);

PPF3_3=load('PPF3_3');
PPF3_3=PPF3_3.PPF3_3;
sys3_3=feedback(system,PPF3_3,2,2,+1);
G11_33=sys3_3(1,1);
G22_33=sys3_3(2,2);

%% Noise Budgetting
Fr=logspace(1,6,2000); %until 2kHz
w=2*pi*Fr;
C=1;
STresp=freqresp(system,w);
STrespc=freqresp(sys1_1,w);

Phiw=C*random('Normal',1e-4,0,size(w));
Phin1=random('Normal',1e-5,0,size(w)); %sensor
Phin2=random('Normal',1e-4,0,size(w)); %actuator

Sw=zeros(5,5);
freq_PPF=freqresp(PPF1_1,w);

for p=1:length(w)
    Sw(1,1)=Phiw(p); %Disturbance 
    Sw(2,2)=Phin2(p); %Noise on actuator 2
    PhiX(:,:,p)=STresp(:,:,p)*Sw(:,:)*STresp(:,:,p)';
    
    n1(p)=STrespc(1,2,p)*(freq_PPF(:,:,p)*Phin1(p))*STrespc(2,1,p)'; %sensor noise
    n2(p)=STrespc(1,2,p)*Sw(2,2)*STrespc(2,1,p)'; %filtered actuator noise
    
    Sw(2,2)=Sw(2,2)+freq_PPF(:,:,p)*Phin1(p); %noise on sensor is fedback in the loop
    PhiXc(:,:,p)=STrespc(:,:,p)*Sw(:,:)*STrespc(:,:,p)';
end
x=PhiX(1,1,:)+Phin1(1,1,:); %adding a noise on pointSensor
x=abs(x(:));
xc=PhiXc(1,1,:); xc=abs(xc(:));
n1=abs(n1); n2=abs(n2);

figure
bode(frd(abs(Phiw),w)) %signal déplacement
legend('input disturbance')
figure
bode(frd(x,w))
legend('open-loop displacement')
figure
bode(frd(xc,w),frd(n1,w))
legend('closed-loop displacement','filtered noise')

%% Bode Diagram Damping of the first mode
figure('Renderer','painters','Position',[10 10 900 600])
opt1=bodeoptions;
opt1.Title.String='Bode Diagram of G_{22}(s) for 3 differents PPF designs';
opt1.Title.FontSize=14;
opt1.XLabel.FontSize=14; opt1.YLabel.FontSize=14;
opt1.TickLabel.FontSize=12;
opt1.Grid='on';
bodeplot(G22,G22_11,G22_12,G22_13,opt1)
legend({'G22(s)','$g=-24.6\ \zeta=0.082\ \omega n=780\ rad/s$','$g=-80.6\ \zeta=0.15\ \omega n=824\ rad/s$','$g=-269.6\ \zeta=0.3\ \omega n=1055\ rad/s$'},'Interpreter','Latex')

figure('Renderer','painters','Position',[10 10 900 600])
opt2=opt1;
opt2.Title.String='Bode Diagram of G_{12}(s)';
bodeplot(G12,opt2)
legend({'G12(s)'},'Interpreter','Latex')

figure('Renderer','painters','Position',[10 10 900 600])
opt2=opt1;
opt2.Title.String='Bode Diagram of G_{11}(s) for 3 differents PPF designs';
bodeplot(G11,G11_11,G11_12,G11_13,opt2)
legend({'G11(s)','$g=-24.6\ \zeta=0.082\ \omega n=780\ rad/s$','$g=-80.6\ \zeta=0.15\ \omega n=824\ rad/s$','$g=-269.6\ \zeta=0.3\ \omega n=1055\ rad/s$'},'Interpreter','Latex')

figure('Renderer','painters','Position',[10 10 900 600])
opt3=opt1;
opt3.Title.String='Bode Diagram of G_{11}(s) using different piezoelectric patches';
bodeplot(G11,G11_11,G11_14,opt3)
legend({'G11(s)','$g=-24.6\ \zeta=0.082\ \omega n=780\ rad/s$ for piezo (2,2)','$g=-158.3\ \zeta=0.082\ \omega n=864\ rad/s$ for piezo (3,3)'},'Interpreter','Latex')

%% Bode Diagram Damping of the Second mode
figure('Renderer','painters','Position',[10 10 900 600])
opt1=bodeoptions;
opt1.Title.String='Bode Diagram of G_{22}(s) for 3 differents PPF designs';
opt1.Title.FontSize=14;
opt1.XLabel.FontSize=14; opt1.YLabel.FontSize=14;
opt1.TickLabel.FontSize=12;
opt1.Grid='on';
bodeplot(G22,G22_21,G22_22,G22_23,opt1)
legend({'G22(s)','$g=-10\ \zeta=0.082\ \omega n=1592\ rad/s$','$g=-29.5\ \zeta=0.15\ \omega n=1614\ rad/s$','$g=-81.738\ \zeta=0.3\ \omega n=1709\ rad/s$'},'Interpreter','Latex')

figure('Renderer','painters','Position',[10 10 900 600])
opt2=opt1;
opt2.Title.String='Bode Diagram of G_{11}(s) for 3 differents PPF designs';
bodeplot(G11,G11_21,G11_22,G11_23,opt2)
legend({'G11(s)','$g=-10\ \zeta=0.082\ \omega n=1592\ rad/s$','$g=-29.5\ \zeta=0.15\ \omega n=1614\ rad/s$','$g=-81.738\ \zeta=0.3\ \omega n=1709\ rad/s$'},'Interpreter','Latex')

%% Bode Diagram Damping of 3 modes with first order PPF
figure('Renderer','painters','Position',[10 10 900 600])
opt1=bodeoptions;
opt1.Title.String='Bode Diagram of G_{22}(s) for 3 differents PPF designs';
opt1.Title.FontSize=14;
opt1.XLabel.FontSize=14; opt1.YLabel.FontSize=14;
opt1.TickLabel.FontSize=12;
opt1.Grid='on';
bodeplot(G22,G22_31,G22_32,G22_33,opt1)
legend({'G22(s)','$g=-607,\ \omega=1500\ rad/s$','$g=-581,\ \omega=2000\ rad/s$','$g=-583,\ \omega=2500\ rad/s$'},'Interpreter','Latex')

figure('Renderer','painters','Position',[10 10 900 600])
opt2=opt1;
opt2.Title.String='Bode Diagram of G_{11}(s) for 3 differents PPF designs';
bodeplot(G11,G11_31,G11_32,G11_33,opt2)
legend({'G11(s)','$g=-607,\ \omega=1500\ rad/s$','$g=-581,\ \omega=2000\ rad/s$','$g=-583,\ \omega=2500\ rad/s$'},'Interpreter','Latex')

%% Root locus - Mode 1
figure('Renderer','painters','Position',[10 10 900 600])
opt1=pzoptions;
opt1.Title.String='Root locus of 1+gHG22 for the first design';
opt1.Title.FontSize=14;
opt1.XLabel.FontSize=14; opt1.YLabel.FontSize=14;
opt1.TickLabel.FontSize=12;
opt1.Xlim=[-100,50]; opt1.Ylim=[-2500,2500];
rlocusplot(series(-PPF1_1,G22),opt1);
hold on;
my_poles=rlocus(series(-PPF1_1,G22),1);
plot(real(my_poles),imag(my_poles),'ms','MarkerFaceColor','m','Markersize',4) 
legend({'$g=-24.6\ \zeta=0.082\ \omega n=780\ rad/s$'},'Interpreter','Latex')

figure('Renderer','painters','Position',[10 10 900 600])
opt2=opt1;
opt1.Title.String='Root locus of 1+gHG22 for the second design';
rlocusplot(series(-PPF1_2,G22),opt2);
hold on;
my_poles=rlocus(series(-PPF1_2,G22),1);
plot(real(my_poles),imag(my_poles),'ms','MarkerFaceColor','m','Markersize',4)
legend({'$g=-80.6\ \zeta=0.15\ \omega n=824\ rad/s$'},'Interpreter','Latex')

figure('Renderer','painters','Position',[10 10 900 600])
opt3=opt1;
opt3.Title.String='Root locus of 1+gHG22 for the third design';
opt3.Xlim=[-150,50];
rlocusplot(series(-PPF1_3,G22),opt3);
hold on;
my_poles=rlocus(series(-PPF1_3,G22),1);
plot(real(my_poles),imag(my_poles),'ms','MarkerFaceColor','m','Markersize',4)
hold on;

legend({'$g=-269.6\ \zeta=0.3\ \omega n=1055\ rad/s$'},'Interpreter','Latex')

figure('Renderer','painters','Position',[10 10 900 600])
opt4=opt1;
opt4.Title.String='Root locus for controllability analysis';
opt1.GridColor=[1 0 0];
rlocusplot(series(-PPF1_1,G22),opt1);
my_zeros=zero(G22);
hold on;
plot(real(my_zeros),imag(my_zeros),'ms','MarkerFaceColor','r','Markersize',7)
rlocusplot(series(-PPF1_1,G33),opt4);
my_zeros=zero(G33);
plot(real(my_zeros),imag(my_zeros),'ms','MarkerFaceColor','b','Markersize',7)
legend({'$g=-24.6\ \zeta=0.082\ \omega n=780\ rad/s$','$g=-158.3\ \zeta=0.082\ \omega n=864\ rad/s$'},'Interpreter','Latex')

%% Root locus - Mode 2
figure('Renderer','painters','Position',[10 10 900 600])
opt1=pzoptions;
opt1.Title.String='Root locus of 1+gHG22 for the first design';
opt1.Title.FontSize=14;
opt1.XLabel.FontSize=14; opt1.YLabel.FontSize=14;
opt1.TickLabel.FontSize=12;
opt1.Xlim=[-300,50];opt1.Ylim=[-4000,4000];

rlocusplot(series(-PPF2_1,G22),opt1);
hold on;
my_poles=rlocus(series(-PPF2_1,G22),1);
plot(real(my_poles),imag(my_poles),'ms','MarkerFaceColor','m','Markersize',4) 
legend({'$g=-10\ \zeta=0.082\ \omega n=1592\ rad/s$'},'Interpreter','Latex')

figure('Renderer','painters','Position',[10 10 900 600])
opt2=opt1;
opt1.Title.String='Root locus of 1+gHG22 for the second design';
rlocusplot(series(-PPF2_2,G22),opt2);
hold on;
my_poles=rlocus(series(-PPF2_2,G22),1);
plot(real(my_poles),imag(my_poles),'ms','MarkerFaceColor','m','Markersize',4)
legend({'$g=-29.5\ \zeta=0.15\ \omega n=1614\ rad/s$'},'Interpreter','Latex')

figure('Renderer','painters','Position',[10 10 900 600])
opt3=opt1;
opt3.Title.String='Root locus of 1+gHG22 for the third design';
rlocusplot(series(-PPF2_3,G22),opt3);
hold on;
my_poles=rlocus(series(-PPF2_3,G22),1);
plot(real(my_poles),imag(my_poles),'ms','MarkerFaceColor','m','Markersize',4)
legend({'$g=-81.7\ \zeta=0.3\ \omega n=1709\ rad/s$'},'Interpreter','Latex')

%% Root locus - First Order PPF
figure('Renderer','painters','Position',[10 10 900 600])
opt1=pzoptions;
opt1.Title.String='Root locus of 1+gHG22 for the first design';
opt1.Title.FontSize=14;
opt1.XLabel.FontSize=14; opt1.YLabel.FontSize=14;
opt1.TickLabel.FontSize=12;
opt1.Xlim=[-900,50];opt1.Ylim=[-3000,3000];

rlocusplot(series(-PPF3_1,G22),opt1);
hold on;
my_poles=rlocus(series(-PPF3_1,G22),1);
plot(real(my_poles),imag(my_poles),'ms','MarkerFaceColor','m','Markersize',4) 
legend({'$g=-607,\ \omega=1500\ rad/s$'},'Interpreter','Latex')

figure('Renderer','painters','Position',[10 10 900 600])
opt2=opt1;
opt1.Title.String='Root locus of 1+gHG22 for the second design';
rlocusplot(series(-PPF3_2,G22),opt2);
hold on;
my_poles=rlocus(series(-PPF3_2,G22),1);
plot(real(my_poles),imag(my_poles),'ms','MarkerFaceColor','m','Markersize',4)
legend({'$g=-581,\ \omega=2000\ rad/s$'},'Interpreter','Latex')

figure('Renderer','painters','Position',[10 10 900 600])
opt3=opt1;
opt3.Title.String='Root locus of 1+gHG22 for the third design';
rlocusplot(series(-PPF3_3,G22),opt3);
hold on;
my_poles=rlocus(series(-PPF3_3,G22),1);
plot(real(my_poles),imag(my_poles),'ms','MarkerFaceColor','m','Markersize',4)
legend({'$g=-583,\ \omega=2500\ rad/s$'},'Interpreter','Latex')