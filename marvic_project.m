clear all; close all; clc;
%% Part 1 - Plant Model
load('system.mat');
G11=system(1,1);
G22=system(2,2);
G33=system(3,3);
[w,zeta,poles]=damp(G22);

%% Part 2 - Performance of various control laws - Damping of the First mode
%sisotool(G22)
PPF22=load('PPF22_new.mat');
PPF22=PPF22.PPF22_new;
sys_new=feedback(system,PPF22,2,2,+1);
sys_new33=feedback(system,PPF22,3,3,+1); %Jtest en prenant une autre paire de piézo
G11_new=sys_new(1,1);
G22_new=sys_new(2,2);
G33_new=sys_new33(3,3);
isstable(G11_new)
%% Damping of the Second mode
%sisotool(G22)
PPF22_2=load('PPF22_2.mat');
PPF22_2=PPF22_2.PPF22_2;
sys_new=feedback(sys_new,PPF22_2,2,2,+1); %On rajoute une rétroaction
G11_mode2=sys_new(1,1);
G22_mode2=sys_new(2,2);
isstable(G11_mode2)
%% Damping of the Third mode
%sisotool(G22)
%sisotool(G11)
PPF11_3=load('PPF11_3.mat');
PPF11_3=PPF11_3.PPF11_3;
sys_new=feedback(sys_new,PPF11_3,1,1,+1); % Et encore une mais cette fois ci entre 1 et 1
G11_mode3=sys_new(1,1);
G22_mode3=sys_new(2,2);
isstable(G11_mode3)

%% First Order PPF
%sisotool(G22)
PPF_order1=load('PPF_order1.mat');
PPF_order1=PPF_order1.PPF_order1;
sys_new=feedback(system,PPF_order1,1,1,+1);
G11_order1=sys_new(1,1);
G22_order1=sys_new(2,2);
isstable(G11_order1);
%% Bode Diagram Damping of the first mode
figure('Renderer','painters','Position',[10 10 900 600])
opt1=bodeoptions;
opt1.Title.String='Bode Diagram of G_{22}(s) and G_{22,new}(s)';
opt1.Title.FontSize=14;
opt1.XLabel.FontSize=14; opt1.YLabel.FontSize=14;
opt1.TickLabel.FontSize=12;
opt1.Grid='on';
bodeplot(G22,G22_new,opt1)
legend({'G22(s)','G22,new(s)'})

figure('Renderer','painters','Position',[10 10 900 600])
opt2=opt1;
opt2.Title.String='Bode Diagram of G_{11}(s) and G_{11,new}(s)';
bodeplot(G11,G11_new,opt2)
legend({'G11(s)','G11,new(s)'})

figure('Renderer','painters','Position',[10 10 900 600])
opt3=opt1;
opt3.Title.String='Bode Diagram of G_{33}(s) and G_{33,new}(s)';
bodeplot(G33,G33_new,opt3)
legend({'G33(s)','G33,new(s)'})

%% Bode Diagram Damping of the Second mode
figure('Renderer','painters','Position',[10 10 900 600])
opt1=bodeoptions;
opt1.Title.String='Bode Diagram of G_{22}(s) and G_{22,new}(s)';
opt1.Title.FontSize=14;
opt1.XLabel.FontSize=14; opt1.YLabel.FontSize=14;
opt1.TickLabel.FontSize=12;
opt1.Grid='on';
bodeplot(G22,G22_mode2,opt1)
legend({'G22(s)','G22,new(s)'})

figure('Renderer','painters','Position',[10 10 900 600])
opt2=opt1;
opt2.Title.String='Bode Diagram of G_{11}(s) and G_{11,new}(s)';
bodeplot(G11,G11_mode2,opt2)
legend({'G11(s)','G11,new(s)'})

%% Bode Diagram Damping of the Third mode
figure('Renderer','painters','Position',[10 10 900 600])
opt1=bodeoptions;
opt1.Title.String='Bode Diagram of G_{22}(s) and G_{22,new}(s)';
opt1.Title.FontSize=14;
opt1.XLabel.FontSize=14; opt1.YLabel.FontSize=14;
opt1.TickLabel.FontSize=12;
opt1.Grid='on';
bodeplot(G22,G22_mode3,opt1)
legend({'G22(s)','G22,new(s)'})

figure('Renderer','painters','Position',[10 10 900 600])
opt2=opt1;
opt2.Title.String='Bode Diagram of G_{11}(s) and G_{11,new}(s)';
bodeplot(G11,G11_mode3,opt2)
legend({'G11(s)','G11,new(s)'})

%% Bode Diagram Damping with first order PPF
figure('Renderer','painters','Position',[10 10 900 600])
opt1=bodeoptions;
opt1.Title.String='Bode Diagram of G_{22}(s) and G_{22,new}(s)';
opt1.Title.FontSize=14;
opt1.XLabel.FontSize=14; opt1.YLabel.FontSize=14;
opt1.TickLabel.FontSize=12;
opt1.Grid='on';
bodeplot(G22,G22_order1,opt1)
legend({'G22(s)','G22,new(s)'})

figure('Renderer','painters','Position',[10 10 900 600])
opt2=opt1;
opt2.Title.String='Bode Diagram of G_{11}(s) and G_{11,new}(s)';
bodeplot(G11,G11_order1,opt2)
legend({'G11(s)','G11,new(s)'})