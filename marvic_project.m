clear all; close all; clc;
%%
load('system.mat');
A=system.A; B=system.B; C=system.C; D=system.D;
input_size=length(B(1,:));
H=struct;

for i=1:input_size
    for j=1:input_size
        field=strcat(int2str(i),int2str(j));
        [b,a]=ss2tf(A,B,C,D,j); %Get the transfer functions from input j (Hxj: size 5)
        H(i).u(j)=tf(b(i,:),a); %Get the transfer function from input j to output i
    end
end

%%
%============== Plot ================%

figure('Renderer','painters','Position',[10 10 900 600])
opt1=bodeoptions;
opt1.Title.String='Bode Diagram of H_{23}, H_{32}, H_{45} and H_{54}';
opt1.Title.FontSize=14;
opt1.XLabel.FontSize=14; opt1.YLabel.FontSize=14;
opt1.TickLabel.FontSize=12;
opt1.Grid='on';
bodeplot(H(2).u(3),H(3).u(2),H(4).u(5),H(5).u(4),opt1)
legend({'H_{23}', 'H_{32}', 'H_{45}', 'H_{54}'})

figure('Renderer','painters','Position',[10 10 900 600])
opt2=opt1;
opt2.Title.String='Bode Diagram of H_{11}, H_{22}';
bodeplot(H(1).u(1),H(2).u(2),opt2)
legend({'H_{11}','H_{22}'})

figure('Renderer','painters','Position',[10 10 900 600])
opt3=opt2;
opt3.Title.String='Bode Diagram of H_{12}, H_{13}, H_{14} and H_{15}';
bodeplot(H(1).u(2),H(1).u(3),H(1).u(4),H(1).u(5),opt3)
legend({'H_{12}', 'H_{13}','H_{14}', 'H_{15}'})