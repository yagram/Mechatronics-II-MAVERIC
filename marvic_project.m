clear all; close all; clc;
%% Part 1 - Plant Model
load('system.mat');
A=system.A; B=system.B; C=system.C; D=system.D;
input_size=length(B(1,:));
G=struct;
eVal=eig(A);
w=sort(abs(imag(eVal))); %eigenfrequencies of the system

%[eVal,eVecs]=eig(A);
%test=eVecs*A*inv(eVecs);
%isdiag(test)

for i=1:input_size
    for j=1:input_size
        field=strcat(int2str(i),int2str(j));
        [b,a]=ss2tf(A,B,C,D,j); %Get the transfer functions from input j (Hxj: size 5)
        G(i).u(j)=tf(b(i,:),a); %Get the transfer function from input j to output i
    end
end

%% Part 2 - Performance of various control laws

wf=w(1); %Since 762.7435 rad/s is the first EF, 750 seems to be 
zetaf=0.8;

H=tf(-10,[1,2*zetaf*wf,wf^2]);
Tbf22=feedback(H*G(2).u(2),1);
evalfr(Tbf22,0)
%%
%============== Plot Part 1 ================%

figure('Renderer','painters','Position',[10 10 900 600])
opt1=bodeoptions;
opt1.Title.String='Bode Diagram of H_{23}, H_{32}, H_{45} and H_{54}';
opt1.Title.FontSize=14;
opt1.XLabel.FontSize=14; opt1.YLabel.FontSize=14;
opt1.TickLabel.FontSize=12;
opt1.Grid='on';
bodeplot(G(2).u(3),G(3).u(2),G(4).u(5),G(5).u(4),opt1)
legend({'H_{23}', 'H_{32}', 'H_{45}', 'H_{54}'})

figure('Renderer','painters','Position',[10 10 900 600])
opt2=opt1;
opt2.Title.String='Bode Diagram of H_{11}, H_{22}';
bodeplot(G(1).u(1),G(2).u(2),opt2)
legend({'H_{11}','H_{22}'})

figure('Renderer','painters','Position',[10 10 900 600])
opt3=opt2;
opt3.Title.String='Bode Diagram of H_{12}, H_{13}, H_{14} and H_{15}';
bodeplot(G(1).u(2),G(1).u(3),G(1).u(4),G(1).u(5),opt3)
legend({'H_{12}', 'H_{13}','H_{14}', 'H_{15}'})

%% 
%============== Plot Part 2================%
figure('Renderer','painters','Position',[10 10 900 600])
opt4=bodeoptions;
opt4.Title.String='Bode Diagram of H_{22}(s) and Tbf_{22}(s)';
opt4.Title.FontSize=14;
opt4.XLabel.FontSize=14; opt4.YLabel.FontSize=14;
opt4.TickLabel.FontSize=12;
opt4.Grid='on';
bodeplot(G(2).u(2),Tbf22,opt4)
legend({'G22','Tbf22(s)'})