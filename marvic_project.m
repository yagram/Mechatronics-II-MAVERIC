clear all; close all; clc;

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

[b,a]=ss2tf(A,B,C,D,1);
test_tf=tf(b(1,:),a); %supposed to be H11

%============== Plot ================%
bode(H(1).u(1))