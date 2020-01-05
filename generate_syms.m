clc
clear all

syms theta1 theta2 theta3 theta4 theta5

%%%%%%%%%%  Joint_1  %%%%%%%%%%%%%%
l1=9.5;
%d1=16.8;
d1=14.5;
alpha1=0;

%%%%%%%%%%  Joint_2  %%%%%%%%%%%%%%%
l2=9.6;
%d2=1.5;
d2=2.5;
alpha2=90;

%%%%%%%%%%  Joint_3  %%%%%%%%%%%%%%%
l3=4.0;  
%d3=-3.2;
d3=0;
alpha3=0;

%%%%%%%%%%  Joint_4  %%%%%%%%%%%%%%%
l4=0;
d4=-3;
%d4=0;
alpha4=90;

%%%%%%%%%%  Joint_5  %%%%%%%%%%%%%%%
l5=0;
%d5=8;
d5=8.5;
alpha5=0;



 A1=[[cos(theta1),-sin(theta1)*cosd(alpha1),sin(theta1)*sind(alpha1),l1*cos(theta1)];
        [sin(theta1),cos(theta1)*cosd(alpha1),-cos(theta1)*sind(alpha1),l1*sin(theta1)];
        [0,sind(alpha1),cosd(alpha1),d1];
        [0,0,0,1]];

A2=[[cos(theta2),-sin(theta2)*cosd(alpha2),sin(theta2)*sind(alpha2),l2*cos(theta2)];
    [sin(theta2),cos(theta2)*cosd(alpha2),-cos(theta2)*sind(alpha2),l2*sin(theta2)];
    [0,sind(alpha2),cosd(alpha2),d2];
    [0,0,0,1]];
A3=[[cos(theta3),-sin(theta3)*cosd(alpha3),sin(theta3)*sind(alpha3),l3*cos(theta3)];
    [sin(theta3),cos(theta3)*cosd(alpha3),-cos(theta3)*sind(alpha3),l3*sin(theta3)];
    [0,sind(alpha3),cosd(alpha3),d3];
    [0,0,0,1]];
A4=[[cos(theta4),-sin(theta4)*cosd(alpha4),sin(theta4)*sind(alpha4),l4*cos(theta4)];
    [sin(theta4),cos(theta4)*cosd(alpha4),-cos(theta4)*sind(alpha4),l4*sin(theta4)];
    [0,sind(alpha4),cosd(alpha4),d4];
    [0,0,0,1]];
disp(A4)
A5=[[cos(theta5),-sin(theta5)*cosd(alpha5),sin(theta5)*sind(alpha5),l5*cos(theta5)];
    [sin(theta5),cos(theta5)*cosd(alpha5),-cos(theta5)*sind(alpha5),l5*sin(theta5)];
    [0,sind(alpha5),cosd(alpha5),d5];
    [0,0,0,1]];
A=A1*A2*A3*A4*A5

simplify(A)

A11=sin(theta1+theta2)*sin(theta5)+cos(theta1+theta2)*cos(theta3+theta4)*cos(theta5);
A12=sin(theta1+theta2)*cos(theta5)-cos(theta1+theta2)*cos(theta3+theta4)*sin(theta5);
A21=-cos(theta1+theta2)*sin(theta5)+cos(theta3+theta4)*sin(theta1+theta2)*cos(theta5);
A22=-cos(theta1+theta2)*cos(theta5)-sin(theta1+theta2)*cos(theta3+theta4)*sin(theta5);

%A1112=simplify(A11^2+A12^2);
%A1121=simplify(A11^2+A21^2);

syms k C1  C2
Del=(1-k^2)^2*(C1+C2)^2-4*(1+k^2*C1*C2)*(k^2+C1*C2)
simplify(Del)



