% General initialization sequence
% connect Robix via USB port before launching initialization
% uncomment 'initialize_cam()' when also using the camera

%clear all; 
%global  a s1 s2 s3 s4 s5 s6 z cam ;
%initialize_cam();
%z=init_angles([-110; 90; -99; 85; -91;85; -93; 100; -113; 90]); % by default min and min angular joint values on J1 to J5, to be updated
%[s1,s2,s3,s4,s5,s6]=robix_connect(a);




% add your function calls from 'Robix_toolbox' below
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
theta1=-45;
theta2=0;
theta3=0;
theta4=0;%theta4=90+theta4;
theta5=0;
%move_1_by(theta1);
%move_2_by(theta2);
%move_3_by(theta3);
%move_4_by(theta4);
%move_5_by(theta5);
%%%%%%%%%%%%  Calculation    %%%%%%%%%%%%%

    A1=[[cosd(theta1),-sind(theta1)*cosd(alpha1),sind(theta1)*sind(alpha1),l1*cosd(theta1)];
        [sind(theta1),cosd(theta1)*cosd(alpha1),-cosd(theta1)*sind(alpha1),l1*sind(theta1)];
        [0,sind(alpha1),cosd(alpha1),d1];
        [0,0,0,1]];

A2=[[cosd(theta2),-sind(theta2)*cosd(alpha2),sind(theta2)*sind(alpha2),l2*cosd(theta2)];
    [sind(theta2),cosd(theta2)*cosd(alpha2),-cosd(theta2)*sind(alpha2),l2*sind(theta2)];
    [0,sind(alpha2),cosd(alpha2),d2];
    [0,0,0,1]];
A3=[[cosd(theta3),-sind(theta3)*cosd(alpha3),sind(theta3)*sind(alpha3),l3*cosd(theta3)];
    [sind(theta3),cosd(theta3)*cosd(alpha3),-cosd(theta3)*sind(alpha3),l3*sind(theta3)];
    [0,sind(alpha3),cosd(alpha3),d3];
    [0,0,0,1]];
A4=[[cosd(theta4+80),-sind(theta4+80)*cosd(alpha4),sind(theta4+80)*sind(alpha4),l4*cosd(theta4+80)];
    [sind(theta4+80),cosd(theta4+80)*cosd(alpha4),-cosd(theta4+80)*sind(alpha4),l4*sind(theta4+80)];
    [0,sind(alpha4),cosd(alpha4),d4];
    [0,0,0,1]];
disp(A4)
A5=[[cosd(theta5),-sind(theta5)*cosd(alpha5),sind(theta5)*sind(alpha5),l5*cosd(theta5)];
    [sind(theta5),cosd(theta5)*cosd(alpha5),-cosd(theta5)*sind(alpha5),l5*sind(theta5)];
    [0,sind(alpha5),cosd(alpha5),d5];
    [0,0,0,1]];
A=A1*A2*A3*A4*A5

%%%%%%%%%%%%%%  elements of forward kinematic matrice  %%%%%%%%%
xT=A(1,4);
yT=A(2,4);
zT=A(3,4);

Xx=A(1,1);
Xy=A(2,1);
Xz=A(3,1);
Yz=A(3,2);
Zz=A(3,3);

thetad=rad2deg(atan2(Xy,Xx))
feid=rad2deg(atan2(-Xz,Xx*cosd(thetad)+Xy*sind(thetad)))
pseid=rad2deg(atan2(Yz,Zz))

% t5d=rad2deg(atan2(-1*A(3,2),A(3,1)))
% t1p2d=rad2deg(atan2(A(2,3),A(1,3)))
% 
% cost3pt4=(A(1,1)-sind(t1p2d)*sind(t5d))/(cosd(t1p2d)*cosd(t5d));
% sint3pt4=A(1,3)/cosd(t1p2d);
% t3p4d=rad2deg(atan2(sint3pt4,cost3pt4))-80
%rott4=t4d-80
sint3=(A(3,4)-17+17/2*(-A(3,3)))/4
t3=rad2deg(asin(sint3))
if -A(3,3)>0
t4=rad2deg(acos(-A(3,3)))-t3
t4=t4-80
elseif -A(3,3)<0
        t4=-rad2deg(acos(-A(3,3)))-t3
t4=t4-80
end





