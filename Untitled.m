clc; 
clear all


a = [0,0,0,0,0]; %theta 1-560
%theta input
t1=deg2rad(a(1));
t2=deg2rad(a(2));
t3=deg2rad(a(3));
t4=deg2rad(a(4)+90);
t5=deg2rad(a(5));
%%%%%%deg2rad

t=[t1,t2,t3,t4,t5];
A1=[cos(t1),-sin(t1),0,9.9*cos(t1);
sin(t1),cos(t1),0,9.9*sin(t1);
0,0,1,13.4;
0,0,0,1];

A2=[cos(t2),0,-sin(t2),9.7*cos(t2);
sin(t2),0,cos(-t2),9.7*sin(t2);
0,1,0,0;
0,0,0,1];

A3=[cos(t3),-sin(t3),0,6.2*cos(t3);
sin(t3),cos(t3),0,6.2*sin(t3);
0,0,1,0;
0,0,0,1];

A4=[cos(t4),-sin(t4),0,0,;
sin(t4),cos(t4),0,3.5;
0,0,1,3.5;
0,0,0,1];

A5=[cos(t5),-sin(t5),0,0;
sin(t5),cos(t5),0,0;
0,0,1,4.9;
0,0,0,1];

Q=A1*A2*A3*A4*A5;


teta3=asin((Q(3,4)-80*Q(3,3)-134)/62)+pi/2;%3
teta4=asin(Q(3,3))-teta3;%%4

teta5=-atan(Q(3,2)/Q(3,1));%%5
teta12=asin(Q(1,1)*sin(teta5)+Q(1,2)*cos(teta5));
teta1=asin((31*sin(teta12+teta3)+40*sin(teta12-teta3-teta4)+31*sin(teta12-teta3)+40*sin(teta12+teta3+teta4)-35*cos(teta12)+97*sin(teta12)-Q(2,4))/99);
teta2=teta12-teta1;
teta=[ 180*teta1/pi+90,180*teta2/pi,180*teta3/pi,180*teta4/pi,180*teta5/pi];
teta=real(teta);
teta=vpa(teta);
disp(teta);
% x=conv(teta);
% x=real(x);
% x=vpa(x);
% disp(x);