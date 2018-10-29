function [ p1,p2,p3,p4,p5,p6,p7,p8,p9,p10] = getPositions()
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
syms ddphi_body dphi_body phi_body ddtheta_body dtheta_body theta_body ddpsi_body dpsi_body psi_body ddtheta1 dtheta1 theta1 ddtheta2 dtheta2 theta2 ddtheta3 dtheta3 theta3 ddtheta4 dtheta4 theta4 T
syms L1 L2 L3 L4 L5 L6 L7 L8 L9 L10

angle=[phi_body,theta_body,psi_body];
R_body=R(angle);
Ry1=[cos(theta1),0,sin(theta1);0,1,0;-sin(theta1),0,cos(theta1)];
Ry2=[cos(theta2),0,sin(theta2);0,1,0;-sin(theta2),0,cos(theta2)];
Ry3=[cos(theta3),0,sin(theta3);0,1,0;-sin(theta3),0,cos(theta3)];
Ry4=[cos(theta4),0,sin(theta4);0,1,0;-sin(theta4),0,cos(theta4)];

p1=R_body*[L1;0;0];          %   Front Body
p2=p1+R_body*Ry1*[L2;0;0];          %Right Knee
p3=p2+R_body*Ry1*Ry2*[L3;0;0];          %Right Foot

p4=R_body*[L4;0;0];          %   Back Body
p5=p4+R_body*Ry1*[L5;0;0];          %Right Calf
p6=p5+R_body*Ry1*Ry2*[L6;0;0];          %Right Heel

p7=p1+R_body*Ry3*[L7;0;0];          %Left Knee
p8=p7+R_body*Ry3*Ry4*[L8;0;0];          %Left Foot

p9=p4+R_body*Ry3*[L9;0;0];          %Left Calf
p10=p9+R_body*Ry3*Ry4*[L10;0;0];          %Left Calf

end

