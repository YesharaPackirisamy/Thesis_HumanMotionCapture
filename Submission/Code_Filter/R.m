function [ R_matrix ] = R( angle )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% This takes angles in the order roll, pitch, yaw (phi, theta, psi) and
% rotates them using Euler 3-1-2
phi_body = angle(1);
theta_body = angle(2);
psi_body = angle(3);

% R(phi)
Rphi = [1,0,0; 0,cos(phi_body), sin(phi_body); 0, -sin(phi_body), cos(phi_body)];
% R(th)    
Rtheta = [cos(theta_body),0,-sin(theta_body); 0,1, 0; sin(theta_body), 0, cos(theta_body)];
% R(psi)    
Rpsi = [cos(psi_body),sin(psi_body),0; -sin(psi_body),cos(psi_body), 0; 0, 0, 1];

% %Euler 3-1-2 
R_matrix= Rtheta*Rphi*Rpsi;
%Euler 3-2-1 
%R_matrix= Rphi*Rtheta*Rpsi;
%Euler 2-1-3
%R_matrix= Rpsi*Rphi*Rtheta;

end