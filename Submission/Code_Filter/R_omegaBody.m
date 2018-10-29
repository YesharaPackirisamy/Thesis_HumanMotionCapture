function [ totalOmega ] = R_omegaBody( angle, omega )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
% This takes angles in the order roll, pitch, yaw (phi, theta, psi) and
% rotates them using Euler 3-1-2
phi_body = angle(1);
theta_body = angle(2);
psi_body = angle(3);

dphi_body = omega(1);
dtheta_body = omega(2);
dpsi_body = omega(3);

% R(phi)
Rphi = [1,0,0; 0,cos(phi_body), sin(phi_body); 0, -sin(phi_body), cos(phi_body)];
% R(th)    
Rtheta = [cos(theta_body),0,-sin(theta_body); 0,1, 0; sin(theta_body), 0, cos(theta_body)];
% R(psi)    
Rpsi = [cos(psi_body),sin(psi_body),0; -sin(psi_body),cos(psi_body), 0; 0, 0, 1];

% %Euler 3-1-2 in Body fixed
totalOmega =  Rtheta*Rphi*[0;0;dpsi_body] + Rtheta*[dphi_body; 0; 0] +[0; dtheta_body; 0];

%Euler 3-2-1 in Body fixed
%totalOmega =  Rphi*Rtheta*[0;0;wPsi] + Rphi*[0; wTh; 0] +[wPhi; 0; 0];

totalOmega = simplify(totalOmega);
end