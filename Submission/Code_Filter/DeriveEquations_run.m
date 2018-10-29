% DERIVE EQUATIONS FOR HUMAN LIMBS
% This script will generate the matrices for the prediction and update
% phases for the human lower limbs
% 
% 

%% admin
clear all
close all
clc

load('CamMatrix.mat');
n = 1;

%% define the symbolic variables:

%syms theta phi psi dtheta dphi dpsi T
syms ddphi_body dphi_body phi_body ddtheta_body dtheta_body theta_body ddpsi_body dpsi_body psi_body ddtheta1 dtheta1 theta1 ddtheta2 dtheta2 theta2 ddtheta3 dtheta3 theta3 ddtheta4 dtheta4 theta4 T

% Lengths between markers
syms L1 L2 L3 L4 L5 L6 L7 L8 L9 L10 p1 p2 p3 p4 p5 p6 p7 p8 p9 p10

syms x y z dx dy dz

%% Pack State Vector

% State vector - dX and then X
P = [phi_body;theta_body;psi_body;theta1;theta2;theta3;theta4];
V = [dphi_body;dtheta_body;dpsi_body;dtheta1;dtheta2;dtheta3;dtheta4];
A= [ddphi_body;ddtheta_body;ddpsi_body;ddtheta1;ddtheta2;ddtheta3;ddtheta4];

X = [A;V;P];

%% Rotation Matrices  - These take a vector in inertial to the corresponding link frame
%RI_S = [cos(thS),0,sin(thS);0,1,0;-sin(thS),0,cos(thS)];
%RI_S = [cos(thS),-sin(thS);sin(thS),cos(thS)];
% R_body=[0,0,1;0,1,0;-1,0,0];
angle=[phi_body,theta_body,psi_body];
omega=[dphi_body,dtheta_body,dpsi_body];

R_body=R(angle);
Ry1=[cos(theta1),0,sin(theta1);0,1,0;-sin(theta1),0,cos(theta1)];
Ry2=[cos(theta2),0,sin(theta2);0,1,0;-sin(theta2),0,cos(theta2)];
Ry3=[cos(theta3),0,sin(theta3);0,1,0;-sin(theta3),0,cos(theta3)];
Ry4=[cos(theta4),0,sin(theta4);0,1,0;-sin(theta4),0,cos(theta4)];

%% Inverse Kinematics

% POSITIONS
[ p1,p2,p3,p4,p5,p6,p7,p8,p9,p10]=getPositions();

% Velocities
% Link 1 
dp1 = jacobian(p1,P)*V; % The origin is attached to the phone

% Acceleration
%% Prediction
predict_eq_angAcc=A;
predict_eq_angVel=V+T*A;
predict_eq_angPos=P+T*V; %0.5*T^2*A;

predict_eq = [predict_eq_angAcc;predict_eq_angVel; predict_eq_angPos];
predict_eq = simplify(predict_eq);

matlabFunction(predict_eq,'file','state_predict_function');

%F matrix
Fmatrix_eq=jacobian(predict_eq,X);
matlabFunction(Fmatrix_eq,'file','Fmatrix_function');

%Camera parameters
camMatrix_FL = cameraMatrix(stereoParams_Front.CameraParameters1,eye(3),zeros(3,1));
c_FL=[stereoParams_Front.CameraParameters1.IntrinsicMatrix(3,1);stereoParams_Front.CameraParameters1.IntrinsicMatrix(3,2)];     %principal point (specific to camera)
camMatrix_FR = cameraMatrix(stereoParams_Front.CameraParameters2,eye(3),zeros(3,1));
c_FR=[stereoParams_Front.CameraParameters2.IntrinsicMatrix(3,1);stereoParams_Front.CameraParameters2.IntrinsicMatrix(3,2)];             %principal point (specific to camera)
camMatrix_BL = cameraMatrix(stereoParams_Back.CameraParameters1,eye(3),zeros(3,1));
c_BL=[stereoParams_Back.CameraParameters1.IntrinsicMatrix(3,1);stereoParams_Back.CameraParameters1.IntrinsicMatrix(3,2)];     %principal point (specific to camera)
camMatrix_BR = cameraMatrix(stereoParams_Back.CameraParameters2,eye(3),zeros(3,1));
c_BR=[stereoParams_Back.CameraParameters2.IntrinsicMatrix(3,1);stereoParams_Back.CameraParameters2.IntrinsicMatrix(3,2)];             %principal point (specific to camera)

T_B2C_Front = [0; 0; 0.07]; 
T_B2C_Back = [0; 0; -0.07]; 

alpha1 =16;  
alpha2=1;
RB2C_Front = [cosd(alpha1),0,-sind(alpha1); 0,1, 0; sind(alpha1), 0, cosd(alpha1)]; 
RB2C_Back = [cosd(alpha2),0,-sind(alpha2); 0,1, 0; sind(alpha2), 0, cosd(alpha2)];
%% Measurements
R_body_mag=[cos(theta_body-1.57),0,sin(theta_body-1.57);0,1,0;-sin(theta_body-1.57),0,cos(theta_body-1.57)];
R_phone=[cos(0),0,sin(0);0,1,0;-sin(0),0,cos(0)];

Mag=(R_body.')*[0.33;0;22.6];
Gyro= R_omegaBody( angle,omega );
h0_eqn=[Mag; Gyro];
h0_eqn = simplify(h0_eqn);
matlabFunction(h0_eqn,'file','h0_eqn');

% Jacobian
H0_matrix_eq=jacobian(h0_eqn,X);
matlabFunction(H0_matrix_eq,'file','H0_matrix');

%Right Knee (Pt_5)
%Cam 1: FL:          
mark1Pos = (transpose(R_body)*p2+T_B2C_Front)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
mark1Pos = RB2C_Front*mark1Pos;
mark1Pos = [mark1Pos(2);mark1Pos(3);mark1Pos(1);1];    
 mark1Pos=mark1Pos.';
 h1_eqn=(mark1Pos)*camMatrix_FL;
 w=h1_eqn(3);                       %compute scaling factor
 h1_eqn=[h1_eqn(1)/w;h1_eqn(2)/w];  %get x y points    (pixel coordinates)
 h1_eqn = simplify(h1_eqn); 
 h1_eqn=h1_eqn-c_FL;
 matlabFunction(h1_eqn,'file','h1_eqn');
 
%%Jacobian
 H1_matrix_eq=jacobian(h1_eqn,X);
 H1_matrix_eq = simplify(H1_matrix_eq);
 matlabFunction(H1_matrix_eq,'file','H1_matrix'); 
 
% % % %Right Knee (Pt_5)
% % % %Cam 2: FR  
mark1Pos = (transpose(R_body)*p2+T_B2C_Front)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position 
mark1Pos = RB2C_Front*mark1Pos; 
 mark1Pos = [mark1Pos(2);mark1Pos(3);mark1Pos(1);1] ;
 mark1Pos=mark1Pos.';
 h2_eqn=(mark1Pos)*camMatrix_FR;
 w=h2_eqn(3);                       %compute scaling factor
 h2_eqn=[h2_eqn(1)/w;h2_eqn(2)/w];  %get x y points    (pixel coordinates)
 h2_eqn = simplify(h2_eqn);
 h2_eqn=h2_eqn-c_FR;
 matlabFunction(h2_eqn,'file','h2_eqn');
%%Jacobian
 H2_matrix_eq=jacobian(h2_eqn,X);
 H2_matrix_eq = simplify(H2_matrix_eq);
 matlabFunction(H2_matrix_eq,'file','H2_matrix');
%  
 %Right Foot (Pt_9)
 %Cam 1: FL:          
mark2Pos = (transpose(R_body)*p3+T_B2C_Front)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
mark2Pos = RB2C_Front*mark2Pos; 
 mark2Pos = [mark2Pos(2);mark2Pos(3);mark2Pos(1);1] ;
 mark2Pos=mark2Pos.';
 h3_eqn=(mark2Pos)*camMatrix_FL;
 w=h3_eqn(3);                       %compute scaling factor
 h3_eqn=[h3_eqn(1)/w;h3_eqn(2)/w];  %get x y points    (pixel coordinates)
 h3_eqn = simplify(h3_eqn);
 h3_eqn=h3_eqn-c_FL;
 matlabFunction(h3_eqn,'file','h3_eqn');
%%Jacobian
 H3_matrix_eq=jacobian(h3_eqn,X);
 H3_matrix_eq = simplify(H3_matrix_eq);
 matlabFunction(H3_matrix_eq,'file','H3_matrix');
% 
 %Right Foot (Pt_9)
 %Cam 2: FR:          
mark2Pos = (transpose(R_body)*p3+T_B2C_Front)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
mark2Pos = RB2C_Front*mark2Pos; 
 mark2Pos = [mark2Pos(2);mark2Pos(3);mark2Pos(1);1] ;
 mark2Pos=mark2Pos.';
 h4_eqn=(mark2Pos)*camMatrix_FR;
 w=h4_eqn(3);                       %compute scaling factor
 h4_eqn=[h4_eqn(1)/w;h4_eqn(2)/w];  %get x y points    (pixel coordinates)
 h4_eqn = simplify(h4_eqn);
 h4_eqn=h4_eqn-c_FR;
 matlabFunction(h4_eqn,'file','h4_eqn');
%%Jacobian
 H4_matrix_eq=jacobian(h4_eqn,X);
 H4_matrix_eq = simplify(H4_matrix_eq);
 matlabFunction(H4_matrix_eq,'file','H4_matrix');
%  

% %Right Calf (Pt_7)
% %Cam 3: BL:          
% mark3Pos = (transpose(R_body)*p5+T_B2C_Back)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
% mark3Pos = RB2C_Back*mark3Pos;
% %Rz=[cos(pi),-sin(pi),0;sin(pi),cos(pi),0;0,0,1];
% %mark1Pos = Rz*mark1Pos;
% mark3Pos = [-mark3Pos(2);-mark3Pos(3);mark3Pos(1);1];    
%  mark3Pos=mark3Pos.';
%  h5_eqn=(mark3Pos)*camMatrix_BL;
%  w=h5_eqn(3);                       %compute scaling factor
%  h5_eqn=[h5_eqn(1)/w;h5_eqn(2)/w];  %get x y points    (pixel coordinates)
%  h5_eqn = simplify(h5_eqn); 
%  h5_eqn=h5_eqn-c_BL;
%  matlabFunction(h5_eqn,'file','h5_eqn');
%  
% %%Jacobian
%  H5_matrix_eq=jacobian(h5_eqn,X);
%  H5_matrix_eq = simplify(H5_matrix_eq);
%  matlabFunction(H5_matrix_eq,'file','H5_matrix'); 
%  
%  %Right Calf (Pt_7)
% %Cam 4: BR:          
% mark3Pos = (transpose(R_body)*p5+T_B2C_Back)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
% mark3Pos = RB2C_Back*mark3Pos;
% %Rz=[cos(pi),-sin(pi),0;sin(pi),cos(pi),0;0,0,1];
% %mark1Pos = Rz*mark1Pos;
% mark3Pos = [-mark3Pos(2);-mark3Pos(3);mark3Pos(1);1];    
%  mark3Pos=mark3Pos.';
%  h6_eqn=(mark3Pos)*camMatrix_BR;
%  w=h6_eqn(3);                       %compute scaling factor
%  h6_eqn=[h6_eqn(1)/w;h6_eqn(2)/w];  %get x y points    (pixel coordinates)
%  h6_eqn = simplify(h6_eqn); 
%  h6_eqn=h6_eqn-c_BR;
%  matlabFunction(h6_eqn,'file','h6_eqn');
%  
% %%Jacobian
%  H6_matrix_eq=jacobian(h6_eqn,X);
%  H6_matrix_eq = simplify(H6_matrix_eq);
%  matlabFunction(H6_matrix_eq,'file','H6_matrix'); 
 
 %Right Heel (Pt_11)
%Cam 3: BL:          
mark4Pos = (transpose(R_body)*p6+T_B2C_Back)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
mark4Pos = RB2C_Back*mark4Pos;
%Rz=[cos(pi),-sin(pi),0;sin(pi),cos(pi),0;0,0,1];
%mark1Pos = Rz*mark1Pos;
mark4Pos = [-mark4Pos(2);-mark4Pos(3);mark4Pos(1);1];    
 mark4Pos=mark4Pos.';
 h7_eqn=(mark4Pos)*camMatrix_BL;
 w=h7_eqn(3);                       %compute scaling factor
 h7_eqn=[h7_eqn(1)/w;h7_eqn(2)/w];  %get x y points    (pixel coordinates)
 h7_eqn = simplify(h7_eqn); 
 h7_eqn=h7_eqn-c_BL;
 matlabFunction(h7_eqn,'file','h7_eqn');
 
%%Jacobian
 H7_matrix_eq=jacobian(h7_eqn,X);
 H7_matrix_eq = simplify(H7_matrix_eq);
 matlabFunction(H7_matrix_eq,'file','H7_matrix'); 
 
 
 %Right Heel (Pt_11)
%Cam 4: BR:          
mark4Pos = (transpose(R_body)*p6+T_B2C_Back)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
mark4Pos = RB2C_Back*mark4Pos;
%Rz=[cos(pi),-sin(pi),0;sin(pi),cos(pi),0;0,0,1];
%mark1Pos = Rz*mark1Pos;
mark4Pos = [-mark4Pos(2);-mark4Pos(3);mark4Pos(1);1];    
 mark4Pos=mark4Pos.';
 h8_eqn=(mark4Pos)*camMatrix_BR;
 w=h8_eqn(3);                       %compute scaling factor
 h8_eqn=[h8_eqn(1)/w;h8_eqn(2)/w];  %get x y points    (pixel coordinates)
 h8_eqn = simplify(h8_eqn); 
 h8_eqn=h8_eqn-c_BL;
 matlabFunction(h8_eqn,'file','h8_eqn');
 
%%Jacobian
 H8_matrix_eq=jacobian(h8_eqn,X);
 H8_matrix_eq = simplify(H8_matrix_eq);
 matlabFunction(H8_matrix_eq,'file','H8_matrix'); 
 
 %Left Knee (Pt_6)
%Cam 1: FL:          
mark1Pos = (transpose(R_body)*p7+T_B2C_Front)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
mark1Pos = RB2C_Front*mark1Pos;
%Rz=[cos(pi),-sin(pi),0;sin(pi),cos(pi),0;0,0,1];
%mark1Pos = Rz*mark1Pos;
mark1Pos = [mark1Pos(2);mark1Pos(3);mark1Pos(1);1];    
 mark1Pos=mark1Pos.';
 h9_eqn=(mark1Pos)*camMatrix_FL;
 w=h9_eqn(3);                       %compute scaling factor
 h9_eqn=[h9_eqn(1)/w;h9_eqn(2)/w];  %get x y points    (pixel coordinates)
 h9_eqn = simplify(h9_eqn); 
 h9_eqn=h9_eqn-c_FL;
 matlabFunction(h9_eqn,'file','h9_eqn');
 
%%Jacobian
 H9_matrix_eq=jacobian(h9_eqn,X);
 H9_matrix_eq = simplify(H9_matrix_eq);
 matlabFunction(H9_matrix_eq,'file','H9_matrix'); 
 
% % % %Left Knee (Pt_6)
% % % %Cam 2: FR  
mark1Pos = (transpose(R_body)*p7+T_B2C_Front)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position 
mark1Pos = RB2C_Front*mark1Pos; 
 mark1Pos = [mark1Pos(2);mark1Pos(3);mark1Pos(1);1] ;
 mark1Pos=mark1Pos.';
 h10_eqn=(mark1Pos)*camMatrix_FR;
 w=h10_eqn(3);                       %compute scaling factor
 h10_eqn=[h10_eqn(1)/w;h10_eqn(2)/w];  %get x y points    (pixel coordinates)
 h10_eqn = simplify(h10_eqn);
 h10_eqn=h10_eqn-c_FR;
 matlabFunction(h10_eqn,'file','h10_eqn');
%%Jacobian
 H10_matrix_eq=jacobian(h10_eqn,X);
 H10_matrix_eq = simplify(H10_matrix_eq);
 matlabFunction(H10_matrix_eq,'file','H10_matrix');
%  
 %Left Foot (Pt_10)
 %Cam 1: FL:          
mark2Pos = (transpose(R_body)*p8+T_B2C_Front)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
mark2Pos = RB2C_Front*mark2Pos; 
 mark2Pos = [mark2Pos(2);mark2Pos(3);mark2Pos(1);1] ;
 mark2Pos=mark2Pos.';
 h11_eqn=(mark2Pos)*camMatrix_FL;
 w=h11_eqn(3);                       %compute scaling factor
 h11_eqn=[h11_eqn(1)/w;h11_eqn(2)/w];  %get x y points    (pixel coordinates)
 h11_eqn = simplify(h11_eqn);
 h11_eqn=h11_eqn-c_FL;
 matlabFunction(h11_eqn,'file','h11_eqn');
%%Jacobian
 H11_matrix_eq=jacobian(h11_eqn,X);
 H11_matrix_eq = simplify(H11_matrix_eq);
 matlabFunction(H11_matrix_eq,'file','H11_matrix');
% 
 %Left Foot (Pt_10)
 %Cam 2: FR:          
mark2Pos = (transpose(R_body)*p8+T_B2C_Front)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
mark2Pos = RB2C_Front*mark2Pos; 
 mark2Pos = [mark2Pos(2);mark2Pos(3);mark2Pos(1);1] ;
 mark2Pos=mark2Pos.';
 h12_eqn=(mark2Pos)*camMatrix_FR;
 w=h12_eqn(3);                       %compute scaling factor
 h12_eqn=[h12_eqn(1)/w;h12_eqn(2)/w];  %get x y points    (pixel coordinates)
 h12_eqn = simplify(h12_eqn);
 h12_eqn=h12_eqn-c_FR;
 matlabFunction(h12_eqn,'file','h12_eqn');
%%Jacobian
 H12_matrix_eq=jacobian(h12_eqn,X);
 H12_matrix_eq = simplify(H12_matrix_eq);
 matlabFunction(H12_matrix_eq,'file','H12_matrix');
%  

% %Left Calf (Pt_8)
% %Cam 3: BL:          
% mark3Pos = (transpose(R_body)*p9+T_B2C_Back)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
% mark3Pos = RB2C_Back*mark3Pos;
% %Rz=[cos(pi),-sin(pi),0;sin(pi),cos(pi),0;0,0,1];
% %mark1Pos = Rz*mark1Pos;
% mark3Pos = [-mark3Pos(2);-mark3Pos(3);mark3Pos(1);1];    
%  mark3Pos=mark3Pos.';
%  h13_eqn=(mark3Pos)*camMatrix_BL;
%  w=h13_eqn(3);                       %compute scaling factor
%  h13_eqn=[h13_eqn(1)/w;h13_eqn(2)/w];  %get x y points    (pixel coordinates)
%  h13_eqn = simplify(h13_eqn); 
%  h13_eqn=h13_eqn-c_BL;
%  matlabFunction(h13_eqn,'file','h13_eqn');
%  
% %%Jacobian
%  H13_matrix_eq=jacobian(h13_eqn,X);
%  H13_matrix_eq = simplify(H13_matrix_eq);
%  matlabFunction(H13_matrix_eq,'file','H13_matrix'); 
%  
%  %Left Calf (Pt_8)
% %Cam 4: BR:          
% mark3Pos = (transpose(R_body)*p9+T_B2C_Back)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
% mark3Pos = RB2C_Back*mark3Pos;
% %Rz=[cos(pi),-sin(pi),0;sin(pi),cos(pi),0;0,0,1];
% %mark1Pos = Rz*mark1Pos;
% mark3Pos = [-mark3Pos(2);-mark3Pos(3);mark3Pos(1);1];    
%  mark3Pos=mark3Pos.';
%  h14_eqn=(mark3Pos)*camMatrix_BR;
%  w=h14_eqn(3);                       %compute scaling factor
%  h14_eqn=[h14_eqn(1)/w;h14_eqn(2)/w];  %get x y points    (pixel coordinates)
%  h14_eqn = simplify(h14_eqn); 
%  h14_eqn=h14_eqn-c_BR;
%  matlabFunction(h14_eqn,'file','h14_eqn');
%  
% %%Jacobian
%  H14_matrix_eq=jacobian(h14_eqn,X);
%  H14_matrix_eq = simplify(H14_matrix_eq);
%  matlabFunction(H14_matrix_eq,'file','H14_matrix'); 
 
 %Left Heel (Pt_12)
%Cam 3: BL:          
mark4Pos = (transpose(R_body)*p10+T_B2C_Back)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
mark4Pos = RB2C_Back*mark4Pos;
%Rz=[cos(pi),-sin(pi),0;sin(pi),cos(pi),0;0,0,1];
%mark1Pos = Rz*mark1Pos;
mark4Pos = [-mark4Pos(2);-mark4Pos(3);mark4Pos(1);1];    
 mark4Pos=mark4Pos.';
 h15_eqn=(mark4Pos)*camMatrix_BL;
 w=h15_eqn(3);                       %compute scaling factor
 h15_eqn=[h15_eqn(1)/w;h15_eqn(2)/w];  %get x y points    (pixel coordinates)
 h15_eqn = simplify(h15_eqn); 
 h15_eqn=h15_eqn-c_BL;
 matlabFunction(h15_eqn,'file','h15_eqn');
 
%%Jacobian
 H15_matrix_eq=jacobian(h15_eqn,X);
 H15_matrix_eq = simplify(H15_matrix_eq);
 matlabFunction(H15_matrix_eq,'file','H15_matrix'); 
 
 
 %Left Heel (Pt_12)
%Cam 4: BR:          
mark4Pos = (transpose(R_body)*p10+T_B2C_Back)*1000;%TODO      %*1000, world points in mm!     camera offset, relative position
mark4Pos = RB2C_Back*mark4Pos;
%Rz=[cos(pi),-sin(pi),0;sin(pi),cos(pi),0;0,0,1];
%mark1Pos = Rz*mark1Pos;
mark4Pos = [-mark4Pos(2);-mark4Pos(3);mark4Pos(1);1];    
 mark4Pos =mark4Pos.';
 h16_eqn=(mark4Pos)*camMatrix_BR;
 w=h16_eqn(3);                       %compute scaling factor
 h16_eqn=[h16_eqn(1)/w;h16_eqn(2)/w];  %get x y points    (pixel coordinates)
 h16_eqn = simplify(h16_eqn); 
 h16_eqn=h16_eqn-c_BL;
 matlabFunction(h16_eqn,'file','h16_eqn');
 
%%Jacobian
 H16_matrix_eq=jacobian(h16_eqn,X);
 H16_matrix_eq = simplify(H16_matrix_eq);
 matlabFunction(H16_matrix_eq,'file','H16_matrix'); 